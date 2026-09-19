// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Dvl_hpp
#define msr_airlib_Dvl_hpp

#include <random>
#include "common/Common.hpp"
#include "DvlSimpleParams.hpp"
#include "DvlBase.hpp"
#include "common/GaussianMarkov.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class DvlSimple : public DvlBase
    {
    public:
        DvlSimple(const AirSimSettings::DvlSetting& setting = AirSimSettings::DvlSetting())
            : DvlBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            gauss_altitude = RandomGeneratorGausianR(0.0f, params_.alti.uncorrelated_noise_sigma);
            gauss_depth = RandomGeneratorGausianR(0.0f, params_.depth_aiding_sigma);
            gauss_vrw = RandomVectorGaussianR(0.0f, params_.vel.vrw);
            //correlated_noise_.initialize(params_.correlated_noise_tau, params_.correlated_noise_sigma, 0.0f);

            vel_bias_stability_norm = params_.vel.bias_stability / sqrt(params_.vel.tau);
            gyro_bias_stability_norm = params_.gyro.bias_stability / sqrt(params_.gyro.tau);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
            delay_line_.initialize(params_.update_latency);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            last_time_ = clock()->nowNanos();

            //correlated_noise_.reset();
            gauss_altitude.reset();
            gauss_depth.reset();
            gauss_vrw.reset();
            gauss_dist.reset();
            gauss_.reset();
            state_.velocimeter_bias = params_.vel.turn_on_bias;
            state_.gyroscope_bias = params_.gyro.turn_on_bias;
            state_.orientation_bias = Quaternionr(1, 0, 0, 0);

            freq_limiter_.reset();
            delay_line_.reset();

            delay_line_.push_back(getOutputInternal());
        }

        virtual void update() override
        {
            DvlBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                delay_line_.push_back(getOutputInternal());
            }

            delay_line_.update();

            if (freq_limiter_.isWaitComplete())
                setOutput(delay_line_.getOutput());
        }
        //*** End: UpdatableState implementation ***//

        virtual ~DvlSimple() = default;

        const DvlSimpleParams& getParams() const
        {
            return params_;
        }

    protected:
        virtual real_T getRayLength(const Pose& pose) = 0;
        virtual void postprocessBeamData(DvlData& /*output*/, const Pose& /*sensor_world_pose*/) {}

    private: //methods
        DvlData getOutputInternal()
        {
            DvlData output;
            const GroundTruth& ground_truth = getGroundTruth();

            //order of Pose addition is important here because it also adds quaternions which is not commutative!
            const Pose sensor_world_pose = params_.alti.relative_pose + ground_truth.kinematics->pose;
            auto altitude = getRayLength(sensor_world_pose);

            //add noise in altitude (about 0.2m sigma)
            altitude += gauss_altitude.next();

            output.velocity = ground_truth.kinematics->twist.linear;
            output.angular_velocity = ground_truth.kinematics->twist.angular;

            output.altitude = altitude;
            output.min_distance = params_.alti.min_distance;
            output.max_distance = params_.alti.max_distance;
            output.relative_pose = params_.alti.relative_pose;

            //velocity is in world frame so transform to body frame
            output.velocity = VectorMath::transformToBodyFrame(output.velocity,
                                                               ground_truth.kinematics->pose.orientation,
                                                               true);
            estimated_pose.orientation = ground_truth.kinematics->pose.orientation;

            //add noise
            dt_ = clock()->updateSince(last_time_);
            addNoise(output.velocity, output.angular_velocity, estimated_pose.orientation);

            Quaternionr del_orientation  = VectorMath::toQuaternion(output.angular_velocity.y()*dt_,//pitch
                                                                    output.angular_velocity.x()*dt_,//roll
                                                                    output.angular_velocity.z()*dt_);//yaw

            if (VectorMath::hasNan(estimated_pose))
                estimated_pose = ground_truth.kinematics->pose;

            // output.velocity = output.velocity;

            // orientation_ = VectorMath::rotateQuaternion(orientation_, del_orientation, true);
            estimated_pose.orientation = estimated_pose.orientation * del_orientation;

            estimated_pose.position += estimated_pose.orientation * output.velocity * dt_;

            //pressure-depth aiding: real AUVs bound depth with a cm-accurate pressure sensor, never
            //dead-reckon z from DVL velocity. gt z = pressure depth in estimated_pose's NED frame + noise.
            estimated_pose.position.z() = ground_truth.kinematics->pose.position.z() + gauss_depth.next();

            output.estimated_pose = estimated_pose;

            output.time_stamp = clock()->nowNanos();

            postprocessBeamData(output, sensor_world_pose);

            return output;
        }
        
        void addNoise(Vector3r& velocity, Vector3r& angular_velocity, Quaternionr& orientation)
        {
            // TTimeDelta dt = clock()->updateSince(last_time_);

            //ref: An introduction to inertial navigation, Oliver J. Woodman, Sec 3.2, pp 10-12
            //https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf

            real_T sqrt_dt = static_cast<real_T>(sqrt(std::max<TTimeDelta>(dt_, 1 / params_.update_frequency)));

            Vector3r vrw_sigma = gauss_vrw.next();
            // Vector3r linear_vel_sigma = gauss_dist.next() * params_.vel.linear_velocity_noise_sigma;
            // Vector3r angular_vel_sigma = gauss_dist.next() * params_.vel.angular_velocity_noise_sigma;

            // velocity += vrw_sigma + linear_vel_sigma.cwiseProduct(velocity) + angular_vel_sigma * angular_velocity.norm();
            velocity += vrw_sigma + state_.velocimeter_bias;
            //update bias random walk
            real_T vel_sigma_bias = vel_bias_stability_norm * sqrt_dt;
            state_.velocimeter_bias += gauss_dist.next() * vel_sigma_bias;

            real_T gyro_sigma_arw = params_.gyro.arw / sqrt_dt;
            // Orientation
            orientation = orientation * state_.orientation_bias;

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(state_.orientation_bias,
                                        pitch, roll, yaw); // pitch roll yaw
            state_.orientation_bias = VectorMath::toQuaternion(0,
                                                               0,
                                                               0.99 * yaw);

            //gravity-referenced AUV: only yaw/heading drifts, pitch/roll stay level (accelerometer).
            //Injecting gyro pitch/roll bias here leaked horizontal velocity into the dead-reckoned z.
            const real_T bias_dt = static_cast<real_T>(dt_);
            state_.orientation_bias = state_.orientation_bias * 
                                        VectorMath::toQuaternion(0,//pitch: gravity-referenced, no drift
                                                                 0,//roll: gravity-referenced, no drift
                                                                 state_.gyroscope_bias.z() * bias_dt);//yaw

            state_.orientation_bias = state_.orientation_bias * 
                                        VectorMath::toQuaternion(0,//pitch
                                                                 0,//roll
                                                                 gauss_.next() * gyro_sigma_arw);//yaw
            // Gyrosocpe
            //convert arw to stddev

            angular_velocity += gauss_dist.next() * gyro_sigma_arw + state_.gyroscope_bias;
            //update bias random walk
            real_T gyro_sigma_bias = gyro_bias_stability_norm * sqrt_dt;
            state_.gyroscope_bias += gauss_dist.next() * gyro_sigma_bias;
        }

    public:
        virtual Pose getRelativePose() const override
        {
            return params_.alti.relative_pose;
        }

    private:
        DvlSimpleParams params_;
        RandomVectorGaussianR gauss_dist = RandomVectorGaussianR(0, 1);
        RandomVectorGaussianR gauss_vrw;
        RandomGeneratorGausianR gauss_altitude;
        RandomGeneratorGausianR gauss_depth;
        RandomGeneratorGausianR gauss_ = RandomGeneratorGausianR(0.0f, 1);

        //cached calculated values
        real_T vel_bias_stability_norm, gyro_bias_stability_norm;

        // Quaternionr orientation_ = VectorMath::nanQuaternion();//estimated orientation
        Pose estimated_pose = VectorMath::Pose::nanPose();//estimated pose

        struct State
        {
            Vector3r velocimeter_bias;
            Vector3r gyroscope_bias;
            Quaternionr orientation_bias;
        } state_;

        FrequencyLimiter freq_limiter_;
        DelayLine<DvlData> delay_line_;

        TTimePoint last_time_;
        TTimeDelta dt_;
    };
}
} //namespace
#endif
