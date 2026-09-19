// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpuSonar_hpp
#define msr_airlib_GpuSonar_hpp

#include <random>
#include "common/Common.hpp"
#include "GpuSonarSimpleParams.hpp"
#include "GpuSonarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class GpuSonarSimple : public GpuSonarBase
    {
    public:
        GpuSonarSimple(const AirSimSettings::GpuSonarSetting& setting = AirSimSettings::GpuSonarSetting())
            : GpuSonarBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            freq_limiter_.reset();
            last_time_ = clock()->nowNanos();

            updateOutput();
        }

        virtual void update() override
        {
            GpuSonarBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                updateOutput();
            }
        }

        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            GpuSonarBase::reportState(reporter);

            reporter.writeValue("GpuSonar-NumLaserSamples", params_.number_of_laser_samples);
            reporter.writeValue("GpuSonar-NumSamples", params_.number_of_samples);
            reporter.writeValue("GpuSonar-NumBeams", params_.number_of_beams);
            reporter.writeValue("GpuSonar-Range", params_.max_range);
            reporter.writeValue("GpuSonar-MaxBounceRay", params_.max_bounce_ray);
            reporter.writeValue("GpuSonar-MaxDiffuseRayNum", params_.max_diffuse_ray_num);
            reporter.writeValue("GpuSonar-Vertical-FOV", params_.vertical_FOV);
            reporter.writeValue("GpuSonar-Horizontal-FOV", params_.horizontal_FOV);
        }
        //*** End: UpdatableState implementation ***//

        virtual ~GpuSonarSimple() = default;

        const GpuSonarSimpleParams& getParams() const
        {
            return params_;
        }

    protected:
        virtual void getSonarData(const Pose& sonar_pose, const Pose& vehicle_pose, 
                                  vector<real_T>& point_cloud, vector<int>& segmentation_cloud,
                                  vector<real_T>& sonar_raw_data, msr::airlib::vector<int>& segmentation_sonar) = 0;

    private: //methods
        void updateOutput()
        {
            point_cloud_.clear();
            segmentation_cloud_.clear();
            sonar_raw_data_.clear();
            segmentation_sonar_.clear();

            const GroundTruth& ground_truth = getGroundTruth();

            // calculate the pose before obtaining the point-cloud. Before/after is a bit arbitrary
            // decision here. If the pose can change while obtaining the point-cloud (could happen for drones)
            // then the pose won't be very accurate either way.
            //
            // TODO: Seems like pose is in vehicle inertial-frame (NOT in Global NED frame).
            //    That could be a bit unintuitive but seems consistent with the position/orientation returned as part of
            //    ImageResponse for cameras and pose returned by getCameraInfo API.
            //    Do we need to convert pose to Global NED frame before returning to clients?            
            Pose sonar_pose = params_.relative_pose + ground_truth.kinematics->pose;
            getSonarData(params_.relative_pose, // relative lidar pose
                          ground_truth.kinematics->pose, // relative vehicle pose
                          point_cloud_,
                          segmentation_cloud_,
                          sonar_raw_data_,
                          segmentation_sonar_);

            GpuSonarData output;
            output.point_cloud = point_cloud_;
            output.time_stamp = clock()->nowNanos();
            output.pose = sonar_pose;
            output.segmentation = segmentation_cloud_;
            output.sonar_raw_data = sonar_raw_data_;
            output.segmentation_sonar = segmentation_sonar_;
            output.min_range = params_.min_range;
            output.max_range = params_.max_range;

            last_time_ = output.time_stamp;

            setOutput(output);
        }

    public:
        virtual Pose getRelativePose() const override
        {
            return params_.relative_pose;
        }

    private:
        GpuSonarSimpleParams params_;
        vector<real_T> point_cloud_;
        vector<int> segmentation_cloud_;
        vector<real_T> sonar_raw_data_;
        vector<int> segmentation_sonar_;

        FrequencyLimiter freq_limiter_;
        TTimePoint last_time_;
    };
}
} //namespace
#endif
