// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpuSidescanSonar_hpp
#define msr_airlib_GpuSidescanSonar_hpp

#include <random>
#include "common/Common.hpp"
#include "GpuSidescanSonarSimpleParams.hpp"
#include "GpuSidescanSonarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class GpuSidescanSonarSimple : public GpuSidescanSonarBase
    {
    public:
        GpuSidescanSonarSimple(const AirSimSettings::GpuSidescanSonarSetting& setting = AirSimSettings::GpuSidescanSonarSetting())
            : GpuSidescanSonarBase(setting.sensor_name)
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
            GpuSidescanSonarBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                updateOutput();
            }
        }

        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            GpuSidescanSonarBase::reportState(reporter);

            reporter.writeValue("GpuSidescanSonar-NumLaserSamples", params_.number_of_laser_samples);
            reporter.writeValue("GpuSidescanSonar-NumSamples", params_.number_of_samples);
            reporter.writeValue("GpuSidescanSonar-NumBeams", params_.number_of_beams);
            reporter.writeValue("GpuSidescanSonar-Range", params_.max_range);
            reporter.writeValue("GpuSidescanSonar-MaxBounceRay", params_.max_bounce_ray);
            reporter.writeValue("GpuSidescanSonar-MaxDiffuseRayNum", params_.max_diffuse_ray_num);
            reporter.writeValue("GpuSidescanSonar-Vertical-FOV", params_.vertical_FOV);
            reporter.writeValue("GpuSidescanSonar-Horizontal-FOV", params_.horizontal_FOV);
        }
        //*** End: UpdatableState implementation ***//

        virtual ~GpuSidescanSonarSimple() = default;

        const GpuSidescanSonarSimpleParams& getParams() const
        {
            return params_;
        }

    protected:
        virtual void getSidescanData(const Pose& sonar_pose, const Pose& vehicle_pose, 
                                     vector<real_T>& port_point_cloud, vector<int>& port_segmentation_cloud,
                                     vector<real_T>& starboard_point_cloud, vector<int>& starboard_segmentation_cloud,
                                     vector<real_T>& port_raw_data, vector<real_T>& starboard_raw_data,
                                     msr::airlib::vector<int>& port_segmentation, msr::airlib::vector<int>& starboard_segmentation) = 0;

    private: //methods
        void updateOutput()
        {
            port_raw_data_.clear();
            port_segmentation_.clear();
            port_point_cloud_.clear();
            port_segmentation_cloud_.clear();

            starboard_raw_data_.clear();
            starboard_segmentation_.clear();
            starboard_point_cloud_.clear();
            starboard_segmentation_cloud_.clear();

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
            getSidescanData(params_.relative_pose, // relative lidar pose
                            ground_truth.kinematics->pose, // relative vehicle pose
                            port_point_cloud_,
                            port_segmentation_cloud_,
                            starboard_point_cloud_,
                            starboard_segmentation_cloud_,
                            port_raw_data_,
                            starboard_raw_data_,
                            port_segmentation_,
                            starboard_segmentation_);

            GpuSidescanSonarData output;
            output.port_point_cloud = port_point_cloud_;
            output.starboard_point_cloud = starboard_point_cloud_;
            output.port_segmentation_cloud = port_segmentation_cloud_;
            output.starboard_segmentation_cloud = starboard_segmentation_cloud_;
            output.time_stamp = clock()->nowNanos();
            output.pose = sonar_pose;
            output.port_raw_data = port_raw_data_;
            output.starboard_raw_data = starboard_raw_data_;
            output.port_segmentation = port_segmentation_;
            output.starboard_segmentation = starboard_segmentation_;
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
        GpuSidescanSonarSimpleParams params_;
        vector<real_T> port_raw_data_;
        vector<int> port_segmentation_;
        vector<real_T> port_point_cloud_;
        vector<int> port_segmentation_cloud_;

        vector<real_T> starboard_raw_data_;
        vector<int> starboard_segmentation_;
        vector<real_T> starboard_point_cloud_;
        vector<int> starboard_segmentation_cloud_;

        FrequencyLimiter freq_limiter_;
        TTimePoint last_time_;
    };
}
} //namespace
#endif
