// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpuSidescanSonarSimpleParams_hpp
#define msr_airlib_GpuSidescanSonarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    struct GpuSidescanSonarSimpleParams
    {

        // default settings

        std::string sensor_name = "";
        uint number_of_laser_samples = 456;
        uint number_of_samples = 1024;
        uint number_of_beams = 10;
        real_T max_range = 75.0f; // meters
        real_T min_range = 0.5f; // meters
        real_T horizontal_FOV = 80.0f; // degrees
        real_T vertical_FOV = 2.0f; // degrees
        real_T transmission_loss_scale = 0.5f;
        real_T rayleigh_noise_scale = 0.1f; // scale of rayleigh noise, between 0 and 1, larger value means more intensity noise
        real_T range_noise_sigma = 0.01f; // meter
        uint max_bounce_ray = 2; // max bounce ray
        uint max_diffuse_ray_num = 0; // max diffuse ray number
        uint max_depth = 4 * max_bounce_ray; // max depth of ray trace

        Pose relative_pose{
            Vector3r(0, 0, -1), // position - a little above vehicle (especially for cars) or Vector3r::Zero()
            Quaternionr::Identity() // orientation - by default Quaternionr(1, 0, 0, 0)
        };

        bool draw_debug_points = false;
        bool draw_beam = false;
        bool draw_fov = false;
        bool open_sonar_viewer = true; // whether to open sonar viewer window
        // Anamorphic render: render target width = number_of_beams * horizontal_render_scale.
        // scale=1 = pure anamorphic (min GPU). Raise if horizontal post-process sampling suffers at 1px width.
        uint horizontal_render_scale = 1;
        bool enable_beam_pattern = true;
        bool enable_transmission_loss = true;
        bool enable_rayleigh_noise = true;
        bool enable_range_noise = true;
        // Multiplicative speckle: I' = I * X, X ~ Gamma(shape, scale). With E[X]=shape*scale=1 the
        // sample is mean-preserving; CV = 1/sqrt(shape). Default (25, 0.04) gives CV=0.2 to match
        // the forward GpuSonar speckle model. Applied AFTER transmission loss so the absolute speckle
        // amplitude (= I * (X-1)) scales with the signal and therefore decreases with range.
        bool enable_speckle_multiplicative = false;
        real_T speckle_shape = 25.0f;
        real_T speckle_scale = 0.04f;
        AirSimSettings::GpuSidescanSonarSetting::DataFrame data_frame;

        bool external_controller = true;

        real_T update_frequency = 10; // Hz
        real_T startup_delay = 0; // sec

        void initializeFromSettings(const AirSimSettings::GpuSidescanSonarSetting& settings)
        {
            std::string simmode_name = AirSimSettings::singleton().simmode_name;
            sensor_name = settings.sensor_name;

            const auto& settings_json = settings.settings;
            number_of_laser_samples = settings_json.getInt("NumberOfLaserSamples", number_of_laser_samples);
            number_of_samples = settings_json.getInt("NumberOfSamples", number_of_samples);
            number_of_beams = settings_json.getInt("NumberOfBeams", number_of_beams);
            max_range = settings_json.getFloat("MaxRange", max_range);
            min_range = settings_json.getFloat("MinRange", min_range);
            update_frequency = settings_json.getInt("MaxFrameRate", update_frequency);
            max_bounce_ray = settings_json.getInt("MaxBounceRay", max_bounce_ray);
            max_diffuse_ray_num = settings_json.getInt("MaxDiffuseRayNum", max_diffuse_ray_num);
            draw_debug_points = settings_json.getBool("DrawDebugPoints", draw_debug_points);
            draw_beam = settings_json.getBool("DrawDebugBeam", settings_json.getBool("DrawBeam", draw_beam));
            draw_fov = settings_json.getBool("DrawDebugFov", settings_json.getBool("DrawFovGizmo", draw_fov));
            open_sonar_viewer = settings_json.getBool("OpenSonarViewer", open_sonar_viewer);
            horizontal_render_scale = std::max(1, settings_json.getInt("HorizontalRenderScale", static_cast<int>(horizontal_render_scale)));
            enable_beam_pattern = settings_json.getBool("EnableBeamPattern", enable_beam_pattern);
            enable_transmission_loss = settings_json.getBool("EnableTransmissionLoss", enable_transmission_loss);
            transmission_loss_scale = settings_json.getFloat("TransmissionLossScale", transmission_loss_scale);
            enable_rayleigh_noise = settings_json.getBool("EnableRayleighNoise", enable_rayleigh_noise);
            rayleigh_noise_scale = settings_json.getFloat("RayleighNoiseScale", rayleigh_noise_scale);
            enable_range_noise = settings_json.getBool("EnableRangeNoise", enable_range_noise);
            range_noise_sigma = settings_json.getFloat("RangeNoiseSigma", range_noise_sigma);
            enable_speckle_multiplicative = settings_json.getBool("EnableSpeckleMultiplicative", enable_speckle_multiplicative);
            speckle_shape = settings_json.getFloat("SpeckleShape", speckle_shape);
            speckle_scale = settings_json.getFloat("SpeckleScale", speckle_scale);
            std::string frame = settings_json.getString("DataFrame", AirSimSettings::kVehicleInertialFrame);
            if (frame == AirSimSettings::kVehicleInertialFrame) {
                data_frame = AirSimSettings::GpuSidescanSonarSetting::DataFrame::VehicleInertialFrame;
            }
            else if (frame == AirSimSettings::kSensorLocalFrame) {
                data_frame = AirSimSettings::GpuSidescanSonarSetting::DataFrame::SensorLocalFrame;
            }
            else {
                throw std::runtime_error("Unknown requested data frame");
            }
            external_controller = settings_json.getBool("ExternalController", external_controller);

            vertical_FOV = settings_json.getFloat("VerticalFOV", vertical_FOV);
            horizontal_FOV = settings_json.getFloat("HorizontalFOV", horizontal_FOV);

            relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
            auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

            if (std::isnan(relative_pose.position.x()))
                relative_pose.position.x() = 0;
            if (std::isnan(relative_pose.position.y()))
                relative_pose.position.y() = 0;
            if (std::isnan(relative_pose.position.z())) {
                if (simmode_name == AirSimSettings::kSimModeTypeMultirotor ||
                    simmode_name == AirSimSettings::kSimModeTypeAuv)
                    relative_pose.position.z() = 0;
                else
                    relative_pose.position.z() = -1; // a little bit above for cars
            }

            float pitch, roll, yaw;
            pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
            roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
            yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
            relative_pose.orientation = VectorMath::toQuaternion(
                Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
                Utils::degreesToRadians(roll), // roll  - rotation around X axis
                Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis
        }
    };
}
} //namespace
#endif
