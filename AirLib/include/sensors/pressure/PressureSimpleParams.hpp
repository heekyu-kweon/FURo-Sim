// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_PressureSimpleParams_hpp
#define msr_airlib_PressureSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    struct PressureSimpleParams
    {
        real_T pressure_factor_sigma = 0.0005f;
        real_T pressure_factor_tau = 1800.0f;
        real_T uncorrelated_noise_sigma = 150.0f;

        real_T update_latency = 0.0f; //sec
        real_T update_frequency = 50; //Hz
        real_T startup_delay = 0; //sec

        Pose relative_pose = Pose::zero();

        void initializeFromSettings(const AirSimSettings::PressureSetting& settings)
        {
            const auto& json = settings.settings;
            relative_pose = AirSimSettings::createPoseSetting(json);
            pressure_factor_sigma = json.getFloat("PressureFactorSigma", pressure_factor_sigma);
            pressure_factor_tau = json.getFloat("PressureFactorTau", pressure_factor_tau);
            uncorrelated_noise_sigma = json.getFloat("UncorrelatedNoiseSigma", uncorrelated_noise_sigma);
            update_latency = json.getFloat("UpdateLatency", update_latency);
            update_frequency = json.getFloat("UpdateFrequency", update_frequency);
            startup_delay = json.getFloat("StartupDelay", startup_delay);
        }
    };
}
} //namespace
#endif
