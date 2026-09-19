// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Pressure_hpp
#define msr_airlib_Pressure_hpp

#include <random>
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "PressureSimpleParams.hpp"
#include "PressureBase.hpp"
#include "common/GaussianMarkov.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class PressureSimple : public PressureBase
    {
    public:
        PressureSimple(const AirSimSettings::PressureSetting& setting = AirSimSettings::PressureSetting())
            : PressureBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //GM process that would do random walk for pressure factor
            pressure_factor_.initialize(params_.pressure_factor_tau, params_.pressure_factor_sigma, 0);

            uncorrelated_noise_ = RandomGeneratorGausianR(0.0f, params_.uncorrelated_noise_sigma);
            //correlated_noise_.initialize(params_.correlated_noise_tau, params_.correlated_noise_sigma, 0.0f);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
            delay_line_.initialize(params_.update_latency);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            pressure_factor_.reset();
            //correlated_noise_.reset();
            uncorrelated_noise_.reset();

            freq_limiter_.reset();
            delay_line_.reset();

            delay_line_.push_back(getOutputInternal());
        }

        virtual void update() override
        {
            PressureBase::update();

            freq_limiter_.update();

            if (freq_limiter_.isWaitComplete()) {
                delay_line_.push_back(getOutputInternal());
            }

            delay_line_.update();

            if (freq_limiter_.isWaitComplete())
                setOutput(delay_line_.getOutput());
        }
        //*** End: UpdatableState implementation ***//

        virtual ~PressureSimple() = default;

    private: //methods
        Output getOutputInternal()
        {
            Output output;
            const GroundTruth& ground_truth = getGroundTruth();

            auto depth = std::max<real_T>(-ground_truth.environment->getState().geo_point.altitude, 0.0f);
            auto gravity = EarthUtils::getGravity(ground_truth.environment->getState().geo_point.altitude);
            auto pressure = EarthUtils::SeaLevelPressure + EarthUtils::getFluidDensity() * gravity * depth;

            pressure_factor_.update();
            pressure += pressure * pressure_factor_.getOutput();

            pressure += uncorrelated_noise_.next();
            pressure = std::max(pressure, EarthUtils::SeaLevelPressure);

            output.pressure = pressure;
            output.depth = (pressure - EarthUtils::SeaLevelPressure) / (EarthUtils::getFluidDensity() * gravity);

            output.time_stamp = clock()->nowNanos();

            return output;
        }

    public:
        virtual Pose getRelativePose() const override
        {
            return params_.relative_pose;
        }

    private:
        PressureSimpleParams params_;

        GaussianMarkov pressure_factor_;
        RandomGeneratorGausianR uncorrelated_noise_;

        FrequencyLimiter freq_limiter_;
        DelayLine<Output> delay_line_;
    };
}
} //namespace
#endif
