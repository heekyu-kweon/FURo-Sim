// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AsvSimpleController_hpp
#define msr_airlib_AsvSimpleController_hpp

#include "vehicles/asv/api/AsvApiBase.hpp"

namespace msr
{
namespace airlib
{

    class AsvSimpleApi : public AsvApiBase
    {
    public:
        AsvSimpleApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
                    const Kinematics::State& state, const Environment& environment)
            : AsvApiBase(vehicle_setting, sensor_factory, state, environment), home_geopoint_(environment.getHomeGeoPoint())
        {
            params_.surface_constrained = vehicle_setting->surface_constrained;
            params_.surface_constrained_auto = vehicle_setting->surface_constrained_auto;
            params_.hull_buoyancy = vehicle_setting->hull_buoyancy;
            params_.draw_debug_buoyancy = vehicle_setting->draw_debug_buoyancy;
            if (!std::isnan(vehicle_setting->mass))
                params_.mass = vehicle_setting->mass;
            for (int i = 0; i < 3; ++i)
                if (!std::isnan(vehicle_setting->center_of_gravity[i]))
                    params_.center_of_gravity[i] = vehicle_setting->center_of_gravity[i];
        }

        ~AsvSimpleApi()
        {
        }

    protected:
        virtual void resetImplementation() override
        {
            AsvApiBase::resetImplementation();
        }

    public:
        virtual void update() override
        {
            AsvApiBase::update();
        }

        virtual const SensorCollection& getSensors() const override
        {
            return AsvApiBase::getSensors();
        }

        // VehicleApiBase Implementation
        virtual void enableApiControl(bool is_enabled) override
        {
            if (api_control_enabled_ != is_enabled) {
                last_controls_ = AsvControls();
                api_control_enabled_ = is_enabled;
            }
        }

        virtual bool isApiControlEnabled() const override
        {
            return api_control_enabled_;
        }

        virtual GeoPoint getHomeGeoPoint() const override
        {
            return home_geopoint_;
        }

        virtual bool armDisarm(bool arm) override
        {
            //TODO: implement arming for asv
            unused(arm);
            return true;
        }

    public:
        virtual void setAsvControls(const AsvControls& controls) override
        {
            last_controls_ = controls;
        }

        virtual void updateAsvState(const AsvState& asv_state) override
        {
            last_asv_state_ = asv_state;
        }

        virtual const AsvState& getAsvState() const override
        {
            return last_asv_state_;
        }

        virtual const AsvControls& getAsvControls() const override
        {
            return last_controls_;
        }

        virtual const AsvVehicleParams& getVehicleParams() const override
        {
            return params_;
        }

    private:
        bool api_control_enabled_ = false;
        GeoPoint home_geopoint_;
        AsvControls last_controls_;
        AsvState last_asv_state_;
        AsvVehicleParams params_;  // default values defined in AsvParams.hpp
    };
}
}

#endif
