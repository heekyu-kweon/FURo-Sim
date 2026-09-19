// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AuvSimpleController_hpp
#define msr_airlib_AuvSimpleController_hpp

#include "vehicles/auv/api/AuvApiBase.hpp"
#include "vehicles/auv/firmwares/auv_simple/AuvAutopilot.hpp"
#include <mutex>
#include <thread>
#include <chrono>

namespace msr
{
namespace airlib
{

    class AuvSimpleApi : public AuvApiBase
    {
    public:
        AuvSimpleApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
                    const Kinematics::State& state, const Environment& environment)
            : AuvApiBase(vehicle_setting, sensor_factory, state, environment), home_geopoint_(environment.getHomeGeoPoint())
        {
            autopilot_.init(params_);
        }

        ~AuvSimpleApi()
        {
        }

    protected:
        virtual void resetImplementation() override
        {
            AuvApiBase::resetImplementation();
            std::lock_guard<std::mutex> lock(autopilot_mutex_);
            autopilot_.clear();
            last_update_nanos_ = 0;
        }

    public:
        virtual void update() override
        {
            AuvApiBase::update();

            const TTimePoint now = clock()->nowNanos();
            const float dt = last_update_nanos_ == 0 ? 0.0f
                                                     : Utils::clip(static_cast<float>((now - last_update_nanos_) * 1e-9), 1e-3f, 0.2f);
            last_update_nanos_ = now;

            std::lock_guard<std::mutex> lock(autopilot_mutex_);
            if (api_control_enabled_ && autopilot_.isActive() && dt > 0.0f)
                last_controls_ = AuvControls(autopilot_.update(last_auv_state_.kinematics_estimated, dt));
        }

        virtual const SensorCollection& getSensors() const override
        {
            return AuvApiBase::getSensors();
        }

        // VehicleApiBase Implementation
        virtual void enableApiControl(bool is_enabled) override
        {
            if (api_control_enabled_ != is_enabled) {
                std::lock_guard<std::mutex> lock(autopilot_mutex_);
                autopilot_.clear();
                last_controls_ = AuvControls();
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
            //TODO: implement arming for auv
            unused(arm);
            return true;
        }

    public:
        virtual const AuvVehicleParams& getVehicleParams() const override
        {
            return params_;
        }

        virtual void setAuvControls(const AuvControls& controls) override
        {
            std::lock_guard<std::mutex> lock(autopilot_mutex_);
            autopilot_.clear();
            last_controls_ = controls;
        }

        virtual void updateAuvState(const AuvState& auv_state) override
        {
            last_auv_state_ = auv_state;
        }

        virtual const AuvState& getAuvState() const override
        {
            return last_auv_state_;
        }

        virtual const AuvControls& getAuvControls() const override
        {
            return last_controls_;
        }

        virtual void setOceanCurrent(const Vector3r& current) override
        {
            ocean_current_ = current;
        }

        virtual Vector3r getOceanCurrent() const override
        {
            return ocean_current_;
        }

        /************************* high level move APIs *********************************/
        virtual bool moveToPosition(float x, float y, float z, float velocity, float timeout_sec, const YawMode& yaw_mode) override
        {
            return moveOnPath({ Vector3r(x, y, z) }, velocity, timeout_sec, yaw_mode);
        }

        virtual bool moveToZ(float z, float velocity, float timeout_sec, const YawMode& yaw_mode) override
        {
            const Vector3r pos = last_auv_state_.getPosition();
            return moveOnPath({ Vector3r(pos.x(), pos.y(), z) }, velocity, timeout_sec, yaw_mode);
        }

        virtual bool moveOnPath(const vector<Vector3r>& path, float velocity, float timeout_sec, const YawMode& yaw_mode) override
        {
            if (path.empty() || velocity <= 0.0f)
                return false;

            SingleTaskCall task(this);
            {
                std::lock_guard<std::mutex> lock(autopilot_mutex_);
                autopilot_.setPath(path, velocity, yaw_mode, last_auv_state_.kinematics_estimated);
            }

            const bool arrived = waitForTaskDone(timeout_sec);
            if (!arrived) {
                std::lock_guard<std::mutex> lock(autopilot_mutex_);
                autopilot_.setHold(last_auv_state_.getPosition(), YawMode(), last_auv_state_.kinematics_estimated);
            }
            return arrived;
        }

        virtual bool hover() override
        {
            SingleTaskCall task(this);
            std::lock_guard<std::mutex> lock(autopilot_mutex_);
            autopilot_.setHold(last_auv_state_.getPosition(), YawMode(), last_auv_state_.kinematics_estimated);
            return true;
        }

    private:
        // polls the autopilot on sim-clock time so simPause and clock_speed stretch the timeout
        // the same way as the motion, returns early on cancelLastTask or a newer move call
        bool waitForTaskDone(float timeout_sec)
        {
            if (timeout_sec <= 0.0f)
                return false;

            const TTimePoint start = clock()->nowNanos();
            while (!getCancelToken().isCancelled()) {
                {
                    std::lock_guard<std::mutex> lock(autopilot_mutex_);
                    if (autopilot_.isDone())
                        return true;
                }
                if (clock()->elapsedSince(start) >= timeout_sec)
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        }

    private:
        bool api_control_enabled_ = false;
        GeoPoint home_geopoint_;
        AuvControls last_controls_;
        AuvState last_auv_state_;
        AuvVehicleParams params_;                    // default values defined in AuvParams.hpp
        Vector3r ocean_current_ = Vector3r::Zero();  // NED world frame [m/s]

        AuvAutopilot autopilot_;
        std::mutex autopilot_mutex_;
        TTimePoint last_update_nanos_ = 0;
    };
}
}

#endif
