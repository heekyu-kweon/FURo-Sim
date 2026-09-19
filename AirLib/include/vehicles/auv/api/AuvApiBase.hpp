// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_AuvApiBase_hpp
#define air_AuvApiBase_hpp

#include "common/VectorMath.hpp"
// #include "AuvCommon.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/auv/AuvParams.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "common/CancelToken.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"

namespace msr
{
namespace airlib
{

    class AuvApiBase : public VehicleApiBase
    {
    public:
        struct AuvControls
        {
            Wrench wrench = Wrench::zero();  // body-frame [N] / [N.m]

            AuvControls() {}
            AuvControls(const Vector3r& force, const Vector3r& torque)
                : wrench(force, torque) {}
            AuvControls(const Wrench& wrench)
                : wrench(wrench) {}
        };


        struct AuvState
        {
            Kinematics::State kinematics_estimated;
            uint64_t timestamp;

            AuvState()
            {
            }

            AuvState(const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
                : kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const Vector3r& getPosition() const
            {
                return kinematics_estimated.pose.position;
            }
            const Quaternionr& getOrientation() const
            {
                return kinematics_estimated.pose.orientation;
            }
        };
    
    public:
        /************************* high level move APIs *********************************/
        // Waypoints and z are NED [m], velocity is the surge speed cap [m/s]. Blocking: returns true on
        // arrival, false on timeout or cancel (cancelLastTask / a newer move call). The vehicle holds
        // station where it ends up. yaw_mode.is_rate=true means face the direction of travel;
        // is_rate=false holds the absolute heading yaw_or_rate [deg].
        virtual bool moveToPosition(float x, float y, float z, float velocity, float timeout_sec, const YawMode& yaw_mode) = 0;
        virtual bool moveOnPath(const vector<Vector3r>& path, float velocity, float timeout_sec, const YawMode& yaw_mode) = 0;
        virtual bool moveToZ(float z, float velocity, float timeout_sec, const YawMode& yaw_mode) = 0;
        virtual bool hover() = 0;

        virtual void cancelLastTask() override
        {
            token_.cancel();
        }

        CancelToken& getCancelToken()
        {
            return token_;
        }

    protected:
        // Serializes move calls the same way MultirotorApiBase::SingleTaskCall does: a new call cancels the
        // one in flight and waits for it to unwind before taking over, so the older call's cleanup can never
        // overwrite the newer task.
        class SingleTaskCall
        {
        public:
            SingleTaskCall(AuvApiBase* api)
                : api_(api)
            {
                auto& token = api_->getCancelToken();
                if (!token.try_lock()) {
                    token.cancel();
                    token.lock();
                }
                if (token.getRecursionCount() == 1)
                    token.reset();
            }

            ~SingleTaskCall()
            {
                auto& token = api_->getCancelToken();
                if (token.getRecursionCount() == 1)
                    token.reset();
                token.unlock();
            }

        private:
            AuvApiBase* api_;
        };

    public:
        // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
        //AuvApiBase() {}

        AuvApiBase(const AirSimSettings::VehicleSetting* vehicle_setting,
                   std::shared_ptr<SensorFactory> sensor_factory,
                   const Kinematics::State& state, const Environment& environment)
        {
            initialize(vehicle_setting, sensor_factory, state, environment);
        }

        virtual void update() override
        {
            VehicleApiBase::update();

            getSensors().update();
        }

        void reportState(StateReporter& reporter) override
        {
            getSensors().reportState(reporter);
        }

        // sensor helpers
        virtual const SensorCollection& getSensors() const override
        {
            return sensors_;
        }

        SensorCollection& getSensors()
        {
            return sensors_;
        }

        void initialize(const AirSimSettings::VehicleSetting* vehicle_setting,
                        std::shared_ptr<SensorFactory> sensor_factory,
                        const Kinematics::State& state, const Environment& environment)
        {
            sensor_factory_ = sensor_factory;

            sensor_storage_.clear();
            sensors_.clear();

            addSensorsFromSettings(vehicle_setting);

            getSensors().initialize(&state, &environment);
        }

        void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            const auto& sensor_settings = vehicle_setting->sensors;

            sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
        }

        virtual void setAuvControls(const AuvControls& controls) = 0;
        virtual void updateAuvState(const AuvState& state) = 0;
        virtual const AuvState& getAuvState() const = 0;
        virtual const AuvControls& getAuvControls() const = 0;
        virtual const AuvVehicleParams& getVehicleParams() const = 0;

        // Ocean current velocity in NED world frame [m/s].
        // Hydrodynamic forces are computed relative to this fluid velocity.
        virtual void setOceanCurrent(const Vector3r& current) { unused(current); }
        virtual Vector3r getOceanCurrent() const { return Vector3r::Zero(); }

        virtual ~AuvApiBase() = default;

        std::shared_ptr<const SensorFactory> sensor_factory_;
        SensorCollection sensors_; //maintains sensor type indexed collection of sensors
        vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors

    protected:
        virtual void resetImplementation() override
        {
            //reset sensors last after their ground truth has been reset
            getSensors().reset();
        }
    
    private: //variables
        CancelToken token_;
    };
}
} //namespace
#endif
