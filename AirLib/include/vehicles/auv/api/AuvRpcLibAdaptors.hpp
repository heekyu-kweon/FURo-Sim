// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_AuvRpcLibAdaptors_hpp
#define air_AuvRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/auv/api/AuvApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr
{
namespace airlib_rpclib
{

    class AuvRpcLibAdaptors : public RpcLibAdaptorsBase
    {
    public:
        struct YawMode
        {
            bool is_rate = true;
            float yaw_or_rate = 0;
            MSGPACK_DEFINE_ARRAY(is_rate, yaw_or_rate);

            YawMode()
            {
            }

            YawMode(const msr::airlib::YawMode& s)
            {
                is_rate = s.is_rate;
                yaw_or_rate = s.yaw_or_rate;
            }
            msr::airlib::YawMode to() const
            {
                return msr::airlib::YawMode(is_rate, yaw_or_rate);
            }
        };

        struct AuvControls
        {
            Vector3r force;   // body-frame force  [N]
            Vector3r torque;  // body-frame torque [N.m]

            MSGPACK_DEFINE_ARRAY(force, torque);

            AuvControls() {}

            AuvControls(const msr::airlib::AuvApiBase::AuvControls& s)
            {
                force  = s.wrench.force;
                torque = s.wrench.torque;
            }

            msr::airlib::AuvApiBase::AuvControls to() const
            {
                return msr::airlib::AuvApiBase::AuvControls(force.to(), torque.to());
            }
        };


        // struct ThrusterParameters
        // {
        //     msr::airlib::real_T thrust;
        //     msr::airlib::real_T torque_scaler;
        //     msr::airlib::real_T speed;

        //     MSGPACK_DEFINE_ARRAY(thrust, torque_scaler, speed);

        //     ThrusterParameters()
        //     {
        //     }

        //     ThrusterParameters(const msr::airlib::ThrusterParameters& s)
        //     {
        //         thrust = s.thrust;
        //         torque_scaler = s.torque_scaler;
        //         speed = s.speed;
        //     }

        //     msr::airlib::ThrusterParameters to() const
        //     {
        //         return msr::airlib::ThrusterParameters(thrust, torque_scaler, speed);
        //     }
        // };

        // struct ThrusterStates
        // {
        //     std::vector<ThrusterParameters> thrusters;
        //     uint64_t timestamp;

        //     MSGPACK_DEFINE_ARRAY(thrusters, timestamp);

        //     ThrusterStates()
        //     {
        //     }

        //     ThrusterStates(const msr::airlib::ThrusterStates& s)
        //     {
        //         for (const auto& r : s.thrusters) {
        //             thrusters.push_back(ThrusterParameters(r));
        //         }
        //         timestamp = s.timestamp;
        //     }

        //     msr::airlib::ThrusterStates to() const
        //     {
        //         std::vector<msr::airlib::ThrusterParameters> d;
        //         for (const auto& r : thrusters) {
        //             d.push_back(r.to());
        //         }
        //         return msr::airlib::ThrusterStates(d, timestamp);
        //     }
        // };

        struct AuvState
        {
            KinematicsState kinematics_estimated;
            uint64_t timestamp;

            MSGPACK_DEFINE_ARRAY(kinematics_estimated, timestamp);

            AuvState()
            {
            }

            AuvState(const msr::airlib::AuvApiBase::AuvState& s)
            {
                timestamp = s.timestamp;
                kinematics_estimated = s.kinematics_estimated;
            }
            msr::airlib::AuvApiBase::AuvState to() const
            {
                return msr::airlib::AuvApiBase::AuvState(
                    kinematics_estimated.to(), timestamp);
            }
        };
    };
}
} //namespace

#endif
