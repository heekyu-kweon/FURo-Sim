// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/asv/api/AsvRpcLibClient.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
// UE defines a "check" macro that conflicts with the msgpack library, so it is undefined around the rpclib headers
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/asv/api/AsvRpcLibAdaptors.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif

    namespace msr
{
    namespace airlib
    {

        typedef msr::airlib_rpclib::AsvRpcLibAdaptors AsvRpcLibAdaptors;

        AsvRpcLibClient::AsvRpcLibClient(const string& ip_address, uint16_t port, float timeout_sec)
            : RpcLibClientBase(ip_address, port, timeout_sec)
        {
        }

        AsvRpcLibClient::~AsvRpcLibClient()
        {
        }

        void AsvRpcLibClient::setAsvControls(const AsvApiBase::AsvControls& controls, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->call("setAsvControls", AsvRpcLibAdaptors::AsvControls(controls), vehicle_name);
        }

        AsvApiBase::AsvState AsvRpcLibClient::getAsvState(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->call("getAsvState", vehicle_name).as<AsvRpcLibAdaptors::AsvState>().to();
        }
        AsvApiBase::AsvControls AsvRpcLibClient::getAsvControls(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->call("getAsvControls", vehicle_name).as<AsvRpcLibAdaptors::AsvControls>().to();
        }
    }
} //namespace

#endif
#endif
