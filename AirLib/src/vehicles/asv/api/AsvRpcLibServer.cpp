// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/asv/api/AsvRpcLibServer.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
// UE defines a "check" macro that conflicts with the msgpack library, so it is undefined around the rpclib headers
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/asv/api/AsvRpcLibAdaptors.hpp"

STRICT_MODE_ON

namespace msr
{
namespace airlib
{

    typedef msr::airlib_rpclib::AsvRpcLibAdaptors AsvRpcLibAdaptors;

    AsvRpcLibServer::AsvRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port)
        : RpcLibServerBase(api_provider, server_address, port)
    {
        (static_cast<rpc::server*>(getServer()))->bind("getAsvState", [&](const std::string& vehicle_name) -> AsvRpcLibAdaptors::AsvState {
            return AsvRpcLibAdaptors::AsvState(getVehicleApi(vehicle_name)->getAsvState());
        });

        (static_cast<rpc::server*>(getServer()))->bind("setAsvControls", [&](const AsvRpcLibAdaptors::AsvControls& controls, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setAsvControls(controls.to());
        });
        (static_cast<rpc::server*>(getServer()))->bind("getAsvControls", [&](const std::string& vehicle_name) -> AsvRpcLibAdaptors::AsvControls {
            return AsvRpcLibAdaptors::AsvControls(getVehicleApi(vehicle_name)->getAsvControls());
        });
    }

    //required for pimpl
    AsvRpcLibServer::~AsvRpcLibServer()
    {
    }
}
} //namespace

#endif
#endif
