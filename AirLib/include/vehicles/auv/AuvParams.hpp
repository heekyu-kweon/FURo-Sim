// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_AuvParams_hpp
#define air_AuvParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

/**
 * AuvAutopilotParams
 *
 * Guidance and control gains used by the AUV high-level move APIs
 * (moveToPosition / moveOnPath / moveToZ / hover). The autopilot runs a
 * world-frame (NED) velocity PI loop fed by either a 3D line-of-sight
 * guidance law (path following) or a position P loop (station keeping),
 * plus a yaw PD loop and roll/pitch rate damping. Output is a body-frame
 * wrench that goes through the same setAuvControls() path as manual input.
 * Defaults are tuned for the BlueROV2-class coefficients in AuvVehicleParams.
 */
struct AuvAutopilotParams
{
    // guidance
    float waypoint_radius = 0.3f;  // [m]   arrival tolerance around a waypoint
    float lookahead       = 2.0f;  // [m]   LOS lookahead distance along the path
    float slowdown_dist   = 2.0f;  // [m]   start reducing speed within this distance of the final waypoint
    float min_speed       = 0.15f; // [m/s] floor for the reduced approach speed
    float hold_max_speed  = 0.5f;  // [m/s] speed cap for station keeping / moveToZ

    // position -> velocity outer loop (station keeping)
    float kp_pos = 0.8f;           // [1/s]

    // world-frame velocity PI loop -> force
    float kp_vel = 40.0f;          // [N/(m/s)]
    float ki_vel = 15.0f;          // [N/m]

    // yaw PD loop -> yaw torque
    float kp_yaw = 3.0f;           // [N.m/rad]
    float kd_yaw = 1.0f;           // [N.m.s/rad]

    // roll / pitch rate damping (restoring moment comes from the CoG/CoB offset)
    float kd_roll  = 1.0f;         // [N.m.s/rad]
    float kd_pitch = 1.0f;         // [N.m.s/rad]

    // actuator saturation (per axis)
    float max_force  = 60.0f;      // [N]
    float max_torque = 5.0f;       // [N.m]
};

/**
 * AuvVehicleParams
 *
 * Physical and hydrodynamic parameters for an AUV body.
 * All quantities use SI units; vectors are in NED frame.
 *
 * Notation follows SNAME (1950) convention:
 *   u/v/w  : surge / sway / heave  (linear)
 *   p/q/r  : roll  / pitch / yaw   (angular)
 */
struct AuvVehicleParams
{
    // basic properties
    float mass     = 11.5f;   // [kg]
    float weight   = 112.8f;  // [N]
    float buoyancy = 114.8f;  // [N]

    Vector3r center_of_gravity  = Vector3r(0.f, 0.f, 0.02f); // NED [m]
    Vector3r center_of_buoyancy = Vector3r::Zero();            // NED [m]

    // moments of inertia
    float Ixx = 0.16f, Iyy = 0.16f, Izz = 0.16f; // [kg.m^2]

    // linear damping
    float xu = -4.03f, xv = -6.22f, xw = -5.18f; // [N.s/m]
    float kp = -0.07f, mq = -0.07f, nr = -0.07f;  // [N.m.s/rad]

    // quadratic damping
    float xuu = -18.18f, xvv = -21.66f, xww = -36.99f; // [N.s^2/m^2]
    float kpp = -1.55f,  mqq = -1.55f,  nrr = -1.55f;  // [N.m.s^2/rad^2]

    // added mass
    float axu = -5.5f,  ayv = -12.7f,  azw = -14.57f; // [kg]
    float akp = -0.12f, amq = -0.12f,  anr = -0.12f;  // [kg.m^2]

    AuvAutopilotParams autopilot;
};

} //namespace airlib
} //namespace msr

#endif
