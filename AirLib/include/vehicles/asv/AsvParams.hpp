// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_AsvParams_hpp
#define air_AsvParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

/**
 * AsvVehicleParams
 *
 * Physical and hydrodynamic parameters for a catamaran-type ASV (surface vehicle).
 * Models 3-DOF horizontal plane motion: surge (u), sway (v), yaw (r).
 * In full 6-DOF fluid-flux mode (surface_constrained = false), Ixx/Iyy are also used.
 *
 * All quantities use SI units.
 * Coordinate convention: NED body frame (X forward, Y starboard, Z down).
 * Notation follows SNAME (1950) / Fossen convention.
 *
 * Reference: T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
 */
struct AsvVehicleParams
{
    // simulation mode
    // true: water surface constraint (lock Z translation and X/Y rotation, disable gravity)
    // false: full 6-DOF simulation on the fluid-flux water surface
    bool surface_constrained = true;
    // when "SurfaceConstrained" is not given in settings the physics body decides at init from the pawn:
    // a fluid-flux buoyancy component on the pawn selects 6-DOF, otherwise the DOF lock above
    bool surface_constrained_auto = true;
    // true: buoyancy from the hull's own collision volume (voxelised in C++) on the fluid-flux water surface,
    //       the fluid-flux Blueprint pontoon buoyancy on the pawn is switched off
    // false: leave buoyancy to the fluid-flux Blueprint component (settings "HullBuoyancy")
    bool hull_buoyancy = true;
    // settings "DrawDebugBuoyancy": draw submerged hull voxels, water samples, CoM and CoB in the viewport
    // (console variable furosim.Asv.DrawBuoyancy overrides at runtime)
    bool draw_debug_buoyancy = false;

    // basic properties
    // Source: BlueBoat datasheet v1.1 (Jan 2025), bluerobotics.com
    float mass = 18.5f;  // [kg]     operating mass: 14.5 kg dry + two battery packs and a light payload
    float Izz  = 1.02f;  // [kg.m^2]  yaw   moment of inertia (0.80 at 14.5 kg, scaled with the mass)
    float Ixx  = 0.26f;  // [kg.m^2]  roll  moment of inertia (6-DOF mode only)
    float Iyy  = 0.77f;  // [kg.m^2]  pitch moment of inertia (6-DOF mode only)

    // geometry
    // All positions in NED body frame, relative to the mesh/physics origin. [m]
    //
    // center_of_gravity: CoM position from the mesh origin (settings "CenterOfGravity" {X,Y,Z}).
    //   NaN components are resolved at start-up: x/y at the hull's centre of buoyancy (even keel),
    //   z at the collision mesh's own centre of mass.
    Vector3r center_of_gravity = VectorMath::nanVector();  // [m] NED

    // thruster_left_pos / thruster_right_pos:
    //   Position of each thruster's thrust application point relative to CoM. [m] NED
    //   BlueBoat: thrusters mounted ~0.35 m port/starboard of centerline,
    //             roughly at CoM longitudinal station, at waterline depth.
    //   -> Mz = r_left x F_left + r_right x F_right  (2-D: Mz = -y_L*F_L - y_R*F_R
    //           but r_L.y = -half_beam, r_R.y = +half_beam, so Mz = (F_R - F_L)*half_beam)
    Vector3r thruster_left_pos  = Vector3r(-0.5f, -0.3f, 0.28f);  // [m] NED  (port)
    Vector3r thruster_right_pos = Vector3r(-0.5f,  0.3f, 0.28f);  // [m] NED  (starboard)

    float max_thrust_N = 50.0f;  // [N]  max forward thrust per M200 (5.1 kgf @ 16V)

    // linear damping
    // D_lin = diag(Xu, Yv, Nr)
    float Xu = -5.0f;   // [N.s/m]      surge linear damping
    float Yv = -8.0f;   // [N.s/m]      sway  linear damping
    float Nr = -2.0f;   // [N.m.s/rad]  yaw   linear damping
    // sway/yaw cross terms. Nv > 0 is the twin-hull keel effect that gives course stability against
    // the Munk moment (m22-m11)uv. Linearised at u=1.5 m/s, stability needs Nv > ~8.
    float Yr = 0.0f;    // [N.s/rad]    sway force from yaw rate
    float Nv = 12.0f;   // [N.m.s/m]    yaw moment from sway velocity

    // quadratic damping
    // D_quad = diag(Xuu|u|, Yvv|v|, Nrr|r|)
    float Xuu = -10.0f;  // [N.s^2/m^2]      surge quadratic damping
    float Yvv = -15.0f;  // [N.s^2/m^2]      sway  quadratic damping
    float Nrr =  -5.0f;  // [N.m.s^2/rad^2]  yaw   quadratic damping

    // added mass
    // M_A = diag(-Xdu, -Ydv): used for added mass Coriolis (C_A)
    float Xdu = -2.0f;   // [kg]   surge added mass
    float Ydv = -8.0f;   // [kg]   sway  added mass
    float Ndr = -0.3f;   // [kg.m^2] yaw   added inertia (N_rdot)

    // heave damping (surface_constrained = false only). Restoring comes from the fluid-flux buoyancy.
    // Zw/Kp/Mq are set for damping ratios of ~0.35-0.45 against the pontoon restoring stiffness measured
    // on the Calm ocean map (k_heave 1650 N/m, k_roll 164 N.m/rad, k_pitch 157 N.m/rad, 2026-09-13):
    // the 0.2-0.6 s natural modes are then quiet while the hull still tracks a 3 s swell with <8 deg lag.
    float Zw  = -110.0f; // [N.s/m]     heave linear damping
    float Zww = -60.0f;  // [N.s^2/m^2]   heave quadratic damping

    // 6-DOF roll/pitch damping (surface_constrained = false only)
    // Restoring moments are handled externally by fluid flux.
    float Kp = -5.0f;   // [N.m.s/rad]  roll  damping
    float Mq = -9.0f;   // [N.m.s/rad]  pitch damping
};

} //namespace airlib
} //namespace msr

#endif