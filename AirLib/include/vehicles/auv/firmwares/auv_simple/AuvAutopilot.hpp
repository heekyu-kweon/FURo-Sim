// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AuvAutopilot_hpp
#define msr_airlib_AuvAutopilot_hpp

#include "common/Common.hpp"
#include "common/VectorMath.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/auv/AuvParams.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"

namespace msr
{
namespace airlib
{

    /**
     * AuvAutopilot
     *
     * Guidance + control behind the AUV high-level move APIs. Two modes:
     *   Path : 3D line-of-sight following of a waypoint list. The commanded
     *          velocity points at a lookahead point on the path and slows down
     *          near the final waypoint. Completes inside waypoint_radius of the
     *          last point and then holds station there.
     *   Hold : station keeping at a fixed NED point (hover / moveToZ target).
     *
     * Both feed a world-frame velocity PI loop whose force is rotated into the
     * body frame; yaw is a PD loop on heading, roll/pitch get rate damping only
     * (the CoG/CoB offset provides the restoring moment). Frames: positions and
     * velocities NED world, output wrench NED body, matching AuvControls.
     */
    class AuvAutopilot
    {
    public:
        enum class Mode
        {
            Idle,
            Path,
            Hold
        };

        void init(const AuvVehicleParams& params)
        {
            params_ = params;
            clear();
        }

        void clear()
        {
            mode_ = Mode::Idle;
            done_ = false;
            path_.clear();
            seg_index_ = 0;
            vel_integral_ = Vector3r::Zero();
            has_prev_yaw_ = false;
        }

        void setPath(const vector<Vector3r>& waypoints, float speed, const YawMode& yaw_mode, const Kinematics::State& state)
        {
            startTask(state);
            mode_ = Mode::Path;
            speed_ = speed;
            yaw_mode_ = yaw_mode;
            path_.clear();
            path_.push_back(state.pose.position);
            for (const auto& wp : waypoints)
                path_.push_back(wp);
            seg_index_ = 0;
        }

        void setHold(const Vector3r& target, const YawMode& yaw_mode, const Kinematics::State& state)
        {
            startTask(state);
            mode_ = Mode::Hold;
            speed_ = params_.autopilot.hold_max_speed;
            yaw_mode_ = yaw_mode;
            hold_target_ = target;
        }

        bool isActive() const
        {
            return mode_ != Mode::Idle;
        }

        bool isDone() const
        {
            return done_;
        }

        Wrench update(const Kinematics::State& state, float dt)
        {
            if (mode_ == Mode::Idle)
                return Wrench::zero();

            const Vector3r pos = state.pose.position;
            const Quaternionr q = state.pose.orientation;

            Vector3r v_des = Vector3r::Zero();
            if (mode_ == Mode::Path) {
                v_des = pathGuidance(pos);
            }
            if (mode_ == Mode::Hold) {
                v_des = params_.autopilot.kp_pos * (hold_target_ - pos);
                v_des = clampNorm(v_des, speed_);
            }

            const Vector3r force_world = velocityLoop(v_des, state.twist.linear, dt);
            const Vector3r force_body = VectorMath::transformToBodyFrame(force_world, q, true);

            const Vector3r torque_body = attitudeLoop(v_des, q, state.twist.angular, dt);

            return Wrench(force_body, torque_body);
        }

    private:
        void startTask(const Kinematics::State& state)
        {
            done_ = false;
            vel_integral_ = Vector3r::Zero();
            yaw_des_ = VectorMath::yawFromQuaternion(state.pose.orientation);
            prev_yaw_ = yaw_des_;
            has_prev_yaw_ = true;
        }

        // LOS guidance: advance along the path, aim at a lookahead point, slow down at the end.
        // Switches to Hold at the final waypoint once inside waypoint_radius.
        Vector3r pathGuidance(const Vector3r& pos)
        {
            const auto& ap = params_.autopilot;
            const size_t last_seg = path_.size() - 2;

            while (seg_index_ < last_seg) {
                const Vector3r& a = path_[seg_index_];
                const Vector3r& b = path_[seg_index_ + 1];
                const float t = projectOnSegment(pos, a, b);
                if (t >= 1.0f || (b - pos).norm() < ap.waypoint_radius)
                    ++seg_index_;
                else
                    break;
            }

            const Vector3r& final_pt = path_.back();
            const float dist_final = (final_pt - pos).norm();
            if (seg_index_ == last_seg && dist_final < ap.waypoint_radius) {
                done_ = true;
                mode_ = Mode::Hold;
                hold_target_ = final_pt;
                speed_ = ap.hold_max_speed;
                return clampNorm(ap.kp_pos * (final_pt - pos), speed_);
            }

            const Vector3r target = lookaheadPoint(pos);
            Vector3r dir = target - pos;
            const float dist_target = dir.norm();
            if (dist_target < 1e-4f)
                return Vector3r::Zero();
            dir /= dist_target;

            float v_cmd = speed_;
            if (dist_final < ap.slowdown_dist)
                v_cmd = std::max(ap.min_speed, speed_ * dist_final / ap.slowdown_dist);
            return dir * v_cmd;
        }

        // Lookahead stays on the current segment (clamped at its end) so the vehicle passes through each
        // waypoint instead of cutting the corner onto the next leg.
        Vector3r lookaheadPoint(const Vector3r& pos) const
        {
            const Vector3r& a = path_[seg_index_];
            const Vector3r& b = path_[seg_index_ + 1];
            const float t = Utils::clip(projectOnSegment(pos, a, b), 0.0f, 1.0f);
            const Vector3r point = a + (b - a) * t;

            const Vector3r to_end = b - point;
            const float len = to_end.norm();
            if (len <= params_.autopilot.lookahead || len < 1e-6f)
                return b;
            return point + to_end * (params_.autopilot.lookahead / len);
        }

        static float projectOnSegment(const Vector3r& p, const Vector3r& a, const Vector3r& b)
        {
            const Vector3r ab = b - a;
            const float len2 = ab.squaredNorm();
            if (len2 < 1e-8f)
                return 1.0f;
            return (p - a).dot(ab) / len2;
        }

        Vector3r velocityLoop(const Vector3r& v_des, const Vector3r& v, float dt)
        {
            const auto& ap = params_.autopilot;
            const Vector3r err = v_des - v;

            const float integral_limit = 0.5f * ap.max_force / std::max(ap.ki_vel, 1e-3f);
            vel_integral_ += err * dt;
            vel_integral_ = clampAbs(vel_integral_, integral_limit);

            // Feedforward: cancel net buoyancy (acts upward, NED -z) and the hydrodynamic drag expected at the
            // commanded speed, using the surge coefficients as an isotropic estimate. Without it the per-axis
            // integrator needs ~(kp+D)/ki seconds to close a 10-15% speed deficit on every new leg.
            const float v_cmd = v_des.norm();
            const float drag_gain = -(params_.xu + params_.xuu * v_cmd);
            const Vector3r feedforward = Vector3r(0.0f, 0.0f, params_.buoyancy - params_.weight) + drag_gain * v_des;

            Vector3r force = ap.kp_vel * err + ap.ki_vel * vel_integral_ + feedforward;
            return clampAbs(force, ap.max_force);
        }

        Vector3r attitudeLoop(const Vector3r& v_des, const Quaternionr& q, const Vector3r& body_rates, float dt)
        {
            const auto& ap = params_.autopilot;

            const float yaw = VectorMath::yawFromQuaternion(q);
            const float yaw_rate = has_prev_yaw_ ? wrapPi(yaw - prev_yaw_) / dt : 0.0f;
            prev_yaw_ = yaw;
            has_prev_yaw_ = true;

            if (!yaw_mode_.is_rate)
                yaw_des_ = Utils::degreesToRadians(yaw_mode_.yaw_or_rate);
            else if (v_des.head<2>().norm() > 0.05f)
                yaw_des_ = std::atan2(v_des.y(), v_des.x());

            const float yaw_err = wrapPi(yaw_des_ - yaw);
            const float tau_yaw = ap.kp_yaw * yaw_err - ap.kd_yaw * yaw_rate;
            const float tau_roll = -ap.kd_roll * body_rates.x();
            const float tau_pitch = -ap.kd_pitch * body_rates.y();

            return clampAbs(Vector3r(tau_roll, tau_pitch, tau_yaw), ap.max_torque);
        }

        static float wrapPi(float angle)
        {
            while (angle > M_PIf)
                angle -= 2.0f * M_PIf;
            while (angle < -M_PIf)
                angle += 2.0f * M_PIf;
            return angle;
        }

        static Vector3r clampAbs(const Vector3r& v, float limit)
        {
            return Vector3r(
                Utils::clip(v.x(), -limit, limit),
                Utils::clip(v.y(), -limit, limit),
                Utils::clip(v.z(), -limit, limit));
        }

        static Vector3r clampNorm(const Vector3r& v, float max_norm)
        {
            const float n = v.norm();
            if (n > max_norm && n > 1e-6f)
                return v * (max_norm / n);
            return v;
        }

    private:
        AuvVehicleParams params_;
        Mode mode_ = Mode::Idle;
        bool done_ = false;

        vector<Vector3r> path_;
        size_t seg_index_ = 0;
        float speed_ = 0.0f;
        YawMode yaw_mode_;
        Vector3r hold_target_ = Vector3r::Zero();

        Vector3r vel_integral_ = Vector3r::Zero();
        float yaw_des_ = 0.0f;
        float prev_yaw_ = 0.0f;
        bool has_prev_yaw_ = false;
    };
}
} //namespace

#endif
