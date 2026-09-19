// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_UnrealClock_hpp
#define airsim_core_UnrealClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"
#include <atomic>
#include <thread>
#include <chrono>

namespace msr
{
namespace airlib
{

    // UnrealClock: sim time advances ONLY when stepBy() is called externally
    // (driven by UE Tick's DeltaSeconds). Use this to keep AirSim's clock
    // aligned with UE simulation time even when Chaos Physics runs slower
    // or faster than wall-clock time.
    //
    // max_step_seconds caps each stepBy increment so sim time stays in sync
    // with the physics integrator (UE clamps physics dt to MaxPhysicsDeltaTime,
    // default 1/30s). 0 disables the cap.
    class UnrealClock : public ClockBase
    {
    public:
        UnrealClock(TTimeDelta max_step_seconds = 0)
            : max_step_seconds_(max_step_seconds)
        {
            TTimePoint t0 = Utils::getTimeSinceEpochNanos();
            start_ = t0;
            current_.store(t0);
        }

        virtual ~UnrealClock() {}

        virtual TTimePoint nowNanos() const override
        {
            return current_.load();
        }

        virtual TTimePoint getStart() const override
        {
            return start_;
        }

        // Driven by UE Tick(DeltaSeconds). amount is in seconds.
        // Clamped to max_step_seconds_ so sim time tracks the physics integrator
        // (which UE clamps to MaxPhysicsDeltaTime).
        virtual TTimePoint stepBy(TTimeDelta amount) override
        {
            if (max_step_seconds_ > 0 && amount > max_step_seconds_)
                amount = max_step_seconds_;
            TTimePoint cur = current_.load();
            TTimePoint next = addTo(cur, amount);
            current_.store(next);
            return next;
        }

        // No-op: UnrealClock advances ONLY via stepBy(DeltaSeconds) from UE Tick.
        // Bare step() calls (e.g. from World::update) must not advance sim time
        // independently of UE.
        virtual TTimePoint step() override
        {
            return current_.load();
        }

        // wall-clock sleep: sim time only advances when UE Tick fires, so spin-waiting
        // on sim time from a non-game thread would deadlock while the game thread is paused
        virtual void sleep_for(TTimeDelta dt) override
        {
            if (dt <= 0)
                return;
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }

    private:
        std::atomic<TTimePoint> current_;
        TTimePoint start_;
        TTimeDelta max_step_seconds_;
    };
}
} //namespace
#endif
