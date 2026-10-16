/**
 * @file FixedTimestep.cpp
 * @brief Implementation of fixed-timestep accumulator logic and spiral-of-death clamping.
 */

#include "Utils/FixedTimestep.h"
#include "Utils/Log.h"
#include <chrono>

namespace FRCBananaSim::Utils
{
    static double NowSeconds()
    {
        using clock = std::chrono::steady_clock;
        using sec = std::chrono::duration<double>;
        return sec(clock::now().time_since_epoch()).count();
    }

    FixedTimestep::FixedTimestep(double hz)
        : fixedDelta(1.0 / hz),
        accumulator(0.0),
        lastTime(NowSeconds()),
        frameDelta(0.0)
    {}

    void FixedTimestep::SetFrequency(double hz)
    {
        fixedDelta = 1.0 / hz;
    }

    double FixedTimestep::FixedDeltaSeconds() const
    {
        return fixedDelta;
    }

    std::uint32_t FixedTimestep::Step()
    {
        double now = NowSeconds();
        frameDelta = now - lastTime;
        lastTime = now;

        // Clamp to avoid spiral of death
        double maxFrameDelta = fixedDelta * 4.0;
        if (frameDelta > maxFrameDelta)
        {
            SIM_LOG_WARN("Frame delta clamped: " << frameDelta << "s -> " << maxFrameDelta
                << "s (possible stall - breakpoint, OS hitch, or heavy frame)");
            frameDelta = maxFrameDelta;
        }

        accumulator += frameDelta;

        std::uint32_t steps = 0;
        while (accumulator >= fixedDelta)
        {
            accumulator -= fixedDelta;
            ++steps;
        }

        return steps;
    }

    double FixedTimestep::Alpha() const
    {
        return accumulator / fixedDelta;
    }

    double FixedTimestep::DeltaSeconds() const
    {
        return frameDelta;
    }
}