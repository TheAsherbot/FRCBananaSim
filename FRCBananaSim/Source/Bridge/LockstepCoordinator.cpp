/**
 * @file LockstepCoordinator.cpp
 * @brief Implementation of the 4-tick lockstep barrier and SimHooks protocol frames.
 */

#include "Bridge/LockstepCoordinator.h"
#include <nlohmann/json.hpp>

namespace FRCBananaSim::Bridge
{
    LockstepCoordinator::LockstepCoordinator(const LockstepConfig& config)
        : config(config),
        currentTickInCycle(0),
        isWaitingForRobot(false),
        completedCycles(0)
    {}

    LockstepCoordinator::~LockstepCoordinator()
    {}

    void LockstepCoordinator::SetEnabled(bool enabled)
    {
        config.enabled = enabled;
        if (!enabled)
        {
            isWaitingForRobot = false;
            currentTickInCycle = 0;
        }
    }

    bool LockstepCoordinator::IsEnabled() const
    {
        return config.enabled;
    }

    void LockstepCoordinator::OnPhysicsTick()
    {
        if (!config.enabled)
        {
            return;
        }

        ++currentTickInCycle;
    }

    bool LockstepCoordinator::IsBarrierReached() const
    {
        if (!config.enabled)
        {
            return false;
        }

        return currentTickInCycle >= config.ticksPerStep;
    }

    bool LockstepCoordinator::CanAdvancePhysics() const
    {
        if (!config.enabled)
        {
            return true;
        }

        if (currentTickInCycle >= config.ticksPerStep && isWaitingForRobot)
        {
            return false;
        }

        return true;
    }

    void LockstepCoordinator::NotifyBarrierDispatched()
    {
        isWaitingForRobot = true;
    }

    void LockstepCoordinator::OnRobotStepAcknowledged()
    {
        if (!config.enabled)
        {
            return;
        }

        if (currentTickInCycle >= config.ticksPerStep)
        {
            currentTickInCycle = 0;
            isWaitingForRobot = false;
            ++completedCycles;
        }
    }

    std::string LockstepCoordinator::GeneratePauseTimingFrame() const
    {
        nlohmann::json root = {
            { "type", "SimHooks" },
            { "device", "" },
            { "data", {
                { ">pause_timing", true }
            } }
        };
        return root.dump();
    }

    std::string LockstepCoordinator::GenerateResumeTimingFrame() const
    {
        nlohmann::json root = {
            { "type", "SimHooks" },
            { "device", "" },
            { "data", {
                { ">resume_timing", true }
            } }
        };
        return root.dump();
    }

    std::string LockstepCoordinator::GenerateStepTimingFrame(double intervalMs) const
    {
        nlohmann::json root = {
            { "type", "SimHooks" },
            { "device", "" },
            { "data", {
                { ">step_timing_ms", intervalMs }
            } }
        };
        return root.dump();
    }

    bool LockstepCoordinator::ProcessIncomingFrame(std::string_view jsonStr)
    {
        try
        {
            auto root = nlohmann::json::parse(jsonStr);
            if (root.contains("type"))
            {
                std::string type = root["type"].get<std::string>();
                if (type == "SimHooks" || type == "DriverStation" || type == "SimDevice")
                {
                    OnRobotStepAcknowledged();
                    return true;
                }
            }
        }
        catch (const std::exception&)
        {
        }
        return false;
    }

    void LockstepCoordinator::Reset()
    {
        currentTickInCycle = 0;
        isWaitingForRobot = false;
        completedCycles = 0;
    }

    uint64_t LockstepCoordinator::GetCompletedCycles() const
    {
        return completedCycles;
    }

    double LockstepCoordinator::GetPeriodicIntervalMs() const
    {
        return config.robotPeriodicIntervalMs;
    }

    uint32_t LockstepCoordinator::GetCurrentTickInCycle() const
    {
        return currentTickInCycle;
    }
}