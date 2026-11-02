/**
 * @file LockstepCoordinator.h
 * @brief Coordinates the 4-tick (20 ms) lockstep synchronization barrier between physics and robot code.
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace FRCBananaSim::Bridge
{
    /**
     * @struct LockstepConfig
     * @brief Timing parameters for WPILib halsim_ws lockstep time-stepping.
     */
    struct LockstepConfig
    {
        bool enabled = false;                  ///< Whether lockstep synchronization is active.
        double robotPeriodicIntervalMs = 20.0; ///< RobotPeriodic loop interval in ms (default: 20 ms / 50 Hz).
        double physicsTickMs = 5.0;            ///< Simulation physics timestep in ms (default: 5 ms / 200 Hz).
        uint32_t ticksPerStep = 4;             ///< Physics sub-steps per robot periodic cycle (20.0 / 5.0 = 4).
        double timeoutMs = 2000.0;             ///< Maximum wait time for robot code response before warning.
    };

    /**
     * @class LockstepCoordinator
     * @brief Manages the 4-tick barrier between 200 Hz physics and 50 Hz robot periodic loops.
     */
    class LockstepCoordinator
    {
    public:
        /**
         * @brief Constructs lockstep coordinator.
         * @param config Timing and barrier parameters.
         */
        explicit LockstepCoordinator(const LockstepConfig& config = LockstepConfig{});

        /**
         * @brief Destructor.
         */
        ~LockstepCoordinator();

        /**
         * @brief Enables or disables lockstep synchronization.
         * @param enabled True to enforce step timing barriers; false for free-running simulation.
         */
        void SetEnabled(bool enabled);

        /**
         * @brief Checks if lockstep synchronization is currently active.
         * @return True if enabled.
         */
        bool IsEnabled() const;

        /**
         * @brief Records completion of a single 5 ms physics sub-step.
         */
        void OnPhysicsTick();

        /**
         * @brief Checks whether the physics step barrier has been reached (e.g. 4 ticks completed).
         * @return True if 4 ticks completed and waiting for robot code step acknowledge.
         */
        bool IsBarrierReached() const;

        /**
         * @brief Checks whether physics is permitted to advance the next tick.
         * @return True if permitted to advance.
         */
        bool CanAdvancePhysics() const;

        /**
         * @brief Marks the barrier as notified to robot code and enters awaiting state.
         */
        void NotifyBarrierDispatched();

        /**
         * @brief Acknowledges robot step completion (called upon receiving new inputs/commands).
         */
        void OnRobotStepAcknowledged();

        /**
         * @brief Generates JSON frame commanding robot to pause its internal FPGA timing.
         * @return JSON string for WebSocket transmission.
         */
        std::string GeneratePauseTimingFrame() const;

        /**
         * @brief Generates JSON frame commanding robot to resume continuous timing.
         * @return JSON string for WebSocket transmission.
         */
        std::string GenerateResumeTimingFrame() const;

        /**
         * @brief Generates JSON frame advancing robot code by the configured periodic interval.
         * @param intervalMs Optional step duration override in ms.
         * @return JSON string for WebSocket transmission.
         */
        std::string GenerateStepTimingFrame(double intervalMs = 20.0) const;

        /**
         * @brief Parses incoming JSON frame to detect SimHooks or robot periodic replies.
         * @param jsonStr Received WebSocket text.
         * @return True if frame acknowledged lockstep progress.
         */
        bool ProcessIncomingFrame(std::string_view jsonStr);

        /**
         * @brief Resets tick counters and state machine.
         */
        void Reset();

        /**
         * @brief Gets total completed lockstep cycles.
         * @return Total 20 ms cycles executed.
         */
        uint64_t GetCompletedCycles() const;

        /**
         * @brief Gets configured periodic interval in milliseconds.
         * @return Interval in ms (default 20.0).
         */
        double GetPeriodicIntervalMs() const;

        /**
         * @brief Gets current physics tick count within the active step cycle [0..ticksPerStep].
         * @return Sub-step index.
         */
        uint32_t GetCurrentTickInCycle() const;

    private:
        LockstepConfig config;
        uint32_t currentTickInCycle = 0;
        bool isWaitingForRobot = false;
        uint64_t completedCycles = 0;
    };
}