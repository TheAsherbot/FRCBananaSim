/**
 * @file SwerveDriveSystem.h
 * @brief Physical swerve drivetrain simulation system driving joints from SimInputs and populating SimOutputs.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Control/SimControlBoundary.h"

namespace FRCBananaSim::Drivetrain::System
{
    /**
     * @class SwerveDriveSystem
     * @brief Translates motor commands into physical torques, simulates voltage sag, and reads encoders.
     */
    class SwerveDriveSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the swerve drive physical simulation system.
         * @param scene Reference to the governing ECS scene.
         */
        explicit SwerveDriveSystem(Core::ECS::Core::Scene& scene);

        /**
         * @brief Virtual destructor.
         */
        virtual ~SwerveDriveSystem() override = default;

        /**
         * @brief Advances physical drivetrain actuation, calculates electrical sag, and extracts telemetry.
         * @param dt Fixed delta time in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Injects incoming control inputs (from HALSim WebSocket or test harness).
         * @param inputs Commanded motor voltages/currents and robot states.
         */
        void SetInputs(const Control::SimInputs& inputs);

        /**
         * @brief Retrieves live simulation outputs (for HALSim WebSocket or test assertions).
         * @return Const reference to latest SimOutputs.
         */
        const Control::SimOutputs& GetOutputs() const;

        /**
         * @brief Retrieves mutable simulation outputs.
         * @return Mutable reference to SimOutputs.
         */
        Control::SimOutputs& GetOutputs();

    private:
        Control::SimInputs currentInputs;
        Control::SimOutputs currentOutputs;
        double lastTotalBusCurrent = 1.5; ///< Breaks the battery-motor algebraic loop.
    };
}