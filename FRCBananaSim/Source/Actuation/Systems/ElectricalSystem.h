/**
 * @file ElectricalSystem.h
 * @brief Stage 1 fixed-timestep system aggregating robot bus current and evaluating battery voltage sag and brownout stages.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Core/ECS/Core/Scene.h"
#include "Control/SimControlBoundary.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"

namespace FRCBananaSim::Actuation::Systems
{
    /**
     * @class ElectricalSystem
     * @brief FixedUpdate system aggregating current draw across all mechanisms and drivetrain, evaluating internal resistance sag.
     */
    class ElectricalSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the electrical power distribution system.
         * @param scene Reference to governing ECS scene.
         * @param swerveSystem Optional pointer to SwerveDriveSystem for drivetrain current aggregation.
         */
        ElectricalSystem(Core::ECS::Core::Scene& scene,
            FRCBananaSim::Robot::Drivetrain::Systems::SwerveDriveSystem* swerveSystem = nullptr);
        
        /**
         * @brief Virtual destructor.
         */
        virtual ~ElectricalSystem() override;

        /**
         * @brief Aggregates electrical load and calculates post-sag terminal voltage across timestep dt.
         * @param dt Timestep duration in seconds (0.005 s for 200 Hz).
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Sets pointer to SwerveDriveSystem for drivetrain current queries.
         * @param swerveSystem Pointer to SwerveDriveSystem.
         */
        void SetSwerveSystem(FRCBananaSim::Robot::Drivetrain::Systems::SwerveDriveSystem* swerveSystem);

        /**
         * @brief Sets external SimOutputs destination for electrical telemetry.
         * @param outputs Pointer to SimOutputs.
         */
        void SetOutputs(Control::SimOutputs* outputs);

        /**
         * @brief Gets current battery terminal voltage in Volts.
         * @return Bus voltage post-sag.
         */
        double GetTerminalVoltage() const;

        /**
         * @brief Gets instantaneous total robot current in Amperes.
         * @return Summed current draw.
         */
        double GetTotalCurrent() const;

        /**
         * @brief Checks if brownout stage 2 (critical motor shutoff) is currently active.
         * @return True if browned out.
         */
        bool IsBrownedOut() const;

        /**
         * @brief Sets base static quiescent current draw (e.g. RoboRIO, radio, LEDs).
         * @param idleCurrentA Quiescent current in Amperes (default: 1.5 A).
         */
        void SetIdleCurrent(double idleCurrentA);

        /**
         * @brief Configures the regulated power rail (e.g. RoboRIO 6V, REV SPM 6V, or FTC Hub).
         * @param nominalVolts Desired output voltage (typically 5.0V or 6.0V, default: 6.0V).
         * @param currentLimitA Maximum continuous current before voltage droop (RoboRIO default: 2.2A).
         * @param efficiency DC-DC buck converter efficiency (default: 0.88).
         */
        void SetRegulatedRail(double nominalVolts, double currentLimitA, double efficiency = 0.88);

        /**
         * @brief Gets instantaneous regulated rail voltage in Volts.
         * @return Regulated voltage post-droop.
         */
        double GetRegulatedRailVoltage() const;

        /**
         * @brief Gets instantaneous total current drawn by all connected servos on the regulated rail in Amperes.
         * @return Total servo rail current.
         */
        double GetRegulatedRailCurrent() const;

        /**
         * @brief Checks if the regulated rail current limit has been exceeded, inducing voltage droop.
         * @return True if regulated rail is overloaded.
         */
        bool IsRegulatedRailBrownedOut() const;

    private:
        FRCBananaSim::Robot::Drivetrain::Systems::SwerveDriveSystem* swerveSystem = nullptr;
        Control::SimOutputs* externalOutputs = nullptr;

        double idleCurrentA = 1.5;
        double currentTerminalVoltage = 12.60;
        double currentTotalCurrent = 1.5;
        bool isBrownedOutState = false;

        // Regulated 6V / 5V power rail properties
        double regulatedRailNominalVoltage = 6.0;
        double regulatedRailCurrentLimit = 2.2; // Default FRC RoboRIO 6V rail limit
        double regulatedRailEfficiency = 0.88;  // Buck regulator efficiency
        double currentRegulatedRailVoltage = 6.0;
        double currentRegulatedRailCurrent = 0.0;
        bool isRegulatedRailBrownedOutState = false;
    };
}
