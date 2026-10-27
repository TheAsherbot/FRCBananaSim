/**
 * @file SwerveDrivetrainModel.h
 * @brief Top-level 6-DOF swerve drivetrain simulation model with multi-substepping and battery integration.
 */

#pragma once

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include "Robot/Drivetrain/Model/SwerveModuleModel.h"
#include "Robot/Drivetrain/Model/SwerveChassisSolver.h"
#include "Robot/Drivetrain/Model/TerrainProvider.h"
#include "Actuation/BatteryModel.h"
#include "Control/SimControlBoundary.h"
#include <array>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct DrivetrainEnergyLedger
     * @brief Energy conservation tracking accounting for mechanical, electrical, and loss terms.
     */
    struct DrivetrainEnergyLedger
    {
        double batteryEnergyDrawnJoules = 0.0;   ///< Total cumulative energy pulled from battery (J).
        double kineticEnergyJoules = 0.0;        ///< Instantaneous mechanical kinetic energy (J).
        double potentialEnergyJoules = 0.0;      ///< Instantaneous gravitational potential energy (J).
        double heatLossJoules = 0.0;             ///< Cumulative ohmic I^2*R and friction dissipation (J).
        double energyResidualJoules = 0.0;       ///< Energy balance error |E_in - (KE + PE + Losses)| (J).
    };

    /**
     * @struct DrivetrainSnapshot
     * @brief Plain-old-data (POD) binary snapshot of the entire drivetrain for instant rewind/reset in Fleet mode.
     */
    struct DrivetrainSnapshot
    {
        ChassisMotionState chassisMotion;
        std::array<SwerveModuleState, 4> moduleStates;
        double batteryTerminalVoltage = 12.60;
        bool isBrownedOut = false;
        double cumulativeEnergyJoules = 0.0;
    };

    /**
     * @class SwerveDrivetrainModel
     * @brief Pure C++ complete swerve drivetrain simulation facade decoupled from ECS and physics middleware.
     */
    class SwerveDrivetrainModel
    {
    public:
        /**
         * @brief Constructs the swerve drivetrain simulation model.
         * @param config Drivetrain parameters, motor models, and geometry.
         * @param battery Battery electrical characteristics.
         */
        explicit SwerveDrivetrainModel(const SwerveDrivetrainConfig& config,
            const Actuation::BatteryModel& battery = Actuation::BatteryModel::FRCStandard());

        /**
         * @brief Advances swerve simulation across physics tick duration using multiple internal sub-steps.
         * @param tickDeltaTime Main tick duration in seconds (e.g. 0.005 s for 200 Hz).
         * @param inputs Incoming motor voltage demands and robot states from robot code or test runner.
         * @param terrain Ground terrain geometry provider for wheel contact probes.
         * @param subSteps Number of internal integration steps (default 4 = 800 Hz).
         */
        void Step(double tickDeltaTime, const Control::SimInputs& inputs,
            const ITerrainProvider& terrain, uint32_t subSteps = 4);

        /**
         * @brief Populates standard SimOutputs struct for bridge communication or test validation.
         * @param outOutputs Mutable reference to SimOutputs.
         */
        void PopulateOutputs(Control::SimOutputs& outOutputs) const;

        /**
         * @brief Computes midpoint velocity handoff for symplectic integration into middleware (Jolt).
         * @param outMidpointLinearVel (v_initial + v_final) / 2 (m/s).
         * @param outMidpointAngularVel (w_initial + w_final) / 2 (rad/s).
         */
        void GetMidpointVelocities(glm::vec3& outMidpointLinearVel, glm::vec3& outMidpointAngularVel) const;

        /**
         * @brief Ingests post-step external collision corrections from middleware (Jolt).
         * @param correctedPosition Actual position after obstacle collision resolution.
         * @param correctedOrientation Actual orientation after collision resolution.
         * @param correctedLinearVel Actual velocity after collision impulse.
         * @param correctedAngularVel Actual angular rate after collision impulse.
         */
        void ApplyExternalCorrections(const glm::vec3& correctedPosition,
            const glm::quat& correctedOrientation,
            const glm::vec3& correctedLinearVel,
            const glm::vec3& correctedAngularVel);

        /**
         * @brief Captures POD binary state snapshot for sub-millisecond reset.
         * @return DrivetrainSnapshot struct.
         */
        DrivetrainSnapshot CaptureSnapshot() const;

        /**
         * @brief Restores complete simulation state from a snapshot.
         * @param snapshot Snapshot struct to restore.
         */
        void RestoreSnapshot(const DrivetrainSnapshot& snapshot);

        // Accessors
        const ChassisMotionState& GetChassisState() const;
        ChassisMotionState& GetChassisState();
        const SwerveModuleModel& GetModule(uint32_t index) const;
        SwerveModuleModel& GetModule(uint32_t index);
        const DrivetrainEnergyLedger& GetEnergyLedger() const;
        double GetBatteryTerminalVoltage() const;
        bool IsBrownedOut() const;

    private:
        SwerveDrivetrainConfig config;
        Actuation::BatteryModel batteryModel;
        SwerveChassisSolver chassisSolver;
        std::array<SwerveModuleModel, 4> modules;

        ChassisMotionState chassisMotion;
        glm::vec3 previousLinearVel{ 0.0f };
        glm::vec3 previousAngularVel{ 0.0f };

        double terminalVoltage = 12.60;
        double totalBusCurrent = 1.5;
        bool isBrownedOutState = false;

        DrivetrainEnergyLedger energyLedger;
    };
}