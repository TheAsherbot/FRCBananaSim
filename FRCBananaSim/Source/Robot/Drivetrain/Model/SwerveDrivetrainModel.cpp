/**
 * @file SwerveDrivetrainModel.cpp
 * @brief Implementation of top-level swerve drivetrain simulation facade.
 */

#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    SwerveDrivetrainModel::SwerveDrivetrainModel(const SwerveDrivetrainConfig& config,
        const Actuation::BatteryModel& battery)
        : config(config),
        batteryModel(battery),
        chassisSolver(config.chassis)
    {
        for (uint32_t i = 0; i < 4; ++i)
        {
            modules[i] = SwerveModuleModel(config.modules[i], config.steer, config.drive, config.tire);
        }

        // Initialize at exact static equilibrium sag to prevent artificial launch heave
        double staticSag = (config.chassis.mass * 9.80665) / (4.0 * config.tire.radialStiffness);
        chassisMotion.position = glm::vec3(0.0f, 0.0f, static_cast<float>(config.chassis.centerOfMassHeight - staticSag));
    }

    void SwerveDrivetrainModel::Step(double tickDeltaTime, const Control::SimInputs& inputs,
        const ITerrainProvider& terrain, uint32_t subSteps)
    {
        SIM_PROFILE_SCOPE("SwerveDrivetrainModel::Step");

        if (tickDeltaTime <= 1e-7 || subSteps == 0)
        {
            return;
        }

        previousLinearVel = chassisMotion.linearVelocity;
        previousAngularVel = chassisMotion.angularVelocity;

        double subDt = tickDeltaTime / static_cast<double>(subSteps);

        for (uint32_t step = 0; step < subSteps; ++step)
        {
            // 1. Compute realistic active motor electrical supply current through buck PWM duty cycle
            double totalSupplyCurrent = 0.0;
            if (inputs.robotEnabled && !isBrownedOutState)
            {
                for (uint32_t i = 0; i < 4; ++i)
                {
                    // Steer motor power
                    double steerCmdVolts = inputs.motorCommands[i].demand;
                    double steerAppliedVolts = std::clamp(steerCmdVolts, -terminalVoltage, terminalVoltage);
                    double steerCurrent = modules[i].GetSteerMotor().CalculateCurrent(
                        steerAppliedVolts, modules[i].GetState().steerVelocity);
                    steerCurrent = std::clamp(std::abs(steerCurrent), 0.0, inputs.motorCommands[i].statorCurrentLimit);
                    double steerBackEmf = modules[i].GetSteerMotor().GetBackEmfConstantKe() *
                        (modules[i].GetState().steerVelocity * modules[i].GetSteerMotor().GetGearRatio());
                    double steerVw = steerCurrent * modules[i].GetSteerMotor().GetResistance() + std::abs(steerBackEmf);
                    double steerDuty = std::clamp(steerVw / std::max(1.0, terminalVoltage), 0.0, 1.0);
                    totalSupplyCurrent += std::min(40.0, steerDuty * steerCurrent);

                    // Drive motor power
                    double driveCmdVolts = inputs.motorCommands[4 + i].demand;
                    double driveAppliedVolts = std::clamp(driveCmdVolts, -terminalVoltage, terminalVoltage);
                    double driveCurrent = modules[i].GetDriveMotor().CalculateCurrent(
                        driveAppliedVolts, modules[i].GetState().wheelVelocity);
                    driveCurrent = std::clamp(std::abs(driveCurrent), 0.0, inputs.motorCommands[4 + i].statorCurrentLimit);
                    double driveBackEmf = modules[i].GetDriveMotor().GetBackEmfConstantKe() *
                        (modules[i].GetState().wheelVelocity * modules[i].GetDriveMotor().GetGearRatio());
                    double driveVw = driveCurrent * modules[i].GetDriveMotor().GetResistance() + std::abs(driveBackEmf);
                    double driveDuty = std::clamp(driveVw / std::max(1.0, terminalVoltage), 0.0, 1.0);
                    totalSupplyCurrent += std::min(40.0, driveDuty * driveCurrent);
                }
            }

            // Sagged battery terminal voltage with double-layer RC filtering (tau ~ 5ms at 800 Hz)
            double targetTerminalVoltage = batteryModel.CalculateTerminalVoltage(totalSupplyCurrent);
            constexpr double batteryFilterAlpha = 0.20;
            terminalVoltage += batteryFilterAlpha * (targetTerminalVoltage - terminalVoltage);
            isBrownedOutState = batteryModel.IsBrownoutCritical(terminalVoltage);

            // 2. Step steer azimuth heading ODEs
            for (uint32_t i = 0; i < 4; ++i)
            {
                modules[i].StepSteer(inputs.motorCommands[i], terminalVoltage,
                    chassisMotion.angularVelocity.z, subDt);
            }

            // 3. Query ground terrain contact geometry for all 4 wheels
            glm::mat3 rotMatrix = glm::mat3_cast(chassisMotion.orientation);
            std::array<SolverWheelContact, 4> wheelContacts;

            for (uint32_t i = 0; i < 4; ++i)
            {
                glm::vec2 modLoc = config.modules[i].locationOnChassis;
                glm::vec3 localHubPos(modLoc.x, modLoc.y, -config.chassis.centerOfMassHeight + config.tire.wheelRadius);
                glm::vec3 worldHubPos = chassisMotion.position + (rotMatrix * localHubPos);

                WheelContactProbe probe = terrain.QueryWheelContact(worldHubPos, config.tire.wheelRadius, config.tire.wheelWidth);

                double cosA = std::cos(modules[i].GetState().steerAngle);
                double sinA = std::sin(modules[i].GetState().steerAngle);
                glm::vec3 localHeading(cosA, sinA, 0.0f);
                wheelContacts[i].wheelHeading = rotMatrix * localHeading;

                wheelContacts[i].contactPoint = probe.contactPoint;
                wheelContacts[i].surfaceNormal = probe.surfaceNormal;
                wheelContacts[i].penetrationDepth = probe.penetrationDepth;
                wheelContacts[i].hasContact = probe.hasContact;
                wheelContacts[i].springStiffnessKt = config.tire.radialStiffness;
                wheelContacts[i].muX = config.tire.muStatic;
                wheelContacts[i].muY = config.tire.muStatic * config.tire.lateralFrictionRatio;

                if (probe.hasContact)
                {
                    wheelContacts[i].normalStiffnessKn = (config.tire.radialStiffness * subDt) + config.tire.radialDamping;
                    wheelContacts[i].tangentialStiffness = modules[i].ComputeTangentialStiffness(
                        inputs.motorCommands[4 + i], terminalVoltage,
                        config.tire.radialStiffness * probe.penetrationDepth, subDt);
                }
            }

            // 4. Solve 6-DOF implicit equations of motion
            glm::vec3 newLinVel{ 0.0f };
            glm::vec3 newAngVel{ 0.0f };

            chassisSolver.SolveStep(chassisMotion, wheelContacts, subDt, newLinVel, newAngVel);

            chassisMotion.linearVelocity = newLinVel;
            chassisMotion.angularVelocity = newAngVel;

            // 5. Finalize module drive steps and update wheel bristle states
            double sumCurrents = 0.0;
            for (uint32_t i = 0; i < 4; ++i)
            {
                if (wheelContacts[i].hasContact)
                {
                    glm::vec3 r = wheelContacts[i].contactPoint - chassisMotion.position;
                    glm::vec3 patchVel = chassisMotion.linearVelocity + glm::cross(chassisMotion.angularVelocity, r);

                    double normalForce = (config.tire.radialStiffness * wheelContacts[i].penetrationDepth) -
                        (config.tire.radialDamping * glm::dot(patchVel, wheelContacts[i].surfaceNormal));
                    normalForce = std::max(0.0, normalForce);

                    modules[i].FinalizeDriveStep(inputs.motorCommands[4 + i], terminalVoltage,
                        normalForce, patchVel, wheelContacts[i].surfaceNormal,
                        wheelContacts[i].wheelHeading, subDt);
                }

                sumCurrents += modules[i].GetState().steerStatorCurrent + modules[i].GetState().driveStatorCurrent;
            }

            totalBusCurrent = sumCurrents + batteryModel.GetQuiescentCurrent();

            // 6. Integrate orientation and world position
            glm::vec3 angVel = chassisMotion.angularVelocity;
            double angRate = glm::length(angVel);
            if (angRate > 1e-6)
            {
                glm::quat deltaRot = glm::angleAxis(static_cast<float>(angRate * subDt), angVel / static_cast<float>(angRate));
                chassisMotion.orientation = glm::normalize(deltaRot * chassisMotion.orientation);
            }

            chassisMotion.position += chassisMotion.linearVelocity * static_cast<float>(subDt);
        }

        // Energy ledger accounting
        double m = config.chassis.mass;
        double speedSq = glm::dot(chassisMotion.linearVelocity, chassisMotion.linearVelocity);
        energyLedger.kineticEnergyJoules = 0.5 * m * speedSq;
        energyLedger.potentialEnergyJoules = m * 9.80665 * chassisMotion.position.z;
    }

    void SwerveDrivetrainModel::PopulateOutputs(Control::SimOutputs& outOutputs) const
    {
        for (uint32_t i = 0; i < 4; ++i)
        {
            const auto& modState = modules[i].GetState();
            outOutputs.encoderPositions[i] = modState.unwrappedSteerAngle;
            outOutputs.encoderVelocities[i] = modState.steerVelocity;
            outOutputs.motorCurrents[i] = modState.steerStatorCurrent;

            outOutputs.encoderPositions[4 + i] = modState.wheelPosition;
            outOutputs.encoderVelocities[4 + i] = modState.wheelVelocity;
            outOutputs.motorCurrents[4 + i] = modState.driveStatorCurrent;
        }

        // Pigeon 2 IMU Euler extraction in NWU (Yaw CCW about +Z)
        glm::mat3 rot = glm::mat3_cast(chassisMotion.orientation);
        glm::vec3 forward = rot * glm::vec3(1.0f, 0.0f, 0.0f);
        glm::vec3 left = rot * glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 up = rot * glm::vec3(0.0f, 0.0f, 1.0f);

        outOutputs.gyroYaw = std::atan2(forward.y, forward.x);
        outOutputs.gyroPitch = std::asin(std::clamp(-forward.z, -1.0f, 1.0f));
        outOutputs.gyroRoll = std::atan2(left.z, up.z);
        outOutputs.gyroYawRate = chassisMotion.angularVelocity.z;

        outOutputs.batteryTerminalVoltage = terminalVoltage;
        outOutputs.totalBusCurrent = totalBusCurrent;
        outOutputs.isBrownedOut = isBrownedOutState;
    }

    void SwerveDrivetrainModel::GetMidpointVelocities(glm::vec3& outMidpointLinearVel, glm::vec3& outMidpointAngularVel) const
    {
        outMidpointLinearVel = 0.5f * (previousLinearVel + chassisMotion.linearVelocity);
        outMidpointAngularVel = 0.5f * (previousAngularVel + chassisMotion.angularVelocity);
    }

    void SwerveDrivetrainModel::ApplyExternalCorrections(const glm::vec3& correctedPosition,
        const glm::quat& correctedOrientation,
        const glm::vec3& correctedLinearVel,
        const glm::vec3& correctedAngularVel)
    {
        chassisMotion.position = correctedPosition;
        chassisMotion.orientation = correctedOrientation;
        chassisMotion.linearVelocity = correctedLinearVel;
        chassisMotion.angularVelocity = correctedAngularVel;
    }

    DrivetrainSnapshot SwerveDrivetrainModel::CaptureSnapshot() const
    {
        DrivetrainSnapshot snap;
        snap.chassisMotion = chassisMotion;
        for (uint32_t i = 0; i < 4; ++i)
        {
            snap.moduleStates[i] = modules[i].GetState();
        }
        snap.batteryTerminalVoltage = terminalVoltage;
        snap.isBrownedOut = isBrownedOutState;
        snap.cumulativeEnergyJoules = energyLedger.batteryEnergyDrawnJoules;
        return snap;
    }

    void SwerveDrivetrainModel::RestoreSnapshot(const DrivetrainSnapshot& snapshot)
    {
        chassisMotion = snapshot.chassisMotion;
        for (uint32_t i = 0; i < 4; ++i)
        {
            modules[i].GetState() = snapshot.moduleStates[i];
        }
        terminalVoltage = snapshot.batteryTerminalVoltage;
        isBrownedOutState = snapshot.isBrownedOut;
        energyLedger.batteryEnergyDrawnJoules = snapshot.cumulativeEnergyJoules;
    }

    const ChassisMotionState& SwerveDrivetrainModel::GetChassisState() const { return chassisMotion; }
    ChassisMotionState& SwerveDrivetrainModel::GetChassisState() { return chassisMotion; }
    const SwerveModuleModel& SwerveDrivetrainModel::GetModule(uint32_t index) const { return modules[index]; }
    SwerveModuleModel& SwerveDrivetrainModel::GetModule(uint32_t index) { return modules[index]; }
    const DrivetrainEnergyLedger& SwerveDrivetrainModel::GetEnergyLedger() const { return energyLedger; }
    double SwerveDrivetrainModel::GetBatteryTerminalVoltage() const { return terminalVoltage; }
    bool SwerveDrivetrainModel::IsBrownedOut() const { return isBrownedOutState; }
}