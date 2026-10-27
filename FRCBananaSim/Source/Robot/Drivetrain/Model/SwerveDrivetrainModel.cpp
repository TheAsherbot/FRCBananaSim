/**
 * @file SwerveDrivetrainModel.cpp
 * @brief Implementation of top-level swerve drivetrain simulation facade.
 */

#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Telemetry/TelemetryMacros.h"
#include <glm/gtx/quaternion.hpp>
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

        chassisMotion.position = glm::vec3(0.0f, 0.0f, static_cast<float>(config.chassis.centerOfMassHeight));
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
            // 1. Solve closed-form battery voltage from aggregate motor power
            double totalPowerWatts = 0.0;
            if (inputs.robotEnabled && !isBrownedOutState)
            {
                for (uint32_t i = 0; i < 4; ++i)
                {
                    double steerVolts = inputs.motorCommands[i].demand;
                    double steerRes = modules[i].GetSteerMotor().GetResistance();
                    totalPowerWatts += (steerVolts * steerVolts) / std::max(1e-4, steerRes);

                    double driveVolts = inputs.motorCommands[4 + i].demand;
                    double driveRes = modules[i].GetDriveMotor().GetResistance();
                    totalPowerWatts += (driveVolts * driveVolts) / std::max(1e-4, driveRes);
                }
            }

            terminalVoltage = batteryModel.SolveBusVoltageClosedForm(totalPowerWatts, isBrownedOutState, isBrownedOutState);

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

                wheelContacts[i].contactPoint = probe.contactPoint;
                wheelContacts[i].surfaceNormal = probe.surfaceNormal;
                wheelContacts[i].penetrationDepth = probe.penetrationDepth;
                wheelContacts[i].hasContact = probe.hasContact;

                if (probe.hasContact)
                {
                    wheelContacts[i].normalStiffnessKn = (config.tire.radialStiffness * subDt) + config.tire.radialDamping;
                    wheelContacts[i].tangentialStiffness = modules[i].ComputeTangentialStiffness(
                        inputs.motorCommands[4 + i], terminalVoltage,
                        wheelContacts[i].normalStiffnessKn * probe.penetrationDepth, subDt);
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
                        normalForce, patchVel, wheelContacts[i].surfaceNormal, subDt);
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
        previousLinearVel = chassisMotion.linearVelocity;
        previousAngularVel = chassisMotion.angularVelocity;

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