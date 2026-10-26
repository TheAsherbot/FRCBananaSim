/**
 * @file SwerveDriveSystem.cpp
 * @brief Implementation of the physical swerve drive simulation system.
 */

#include "Drivetrain/Systems/SwerveDriveSystem.h"
#include "Drivetrain/Components/SwerveChassisComponent.h"
#include "Drivetrain/Components/SwerveModuleComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Telemetry/TelemetryMacros.h"

#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Drivetrain::System
{
    using namespace Control;
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Drivetrain::Components;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    SwerveDriveSystem::SwerveDriveSystem(Core::ECS::Core::Scene& scene)
        : ISystem(scene)
    {}

    void SwerveDriveSystem::SetInputs(const SimInputs& inputs)
    {
        currentInputs = inputs;
    }

    const SimOutputs& SwerveDriveSystem::GetOutputs() const
    {
        return currentOutputs;
    }

    SimOutputs& SwerveDriveSystem::GetOutputs()
    {
        return currentOutputs;
    }

    void SwerveDriveSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("SwerveDriveSystem::FixedUpdate");

        auto chasses = scene.Query<SwerveChassisComponent, TransformComponent, RigidBodyComponent>();
        if (chasses.empty())
        {
            return;
        }

        // 1. Solve battery voltage sag using previous step's current (breaks algebraic loop)
        double terminalVoltage = 12.60_V;
        BatteryModel batteryModel;
        auto* batteryComp = scene.GetComponent<BatteryComponent>(chasses[0]);
        if (!batteryComp && scene.HasComponent<SwerveChassisComponent>(chasses[0]))
        {
            auto* chassis = scene.GetComponent<SwerveChassisComponent>(chasses[0]);
            if (chassis && chassis->batteryEntity != Core::ECS::Core::invalidEntity)
            {
                batteryComp = scene.GetComponent<BatteryComponent>(chassis->batteryEntity);
            }
        }

        if (batteryComp)
        {
            batteryModel = batteryComp->batteryModel;
            terminalVoltage = batteryModel.CalculateTerminalVoltage(lastTotalBusCurrent);
            batteryComp->terminalVoltage = terminalVoltage;
            batteryComp->isBrownoutWarning = batteryModel.IsBrownoutWarning(terminalVoltage);
            batteryComp->isBrownoutCritical = batteryModel.IsBrownoutCritical(terminalVoltage);
        }

        double accumulatedSupplyCurrent = 0.0;
        bool isBrownedOut = batteryModel.IsBrownoutCritical(terminalVoltage);

        // 2. Iterate through swerve chassis and simulate the 4 physical modules
        for (auto chassisEntity : chasses)
        {
            auto* chassis = scene.GetComponent<SwerveChassisComponent>(chassisEntity);
            auto* chassisTransform = scene.GetComponent<TransformComponent>(chassisEntity);
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(chassisEntity);
            if (!chassis)
            {
                continue;
            }

            for (uint32_t i = 0; i < 4; ++i)
            {
                auto moduleEntity = chassis->moduleEntities[i];
                auto* module = scene.GetComponent<SwerveModuleComponent>(moduleEntity);
                if (!module)
                {
                    continue;
                }

                auto* steerJoint = scene.GetComponent<JointComponent>(module->steerJointEntity);
                auto* driveJoint = scene.GetComponent<JointComponent>(module->driveJointEntity);

                // --- STEER ACTUATION ---
                double steerTorque = 0.0_Nm;
                double steerCurrent = 0.0_A;

                if (steerJoint && currentInputs.robotEnabled && !isBrownedOut)
                {
                    const auto& steerCmd = currentInputs.motorCommands[i];
                    double commandedVolts = 0.0_V;

                    switch (steerCmd.mode)
                    {
                    case MotorControlMode::Voltage:
                        commandedVolts = std::clamp(steerCmd.demand, -terminalVoltage, terminalVoltage);
                        break;
                    case MotorControlMode::DutyCycle:
                        commandedVolts = std::clamp(steerCmd.demand, -1.0, 1.0) * terminalVoltage;
                        break;
                    case MotorControlMode::TorqueCurrent:
                    {
                        double kt = module->steerMotor.GetTorqueConstantKt();
                        double r = module->steerMotor.GetResistance();
                        double ke = module->steerMotor.GetBackEmfConstantKe();
                        double motorRadps = steerJoint->currentVelocity * module->steerMotor.GetGearRatio();
                        commandedVolts = std::clamp(steerCmd.demand * r + ke * motorRadps, -terminalVoltage, terminalVoltage);
                        break;
                    }
                    case MotorControlMode::Disabled:
                        commandedVolts = 0.0_V;
                        break;
                    }

                    // Apply supply current limit
                    double appliedVolts = batteryModel.ApplySupplyCurrentLimit(commandedVolts,
                        module->steerMotor.CalculateCurrent(commandedVolts, steerJoint->currentVelocity), terminalVoltage);

                    steerTorque = module->steerMotor.CalculateTorque(appliedVolts, steerJoint->currentVelocity, 0.001, dt);
                    steerCurrent = std::abs(module->steerMotor.CalculateCurrent(appliedVolts, steerJoint->currentVelocity));

                    // Direct torque drive applied along hinge axis
                    steerJoint->motorMode = JointMotorMode::Torque;
                    steerJoint->appliedTorque = static_cast<float>(steerTorque);

                    accumulatedSupplyCurrent += (steerCurrent * std::abs(appliedVolts)) / std::max(1.0, terminalVoltage);
                }
                else if (steerJoint)
                {
                    steerJoint->motorMode = JointMotorMode::Torque;
                    steerJoint->appliedTorque = 0.0f;
                }

                // --- DRIVE ACTUATION ---
                double driveTorque = 0.0_Nm;
                double driveCurrent = 0.0_A;

                if (driveJoint && currentInputs.robotEnabled && !isBrownedOut)
                {
                    const auto& driveCmd = currentInputs.motorCommands[4 + i];
                    double commandedVolts = 0.0_V;

                    switch (driveCmd.mode)
                    {
                    case MotorControlMode::Voltage:
                        commandedVolts = std::clamp(driveCmd.demand, -terminalVoltage, terminalVoltage);
                        break;
                    case MotorControlMode::DutyCycle:
                        commandedVolts = std::clamp(driveCmd.demand, -1.0, 1.0) * terminalVoltage;
                        break;
                    case MotorControlMode::TorqueCurrent:
                    {
                        double r = module->driveMotor.GetResistance();
                        double ke = module->driveMotor.GetBackEmfConstantKe();
                        double motorRadps = driveJoint->currentVelocity * module->driveMotor.GetGearRatio();
                        commandedVolts = std::clamp(driveCmd.demand * r + ke * motorRadps, -terminalVoltage, terminalVoltage);
                        break;
                    }
                    case MotorControlMode::Disabled:
                        commandedVolts = 0.0_V;
                        break;
                    }

                    double appliedVolts = batteryModel.ApplySupplyCurrentLimit(commandedVolts,
                        module->driveMotor.CalculateCurrent(commandedVolts, driveJoint->currentVelocity), terminalVoltage);

                    driveTorque = module->driveMotor.CalculateTorque(appliedVolts, driveJoint->currentVelocity, 0.005, dt);
                    driveCurrent = std::abs(module->driveMotor.CalculateCurrent(appliedVolts, driveJoint->currentVelocity));

                    driveJoint->motorMode = JointMotorMode::Torque;
                    driveJoint->appliedTorque = static_cast<float>(driveTorque);

                    accumulatedSupplyCurrent += (driveCurrent * std::abs(appliedVolts)) / std::max(1.0, terminalVoltage);
                }
                else if (driveJoint)
                {
                    driveJoint->motorMode = JointMotorMode::Torque;
                    driveJoint->appliedTorque = 0.0f;
                }

                // --- TELEMETRY READBACK ---
                if (steerJoint)
                {
                    module->currentSteerAngle = steerJoint->currentAngle;
                    module->currentSteerVelocity = steerJoint->currentVelocity;
                }

                if (driveJoint)
                {
                    module->currentWheelPosition += driveJoint->currentVelocity * dt;
                    module->currentWheelVelocity = driveJoint->currentVelocity;
                    module->currentLinearVelocity = module->currentWheelVelocity * module->wheelRadius;
                    module->distanceTraveled += module->currentLinearVelocity * dt;
                }

                // Populate SimOutputs
                currentOutputs.encoderPositions[i] = module->currentSteerAngle;
                currentOutputs.encoderVelocities[i] = module->currentSteerVelocity;
                currentOutputs.motorCurrents[i] = steerCurrent;

                currentOutputs.encoderPositions[4 + i] = module->currentWheelPosition;
                currentOutputs.encoderVelocities[4 + i] = module->currentWheelVelocity;
                currentOutputs.motorCurrents[4 + i] = driveCurrent;
            }

            // Chassis IMU readback (CTRE Pigeon 2: Yaw around vertical axis)
            if (chassisTransform)
            {
                const glm::quat& q = chassisTransform->rotation;
                // Analytical yaw extraction from quaternion (counter-clockwise positive)
                double sinyCosp = 2.0 * (q.w * q.y - q.x * q.z);
                double cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                chassis->gyroYaw = std::atan2(sinyCosp, cosyCosp);

                double sinp = 2.0 * (q.w * q.x + q.y * q.z);
                chassis->gyroPitch = (std::abs(sinp) >= 1.0) ? std::copysign(3.14159265358979323846 / 2.0, sinp) : std::asin(sinp);

                double sinrCosp = 2.0 * (q.w * q.z - q.x * q.y);
                double cosrCosp = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
                chassis->gyroRoll = std::atan2(sinrCosp, cosrCosp);
            }

            if (chassisRb)
            {
                chassis->gyroYawRate = chassisRb->angularVelocity.y;
            }

            currentOutputs.gyroYaw = chassis->gyroYaw;
            currentOutputs.gyroPitch = chassis->gyroPitch;
            currentOutputs.gyroRoll = chassis->gyroRoll;
            currentOutputs.gyroYawRate = chassis->gyroYawRate;
        }

        lastTotalBusCurrent = accumulatedSupplyCurrent;
        currentOutputs.batteryTerminalVoltage = terminalVoltage;
        currentOutputs.totalBusCurrent = accumulatedSupplyCurrent;
        currentOutputs.isBrownedOut = isBrownedOut;
    }
}