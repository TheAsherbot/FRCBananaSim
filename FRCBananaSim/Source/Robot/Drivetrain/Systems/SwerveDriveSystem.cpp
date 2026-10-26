/**
 * @file SwerveDriveSystem.cpp
 * @brief Implementation of the physical swerve drive simulation system.
 */

#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Telemetry/TelemetryMacros.h"

#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::System
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
                if (moduleEntity == Core::ECS::Core::invalidEntity)
                {
                    continue;
                }

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

                if (steerJoint)
                {
                    const auto& steerCmd = currentInputs.motorCommands[i];
                    constexpr double steerHousingInertia = 0.00036; // 0.45 kg, 0.04m radius cylinder
                    double effectiveSteerInertia = module->steerMotor.GetReflectedInertia(steerHousingInertia);

                    if (currentInputs.robotEnabled && !isBrownedOut && steerCmd.mode != MotorControlMode::Disabled)
                    {
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

                        steerTorque = module->steerMotor.CalculateTorque(appliedVolts, steerJoint->currentVelocity, steerHousingInertia, dt);
                        double rawCurrent = module->steerMotor.CalculateCurrent(appliedVolts, steerJoint->currentVelocity);

                        // Enforce stator current clamp on torque and reported current
                        double statorLimit = steerCmd.statorCurrentLimit;
                        double maxStatorTorque = module->steerMotor.GetGearRatio() * module->steerMotor.GetGearboxEfficiency()
                            * module->steerMotor.GetTorqueConstantKt() * statorLimit;
                        steerTorque = std::clamp(steerTorque, -maxStatorTorque, maxStatorTorque);
                        steerCurrent = std::clamp(std::abs(rawCurrent), 0.0, statorLimit);

                        // Scale torque applied to Jolt link by load fraction to account for reflected rotor inertia
                        double appliedSteerTorque = steerTorque * (steerHousingInertia / effectiveSteerInertia);

                        // Direct torque drive applied along steer hinge axis (+Y)
                        steerJoint->motorMode = JointMotorMode::Torque;
                        steerJoint->appliedTorque = static_cast<float>(appliedSteerTorque);

                        accumulatedSupplyCurrent += (steerCurrent * std::abs(appliedVolts)) / std::max(1.0, terminalVoltage);
                    }
                    else
                    {
                        // Neutral mode handling (Brake vs. Coast)
                        if (steerCmd.neutralMode == NeutralMode::Brake)
                        {
                            steerTorque = module->steerMotor.CalculateTorque(0.0_V, steerJoint->currentVelocity, steerHousingInertia, dt);
                            double appliedSteerTorque = steerTorque * (steerHousingInertia / effectiveSteerInertia);
                            steerJoint->motorMode = JointMotorMode::Torque;
                            steerJoint->appliedTorque = static_cast<float>(appliedSteerTorque);
                        }
                        else
                        {
                            steerJoint->motorMode = JointMotorMode::Torque;
                            steerJoint->appliedTorque = 0.0f;
                        }
                    }
                }

                // --- DRIVE ACTUATION ---
                double driveTorque = 0.0_Nm;
                double driveCurrent = 0.0_A;

                if (driveJoint)
                {
                    const auto& driveCmd = currentInputs.motorCommands[4 + i];
                    // True reflected load inertia under robot mass: J_load = J_wheel + (1/4 * M_chassis * r^2)
                    constexpr double chassisLoadInertia = 0.00084 + (12.5 * 0.0508 * 0.0508); // ~0.0331 kg*m^2

                    if (currentInputs.robotEnabled && !isBrownedOut && driveCmd.mode != MotorControlMode::Disabled)
                    {
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

                        // Apply supply current limit
                        double appliedVolts = batteryModel.ApplySupplyCurrentLimit(commandedVolts,
                            module->driveMotor.CalculateCurrent(commandedVolts, driveJoint->currentVelocity), terminalVoltage);

                        driveTorque = module->driveMotor.CalculateTorque(appliedVolts, driveJoint->currentVelocity, chassisLoadInertia, dt);
                        double rawCurrent = module->driveMotor.CalculateCurrent(appliedVolts, driveJoint->currentVelocity);

                        // Enforce stator current clamp on torque and reported current
                        double statorLimit = driveCmd.statorCurrentLimit;
                        double maxStatorTorque = module->driveMotor.GetGearRatio() * module->driveMotor.GetGearboxEfficiency()
                            * module->driveMotor.GetTorqueConstantKt() * statorLimit;
                        driveTorque = std::clamp(driveTorque, -maxStatorTorque, maxStatorTorque);
                        driveCurrent = std::clamp(std::abs(rawCurrent), 0.0, statorLimit);

                        // Direct torque drive applied along wheel axle (+X)
                        driveJoint->motorMode = JointMotorMode::Torque;
                        driveJoint->appliedTorque = static_cast<float>(driveTorque);

                        accumulatedSupplyCurrent += (driveCurrent * std::abs(appliedVolts)) / std::max(1.0, terminalVoltage);
                    }
                    else
                    {
                        // Neutral mode handling (Brake vs. Coast)
                        if (driveCmd.neutralMode == NeutralMode::Brake)
                        {
                            driveTorque = module->driveMotor.CalculateTorque(0.0_V, driveJoint->currentVelocity, chassisLoadInertia, dt);
                            driveJoint->motorMode = JointMotorMode::Torque;
                            driveJoint->appliedTorque = static_cast<float>(driveTorque);
                        }
                        else
                        {
                            driveJoint->motorMode = JointMotorMode::Torque;
                            driveJoint->appliedTorque = 0.0f;
                        }
                    }
                }

                // --- TELEMETRY READBACK ---
                if (steerJoint)
                {
                    // Continuous unwrapped steer angle tracking across [-pi, +pi] boundary
                    double currentHingeAngle = steerJoint->currentAngle;
                    double delta = currentHingeAngle - module->lastHingeAngle;
                    constexpr double pi = 3.14159265358979323846;

                    while (delta > pi)
                    {
                        delta -= 2.0 * pi;
                    }
                    while (delta < -pi)
                    {
                        delta += 2.0 * pi;
                    }

                    module->unwrappedSteerAngle += delta;
                    module->lastHingeAngle = currentHingeAngle;

                    module->currentSteerAngle = module->unwrappedSteerAngle;
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

            // Chassis IMU readback (CTRE Pigeon 2: CCW positive in X-Z plane, +X Right, +Y Up, +Z Forward)
            if (chassisTransform)
            {
                const glm::quat& q = chassisTransform->rotation;
                glm::vec3 forward = q * glm::vec3(0.0f, 0.0f, 1.0f);
                glm::vec3 up = q * glm::vec3(0.0f, 1.0f, 0.0f);
                glm::vec3 right = q * glm::vec3(1.0f, 0.0f, 0.0f);

                // Yaw: 0 rad facing +Z forward, positive turning CCW towards -X (Left)
                chassis->gyroYaw = std::atan2(-forward.x, forward.z);
                chassis->gyroPitch = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
                chassis->gyroRoll = std::atan2(right.y, up.y);
            }

            if (chassisRb)
            {
                // CCW yaw rate around vertical axis
                chassis->gyroYawRate = -chassisRb->angularVelocity.y;
            }

            currentOutputs.gyroYaw = chassis->gyroYaw;
            currentOutputs.gyroPitch = chassis->gyroPitch;
            currentOutputs.gyroRoll = chassis->gyroRoll;
            currentOutputs.gyroYawRate = chassis->gyroYawRate;
        }

        double totalBusCurrent = accumulatedSupplyCurrent;
        if (batteryComp)
        {
            totalBusCurrent += batteryComp->batteryModel.GetQuiescentCurrent();
        }

        lastTotalBusCurrent = totalBusCurrent;
        currentOutputs.batteryTerminalVoltage = terminalVoltage;
        currentOutputs.totalBusCurrent = totalBusCurrent;
        currentOutputs.isBrownedOut = isBrownedOut;
    }
}