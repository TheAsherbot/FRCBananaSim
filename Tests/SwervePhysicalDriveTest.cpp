/**
 * @file SwervePhysicalDriveTest.cpp
 * @brief Test suite verifying physical swerve drivetrain simulation, torque drive, and control boundary.
 */

#include "SwervePhysicalDriveTest.h"
#include "TestFramework.h"
#include "Control/SimControlBoundary.h"
#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/Units/Units.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Tests
{
    using namespace Control;
    using namespace Robot;
    using namespace Robot::Drivetrain::Components;
    using namespace Robot::Drivetrain::System;
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Physics;
    using namespace Physics::Components;
    using namespace Physics::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    void RunSwervePhysicalDriveTest(TestContext& ctx)
    {
        std::cout << "[RUN ] SwervePhysicalDriveTest\n";
        constexpr double tolerance = 1e-4;

        // -------------------------------------------------------------
        // 1. SimControlBoundary Struct Defaults & Multi-Mode Verification
        // -------------------------------------------------------------
        {
            SimInputs inputs;
            TEST_EXPECT_FALSE(ctx, inputs.robotEnabled);
            TEST_EXPECT_FALSE(ctx, inputs.isAutonomous);
            TEST_EXPECT_NEAR(ctx, inputs.matchTime, 0.0, tolerance);

            for (size_t i = 0; i < 8; ++i)
            {
                TEST_EXPECT_EQ(ctx, static_cast<int>(inputs.motorCommands[i].mode), static_cast<int>(MotorControlMode::Voltage));
                TEST_EXPECT_EQ(ctx, static_cast<int>(inputs.motorCommands[i].neutralMode), static_cast<int>(NeutralMode::Brake));
                TEST_EXPECT_NEAR(ctx, inputs.motorCommands[i].demand, 0.0, tolerance);
                TEST_EXPECT_NEAR(ctx, inputs.motorCommands[i].statorCurrentLimit, 60.0, tolerance);
            }

            SimOutputs outputs;
            TEST_EXPECT_NEAR(ctx, outputs.batteryTerminalVoltage, 12.60, tolerance);
            TEST_EXPECT_NEAR(ctx, outputs.totalBusCurrent, 1.5, tolerance);
            TEST_EXPECT_NEAR(ctx, outputs.gyroYaw, 0.0, tolerance);
            TEST_EXPECT_FALSE(ctx, outputs.isBrownedOut);
        }

        // -------------------------------------------------------------
        // 2. Physical Swerve Module Torque Drive & Wheel Spin-Up
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            SwerveDriveSystem swerveSystem(scene);

            // A. Battery Entity
            Entity batteryEntity = scene.CreateEntity();
            BatteryComponent batteryComp;
            batteryComp.batteryModel = BatteryModel::FRCStandard();
            scene.AddComponent(batteryEntity, batteryComp);

            // B. Chassis Root Entity (50 kg)
            Entity chassisEntity = scene.CreateEntity();
            TransformComponent chassisTransform;
            chassisTransform.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(chassisEntity, chassisTransform);

            RigidBodyComponent chassisRb;
            chassisRb.motionType = BodyMotionType::Dynamic;
            chassisRb.mass = 50.0f;
            scene.AddComponent(chassisEntity, chassisRb);

            ColliderComponent chassisCol;
            chassisCol.shape = ColliderShape::Box;
            chassisCol.boxHalfExtents = glm::vec3(0.35f, 0.075f, 0.35f);
            scene.AddComponent(chassisEntity, chassisCol);

            SwerveChassisComponent chassisComp;
            chassisComp.batteryEntity = batteryEntity;

            // Front-Left module position: X = -0.28m (Left), Z = +0.28m (Forward)
            // C. Steer Link Entity (0.45 kg)
            Entity steerEntity = scene.CreateEntity();
            TransformComponent steerTransform;
            steerTransform.position = glm::vec3(-0.28f, 0.9f, 0.28f);
            scene.AddComponent(steerEntity, steerTransform);

            RigidBodyComponent steerRb;
            steerRb.motionType = BodyMotionType::Dynamic;
            steerRb.mass = 0.45f;
            scene.AddComponent(steerEntity, steerRb);

            ColliderComponent steerCol;
            steerCol.shape = ColliderShape::Cylinder;
            steerCol.radius = 0.04f;
            steerCol.halfHeight = 0.03f;
            scene.AddComponent(steerEntity, steerCol);

            // D. Wheel Link Entity (0.65 kg)
            Entity wheelEntity = scene.CreateEntity();
            TransformComponent wheelTransform;
            wheelTransform.position = glm::vec3(-0.28f, 0.8f, 0.28f);
            scene.AddComponent(wheelEntity, wheelTransform);

            RigidBodyComponent wheelRb;
            wheelRb.motionType = BodyMotionType::Dynamic;
            wheelRb.mass = 0.65f;
            scene.AddComponent(wheelEntity, wheelRb);

            ColliderComponent wheelCol;
            wheelCol.shape = ColliderShape::Cylinder;
            wheelCol.radius = 0.0508f;
            wheelCol.halfHeight = 0.02f;
            wheelCol.localRotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            scene.AddComponent(wheelEntity, wheelCol);

            // E. Vertical Steer Joint (Chassis <-> Steer Link along vertical +Y)
            Entity steerJointEntity = scene.CreateEntity();
            JointComponent steerJoint;
            steerJoint.parentEntity = chassisEntity;
            steerJoint.childEntity = steerEntity;
            steerJoint.SetWorldSpace(glm::vec3(-0.28f, 0.95f, 0.28f), glm::vec3(0.0f, 1.0f, 0.0f));
            steerJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(steerJointEntity, steerJoint);

            // F. Horizontal Drive Axle Joint (Steer Link <-> Wheel Link along horizontal +X)
            Entity driveJointEntity = scene.CreateEntity();
            JointComponent driveJoint;
            driveJoint.parentEntity = steerEntity;
            driveJoint.childEntity = wheelEntity;
            driveJoint.SetWorldSpace(glm::vec3(-0.28f, 0.8f, 0.28f), glm::vec3(1.0f, 0.0f, 0.0f));
            driveJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(driveJointEntity, driveJoint);

            // G. Swerve Module Component
            Entity moduleEntity = scene.CreateEntity();
            SwerveModuleComponent moduleComp;
            moduleComp.moduleIndex = 0;
            moduleComp.moduleLocation = glm::vec2(-0.28f, 0.28f);
            moduleComp.steerEntity = steerEntity;
            moduleComp.wheelEntity = wheelEntity;
            moduleComp.steerJointEntity = steerJointEntity;
            moduleComp.driveJointEntity = driveJointEntity;
            moduleComp.driveMotor = DCMotorModel::KrakenX60(6.12, 0.96);
            moduleComp.steerMotor = DCMotorModel::Falcon500(21.43, 0.92);
            scene.AddComponent(moduleEntity, moduleComp);

            chassisComp.moduleEntities[0] = moduleEntity;
            scene.AddComponent(chassisEntity, chassisComp);

            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            // Command +12.0V on drive motor [index 4 = FL drive] for 200 steps (1.0 second)
            SimInputs inputs;
            inputs.robotEnabled = true;
            inputs.motorCommands[4].mode = MotorControlMode::Voltage;
            inputs.motorCommands[4].demand = 12.0;
            swerveSystem.SetInputs(inputs);

            for (int i = 0; i < 200; ++i)
            {
                swerveSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            const auto& outputs = swerveSystem.GetOutputs();

            // Drive wheel angular velocity must spin up past 10 rad/s under supply current limiting
            TEST_EXPECT_TRUE(ctx, outputs.encoderVelocities[4] > 10.0);
            TEST_EXPECT_TRUE(ctx, outputs.encoderPositions[4] > 10.0);
        }

        // -------------------------------------------------------------
        // 3. Continuous Unwrapped Steer Angle Tracking Across Boundaries
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            SwerveDriveSystem swerveSystem(scene);

            Entity batteryEntity = scene.CreateEntity();
            BatteryComponent batteryComp;
            scene.AddComponent(batteryEntity, batteryComp);

            Entity chassisEntity = scene.CreateEntity();
            TransformComponent chassisTransform;
            chassisTransform.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(chassisEntity, chassisTransform);

            RigidBodyComponent chassisRb;
            chassisRb.motionType = BodyMotionType::Dynamic;
            chassisRb.mass = 50.0f;
            scene.AddComponent(chassisEntity, chassisRb);

            ColliderComponent chassisCol;
            chassisCol.shape = ColliderShape::Box;
            chassisCol.boxHalfExtents = glm::vec3(0.35f, 0.075f, 0.35f);
            scene.AddComponent(chassisEntity, chassisCol);

            SwerveChassisComponent chassisComp;
            chassisComp.batteryEntity = batteryEntity;

            Entity steerEntity = scene.CreateEntity();
            TransformComponent steerTransform;
            steerTransform.position = glm::vec3(-0.28f, 0.9f, 0.28f);
            scene.AddComponent(steerEntity, steerTransform);

            RigidBodyComponent steerRb;
            steerRb.motionType = BodyMotionType::Dynamic;
            steerRb.mass = 0.45f;
            scene.AddComponent(steerEntity, steerRb);

            ColliderComponent steerCol;
            steerCol.shape = ColliderShape::Cylinder;
            steerCol.radius = 0.04f;
            steerCol.halfHeight = 0.03f;
            scene.AddComponent(steerEntity, steerCol);

            Entity steerJointEntity = scene.CreateEntity();
            JointComponent steerJoint;
            steerJoint.parentEntity = chassisEntity;
            steerJoint.childEntity = steerEntity;
            steerJoint.SetWorldSpace(glm::vec3(-0.28f, 0.95f, 0.28f), glm::vec3(0.0f, 1.0f, 0.0f));
            steerJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(steerJointEntity, steerJoint);

            Entity moduleEntity = scene.CreateEntity();
            SwerveModuleComponent moduleComp;
            moduleComp.moduleIndex = 0;
            moduleComp.steerEntity = steerEntity;
            moduleComp.steerJointEntity = steerJointEntity;
            moduleComp.steerMotor = DCMotorModel::Falcon500(21.43, 0.92);
            scene.AddComponent(moduleEntity, moduleComp);

            chassisComp.moduleEntities[0] = moduleEntity;
            scene.AddComponent(chassisEntity, chassisComp);

            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            // Command +12.0V on steer motor [index 0 = FL steer] for 150 steps (750 ms)
            SimInputs inputs;
            inputs.robotEnabled = true;
            inputs.motorCommands[0].mode = MotorControlMode::Voltage;
            inputs.motorCommands[0].demand = 12.0;
            swerveSystem.SetInputs(inputs);

            for (int i = 0; i < 150; ++i)
            {
                swerveSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            const auto& outputs = swerveSystem.GetOutputs();

            // Continuous angle must exceed pi radians without discontinuous negative wrapping
            TEST_EXPECT_TRUE(ctx, outputs.encoderPositions[0] > 3.14159265358979323846);
            TEST_EXPECT_TRUE(ctx, outputs.encoderVelocities[0] > 0.0);
        }

        // -------------------------------------------------------------
        // 4. Stator Current Limiting Clamping & Brownout Hysteresis
        // -------------------------------------------------------------
        {
            // A. Battery Model Hysteresis Test
            BatteryModel battery = BatteryModel::FRCStandard();
            TEST_EXPECT_FALSE(ctx, battery.IsBrownoutCritical(12.0_V));
            TEST_EXPECT_FALSE(ctx, battery.IsBrownoutCritical(6.4_V));
            TEST_EXPECT_TRUE(ctx, battery.IsBrownoutCritical(6.2_V));
            TEST_EXPECT_TRUE(ctx, battery.IsBrownoutCritical(6.5_V));
            TEST_EXPECT_TRUE(ctx, battery.IsBrownoutCritical(7.4_V));
            TEST_EXPECT_FALSE(ctx, battery.IsBrownoutCritical(7.6_V));

            // B. Supply Current Limit Throttling
            double commandedVolts = 12.0_V;
            double statorCurrent = 80.0_A;
            double terminalVoltage = 11.5_V;
            double throttledVolts = battery.ApplySupplyCurrentLimit(commandedVolts, statorCurrent, terminalVoltage);
            TEST_EXPECT_TRUE(ctx, throttledVolts < commandedVolts);
            TEST_EXPECT_NEAR(ctx, throttledVolts, 5.75_V, 0.05);
        }

        // -------------------------------------------------------------
        // 5. Chassis Yaw & Gyro IMU Tracking (CCW Positive)
        // -------------------------------------------------------------
        {
            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);
            SwerveDriveSystem swerveSystem(scene);

            Entity chassisEntity = scene.CreateEntity();
            TransformComponent chassisTransform;
            chassisTransform.position = glm::vec3(0.0f, 1.0f, 0.0f);
            chassisTransform.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, -1.0f, 0.0f));
            scene.AddComponent(chassisEntity, chassisTransform);

            RigidBodyComponent chassisRb;
            chassisRb.motionType = BodyMotionType::Dynamic;
            chassisRb.angularVelocity = glm::vec3(0.0f, -2.5f, 0.0f);
            scene.AddComponent(chassisEntity, chassisRb);

            SwerveChassisComponent chassisComp;
            scene.AddComponent(chassisEntity, chassisComp);

            swerveSystem.FixedUpdate(PhysicsConfig::fixedDeltaTime);

            const auto& outputs = swerveSystem.GetOutputs();

            TEST_EXPECT_NEAR(ctx, outputs.gyroYaw, 3.14159265358979323846 / 4.0, 0.01);
            TEST_EXPECT_NEAR(ctx, outputs.gyroYawRate, 2.5, tolerance);
        }

        // -------------------------------------------------------------
        // 6. Comprehensive Swerve Telemetry & Collision Clearance Test
        // -------------------------------------------------------------
        {
            std::cout << "[INFO] Running Section 6: Comprehensive Swerve Telemetry & Clearance Verification...\n";

            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            SwerveDriveSystem swerveSystem(scene);

            // 1. Static Field Carpet
            Entity ground = scene.CreateEntity();
            TransformComponent groundT;
            groundT.position = glm::vec3(0.0f, -0.05f, 0.0f);
            scene.AddComponent(ground, groundT);
            RigidBodyComponent groundRb;
            groundRb.motionType = BodyMotionType::Static;
            groundRb.friction = 0.8f;
            scene.AddComponent(ground, groundRb);
            ColliderComponent groundCol;
            groundCol.shape = ColliderShape::Box;
            groundCol.boxHalfExtents = glm::vec3(15.0f, 0.05f, 15.0f);
            scene.AddComponent(ground, groundCol);

            // 2. Spawn Full 50 kg Swerve Robot at Y = 0.248 m (wheels resting cleanly on carpet at y = 0)
            SpawnedRobot robot = RobotSpawner::SpawnFromJsonFile(scene,
                "FRCBananaSim/Resources/Robots/robot_swerve.json", glm::vec3(0.0f, 0.248f, 0.0f));
            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            // 3. Verify Clearances
            auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
            auto* chassisCol = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisT != nullptr && chassisCol != nullptr);

            float chassisBottom = chassisT->position.y - chassisCol->boxHalfExtents.y;
            TEST_EXPECT_TRUE(ctx, chassisBottom > 0.08f); // Over 8 cm clearance above ground

            for (uint32_t i = 0; i < 4; ++i)
            {
                auto* steerT = scene.GetComponent<TransformComponent>(robot.steerEntities[i]);
                auto* steerCol = scene.GetComponent<ColliderComponent>(robot.steerEntities[i]);
                auto* wheelT = scene.GetComponent<TransformComponent>(robot.wheelEntities[i]);
                auto* wheelCol = scene.GetComponent<ColliderComponent>(robot.wheelEntities[i]);

                TEST_EXPECT_TRUE(ctx, steerT != nullptr && steerCol != nullptr);
                TEST_EXPECT_TRUE(ctx, wheelT != nullptr && wheelCol != nullptr);

                float steerBottom = steerT->position.y - steerCol->halfHeight;
                float wheelTop = wheelT->position.y + wheelCol->radius;
                float verticalGap = steerBottom - wheelTop;

                // Positive air gap: wheel never penetrates steer housing
                TEST_EXPECT_TRUE(ctx, verticalGap > 0.005f);
            }

            // 4. Initialize Physics Bodies & Joints
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            // 5. Command 10.0V Forward Drive with Stable Heading Hold
            SimInputs inputs;
            inputs.robotEnabled = true;

            for (uint32_t i = 0; i < 4; ++i)
            {
                inputs.motorCommands[i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[i].statorCurrentLimit = 60.0;
                inputs.motorCommands[i].neutralMode = NeutralMode::Brake;

                inputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[4 + i].demand = 10.0; // 10V Forward drive
                inputs.motorCommands[4 + i].statorCurrentLimit = 60.0;
                inputs.motorCommands[4 + i].neutralMode = NeutralMode::Brake;
            }

            double minTerminalVoltage = 12.60;
            double peakTotalBusCurrent = 0.0;

            // 6. Simulate 200 ticks (1.0 second of driving on carpet)
            for (int step = 0; step < 200; ++step)
            {
                // Stable steer heading hold PD
                constexpr double kP = 8.0;
                constexpr double kD = 0.8;
                for (uint32_t i = 0; i < 4; ++i)
                {
                    auto* steerJoint = scene.GetComponent<JointComponent>(robot.steerJointEntities[i]);
                    if (steerJoint)
                    {
                        double err = 0.0 - steerJoint->currentAngle;
                        double vel = static_cast<double>(steerJoint->currentVelocity);
                        inputs.motorCommands[i].demand = std::clamp((err * kP) - (vel * kD), -4.0, 4.0);
                    }
                }

                swerveSystem.SetInputs(inputs);
                swerveSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);

                const auto& outputs = swerveSystem.GetOutputs();
                minTerminalVoltage = std::min(minTerminalVoltage, outputs.batteryTerminalVoltage);
                peakTotalBusCurrent = std::max(peakTotalBusCurrent, outputs.totalBusCurrent);
            }

            const auto& finalOutputs = swerveSystem.GetOutputs();
            auto* finalChassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
            auto* finalChassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);

            // 7. Verify Health Assertions
            TEST_EXPECT_FALSE(ctx, finalOutputs.isBrownedOut);
            TEST_EXPECT_TRUE(ctx, minTerminalVoltage > 9.5); // Healthy launch transient settles > 9.5V
            TEST_EXPECT_TRUE(ctx, finalChassisT->position.z > 0.1f);
            TEST_EXPECT_TRUE(ctx, finalChassisRb->linearVelocity.z > 0.5f);

            // 8. Output Formatted Telemetry Diagnostic Table
            std::cout << "\n======================================================================\n";
            std::cout << " SWERVE TELEMETRY & CLEARANCE DIAGNOSTIC REPORT\n";
            std::cout << "======================================================================\n";
            std::cout << " Battery Terminal Voltage: " << finalOutputs.batteryTerminalVoltage << " V (Minimum during run: " << minTerminalVoltage << " V)\n";
            std::cout << " Total Bus Current:        " << finalOutputs.totalBusCurrent << " A (Peak during run: " << peakTotalBusCurrent << " A)\n";
            std::cout << " Brownout State:           " << (finalOutputs.isBrownedOut ? "TRUE [BROWNOUT ALERT]" : "FALSE [HEALTHY]") << "\n";
            std::cout << "----------------------------------------------------------------------\n";
            std::cout << " Chassis World Position:   X=" << finalChassisT->position.x << " m, Y=" << finalChassisT->position.y << " m, Z=" << finalChassisT->position.z << " m\n";
            std::cout << " Chassis Linear Velocity:  Vx=" << finalChassisRb->linearVelocity.x << " m/s, Vy=" << finalChassisRb->linearVelocity.y << " m/s, Vz=" << finalChassisRb->linearVelocity.z << " m/s\n";
            std::cout << " Chassis Angular Velocity: Wx=" << finalChassisRb->angularVelocity.x << " rad/s, Wy=" << finalChassisRb->angularVelocity.y << " rad/s, Wz=" << finalChassisRb->angularVelocity.z << " rad/s\n";
            std::cout << " Gyro IMU Yaw:             " << finalOutputs.gyroYaw * 180.0 / 3.14159265 << " deg (" << finalOutputs.gyroYaw << " rad)\n";
            std::cout << " Gyro IMU Pitch / Roll:    Pitch=" << finalOutputs.gyroPitch << " rad, Roll=" << finalOutputs.gyroRoll << " rad\n";
            std::cout << " Gyro IMU Yaw Rate:        " << finalOutputs.gyroYawRate << " rad/s\n";
            std::cout << "----------------------------------------------------------------------\n";
            const char* modNames[4] = { "FrontLeft (FL)", "FrontRight (FR)", "BackLeft (BL)", "BackRight (BR)" };
            for (uint32_t i = 0; i < 4; ++i)
            {
                auto* steerJoint = scene.GetComponent<JointComponent>(robot.steerJointEntities[i]);
                auto* driveJoint = scene.GetComponent<JointComponent>(robot.driveJointEntities[i]);
                auto* wheelT = scene.GetComponent<TransformComponent>(robot.wheelEntities[i]);
                auto* modComp = scene.GetComponent<SwerveModuleComponent>(robot.moduleEntities[i]);

                std::cout << " [" << modNames[i] << "]:\n";
                std::cout << "   Steer Angle:      " << finalOutputs.encoderPositions[i] * 180.0 / 3.14159265 << " deg (" << finalOutputs.encoderPositions[i] << " rad)\n";
                std::cout << "   Steer Velocity:   " << finalOutputs.encoderVelocities[i] << " rad/s\n";
                std::cout << "   Steer Stator I:   " << finalOutputs.motorCurrents[i] << " A\n";
                std::cout << "   Drive Distance:   " << modComp->distanceTraveled << " m (" << finalOutputs.encoderPositions[4 + i] << " rad)\n";
                std::cout << "   Drive Velocity:   " << modComp->currentLinearVelocity << " m/s (" << finalOutputs.encoderVelocities[4 + i] << " rad/s)\n";
                std::cout << "   Drive Stator I:   " << finalOutputs.motorCurrents[4 + i] << " A\n";
                std::cout << "   Joint Deflection: Steer=" << steerJoint->currentDeflection * 1000.0f << " mm, Drive=" << driveJoint->currentDeflection * 1000.0f << " mm\n";
                std::cout << "   Wheel World Pos:  X=" << wheelT->position.x << " m, Y=" << wheelT->position.y << " m, Z=" << wheelT->position.z << " m\n";
            }
            std::cout << "======================================================================\n\n";
        }

        std::cout << "[DONE] SwervePhysicalDriveTest\n";
    }
}