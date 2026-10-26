/**
 * @file SwervePhysicalDriveTest.cpp
 * @brief Test suite verifying physical swerve drivetrain simulation, torque drive, and control boundary.
 */

#include "SwervePhysicalDriveTest.h"
#include "TestFramework.h"
#include "Control/SimControlBoundary.h"
#include "Drivetrain/Components/SwerveChassisComponent.h"
#include "Drivetrain/Components/SwerveModuleComponent.h"
#include "Drivetrain/Systems/SwerveDriveSystem.h"
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
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Control;
    using namespace Drivetrain::Components;
    using namespace Drivetrain::System;
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

            // C. Steer Link Entity (0.45 kg)
            Entity steerEntity = scene.CreateEntity();
            TransformComponent steerTransform;
            steerTransform.position = glm::vec3(0.28f, 0.9f, 0.28f);
            scene.AddComponent(steerEntity, steerTransform);

            RigidBodyComponent steerRb;
            steerRb.motionType = BodyMotionType::Dynamic;
            steerRb.mass = 0.45f;
            scene.AddComponent(steerEntity, steerRb);

            ColliderComponent steerCol;
            steerCol.shape = ColliderShape::Cylinder;
            steerCol.radius = 0.04f;
            steerCol.halfHeight = 0.05f;
            scene.AddComponent(steerEntity, steerCol);

            // D. Wheel Link Entity (0.65 kg)
            Entity wheelEntity = scene.CreateEntity();
            TransformComponent wheelTransform;
            wheelTransform.position = glm::vec3(0.28f, 0.8f, 0.28f);
            scene.AddComponent(wheelEntity, wheelTransform);

            RigidBodyComponent wheelRb;
            wheelRb.motionType = BodyMotionType::Dynamic;
            wheelRb.mass = 0.65f;
            scene.AddComponent(wheelEntity, wheelRb);

            ColliderComponent wheelCol;
            wheelCol.shape = ColliderShape::Cylinder;
            wheelCol.radius = 0.0508f;
            wheelCol.halfHeight = 0.02f;
            scene.AddComponent(wheelEntity, wheelCol);

            // E. Vertical Steer Joint (Chassis <-> Steer Link)
            Entity steerJointEntity = scene.CreateEntity();
            JointComponent steerJoint;
            steerJoint.parentEntity = chassisEntity;
            steerJoint.childEntity = steerEntity;
            steerJoint.SetWorldSpace(glm::vec3(0.28f, 0.95f, 0.28f), glm::vec3(0.0f, 1.0f, 0.0f));
            steerJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(steerJointEntity, steerJoint);

            // F. Horizontal Drive Axle Joint (Steer Link <-> Wheel Link)
            Entity driveJointEntity = scene.CreateEntity();
            JointComponent driveJoint;
            driveJoint.parentEntity = steerEntity;
            driveJoint.childEntity = wheelEntity;
            driveJoint.SetWorldSpace(glm::vec3(0.28f, 0.8f, 0.28f), glm::vec3(1.0f, 0.0f, 0.0f));
            driveJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(driveJointEntity, driveJoint);

            // G. Swerve Module Component
            Entity moduleEntity = scene.CreateEntity();
            SwerveModuleComponent moduleComp;
            moduleComp.moduleIndex = 0;
            moduleComp.moduleLocation = glm::vec2(0.28f, 0.28f);
            moduleComp.steerEntity = steerEntity;
            moduleComp.wheelEntity = wheelEntity;
            moduleComp.steerJointEntity = steerJointEntity;
            moduleComp.driveJointEntity = driveJointEntity;
            moduleComp.driveMotor = DCMotorModel::KrakenX60(6.12, 0.96);
            moduleComp.steerMotor = DCMotorModel::Falcon500(21.43, 0.92);
            scene.AddComponent(moduleEntity, moduleComp);

            chassisComp.moduleEntities[0] = moduleEntity;
            scene.AddComponent(chassisEntity, chassisComp);

            // Step 1 to instantiate physics bodies and constraints
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            // Command +6.0V on drive motor [index 4 = FL drive]
            SimInputs inputs;
            inputs.robotEnabled = true;
            inputs.motorCommands[4].mode = MotorControlMode::Voltage;
            inputs.motorCommands[4].demand = 6.0; // 6.0 Volts
            swerveSystem.SetInputs(inputs);

            // Simulate 50 steps (250 ms) of wheel acceleration
            for (int i = 0; i < 50; ++i)
            {
                swerveSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            const auto& outputs = swerveSystem.GetOutputs();

            // Drive wheel angular velocity must be positive and accelerating
            TEST_EXPECT_TRUE(ctx, outputs.encoderVelocities[4] > 0.0);
            TEST_EXPECT_TRUE(ctx, outputs.encoderPositions[4] > 0.0);

            // Battery terminal voltage must sag under active drive motor current
            TEST_EXPECT_TRUE(ctx, outputs.batteryTerminalVoltage < 12.60);
            TEST_EXPECT_TRUE(ctx, outputs.totalBusCurrent > 1.5);
        }

        // -------------------------------------------------------------
        // 3. Physical Steer Azimuth Rotation Test
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
            steerTransform.position = glm::vec3(0.28f, 0.9f, 0.28f);
            scene.AddComponent(steerEntity, steerTransform);

            RigidBodyComponent steerRb;
            steerRb.motionType = BodyMotionType::Dynamic;
            steerRb.mass = 0.45f;
            scene.AddComponent(steerEntity, steerRb);

            ColliderComponent steerCol;
            steerCol.shape = ColliderShape::Cylinder;
            steerCol.radius = 0.04f;
            steerCol.halfHeight = 0.05f;
            scene.AddComponent(steerEntity, steerCol);

            Entity steerJointEntity = scene.CreateEntity();
            JointComponent steerJoint;
            steerJoint.parentEntity = chassisEntity;
            steerJoint.childEntity = steerEntity;
            steerJoint.SetWorldSpace(glm::vec3(0.28f, 0.95f, 0.28f), glm::vec3(0.0f, 1.0f, 0.0f));
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

            // Command +4.0V on steer motor [index 0 = FL steer]
            SimInputs inputs;
            inputs.robotEnabled = true;
            inputs.motorCommands[0].mode = MotorControlMode::Voltage;
            inputs.motorCommands[0].demand = 4.0;
            swerveSystem.SetInputs(inputs);

            for (int i = 0; i < 40; ++i)
            {
                swerveSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            const auto& outputs = swerveSystem.GetOutputs();

            // Steer angle and velocity must reflect positive azimuth rotation
            TEST_EXPECT_TRUE(ctx, outputs.encoderVelocities[0] > 0.0);
            TEST_EXPECT_TRUE(ctx, outputs.encoderPositions[0] > 0.0);
        }

        // -------------------------------------------------------------
        // 4. Current Limiting under Locked-Rotor Stall
        // -------------------------------------------------------------
        {
            BatteryModel battery(12.60_V, 0.023_ohm, 1.5_A, 60.0_A, 30.0_A);

            // Stall condition: high demand at 12V
            double commandedVolts = 12.0_V;
            double statorCurrent = 80.0_A; // Exceeds 30A supply limit
            double terminalVoltage = 11.5_V;

            double throttledVolts = battery.ApplySupplyCurrentLimit(commandedVolts, statorCurrent, terminalVoltage);

            // P_demanded = 80A * 12V = 960W; P_allowed = 30A * 11.5V = 345W
            // Scaled voltage = 12.0 * (345 / 960) = 4.3125V
            TEST_EXPECT_TRUE(ctx, throttledVolts < commandedVolts);
            TEST_EXPECT_NEAR(ctx, throttledVolts, 4.3125_V, 0.05);
        }

        // -------------------------------------------------------------
        // 5. Chassis Yaw & Gyro IMU Tracking
        // -------------------------------------------------------------
        {
            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);
            SwerveDriveSystem swerveSystem(scene);

            Entity chassisEntity = scene.CreateEntity();
            TransformComponent chassisTransform;
            chassisTransform.position = glm::vec3(0.0f, 1.0f, 0.0f);
            // Rotate chassis by 45 degrees around Y-axis
            chassisTransform.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            scene.AddComponent(chassisEntity, chassisTransform);

            RigidBodyComponent chassisRb;
            chassisRb.motionType = BodyMotionType::Dynamic;
            chassisRb.angularVelocity = glm::vec3(0.0f, 2.5f, 0.0f); // 2.5 rad/s yaw rate
            scene.AddComponent(chassisEntity, chassisRb);

            SwerveChassisComponent chassisComp;
            scene.AddComponent(chassisEntity, chassisComp);

            swerveSystem.FixedUpdate(PhysicsConfig::fixedDeltaTime);

            const auto& outputs = swerveSystem.GetOutputs();

            // Heading must reflect +45 degrees (pi/4 radians)
            TEST_EXPECT_NEAR(ctx, outputs.gyroYaw, 3.14159265358979323846 / 4.0, 0.01);
            TEST_EXPECT_NEAR(ctx, outputs.gyroYawRate, 2.5, tolerance);
        }

        std::cout << "[DONE] SwervePhysicalDriveTest\n";
    }
}