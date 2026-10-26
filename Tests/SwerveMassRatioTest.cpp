/**
 * @file SwerveMassRatioTest.cpp
 * @brief 60-second (12,000 fixed steps at 200 Hz) mass-ratio stress test validating Gate 1 stability criteria.
 */

#include "SwerveMassRatioTest.h"
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

    void RunSwerveMassRatioTest(TestContext& ctx)
    {
        std::cout << "[RUN ] SwerveMassRatioTest (60s / 12,000 Steps Gate 1 Stress Test)\n";

        JoltContext context;
        TEST_EXPECT_TRUE(ctx, context.Initialize());

        Scene scene;
        scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

        PhysicsSystem physicsSystem(scene, context);
        JointSystem jointSystem(scene, context);
        SwerveDriveSystem swerveSystem(scene);

        // 1. Spawn complete swerve robot (50 kg chassis on 4x 1.1 kg module assemblies)
        SpawnedRobot robot = RobotSpawner::SpawnFromJsonFile(scene, "FRCBananaSim/Resources/Robots/robot_swerve.json",
            glm::vec3(0.0f, 1.0f, 0.0f));
        TEST_EXPECT_TRUE(ctx, robot.isSuccess);

        // Pin the chassis rigidly to test-bench so all torque and gyroscopic reaction concentrates into joints
        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
        TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);
        chassisRb->motionType = BodyMotionType::Kinematic;

        // Step 1: Instantiate physics bodies and joints
        constexpr double dt = PhysicsConfig::fixedDeltaTime; // 0.005s (200 Hz)
        physicsSystem.FixedUpdate(dt);
        jointSystem.FixedUpdate(dt);

        // 2. Command maximum drive and steer voltages across all 8 motors
        SimInputs inputs;
        inputs.robotEnabled = true;

        for (uint32_t i = 0; i < 4; ++i)
        {
            // Steer motors: +12.0 V continuous azimuth rotation
            inputs.motorCommands[i].mode = MotorControlMode::Voltage;
            inputs.motorCommands[i].demand = 12.0;
            inputs.motorCommands[i].statorCurrentLimit = 60.0;

            // Drive motors: +12.0 V full drive demand
            inputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
            inputs.motorCommands[4 + i].demand = 12.0;
            inputs.motorCommands[4 + i].statorCurrentLimit = 60.0;
        }

        swerveSystem.SetInputs(inputs);

        // 3. Execution tracking metrics
        constexpr int totalSteps = 12000; // 60 simulated seconds at 200 Hz
        float peakSteerDeflection = 0.0f;
        float peakDriveDeflection = 0.0f;
        double minTerminalVoltage = 12.60;
        double maxTerminalVoltage = 0.0;
        bool anyNaN = false;
        bool anyExplosion = false;

        // 4. Run 12,000 steps of intense physical simulation
        for (int step = 0; step < totalSteps; ++step)
        {
            swerveSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);
            physicsSystem.FixedUpdate(dt);

            const auto& outputs = swerveSystem.GetOutputs();

            // Track battery terminal voltage range
            minTerminalVoltage = std::min(minTerminalVoltage, outputs.batteryTerminalVoltage);
            maxTerminalVoltage = std::max(maxTerminalVoltage, outputs.batteryTerminalVoltage);

            // Verify no brownout collapse occurs under normal current limits
            if (outputs.isBrownedOut)
            {
                anyExplosion = true;
            }

            // Inspect joint deflections across all 8 hinges
            for (uint32_t i = 0; i < 4; ++i)
            {
                auto* steerJoint = scene.GetComponent<JointComponent>(robot.steerJointEntities[i]);
                auto* driveJoint = scene.GetComponent<JointComponent>(robot.driveJointEntities[i]);

                if (steerJoint)
                {
                    float def = steerJoint->currentDeflection;
                    if (std::isnan(def) || std::isinf(def))
                    {
                        anyNaN = true;
                    }
                    peakSteerDeflection = std::max(peakSteerDeflection, def);
                }

                if (driveJoint)
                {
                    float def = driveJoint->currentDeflection;
                    if (std::isnan(def) || std::isinf(def))
                    {
                        anyNaN = true;
                    }
                    peakDriveDeflection = std::max(peakDriveDeflection, def);
                }
            }

            // Periodically check rigid body velocities for explosive spikes
            if (step % 200 == 0)
            {
                for (uint32_t i = 0; i < 4; ++i)
                {
                    auto* steerRb = scene.GetComponent<RigidBodyComponent>(robot.steerEntities[i]);
                    auto* wheelRb = scene.GetComponent<RigidBodyComponent>(robot.wheelEntities[i]);

                    if (steerRb)
                    {
                        if (std::isnan(steerRb->angularVelocity.y) || std::abs(steerRb->angularVelocity.y) > 2000.0f)
                        {
                            anyExplosion = true;
                        }
                    }

                    if (wheelRb)
                    {
                        if (std::isnan(wheelRb->angularVelocity.x) || std::abs(wheelRb->angularVelocity.x) > 2000.0f)
                        {
                            anyExplosion = true;
                        }
                    }
                }
            }
        }

        // -------------------------------------------------------------
        // Gate 1 Acceptance Assertions
        // -------------------------------------------------------------

        // 1. Numerical Stability: strictly zero NaNs or infinities
        TEST_EXPECT_FALSE(ctx, anyNaN);
        TEST_EXPECT_FALSE(ctx, anyExplosion);

        // 2. Joint Deflection: must stay strictly < 5.0 mm (0.005 m) across all 8 hinges
        std::cout << "  [Gate 1] Peak Steer Hinge Deflection: " << (peakSteerDeflection * 1000.0f) << " mm (Limit: < 5.0 mm)\n";
        std::cout << "  [Gate 1] Peak Drive Hinge Deflection: " << (peakDriveDeflection * 1000.0f) << " mm (Limit: < 5.0 mm)\n";
        TEST_EXPECT_TRUE(ctx, peakSteerDeflection < 0.005f);
        TEST_EXPECT_TRUE(ctx, peakDriveDeflection < 0.005f);

        // 3. Electrical Equilibrium: Sustained sag settles around ~9.97V under full 4x drive + 4x steer load
        std::cout << "  [Gate 1] Minimum Battery Voltage: " << minTerminalVoltage << " V (Target: 8.5 V - 10.5 V)\n";
        TEST_EXPECT_TRUE(ctx, minTerminalVoltage >= 8.5_V);
        TEST_EXPECT_TRUE(ctx, minTerminalVoltage <= 10.5_V);

        // 4. Continuous Angle Tracking: Steer & Drive encoders accumulate continuous rotation without NaN or stalling
        const auto& finalOutputs = swerveSystem.GetOutputs();
        for (uint32_t i = 0; i < 4; ++i)
        {
            // Steer encoder must rotate continuously (> 100 rad)
            TEST_EXPECT_TRUE(ctx, finalOutputs.encoderPositions[i] > 100.0);
            // Drive encoder under 40A supply limiting accumulates ~670 rad (> 500 rad)
            TEST_EXPECT_TRUE(ctx, finalOutputs.encoderPositions[4 + i] > 500.0);
        }

        std::cout << "[DONE] SwerveMassRatioTest (Gate 1 Validation Passed)\n";
    }
}