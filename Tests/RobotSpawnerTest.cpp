/**
 * @file RobotSpawnerTest.cpp
 * @brief Test suite verifying declarative JSON robot spawner and collision group filtering.
 */

#include "RobotSpawnerTest.h"
#include "TestFramework.h"
#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
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
    using namespace Robot;
    using namespace Robot::Drivetrain::Components;
    using namespace Actuation::Components;
    using namespace Physics;
    using namespace Physics::Components;
    using namespace Physics::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    void RunRobotSpawnerTest(TestContext& ctx)
    {
        std::cout << "[RUN ] RobotSpawnerTest\n";
        constexpr double tolerance = 1e-4;

        // -------------------------------------------------------------
        // 1. Spawning From JSON String Specification
        // -------------------------------------------------------------
        {
            Scene scene;
            const std::string jsonText = R"({
                "chassis": {
                    "mass_kg": 50.0,
                    "width_m": 0.70,
                    "height_m": 0.15,
                    "length_m": 0.70,
                    "wheelbase_m": 0.56,
                    "trackwidth_m": 0.56,
                    "friction": 0.7
                },
                "battery": {
                    "nominalVoltage_V": 12.60,
                    "internalResistance_ohm": 0.023,
                    "quiescentCurrent_A": 1.5,
                    "statorCurrentLimit_A": 60.0,
                    "supplyCurrentLimit_A": 40.0,
                    "brownoutWarning_V": 6.8,
                    "brownoutCutoff_V": 6.3,
                    "brownoutRecovery_V": 7.5
                },
                "motors": {
                    "steer": { "type": "Falcon500", "gearRatio": 21.43, "efficiency": 0.92 },
                    "drive": { "type": "KrakenX60", "gearRatio": 6.12, "efficiency": 0.96 }
                },
                "steerHousing": { "mass_kg": 0.45, "radius_m": 0.04, "height_m": 0.06 },
                "wheel": { "mass_kg": 0.65, "radius_m": 0.0508, "width_m": 0.04, "friction": 1.2 },
                "modules": [
                    { "index": 0, "name": "FL", "location_m": [-0.28,  0.28], "steerCAN": 1, "driveCAN": 2, "canCoder": 1 },
                    { "index": 1, "name": "FR", "location_m": [ 0.28,  0.28], "steerCAN": 3, "driveCAN": 4, "canCoder": 2 },
                    { "index": 2, "name": "BL", "location_m": [-0.28, -0.28], "steerCAN": 5, "driveCAN": 6, "canCoder": 3 },
                    { "index": 3, "name": "BR", "location_m": [ 0.28, -0.28], "steerCAN": 7, "driveCAN": 8, "canCoder": 4 }
                ]
            })";

            SpawnedRobot robot = RobotSpawner::SpawnFromJsonString(scene, jsonText, glm::vec3(0.0f, 1.0f, 0.0f));

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);
            TEST_EXPECT_TRUE(ctx, robot.chassisEntity != invalidEntity);
            TEST_EXPECT_TRUE(ctx, robot.batteryEntity != invalidEntity);

            // Verify 4 modules, 4 steer housings, 4 wheels, and 8 joints were allocated
            for (uint32_t i = 0; i < 4; ++i)
            {
                TEST_EXPECT_TRUE(ctx, robot.moduleEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.steerEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.wheelEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.steerJointEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.driveJointEntities[i] != invalidEntity);
            }

            // Verify Chassis Components
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);
            if (chassisRb)
            {
                TEST_EXPECT_NEAR(ctx, chassisRb->mass, 50.0f, tolerance);
                TEST_EXPECT_EQ(ctx, chassisRb->collisionGroupID, 1u);
                TEST_EXPECT_EQ(ctx, chassisRb->collisionSubGroupID, 0u);
            }

            auto* chassisCol = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisCol != nullptr);
            if (chassisCol)
            {
                TEST_EXPECT_EQ(ctx, static_cast<int>(chassisCol->shape), static_cast<int>(ColliderShape::Box));
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.x, 0.35f, tolerance);
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.y, 0.075f, tolerance);
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.z, 0.35f, tolerance);
            }

            // Verify Module 0 (Front-Left) components and coordinates
            auto* mod0 = scene.GetComponent<SwerveModuleComponent>(robot.moduleEntities[0]);
            TEST_EXPECT_TRUE(ctx, mod0 != nullptr);
            if (mod0)
            {
                TEST_EXPECT_EQ(ctx, mod0->moduleIndex, 0u);
                TEST_EXPECT_NEAR(ctx, mod0->moduleLocation.x, -0.28f, tolerance); // Left
                TEST_EXPECT_NEAR(ctx, mod0->moduleLocation.y,  0.28f, tolerance); // Forward
                TEST_EXPECT_EQ(ctx, mod0->steerCANId, 1u);
                TEST_EXPECT_EQ(ctx, mod0->driveCANId, 2u);
                TEST_EXPECT_EQ(ctx, mod0->canCoderId, 1u);
            }

            // Verify Battery Component
            auto* battery = scene.GetComponent<BatteryComponent>(robot.batteryEntity);
            TEST_EXPECT_TRUE(ctx, battery != nullptr);
            if (battery)
            {
                TEST_EXPECT_NEAR(ctx, battery->batteryModel.GetNominalVoltage(), 12.60_V, tolerance);
                TEST_EXPECT_NEAR(ctx, battery->batteryModel.GetInternalResistance(), 0.023_ohm, tolerance);
            }
        }

        // -------------------------------------------------------------
        // 2. Physical Instantiation & Stepping Stability Under Gravity
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);

            SpawnedRobot robot = RobotSpawner::SpawnFromJsonFile(scene, "FRCBananaSim/Resources/Robots/robot_swerve.json", glm::vec3(0.0f, 1.5f, 0.0f));

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            if (robot.isSuccess)
            {
                // Step 50 ticks (250 ms) of gravity acceleration
                constexpr double dt = PhysicsConfig::fixedDeltaTime;
                for (int i = 0; i < 50; ++i)
                {
                    physicsSystem.FixedUpdate(dt);
                    jointSystem.FixedUpdate(dt);
                }

                auto* chassisTransform = scene.GetComponent<TransformComponent>(robot.chassisEntity);
                auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);

                TEST_EXPECT_TRUE(ctx, chassisTransform != nullptr);
                TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);

                if (chassisTransform && chassisRb)
                {
                    // Chassis should have fallen smoothly in -Y under gravity
                    TEST_EXPECT_TRUE(ctx, chassisTransform->position.y < 1.5f);
                    TEST_EXPECT_TRUE(ctx, chassisRb->linearVelocity.y < 0.0f);
                }

                // Joint deflections across all 8 hinges must stay strictly < 5 mm
                for (uint32_t i = 0; i < 4; ++i)
                {
                    auto* steerJoint = scene.GetComponent<JointComponent>(robot.steerJointEntities[i]);
                    auto* driveJoint = scene.GetComponent<JointComponent>(robot.driveJointEntities[i]);

                    TEST_EXPECT_TRUE(ctx, steerJoint != nullptr);
                    TEST_EXPECT_TRUE(ctx, driveJoint != nullptr);

                    if (steerJoint)
                    {
                        TEST_EXPECT_TRUE(ctx, steerJoint->currentDeflection < 0.005f);
                    }
                    if (driveJoint)
                    {
                        TEST_EXPECT_TRUE(ctx, driveJoint->currentDeflection < 0.005f);
                    }
                }
            }
        }

        std::cout << "[DONE] RobotSpawnerTest\n";
    }
}