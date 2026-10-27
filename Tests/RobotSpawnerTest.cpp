/**
 * @file RobotSpawnerTest.cpp
 * @brief Test suite verifying Schema v2 JSON robot spawner, single Jolt rigid body creation, and visual hierarchy.
 */

#include "RobotSpawnerTest.h"
#include "TestFramework.h"
#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Model/SwerveParams.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Core/Units/Units.h"

#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Robot;
    using namespace Robot::Drivetrain::Model;
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
        // 1. Spawning From Schema v2 JSON Specification
        // -------------------------------------------------------------
        {
            Scene scene;
            const std::string jsonText = R"({
                "name": "Test_Swerve_MK5n",
                "version": "2.0.0",
                "chassis": {
                    "mass_kg": 50.0,
                    "width_m": 0.70,
                    "height_m": 0.15,
                    "length_m": 0.70,
                    "wheelbase_m": 0.56,
                    "trackwidth_m": 0.56,
                    "cgHeight_m": 0.20,
                    "cgOffset_m": [0.0, 0.0],
                    "inertiaTensor_kgm2": [
                        [ 2.1354, 0.0, 0.0 ],
                        [ 0.0, 2.1354, 0.0 ],
                        [ 0.0, 0.0, 4.0833 ]
                    ],
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
                    "steer": { "type": "KrakenX44", "gearRatio": 26.0909, "efficiency": 0.92 },
                    "drive": { "type": "KrakenX60", "gearRatio": 5.27, "efficiency": 0.96 }
                },
                "steerHousing": { "mass_kg": 0.45, "radius_m": 0.04, "height_m": 0.06 },
                "wheel": { "mass_kg": 0.65, "radius_m": 0.0508, "width_m": 0.0381, "axleInertia_kgm2": 0.00084, "friction": 1.2 },
                "tire": {
                    "radialStiffness_Npm": 120000.0,
                    "radialDamping_Nspm": 1200.0,
                    "preSlide_m": 0.001,
                    "mu_static": 1.20,
                    "mu_kinetic": 1.05,
                    "stribeckVelocity_mps": 0.15,
                    "loadSensitivityBeta": 0.08,
                    "rollingResistance": 0.015,
                    "lateralFrictionRatio": 1.0
                },
                "steerModule": {
                    "steerHousingInertia_kgm2": 0.00036,
                    "coulombFriction_Nm": 0.08,
                    "viscousDamping_Nms": 0.002
                },
                "driveModule": {
                    "coulombFriction_Nm": 1.8
                },
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

            // Verify Exactly 1 RigidBodyComponent exists in the entire scene (Chassis)
            auto allRigidBodies = scene.Query<RigidBodyComponent>();
            TEST_EXPECT_EQ(ctx, allRigidBodies.size(), 1u);

            // Verify 4 modules, 4 steer housings, 4 wheels were allocated
            for (uint32_t i = 0; i < 4; ++i)
            {
                TEST_EXPECT_TRUE(ctx, robot.moduleEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.steerEntities[i] != invalidEntity);
                TEST_EXPECT_TRUE(ctx, robot.wheelEntities[i] != invalidEntity);

                // Joints no longer exist in the single-body analytic model
                TEST_EXPECT_FALSE(ctx, scene.HasComponent<JointComponent>(robot.moduleEntities[i]));
                TEST_EXPECT_FALSE(ctx, scene.HasComponent<RigidBodyComponent>(robot.steerEntities[i]));
                TEST_EXPECT_FALSE(ctx, scene.HasComponent<RigidBodyComponent>(robot.wheelEntities[i]));
                TEST_EXPECT_FALSE(ctx, scene.HasComponent<ColliderComponent>(robot.steerEntities[i]));
                TEST_EXPECT_FALSE(ctx, scene.HasComponent<ColliderComponent>(robot.wheelEntities[i]));
            }

            // Verify Chassis RigidBody properties
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);
            if (chassisRb)
            {
                TEST_EXPECT_NEAR(ctx, chassisRb->mass, 50.0f, tolerance);
                TEST_EXPECT_EQ(ctx, chassisRb->collisionGroupID, 1u);
                TEST_EXPECT_EQ(ctx, chassisRb->collisionSubGroupID, 0u);
                TEST_EXPECT_NEAR(ctx, chassisRb->gravityFactor, 0.0f, tolerance);
                TEST_EXPECT_FALSE(ctx, chassisRb->allowSleeping);
                TEST_EXPECT_TRUE(ctx, chassisRb->hasCustomInertia);

                // Verify Jolt-mapped inertia tensor (Pitch around X, Yaw around Y, Roll around Z)
                TEST_EXPECT_NEAR(ctx, chassisRb->inertiaTensor[0][0], 2.1354f, 1e-3f);
                TEST_EXPECT_NEAR(ctx, chassisRb->inertiaTensor[1][1], 4.0833f, 1e-3f);
                TEST_EXPECT_NEAR(ctx, chassisRb->inertiaTensor[2][2], 2.1354f, 1e-3f);
            }

            // Verify Chassis Collider
            auto* chassisCol = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisCol != nullptr);
            if (chassisCol)
            {
                TEST_EXPECT_EQ(ctx, static_cast<int>(chassisCol->shape), static_cast<int>(ColliderShape::Box));
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.x, 0.35f, tolerance);
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.y, 0.075f, tolerance);
                TEST_EXPECT_NEAR(ctx, chassisCol->boxHalfExtents.z, 0.35f, tolerance);
            }

            // Verify Hierarchy Link Graph
            auto* chassisHierarchy = scene.GetComponent<HierarchyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, chassisHierarchy != nullptr);
            if (chassisHierarchy)
            {
                TEST_EXPECT_EQ(ctx, chassisHierarchy->parent, invalidEntity);
                TEST_EXPECT_EQ(ctx, chassisHierarchy->children.size(), 4u);
            }

            for (uint32_t i = 0; i < 4; ++i)
            {
                auto* steerH = scene.GetComponent<HierarchyComponent>(robot.steerEntities[i]);
                TEST_EXPECT_TRUE(ctx, steerH != nullptr);
                if (steerH)
                {
                    TEST_EXPECT_EQ(ctx, steerH->parent, robot.chassisEntity);
                    TEST_EXPECT_EQ(ctx, steerH->children.size(), 1u);
                    TEST_EXPECT_EQ(ctx, steerH->children[0], robot.wheelEntities[i]);
                }

                auto* wheelH = scene.GetComponent<HierarchyComponent>(robot.wheelEntities[i]);
                TEST_EXPECT_TRUE(ctx, wheelH != nullptr);
                if (wheelH)
                {
                    TEST_EXPECT_EQ(ctx, wheelH->parent, robot.steerEntities[i]);
                    TEST_EXPECT_TRUE(ctx, wheelH->children.empty());
                }
            }

            // Verify Module 0
            auto* mod0 = scene.GetComponent<SwerveModuleComponent>(robot.moduleEntities[0]);
            TEST_EXPECT_TRUE(ctx, mod0 != nullptr);
            if (mod0)
            {
                TEST_EXPECT_EQ(ctx, mod0->moduleIndex, 0u);
                TEST_EXPECT_NEAR(ctx, mod0->moduleLocation.x, -0.28f, tolerance);
                TEST_EXPECT_NEAR(ctx, mod0->moduleLocation.y,  0.28f, tolerance);
                TEST_EXPECT_EQ(ctx, mod0->steerCANId, 1u);
                TEST_EXPECT_EQ(ctx, mod0->driveCANId, 2u);
                TEST_EXPECT_EQ(ctx, mod0->canCoderId, 1u);
                TEST_EXPECT_NEAR(ctx, mod0->steerMotor.GetGearRatio(), 26.0909, 1e-3);
                TEST_EXPECT_NEAR(ctx, mod0->driveMotor.GetGearRatio(), 5.27, 1e-3);
            }

            // Verify Battery
            auto* battery = scene.GetComponent<BatteryComponent>(robot.batteryEntity);
            TEST_EXPECT_TRUE(ctx, battery != nullptr);
            if (battery)
            {
                TEST_EXPECT_NEAR(ctx, battery->batteryModel.GetNominalVoltage(), 12.60_V, tolerance);
                TEST_EXPECT_NEAR(ctx, battery->batteryModel.GetInternalResistance(), 0.023_ohm, tolerance);
            }
        }

        // -------------------------------------------------------------
        // 2. Physical Instantiation & Stability of Single Body in Jolt
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);

            SpawnedRobot robot = RobotSpawner::SpawnFromJsonFile(scene, "FRCBananaSim/Resources/Robots/robot_swerve.json", glm::vec3(0.0f, 1.5f, 0.0f));

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            if (robot.isSuccess)
            {
                // Step 50 ticks (250 ms) of simulation
                constexpr double dt = PhysicsConfig::fixedDeltaTime;
                for (int i = 0; i < 50; ++i)
                {
                    physicsSystem.FixedUpdate(dt);
                }

                auto* chassisTransform = scene.GetComponent<TransformComponent>(robot.chassisEntity);
                auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);

                TEST_EXPECT_TRUE(ctx, chassisTransform != nullptr);
                TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);

                if (chassisTransform && chassisRb)
                {
                    // With gravityFactor = 0.0, the chassis does NOT drop ballistically in Jolt
                    TEST_EXPECT_NEAR(ctx, chassisTransform->position.y, 1.5f, 1e-3f);
                    TEST_EXPECT_NEAR(ctx, chassisRb->linearVelocity.y, 0.0f, 1e-3f);
                    TEST_EXPECT_FALSE(ctx, chassisRb->allowSleeping);
                }

                // Verify kinematic step: apply a linear velocity to the single body and verify smooth translation
                if (chassisRb && !chassisRb->bodyID.IsInvalid())
                {
                    context.GetBodyInterface().SetLinearVelocity(chassisRb->bodyID, JPH::Vec3(0.0f, 0.0f, 2.0f));

                    for (int i = 0; i < 20; ++i)
                    {
                        physicsSystem.FixedUpdate(dt);
                    }

                    // Over 20 ticks * 5 ms = 0.1 s at 2 m/s forward -> Delta Z ~ +0.20 m
                    TEST_EXPECT_NEAR(ctx, chassisTransform->position.z, 0.20f, 0.02f);
                    TEST_EXPECT_NEAR(ctx, chassisRb->linearVelocity.z, 2.0f, 1e-3f);
                }
            }
        }

        std::cout << "[DONE] RobotSpawnerTest\n";
    }
}