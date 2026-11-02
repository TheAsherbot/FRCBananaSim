/**
 * @file JointStabilityTest.cpp
 * @brief Test suite verifying physical joints, motor drive modes, and 50:1 mass-ratio stability.
 */

#include "JointStabilityTest.h"
#include "TestFramework.h"
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
    using namespace Physics;
    using namespace Physics::Components;
    using namespace Physics::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    void RunJointStabilityTest(TestContext& ctx)
    {
        std::cout << "[RUN ] JointStabilityTest\n";
        constexpr double tolerance = 1e-4;

        // -------------------------------------------------------------
        // 1. Static World Attachment & Pendulum Constraint Test
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);

            // Child body (10 kg dynamic pendulum)
            Entity childEntity = scene.CreateEntity();
            TransformComponent transform;
            transform.position = glm::vec3(0.0f, 4.0f, 0.0f);
            scene.AddComponent(childEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 10.0f;
            scene.AddComponent(childEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.2f, 0.5f, 0.2f);
            scene.AddComponent(childEntity, col);

            // Joint attaching child to world pivot at (0, 5, 0)
            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.parentEntity = invalidEntity; // Attach to static world
            joint.childEntity = childEntity;
            joint.SetWorldSpace(glm::vec3(0.0f, 5.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            joint.motorMode = JointMotorMode::Off;
            scene.AddComponent(jointEntity, joint);

            // Step 50 steps (0.25s) under gravity
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            for (int i = 0; i < 50; ++i)
            {
                physicsSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
            }

            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);
            TEST_EXPECT_TRUE(ctx, jointComp->isCreated);

            // Deflection from world anchor must stay strictly under 1 mm
            TEST_EXPECT_TRUE(ctx, jointComp->currentDeflection < 0.001f);
        }

        // -------------------------------------------------------------
        // 2. Velocity Motor Tracking Test
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);

            Entity wheelEntity = scene.CreateEntity();
            TransformComponent transform;
            transform.position = glm::vec3(0.0f, 2.0f, 0.0f);
            scene.AddComponent(wheelEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 2.0f;
            scene.AddComponent(wheelEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Cylinder;
            col.radius = 0.05f;
            col.halfHeight = 0.02f;
            scene.AddComponent(wheelEntity, col);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.parentEntity = invalidEntity;
            joint.childEntity = wheelEntity;
            joint.SetWorldSpace(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            joint.motorMode = JointMotorMode::Velocity;
            joint.targetVelocity = 15.0f;   // 15 rad/s
            joint.maxMotorTorque = 30.0f;   // 30 N*m
            scene.AddComponent(jointEntity, joint);

            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            for (int i = 0; i < 50; ++i)
            {
                physicsSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
            }

            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);
            // Verify wheel reached commanded angular velocity
            auto* wheelRb = scene.GetComponent<RigidBodyComponent>(wheelEntity);
            TEST_EXPECT_TRUE(ctx, wheelRb != nullptr);
            TEST_EXPECT_NEAR(ctx, wheelRb->angularVelocity.y, 15.0f, 0.5f);
        }

        // -------------------------------------------------------------
        // 3. Position Limits & Hard Stops Enforcement
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);

            Entity armEntity = scene.CreateEntity();
            TransformComponent transform;
            transform.position = glm::vec3(0.5f, 2.0f, 0.0f);
            scene.AddComponent(armEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 3.0f;
            scene.AddComponent(armEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.5f, 0.05f, 0.05f);
            scene.AddComponent(armEntity, col);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.parentEntity = invalidEntity;
            joint.childEntity = armEntity;
            joint.SetWorldSpace(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            joint.hasLimits = true;
            joint.minLimit = -0.785398f;  // -45 degrees
            joint.maxLimit = 0.785398f;   // +45 degrees
            joint.motorMode = JointMotorMode::Position;
            joint.targetAngle = 2.0f;     // Command well beyond max limit (+114 degrees)
            joint.maxMotorTorque = 50.0f;
            scene.AddComponent(jointEntity, joint);

            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            for (int i = 0; i < 100; ++i)
            {
                physicsSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
            }

            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);
            // Verify angle is clamped at limit (+45 deg = 0.7854 rad) with small elasticity margin
            TEST_EXPECT_TRUE(ctx, jointComp->currentAngle <= 0.785398f + 0.05f);
        }

        // -------------------------------------------------------------
        // 4. Extreme 50:1 Mass-Ratio Deflection under 40 N*m Stall Torque
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);

            // 50 kg dynamic robot chassis
            Entity chassisEntity = scene.CreateEntity();
            TransformComponent chassisTransform;
            chassisTransform.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(chassisEntity, chassisTransform);

            RigidBodyComponent chassisRb;
            chassisRb.motionType = BodyMotionType::Dynamic;
            chassisRb.mass = 50.0f; // 50 kg
            scene.AddComponent(chassisEntity, chassisRb);

            ColliderComponent chassisCol;
            chassisCol.shape = ColliderShape::Box;
            chassisCol.boxHalfExtents = glm::vec3(0.375f, 0.075f, 0.375f);
            scene.AddComponent(chassisEntity, chassisCol);

            // 1 kg dynamic wheel assembly (50:1 mass ratio!)
            Entity wheelEntity = scene.CreateEntity();
            TransformComponent wheelTransform;
            wheelTransform.position = glm::vec3(0.28f, 0.9f, 0.28f);
            scene.AddComponent(wheelEntity, wheelTransform);

            RigidBodyComponent wheelRb;
            wheelRb.motionType = BodyMotionType::Dynamic;
            wheelRb.mass = 1.0f; // 1 kg
            scene.AddComponent(wheelEntity, wheelRb);

            ColliderComponent wheelCol;
            wheelCol.shape = ColliderShape::Cylinder;
            wheelCol.radius = 0.0508f;
            wheelCol.halfHeight = 0.02f;
            scene.AddComponent(wheelEntity, wheelCol);

            // Connect wheel to chassis via vertical hinge constraint
            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.parentEntity = chassisEntity;
            joint.childEntity = wheelEntity;
            joint.SetWorldSpace(glm::vec3(0.28f, 0.95f, 0.28f), glm::vec3(0.0f, 1.0f, 0.0f));
            joint.motorMode = JointMotorMode::Velocity;
            joint.targetVelocity = 0.0f;  // Stall / locked condition
            joint.maxMotorTorque = 40.0f; // 40 N*m stall torque (Kraken X60 geared)
            scene.AddComponent(jointEntity, joint);

            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            constexpr int numSteps = 1000; // 5 seconds of continuous 200 Hz simulation

            float peakDeflection = 0.0f;
            bool noNaNs = true;

            for (int step = 0; step < numSteps; ++step)
            {
                physicsSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);

                auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
                if (!jointComp || !jointComp->isCreated)
                {
                    noNaNs = false;
                    break;
                }

                float def = jointComp->currentDeflection;
                if (std::isnan(def) || std::isinf(def))
                {
                    noNaNs = false;
                    break;
                }

                if (def > peakDeflection)
                {
                    peakDeflection = def;
                }
            }

            TEST_EXPECT_TRUE(ctx, noNaNs);

            // Gate 1 Acceptance Criteria: Deflection strictly < 5 mm (0.005 m)
            TEST_EXPECT_TRUE(ctx, peakDeflection < 0.005f);
        }

        std::cout << "[DONE] JointStabilityTest\n";
    }
}