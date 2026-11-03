/**
 * @file DifferentialMechanismTest.cpp
 * @brief Implementation of differential mechanism kinematic, dynamic, and stability unit tests.
 */

#include "DifferentialMechanismTest.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Components/DifferentialComponent.h"
#include "Robot/Mechanisms/Systems/DifferentialMechanismSystem.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"

#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Physics;
    using namespace Physics::Components;
    using namespace Physics::Systems;
    using namespace Robot::Mechanisms::Components;
    using namespace Robot::Mechanisms::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;

    void RunDifferentialMechanismTest(TestContext& ctx)
    {
        std::cout << "[RUN ] DifferentialMechanismTest\n";

        // -------------------------------------------------------------
        // Test 1a: Symmetric Command (+6V, +6V) -> Must produce Pure Pitch, Roll = 0
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = Physics::PhysicsConfig::fixedDeltaTime; // 0.005s (200 Hz)
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            DifferentialMechanismSystem diffSystem(scene, &context);

            // Mount Base
            Entity baseEntity = scene.CreateEntity();
            TransformComponent tBase;
            tBase.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(baseEntity, tBase);
            RigidBodyComponent rbBase;
            rbBase.motionType = BodyMotionType::Static;
            rbBase.isSensor = true; // Disable contact collision hull locking
            scene.AddComponent(baseEntity, rbBase);
            ColliderComponent colBase;
            colBase.shape = ColliderShape::Box;
            colBase.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(baseEntity, colBase);

            // Pitch Link (Yoke)
            Entity pitchEntity = scene.CreateEntity();
            TransformComponent tPitch;
            tPitch.position = glm::vec3(0.0f, 1.0f, 0.05f);
            scene.AddComponent(pitchEntity, tPitch);
            RigidBodyComponent rbPitch;
            rbPitch.motionType = BodyMotionType::Dynamic;
            rbPitch.mass = 1.0f;
            rbPitch.gravityFactor = 0.0f;
            rbPitch.isSensor = true;
            scene.AddComponent(pitchEntity, rbPitch);
            ColliderComponent colPitch;
            colPitch.shape = ColliderShape::Box;
            colPitch.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(pitchEntity, colPitch);

            // Roll Link (Tool)
            Entity rollEntity = scene.CreateEntity();
            TransformComponent tRoll;
            tRoll.position = glm::vec3(0.0f, 1.0f, 0.15f);
            scene.AddComponent(rollEntity, tRoll);
            RigidBodyComponent rbRoll;
            rbRoll.motionType = BodyMotionType::Dynamic;
            rbRoll.mass = 0.5f;
            rbRoll.gravityFactor = 0.0f;
            rbRoll.isSensor = true;
            scene.AddComponent(rollEntity, rbRoll);
            ColliderComponent colRoll;
            colRoll.shape = ColliderShape::Box;
            colRoll.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(rollEntity, colRoll);

            // Pitch Joint (Hinge around X axis)
            Entity pitchJoint = scene.CreateEntity();
            JointComponent jcPitch;
            jcPitch.type = JointType::Hinge;
            jcPitch.parentEntity = baseEntity;
            jcPitch.childEntity = pitchEntity;
            jcPitch.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            scene.AddComponent(pitchJoint, jcPitch);

            // Roll Joint (Hinge around Z axis)
            Entity rollJoint = scene.CreateEntity();
            JointComponent jcRoll;
            jcRoll.type = JointType::Hinge;
            jcRoll.parentEntity = pitchEntity;
            jcRoll.childEntity = rollEntity;
            jcRoll.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.10f), glm::vec3(0.0f, 0.0f, 1.0f));
            scene.AddComponent(rollJoint, jcRoll);

            // Differential Component (Symmetric Bevel: Gp = 40.0, Gr = 25.0)
            Entity diffEntity = scene.CreateEntity();
            DifferentialComponent diff;
            diff.jointEntityA = pitchJoint;
            diff.jointEntityB = rollJoint;
            diff.linkEntityA = pitchEntity;
            diff.linkEntityB = rollEntity;
            diff.motorModelA = Actuation::DCMotorModel::KrakenX44();
            diff.motorModelB = Actuation::DCMotorModel::KrakenX44();
            diff.efficiency = 1.0;
            diff.coulombFrictionNm = { 0.0, 0.0 };

            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, 40.0, 25.0, false);
            scene.AddComponent(diffEntity, diff);

            // Warm up constraints
            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            // Phase A: Symmetric Command (+6V, +6V) -> Must produce Pure Pitch, Roll = 0
            auto* liveDiff = scene.GetComponent<DifferentialComponent>(diffEntity);
            liveDiff->commandedVoltageA = 6.0;
            liveDiff->commandedVoltageB = 6.0;

            for (int step = 0; step < 100; ++step) // 0.5 seconds
            {
                diffSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            TEST_EXPECT_TRUE(ctx, std::abs(liveDiff->jointPositionsRad.x) > 0.05); // Pitch moved
            TEST_EXPECT_NEAR(ctx, liveDiff->jointPositionsRad.y, 0.0, 1e-3);        // Roll remained exactly 0
        }

        // -------------------------------------------------------------
        // Test 1b: Anti-symmetric Command (+6V, -6V) -> Must produce Pure Roll, Pitch = 0
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = Physics::PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            DifferentialMechanismSystem diffSystem(scene, &context);

            Entity baseEntity = scene.CreateEntity();
            TransformComponent tBase;
            tBase.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(baseEntity, tBase);
            RigidBodyComponent rbBase;
            rbBase.motionType = BodyMotionType::Static;
            rbBase.isSensor = true;
            scene.AddComponent(baseEntity, rbBase);
            ColliderComponent colBase;
            colBase.shape = ColliderShape::Box;
            colBase.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(baseEntity, colBase);

            Entity pitchEntity = scene.CreateEntity();
            TransformComponent tPitch;
            tPitch.position = glm::vec3(0.0f, 1.0f, 0.05f);
            scene.AddComponent(pitchEntity, tPitch);
            RigidBodyComponent rbPitch;
            rbPitch.motionType = BodyMotionType::Dynamic;
            rbPitch.mass = 1.0f;
            rbPitch.gravityFactor = 0.0f;
            rbPitch.isSensor = true;
            scene.AddComponent(pitchEntity, rbPitch);
            ColliderComponent colPitch;
            colPitch.shape = ColliderShape::Box;
            colPitch.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(pitchEntity, colPitch);

            Entity rollEntity = scene.CreateEntity();
            TransformComponent tRoll;
            tRoll.position = glm::vec3(0.0f, 1.0f, 0.15f);
            scene.AddComponent(rollEntity, tRoll);
            RigidBodyComponent rbRoll;
            rbRoll.motionType = BodyMotionType::Dynamic;
            rbRoll.mass = 0.5f;
            rbRoll.gravityFactor = 0.0f;
            rbRoll.isSensor = true;
            scene.AddComponent(rollEntity, rbRoll);
            ColliderComponent colRoll;
            colRoll.shape = ColliderShape::Box;
            colRoll.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f);
            scene.AddComponent(rollEntity, colRoll);

            Entity pitchJoint = scene.CreateEntity();
            JointComponent jcPitch;
            jcPitch.type = JointType::Hinge;
            jcPitch.parentEntity = baseEntity;
            jcPitch.childEntity = pitchEntity;
            jcPitch.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            scene.AddComponent(pitchJoint, jcPitch);

            Entity rollJoint = scene.CreateEntity();
            JointComponent jcRoll;
            jcRoll.type = JointType::Hinge;
            jcRoll.parentEntity = pitchEntity;
            jcRoll.childEntity = rollEntity;
            jcRoll.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.10f), glm::vec3(0.0f, 0.0f, 1.0f));
            scene.AddComponent(rollJoint, jcRoll);

            Entity diffEntity = scene.CreateEntity();
            DifferentialComponent diff;
            diff.jointEntityA = pitchJoint;
            diff.jointEntityB = rollJoint;
            diff.linkEntityA = pitchEntity;
            diff.linkEntityB = rollEntity;
            diff.motorModelA = Actuation::DCMotorModel::KrakenX44();
            diff.motorModelB = Actuation::DCMotorModel::KrakenX44();
            diff.efficiency = 1.0;
            diff.coulombFrictionNm = { 0.0, 0.0 };

            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, 40.0, 25.0, false);
            scene.AddComponent(diffEntity, diff);

            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            auto* liveDiff = scene.GetComponent<DifferentialComponent>(diffEntity);
            liveDiff->commandedVoltageA = 6.0;
            liveDiff->commandedVoltageB = -6.0;

            for (int step = 0; step < 100; ++step)
            {
                diffSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            TEST_EXPECT_TRUE(ctx, std::abs(liveDiff->jointPositionsRad.y) > 0.05); // Roll moved
            TEST_EXPECT_NEAR(ctx, liveDiff->jointPositionsRad.x, 0.0, 1e-3);        // Pitch remained stationary
        }

        // -------------------------------------------------------------
        // Test 2: Dual Torque Summation (Virtual Work Verification)
        // -------------------------------------------------------------
        {
            DifferentialComponent diff;
            diff.motorModelA = Actuation::DCMotorModel::KrakenX44();
            diff.motorModelB = Actuation::DCMotorModel::KrakenX44();
            diff.efficiency = 1.0;
            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, 50.0, 30.0, false);

            // At stall under 12V: tau_stall = 4.05 N*m per motor
            double tauStallMotor = diff.motorModelA.GetStallTorque(); // ~4.05 N*m
            glm::dvec2 tauMotors = { tauStallMotor, tauStallMotor };

            // Joint space torques: tau_q = T_mq^T * tau_m
            glm::dvec2 tauJoints = glm::transpose(diff.t_mq) * tauMotors;

            // Pitch stall torque must equal exactly 2 * tauStall * Gp = 2 * 4.05 * 50 = 405 N*m
            double expectedPitchTorque = 2.0 * tauStallMotor * 50.0;
            TEST_EXPECT_NEAR(ctx, tauJoints.x, expectedPitchTorque, 1e-3);
            TEST_EXPECT_NEAR(ctx, tauJoints.y, 0.0, 1e-3); // Roll torque cancels out completely
        }

        // -------------------------------------------------------------
        // Test 3: Semi-Implicit Stability (Instantaneous 24V Reversal)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = Physics::PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            DifferentialMechanismSystem diffSystem(scene, &context);

            Entity base = scene.CreateEntity();
            TransformComponent tb; tb.position = glm::vec3(0, 1, 0); scene.AddComponent(base, tb);
            RigidBodyComponent rbb; rbb.motionType = BodyMotionType::Static; rbb.isSensor = true; scene.AddComponent(base, rbb);
            ColliderComponent cb; cb.shape = ColliderShape::Box; cb.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f); scene.AddComponent(base, cb);

            Entity yoke = scene.CreateEntity();
            TransformComponent ty; ty.position = glm::vec3(0, 1, 0.05f); scene.AddComponent(yoke, ty);
            RigidBodyComponent rby; rby.motionType = BodyMotionType::Dynamic; rby.mass = 1.0f; rby.isSensor = true; scene.AddComponent(yoke, rby);
            ColliderComponent cy; cy.shape = ColliderShape::Box; cy.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f); scene.AddComponent(yoke, cy);

            Entity tool = scene.CreateEntity();
            TransformComponent tt; tt.position = glm::vec3(0, 1, 0.15f); scene.AddComponent(tool, tt);
            RigidBodyComponent rbt; rbt.motionType = BodyMotionType::Dynamic; rbt.mass = 0.5f; rbt.isSensor = true; scene.AddComponent(tool, rbt);
            ColliderComponent ct; ct.shape = ColliderShape::Box; ct.boxHalfExtents = glm::vec3(0.04f, 0.04f, 0.04f); scene.AddComponent(tool, ct);

            Entity pj = scene.CreateEntity();
            JointComponent jcp; jcp.type = JointType::Hinge; jcp.parentEntity = base; jcp.childEntity = yoke;
            jcp.SetWorldSpace(glm::vec3(0, 1, 0), glm::vec3(1, 0, 0)); scene.AddComponent(pj, jcp);

            Entity rj = scene.CreateEntity();
            JointComponent jcr; jcr.type = JointType::Hinge; jcr.parentEntity = yoke; jcr.childEntity = tool;
            jcr.SetWorldSpace(glm::vec3(0, 1, 0.1f), glm::vec3(0, 0, 1)); scene.AddComponent(rj, jcr);

            Entity diffEntity = scene.CreateEntity();
            DifferentialComponent diff;
            diff.jointEntityA = pj; diff.jointEntityB = rj;
            diff.linkEntityA = yoke; diff.linkEntityB = tool;
            diff.motorModelA = Actuation::DCMotorModel::KrakenX60();
            diff.motorModelB = Actuation::DCMotorModel::KrakenX60();
            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, 60.0, 40.0, false);
            scene.AddComponent(diffEntity, diff);

            physicsSystem.FixedUpdate(dt);
            jointSystem.FixedUpdate(dt);

            auto* liveDiff = scene.GetComponent<DifferentialComponent>(diffEntity);

            // Step full throttle forward (+12V)
            liveDiff->commandedVoltageA = 12.0;
            liveDiff->commandedVoltageB = 12.0;
            for (int i = 0; i < 40; ++i)
            {
                diffSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
            }

            // Step full throttle reverse (-12V) instantaneously (24V transient step)
            liveDiff->commandedVoltageA = -12.0;
            liveDiff->commandedVoltageB = -12.0;
            bool noNaNs = true;

            for (int i = 0; i < 100; ++i)
            {
                diffSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);

                if (std::isnan(liveDiff->jointPositionsRad.x) || std::isnan(liveDiff->jointVelocitiesRadS.x))
                {
                    noNaNs = false;
                }
            }

            TEST_EXPECT_TRUE(ctx, noNaNs);
        }

        std::cout << "[PASS] DifferentialMechanismTest\n";
    }
}