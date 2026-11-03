/**
 * @file ElevatorMechanismTest.cpp
 * @brief Implementation of test suite for elevator prismatic joints, semi-implicit step response, cascade tracking, and zero-creep stick.
 */

#include "ElevatorMechanismTest.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Systems/ElevatorSystem.h"
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
    using namespace Robot::Mechanisms::Components;
    using namespace Robot::Mechanisms::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    void RunElevatorMechanismTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ElevatorMechanismTest\n";

        // -------------------------------------------------------------
        // 1. Single-Stage Step Response vs Analytical ODE (TE1)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime; // 0.005s = 200 Hz
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            ElevatorSystem elevatorSystem(scene, &context);

            constexpr double gRatio = 9.0;
            constexpr double rDrum = 0.01905; // 0.75 in radius
            constexpr double stageMass = 4.0; // kg

            Actuation::DCMotorModel kraken = Actuation::DCMotorModel::KrakenX60(gRatio, 1.0);
            double mReflected = kraken.GetRotorInertia() * (gRatio / rDrum) * (gRatio / rDrum);
            double mEffective = mReflected + stageMass;

            Entity stageEntity = scene.CreateEntity();
            TransformComponent transform;
            scene.AddComponent(stageEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = static_cast<float>(stageMass);
            rb.gravityFactor = 1.0f;
            rb.allowSleeping = false;
            scene.AddComponent(stageEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.15f, 0.15f, 0.15f);
            scene.AddComponent(stageEntity, col);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Slider;
            joint.parentEntity = invalidEntity;
            joint.childEntity = stageEntity;
            joint.SetWorldSpaceSlider(
                glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                3.0f
            );
            scene.AddComponent(jointEntity, joint);

            // Elevator Component
            Entity elevatorEntity = scene.CreateEntity();
            ElevatorComponent elevator;
            elevator.name = "SingleStageElevator";
            elevator.riggingMode = ElevatorRiggingMode::SingleStage;
            elevator.motorModel = kraken;
            elevator.motorCount = 1;
            elevator.gearRatio = gRatio;
            elevator.efficiency = 1.0;
            elevator.drumRadiusM = rDrum;
            elevator.commandedVoltage = 12.0; // 12V step input
            elevator.brakeMode = true;

            ElevatorStageInfo stageInfo;
            stageInfo.stageEntity = stageEntity;
            stageInfo.jointEntity = jointEntity;
            stageInfo.stageMassKg = stageMass;
            stageInfo.maxTravelM = 3.0;
            elevator.stages.push_back(stageInfo);
            scene.AddComponent(elevatorEntity, elevator);

            // Warm up allocation pass
            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            // Analytical ODE Parameters
            auto affine = kraken.CalculateAffineTorqueTerms(12.0);
            double fCmd = affine.stallTorqueCommanded / rDrum;
            double dLin = affine.electricalDamping / (rDrum * rDrum);
            double fGrav = stageMass * 9.80665;
            double vTerm = (fCmd - fGrav) / dLin;
            double tau = mEffective / dLin;

            // Step for 0.20s (40 ticks at 200 Hz)
            constexpr int stepCount = 40;
            for (int i = 0; i < stepCount; ++i)
            {
                elevatorSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();
            }

            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);

            double simulatedVel = jointComp->currentVelocity;
            double simulatedPos = jointComp->currentPosition;
            double tFinal = stepCount * dt;
            double analyticalVel = vTerm * (1.0 - std::exp(-tFinal / tau));

            double velocityErrorPct = (std::abs(simulatedVel - analyticalVel) / analyticalVel) * 100.0;
            TEST_EXPECT_TRUE(ctx, velocityErrorPct < 0.5);
            TEST_EXPECT_TRUE(ctx, simulatedPos > 0.1);

            // Verify stator current decreases as back-EMF opposes armature voltage
            auto* elevComp = scene.GetComponent<ElevatorComponent>(elevatorEntity);
            TEST_EXPECT_TRUE(ctx, elevComp != nullptr);
            TEST_EXPECT_TRUE(ctx, elevComp->statorCurrentA < kraken.GetStallCurrent());
        }

        // -------------------------------------------------------------
        // 2. 2-Stage Cascade Tracking & Hard Stop Test (TE2)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            ElevatorSystem elevatorSystem(scene, &context);

            constexpr double gRatio = 9.0;
            constexpr double rDrum = 0.01905;
            constexpr double m1 = 2.0;
            constexpr double m2 = 2.0;

            Actuation::DCMotorModel motor = Actuation::DCMotorModel::KrakenX60(gRatio, 1.0);
            double mReflected = motor.GetRotorInertia() * (gRatio / rDrum) * (gRatio / rDrum);
            double mEffCascade = mReflected + (1.0 * 1.0 * m1) + (2.0 * 2.0 * m2);
            double gravCascade = (1.0 * m1) + (2.0 * m2);

            // Stage 1 (Intermediate stage carrying cascade lumped effective inertia)
            Entity stage1Entity = scene.CreateEntity();
            TransformComponent t1;
            t1.position = glm::vec3(0.0f, 0.0f, 0.0f);
            scene.AddComponent(stage1Entity, t1);

            RigidBodyComponent rb1;
            rb1.motionType = BodyMotionType::Dynamic;
            rb1.mass = static_cast<float>(mEffCascade);
            rb1.gravityFactor = static_cast<float>(gravCascade / mEffCascade);
            rb1.allowSleeping = false;
            rb1.isSensor = true; // Stages slide inside each other without contact hulls
            scene.AddComponent(stage1Entity, rb1);

            // Slider 1: World to Stage 1 with [0.0, 0.60] meter limits
            Entity joint1Entity = scene.CreateEntity();
            JointComponent joint1;
            joint1.type = JointType::Slider;
            joint1.parentEntity = invalidEntity;
            joint1.childEntity = stage1Entity;
            joint1.SetWorldSpaceSlider(
                glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                0.60f
            );
            scene.AddComponent(joint1Entity, joint1);

            // Stage 2 (Carriage link slave)
            Entity stage2Entity = scene.CreateEntity();
            TransformComponent t2;
            t2.position = glm::vec3(0.0f, 0.1f, 0.0f);
            scene.AddComponent(stage2Entity, t2);

            RigidBodyComponent rb2;
            rb2.motionType = BodyMotionType::Dynamic;
            rb2.mass = 0.1f; // Slave carriage whose physical inertia is represented inside Stage 1
            rb2.gravityFactor = 0.0f;
            rb2.allowSleeping = false;
            rb2.isSensor = true; // Stages slide inside each other without contact hulls
            scene.AddComponent(stage2Entity, rb2);

            // Slider 2: Stage 1 to Stage 2 with [0.0, 0.60] meter limits
            Entity joint2Entity = scene.CreateEntity();
            JointComponent joint2;
            joint2.type = JointType::Slider;
            joint2.parentEntity = stage1Entity;
            joint2.childEntity = stage2Entity;
            joint2.SetWorldSpaceSlider(
                glm::vec3(0.0f, 0.1f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                0.60f
            );
            scene.AddComponent(joint2Entity, joint2);

            // Elevator Component
            Entity elevatorEntity = scene.CreateEntity();
            ElevatorComponent elevator;
            elevator.name = "CascadeElevator";
            elevator.riggingMode = ElevatorRiggingMode::Cascade;
            elevator.motorModel = motor;
            elevator.motorCount = 1;
            elevator.gearRatio = gRatio;
            elevator.efficiency = 1.0;
            elevator.drumRadiusM = rDrum;
            elevator.commandedVoltage = 12.0; // Drive upward into stop
            elevator.maxHeightM = 1.20;

            ElevatorStageInfo s1;
            s1.stageEntity = stage1Entity;
            s1.jointEntity = joint1Entity;
            s1.stageMassKg = m1;
            s1.maxTravelM = 0.60;
            elevator.stages.push_back(s1);

            ElevatorStageInfo s2;
            s2.stageEntity = stage2Entity;
            s2.jointEntity = joint2Entity;
            s2.stageMassKg = m2;
            s2.maxTravelM = 0.60;
            elevator.stages.push_back(s2);
            scene.AddComponent(elevatorEntity, elevator);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            double maxTrackingError = 0.0;
            bool hitStop = false;

            // Step for 1.0s (200 ticks)
            for (int i = 0; i < 200; ++i)
            {
                elevatorSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();

                auto* j1 = scene.GetComponent<JointComponent>(joint1Entity);
                auto* j2 = scene.GetComponent<JointComponent>(joint2Entity);

                if (j1 && j2)
                {
                    double x1 = j1->currentPosition;
                    double x2 = j2->currentPosition;

                    if (x1 > 0.05 && x1 < 0.50)
                    {
                        double trackingError = std::abs(x2 - x1);
                        if (trackingError > maxTrackingError)
                        {
                            maxTrackingError = trackingError;
                        }
                    }

                    if (x1 >= 0.58)
                    {
                        hitStop = true;
                    }

                    TEST_EXPECT_FALSE(ctx, std::isnan(x1));
                    TEST_EXPECT_FALSE(ctx, std::isnan(x2));
                    TEST_EXPECT_FALSE(ctx, std::isnan(j1->currentVelocity));
                    TEST_EXPECT_FALSE(ctx, std::isnan(j2->currentVelocity));
                }
            }

            // Verify Stage 2 displacement tracks Stage 1 displacement within < 0.5 mm
            TEST_EXPECT_TRUE(ctx, maxTrackingError < 0.0005);
            TEST_EXPECT_TRUE(ctx, hitStop);

            // Verify total carriage height reached ~1.20 m
            auto* elevComp = scene.GetComponent<ElevatorComponent>(elevatorEntity);
            TEST_EXPECT_TRUE(ctx, elevComp != nullptr);
            TEST_EXPECT_NEAR(ctx, elevComp->totalCarriageHeightM, 1.20, 0.05);
        }

        // -------------------------------------------------------------
        // 3. 2-Stage Continuous Rigging Sequential Lift Test (TE3)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            ElevatorSystem elevatorSystem(scene, &context);

            // Stage 1
            Entity stage1Entity = scene.CreateEntity();
            TransformComponent t1;
            scene.AddComponent(stage1Entity, t1);

            RigidBodyComponent rb1;
            rb1.motionType = BodyMotionType::Dynamic;
            rb1.mass = 3.0f;
            rb1.gravityFactor = 1.0f;
            rb1.allowSleeping = false;
            rb1.isSensor = true; // Stages slide inside each other without contact hulls
            scene.AddComponent(stage1Entity, rb1);

            Entity joint1Entity = scene.CreateEntity();
            JointComponent joint1;
            joint1.type = JointType::Slider;
            joint1.parentEntity = invalidEntity;
            joint1.childEntity = stage1Entity;
            joint1.SetWorldSpaceSlider(
                glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                0.40f
            );
            scene.AddComponent(joint1Entity, joint1);

            // Stage 2
            Entity stage2Entity = scene.CreateEntity();
            TransformComponent t2;
            t2.position = glm::vec3(0.0f, 0.1f, 0.0f);
            scene.AddComponent(stage2Entity, t2);

            RigidBodyComponent rb2;
            rb2.motionType = BodyMotionType::Dynamic;
            rb2.mass = 3.0f;
            rb2.gravityFactor = 1.0f;
            rb2.allowSleeping = false;
            rb2.isSensor = true; // Stages slide inside each other without contact hulls
            scene.AddComponent(stage2Entity, rb2);

            Entity joint2Entity = scene.CreateEntity();
            JointComponent joint2;
            joint2.type = JointType::Slider;
            joint2.parentEntity = stage1Entity;
            joint2.childEntity = stage2Entity;
            joint2.SetWorldSpaceSlider(
                glm::vec3(0.0f, 0.1f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                0.40f
            );
            scene.AddComponent(joint2Entity, joint2);

            // Elevator
            Entity elevatorEntity = scene.CreateEntity();
            ElevatorComponent elevator;
            elevator.name = "ContinuousElevator";
            elevator.riggingMode = ElevatorRiggingMode::Continuous;
            elevator.drumRadiusM = 0.01905;
            elevator.gearRatio = 9.0;
            elevator.efficiency = 1.0;
            elevator.motorModel = Actuation::DCMotorModel::KrakenX60(9.0, 1.0);
            elevator.commandedVoltage = 6.0;
            elevator.statorCurrentLimitA = 60.0;

            ElevatorStageInfo s1;
            s1.stageEntity = stage1Entity;
            s1.jointEntity = joint1Entity;
            s1.stageMassKg = 3.0;
            s1.maxTravelM = 0.40;
            elevator.stages.push_back(s1);

            ElevatorStageInfo s2;
            s2.stageEntity = stage2Entity;
            s2.jointEntity = joint2Entity;
            s2.stageMassKg = 3.0;
            s2.maxTravelM = 0.40;
            elevator.stages.push_back(s2);
            scene.AddComponent(elevatorEntity, elevator);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            double stage2MaxLiftWhileStage1Active = 0.0;
            bool stage1ReachedLimit = false;

            // Step for 1.0s (200 ticks)
            for (int i = 0; i < 200; ++i)
            {
                elevatorSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();

                auto* j1 = scene.GetComponent<JointComponent>(joint1Entity);
                auto* j2 = scene.GetComponent<JointComponent>(joint2Entity);

                if (j1 && j2)
                {
                    double x1 = j1->currentPosition;
                    double x2 = j2->currentPosition;

                    if (!stage1ReachedLimit)
                    {
                        if (x1 < 0.38)
                        {
                            if (x2 > stage2MaxLiftWhileStage1Active)
                            {
                                stage2MaxLiftWhileStage1Active = x2;
                            }
                        }
                        else
                        {
                            stage1ReachedLimit = true;
                        }
                    }
                }
            }

            // Verify Stage 1 extends fully before Stage 2 lifts > 2 mm
            TEST_EXPECT_TRUE(ctx, stage1ReachedLimit);
            TEST_EXPECT_TRUE(ctx, stage2MaxLiftWhileStage1Active < 0.002);

            // Verify Stage 2 started lifting sequentially once Stage 1 hit limit
            auto* j2Final = scene.GetComponent<JointComponent>(joint2Entity);
            TEST_EXPECT_TRUE(ctx, j2Final != nullptr);
            TEST_EXPECT_TRUE(ctx, j2Final->currentPosition > 0.05);
        }

        // -------------------------------------------------------------
        // 4. Counterbalance Spring & Zero-Creep Stick Test (TE4)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            ElevatorSystem elevatorSystem(scene, &context);

            constexpr double carriageMass = 6.0; // kg
            constexpr double springPreload = 58.8399; // N (exact counterbalance for 6 kg * 9.80665)
            constexpr double coulombFriction = 15.0;  // N
            constexpr double initialHeight = 0.50;    // m

            Entity stageEntity = scene.CreateEntity();
            TransformComponent transform;
            transform.position = glm::vec3(0.0f, static_cast<float>(initialHeight), 0.0f);
            scene.AddComponent(stageEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = static_cast<float>(carriageMass);
            rb.gravityFactor = 1.0f;
            rb.allowSleeping = false;
            scene.AddComponent(stageEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.15f, 0.15f, 0.15f);
            scene.AddComponent(stageEntity, col);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Slider;
            joint.parentEntity = invalidEntity;
            joint.childEntity = stageEntity;
            joint.SetLocalSpaceSlider(
                glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                0.0f,
                1.5f
            );
            scene.AddComponent(jointEntity, joint);

            Entity elevatorEntity = scene.CreateEntity();
            ElevatorComponent elevator;
            elevator.name = "CounterbalancedElevator";
            elevator.riggingMode = ElevatorRiggingMode::SingleStage;
            elevator.motorModel = Actuation::DCMotorModel::KrakenX60(9.0, 1.0);
            elevator.motorCount = 1;
            elevator.gearRatio = 9.0;
            elevator.efficiency = 1.0;
            elevator.drumRadiusM = 0.01905;
            elevator.springPreloadForceN = springPreload;
            elevator.coulombFrictionN = coulombFriction;
            elevator.commandedVoltage = 0.0; // 0V neutral holding
            elevator.brakeMode = true;

            ElevatorStageInfo stageInfo;
            stageInfo.stageEntity = stageEntity;
            stageInfo.jointEntity = jointEntity;
            stageInfo.stageMassKg = carriageMass;
            stageInfo.maxTravelM = 1.5;
            stageInfo.currentPositionM = initialHeight;
            elevator.stages.push_back(stageInfo);
            scene.AddComponent(elevatorEntity, elevator);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            // Step for 6,000 ticks at 200 Hz = 30.0 seconds of simulation
            constexpr int stickTicks = 6000;
            for (int i = 0; i < stickTicks; ++i)
            {
                elevatorSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();
            }

            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);

            double finalPos = jointComp->currentPosition;
            double finalVel = jointComp->currentVelocity;
            double creep = std::abs(finalPos - initialHeight);

            // Assert creep over 30s is under 1 micrometer
            TEST_EXPECT_TRUE(ctx, creep < 1e-6);
            TEST_EXPECT_TRUE(ctx, std::abs(finalVel) < 1e-6);
        }
    }
}