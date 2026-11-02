/**
 * @file ArmFourBarFlywheelTest.cpp
 * @brief Implementation of test suite for arms, four-bar linkages, flywheels, and dynamic composite mass transfer.
 */

#include "ArmFourBarFlywheelTest.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FourBarComponent.h"
#include "Robot/Mechanisms/Components/CouplingComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"
#include "Robot/Mechanisms/Systems/MechanismSystem.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/Mechanisms/Systems/CompositeMassSystem.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Robot/Drivetrain/Model/SwerveParams.h"
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

    void RunArmFourBarFlywheelTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ArmFourBarFlywheelTest\n";

        // -------------------------------------------------------------
        // 1. Arm Gravity Holding Torque & Static Stick (TA1)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            MechanismSystem mechanismSystem(scene, &context);

            constexpr double armMass = 5.0; // kg
            constexpr double cgDistance = 0.40; // m
            constexpr double gRatio = 60.0;
            // Static horizontal gravity torque: tau_g = -m * g * L_cg = -5.0 * 9.80665 * 0.40 = -19.6133 N*m
            constexpr double staticGravityTorque = 19.6133;

            // Arm Link Rigid Body
            Entity armLinkEntity = scene.CreateEntity();
            TransformComponent t;
            t.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(armLinkEntity, t);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = static_cast<float>(armMass);
            rb.gravityFactor = 1.0f;
            rb.allowSleeping = false;
            rb.isSensor = true; // Disable hull collisions
            scene.AddComponent(armLinkEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.4f, 0.05f, 0.05f);
            scene.AddComponent(armLinkEntity, col);

            // Hinge Joint at (0, 1, 0) rotating about World +Z
            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Hinge;
            joint.parentEntity = invalidEntity; // World anchor
            joint.childEntity = armLinkEntity;
            joint.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            joint.hasLimits = false;
            scene.AddComponent(jointEntity, joint);

            // Arm Component with counterbalance spring balancing gravity
            Entity armEntity = scene.CreateEntity();
            ArmComponent arm;
            arm.name = "ShoulderArm";
            arm.jointEntity = jointEntity;
            arm.linkEntity = armLinkEntity;
            arm.motorModel = Actuation::DCMotorModel::KrakenX60(gRatio, 1.0);
            arm.gearRatio = gRatio;
            arm.efficiency = 1.0;
            arm.linkMassKg = armMass;
            arm.centerOfMassDistanceM = cgDistance;
            arm.springType = ArmSpringType::ConstantTorque;
            arm.springPreloadTorqueNm = staticGravityTorque; // Exactly balances horizontal gravity
            arm.coulombFrictionNm = 5.0; // 5 N*m friction threshold
            arm.commandedVoltage = 0.0;  // Neutral holding
            arm.brakeMode = true;
            scene.AddComponent(armEntity, arm);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            // Step for 1.0s (200 ticks)
            for (int i = 0; i < 200; ++i)
            {
                mechanismSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();
            }

            auto* armComp = scene.GetComponent<ArmComponent>(armEntity);
            auto* jointComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, armComp != nullptr);
            TEST_EXPECT_TRUE(ctx, jointComp != nullptr);

            // Static stick must latch and hold with zero drift (< 1e-6 rad)
            TEST_EXPECT_TRUE(ctx, armComp->isStickLocked);
            TEST_EXPECT_TRUE(ctx, std::abs(jointComp->currentAngle) < 1e-6);
            TEST_EXPECT_TRUE(ctx, std::abs(jointComp->currentVelocity) < 1e-6);
        }

        // -------------------------------------------------------------
        // 2. Arm Hard-Stop Impact at High Speed (TA2)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            MechanismSystem mechanismSystem(scene, &context);

            Entity armLink = scene.CreateEntity();
            TransformComponent t;
            t.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(armLink, t);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 4.0f;
            rb.gravityFactor = 0.0f; // Pure impact dynamic test
            rb.allowSleeping = false;
            rb.isSensor = true;
            scene.AddComponent(armLink, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.3f, 0.05f, 0.05f);
            scene.AddComponent(armLink, col);

            // Hinge with limits [-1.0, 1.0 rad] and critically damped spring stops
            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Hinge;
            joint.parentEntity = invalidEntity;
            joint.childEntity = armLink;
            joint.SetWorldSpace(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            joint.hasLimits = true;
            joint.minLimit = -1.0f;
            joint.maxLimit = 1.0f; // 1.0 rad (~57.3 deg)
            joint.hasLimitSpring = true;
            joint.limitSpringFrequency = 40.0f;
            joint.limitSpringDamping = 1.0f; // Critically damped
            scene.AddComponent(jointEntity, joint);

            Entity armEntity = scene.CreateEntity();
            ArmComponent arm;
            arm.name = "SpeedyArm";
            arm.jointEntity = jointEntity;
            arm.linkEntity = armLink;
            arm.motorModel = Actuation::DCMotorModel::KrakenX60(25.0, 1.0);
            arm.gearRatio = 25.0;
            arm.efficiency = 1.0;
            arm.commandedVoltage = 12.0; // Full throttle into hard stop
            arm.brakeMode = true;
            scene.AddComponent(armEntity, arm);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            double peakVelocity = 0.0;
            bool hitStop = false;

            // Step for 1.0s (200 ticks)
            for (int i = 0; i < 200; ++i)
            {
                mechanismSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();

                auto* jc = scene.GetComponent<JointComponent>(jointEntity);
                if (jc)
                {
                    if (jc->currentVelocity > peakVelocity)
                    {
                        peakVelocity = jc->currentVelocity;
                    }
                    if (jc->currentAngle >= 0.95f)
                    {
                        hitStop = true;
                    }

                    TEST_EXPECT_FALSE(ctx, std::isnan(jc->currentAngle));
                    TEST_EXPECT_FALSE(ctx, std::isnan(jc->currentVelocity));
                }
            }

            auto* finalJoint = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, finalJoint != nullptr);

            // Reached high speed before impact
            TEST_EXPECT_TRUE(ctx, peakVelocity >= 5.0);
            TEST_EXPECT_TRUE(ctx, hitStop);

            // Settle smoothly near 1.0 rad without exploding or excessive over-penetration (< 1.05 rad)
            TEST_EXPECT_TRUE(ctx, finalJoint->currentAngle <= 1.05f);
            TEST_EXPECT_TRUE(ctx, finalJoint->currentAngle >= 0.98f);
            TEST_EXPECT_TRUE(ctx, std::abs(finalJoint->currentVelocity) < 1.0);
        }

        // -------------------------------------------------------------
        // 3. Four-Bar Linkage Under 15 kg Load (TA3)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            MechanismSystem mechanismSystem(scene, &context);

            // Parallelogram 4-bar:
            // Base pivots: A0 = (0, 0.5, 0.0), B0 = (0, 0.5, 0.40)
            // Crank (L1, 0.30m): A0 -> A1 = (0, 0.8, 0.0)
            // Follower (L3, 0.30m): B0 -> B1 = (0, 0.8, 0.40)
            // Coupler (L2, 0.40m): A1 -> B1 (mass 15.0 kg!)
            // Shared rotation axis: World +X (1, 0, 0)

            // 1. Crank L1 (Dynamic, 2 kg)
            Entity crankLink = scene.CreateEntity();
            TransformComponent tCrank;
            tCrank.position = glm::vec3(0.0f, 0.65f, 0.0f);
            scene.AddComponent(crankLink, tCrank);

            RigidBodyComponent rbCrank;
            rbCrank.motionType = BodyMotionType::Dynamic;
            rbCrank.mass = 2.0f;
            rbCrank.isSensor = true;
            scene.AddComponent(crankLink, rbCrank);

            ColliderComponent colCrank;
            colCrank.shape = ColliderShape::Box;
            colCrank.boxHalfExtents = glm::vec3(0.05f, 0.15f, 0.05f);
            scene.AddComponent(crankLink, colCrank);

            // 2. Follower L3 (Dynamic, 2 kg)
            Entity followerLink = scene.CreateEntity();
            TransformComponent tFollower;
            tFollower.position = glm::vec3(0.0f, 0.65f, 0.40f);
            scene.AddComponent(followerLink, tFollower);

            RigidBodyComponent rbFollower;
            rbFollower.motionType = BodyMotionType::Dynamic;
            rbFollower.mass = 2.0f;
            rbFollower.isSensor = true;
            scene.AddComponent(followerLink, rbFollower);

            ColliderComponent colFollower;
            colFollower.shape = ColliderShape::Box;
            colFollower.boxHalfExtents = glm::vec3(0.05f, 0.15f, 0.05f);
            scene.AddComponent(followerLink, colFollower);

            // 3. Coupler L2 (Dynamic, 15 kg heavy end-effector)
            Entity couplerLink = scene.CreateEntity();
            TransformComponent tCoupler;
            tCoupler.position = glm::vec3(0.0f, 0.80f, 0.20f);
            scene.AddComponent(couplerLink, tCoupler);

            RigidBodyComponent rbCoupler;
            rbCoupler.motionType = BodyMotionType::Dynamic;
            rbCoupler.mass = 15.0f; // 15 kg heavy payload!
            rbCoupler.gravityFactor = 1.0f;
            rbCoupler.isSensor = true;
            scene.AddComponent(couplerLink, rbCoupler);

            ColliderComponent colCoupler;
            colCoupler.shape = ColliderShape::Box;
            colCoupler.boxHalfExtents = glm::vec3(0.05f, 0.05f, 0.20f);
            scene.AddComponent(couplerLink, colCoupler);

            // Joint J01: World -> Crank at A0 (0, 0.5, 0)
            Entity j01 = scene.CreateEntity();
            JointComponent joint01;
            joint01.type = JointType::Hinge;
            joint01.parentEntity = invalidEntity;
            joint01.childEntity = crankLink;
            joint01.SetWorldSpace(glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            joint01.motorMode = JointMotorMode::Velocity;
            joint01.targetVelocity = 1.5f; // Drive crank at 1.5 rad/s
            joint01.maxMotorTorque = 15000.0f; // High torque to lift 15 kg
            scene.AddComponent(j01, joint01);

            // Joint J12: Crank -> Coupler at A1 (0, 0.8, 0)
            Entity j12 = scene.CreateEntity();
            JointComponent joint12;
            joint12.type = JointType::Hinge;
            joint12.parentEntity = crankLink;
            joint12.childEntity = couplerLink;
            joint12.SetWorldSpace(glm::vec3(0.0f, 0.8f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            scene.AddComponent(j12, joint12);

            // Joint J03: World -> Follower at B0 (0, 0.5, 0.40)
            Entity j03 = scene.CreateEntity();
            JointComponent joint03;
            joint03.type = JointType::Hinge;
            joint03.parentEntity = invalidEntity;
            joint03.childEntity = followerLink;
            joint03.SetWorldSpace(glm::vec3(0.0f, 0.5f, 0.40f), glm::vec3(1.0f, 0.0f, 0.0f));
            scene.AddComponent(j03, joint03);

            // Joint J32: Follower -> Coupler at B1 (0, 0.8, 0.40) - Loop Closer
            Entity j32 = scene.CreateEntity();
            JointComponent joint32;
            joint32.type = JointType::Hinge;
            joint32.parentEntity = followerLink;
            joint32.childEntity = couplerLink;
            joint32.SetWorldSpace(glm::vec3(0.0f, 0.8f, 0.40f), glm::vec3(1.0f, 0.0f, 0.0f));
            scene.AddComponent(j32, joint32);

            // FourBarComponent
            Entity fourBarEntity = scene.CreateEntity();
            FourBarComponent fourBar;
            fourBar.name = "IntakeFourBar";
            fourBar.crankEntity = crankLink;
            fourBar.followerEntity = followerLink;
            fourBar.couplerEntity = couplerLink;
            fourBar.crankJoint = j01;
            fourBar.couplerJoint = j12;
            fourBar.followerJoint = j03;
            fourBar.closureJoint = j32;
            fourBar.closureParentAnchor = glm::vec3(0.0f, 0.15f, 0.0f); // Top of follower (B1 relative to COM)
            fourBar.closureChildAnchor = glm::vec3(0.0f, 0.0f, 0.20f);  // Rear of coupler (B1 relative to COM)
            scene.AddComponent(fourBarEntity, fourBar);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            double maxLoopDrift = 0.0;

            // Step for 1.0s (200 ticks) through 1.5 rad of rotation
            for (int i = 0; i < 200; ++i)
            {
                mechanismSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();

                auto* fb = scene.GetComponent<FourBarComponent>(fourBarEntity);
                if (fb && fb->loopClosureErrorM > maxLoopDrift)
                {
                    maxLoopDrift = fb->loopClosureErrorM;
                }
            }

            // Loop closure error must remain strictly under 1.5 mm under 15 kg load
            TEST_EXPECT_TRUE(ctx, maxLoopDrift < 0.0015);
        }

        // -------------------------------------------------------------
        // 4. Flywheel Spin-Up Time Constant tau = J / D (TA4)
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            FlywheelSystem flywheelSystem(scene, &context);

            // 2 Kraken X60 motors on 1:1 direct drive shooter
            constexpr uint32_t motorCount = 2;
            constexpr double jDisc = 0.003; // kg*m^2
            Actuation::DCMotorModel kraken = Actuation::DCMotorModel::KrakenX60(1.0, 1.0);
            double jRefl = motorCount * kraken.GetRotorInertia() * (1.0 * 1.0);
            double jTotal = jDisc + jRefl; // ~0.003112 kg*m^2

            // Theoretical time constant and terminal velocity
            Actuation::DCMotorModel multiKraken = kraken.WithMotorCount(motorCount);
            auto affine = multiKraken.CalculateAffineTorqueTerms(12.0);
            double tauCmd = affine.stallTorqueCommanded;
            double dRot = affine.electricalDamping;
            double omegaTerm = tauCmd / dRot;
            double tauTime = jTotal / dRot; // ~0.1386 s

            Entity flywheelEntity = scene.CreateEntity();
            FlywheelComponent flywheel;
            flywheel.name = "DualShooter";
            flywheel.motorModel = kraken;
            flywheel.motorCount = motorCount;
            flywheel.gearRatio = 1.0;
            flywheel.efficiency = 1.0;
            flywheel.flywheelInertiaKgm2 = jDisc;
            flywheel.coulombFrictionNm = 0.0; // Pure theoretical ODE comparison
            flywheel.viscousDampingNms = 0.0;
            flywheel.commandedVoltage = 12.0; // Step input
            flywheel.brakeMode = false;
            flywheel.subSteps = 4; // 1.25 ms sub-steps
            flywheel.statorCurrentLimitA = 1000.0; // Unconstrained for pure linear ODE benchmark
            scene.AddComponent(flywheelEntity, flywheel);

            // Simulate to t = 0.10s (20 ticks at 200 Hz)
            constexpr int ticks = 20;
            for (int i = 0; i < ticks; ++i)
            {
                flywheelSystem.FixedUpdate(dt);
            }

            auto* fwComp = scene.GetComponent<FlywheelComponent>(flywheelEntity);
            TEST_EXPECT_TRUE(ctx, fwComp != nullptr);

            double tFinal = ticks * dt; // 0.10 s
            double theoreticalOmega = omegaTerm * (1.0 - std::exp(-tFinal / tauTime));
            double simulatedOmega = fwComp->angularVelocityRadPerSec;

            double errorPct = (std::abs(simulatedOmega - theoreticalOmega) / theoreticalOmega) * 100.0;
            // Simulated ODE matches theoretical within 1.0%
            TEST_EXPECT_TRUE(ctx, errorPct < 1.0);
            // Reaches high RPM
            TEST_EXPECT_TRUE(ctx, fwComp->angularVelocityRpm > 2500.0);
        }

        // -------------------------------------------------------------
        // 5. Dynamic Composite Mass & CG Load Transfer (TA5)
        // -------------------------------------------------------------
        {
            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            scene.SetFixedDeltaTime(dt);

            // Drivetrain configuration
            Robot::Drivetrain::Model::SwerveDrivetrainConfig config;
            config.chassis.mass = 45.0; // Base chassis
            config.chassis.centerOfMassOffset = glm::vec3(0.0f, 0.0f, 0.20f);
            Robot::Drivetrain::Model::SwerveDrivetrainModel swerveModel(config);

            CompositeMassSystem massSystem(scene, &swerveModel, nullptr);

            // 1. Chassis Entity (45 kg at origin)
            Entity chassis = scene.CreateEntity();
            TransformComponent tChassis;
            tChassis.position = glm::vec3(0.0f, 0.20f, 0.0f); // Engine frame
            scene.AddComponent(chassis, tChassis);

            RigidBodyComponent rbChassis;
            rbChassis.mass = 45.0f;
            scene.AddComponent(chassis, rbChassis);

            // 2. Articulated Mechanism Link (15 kg at 0.50m height)
            Entity armLink = scene.CreateEntity();
            TransformComponent tArm;
            tArm.position = glm::vec3(0.0f, 0.50f, 0.0f); // Centered horizontally
            scene.AddComponent(armLink, tArm);

            RigidBodyComponent rbArm;
            rbArm.mass = 15.0f;
            scene.AddComponent(armLink, rbArm);

            // CompositeMassComponent
            Entity compositeEntity = scene.CreateEntity();
            CompositeMassComponent comp;
            comp.chassisEntity = chassis;
            comp.mechanismLinks.push_back(armLink);
            scene.AddComponent(compositeEntity, comp);

            // Evaluate initial centered pose
            massSystem.FixedUpdate(dt);

            auto* compOut = scene.GetComponent<CompositeMassComponent>(compositeEntity);
            TEST_EXPECT_TRUE(ctx, compOut != nullptr);
            TEST_EXPECT_NEAR(ctx, compOut->totalMassKg, 60.0, 1e-4); // 45 + 15 = 60 kg

            // Initial centered CG relative to chassis origin:
            // (45 * 0.0 + 15 * (0.50 - 0.20)) / 60 = +0.075 m in Engine +Y (NWU +Z)
            TEST_EXPECT_NEAR(ctx, compOut->compositeCenterOfMassEngine.y, 0.075, 1e-3);
            TEST_EXPECT_NEAR(ctx, compOut->compositeCenterOfMassNWU.z, 0.075, 1e-3);
            TEST_EXPECT_NEAR(ctx, compOut->compositeCenterOfMassNWU.x, 0.0, 1e-3);

            // Now articulate the arm link forward by +0.40m (Engine +Z = NWU +X Forward)
            auto* armTransform = scene.GetComponent<TransformComponent>(armLink);
            armTransform->position = glm::vec3(0.0f, 0.50f, 0.40f);

            massSystem.FixedUpdate(dt);

            // Composite CG shifts forward by (15 * 0.40) / 60 = +0.10 m
            TEST_EXPECT_NEAR(ctx, compOut->compositeCenterOfMassEngine.z, 0.10, 1e-3);
            TEST_EXPECT_NEAR(ctx, compOut->compositeCenterOfMassNWU.x, 0.10, 1e-3); // NWU +X is Forward!
        }
    }
}