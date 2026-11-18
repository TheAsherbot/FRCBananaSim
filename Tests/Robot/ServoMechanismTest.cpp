/**
 * @file ServoMechanismTest.cpp
 * @brief Implementation of unit tests for servo-driven arm pivots, differential wrists, elevators, swerve azimuth, and regulated electrical rail.
 */

#include "ServoMechanismTest.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/DifferentialComponent.h"
#include "Robot/Mechanisms/Systems/MechanismSystem.h"
#include "Robot/Mechanisms/Systems/ElevatorSystem.h"
#include "Robot/Mechanisms/Systems/DifferentialMechanismSystem.h"
#include "Robot/Drivetrain/Model/SwerveModuleModel.h"
#include "Actuation/ServoModel.h"
#include "Actuation/Components/ServoComponent.h"
#include "Actuation/Systems/ElectricalSystem.h"
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
    using namespace Robot::Drivetrain::Model;
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Actuation::Systems;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    void RunServoMechanismTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ServoMechanismTest\n";

        // -------------------------------------------------------------
        // Test 1: Arm Servo Tracking and Gravity Holding
        // -------------------------------------------------------------
        {
            JoltContext context;
            TEST_EXPECT_TRUE(ctx, context.Initialize());

            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime; // 0.005s (200 Hz)
            scene.SetFixedDeltaTime(dt);

            PhysicsSystem physicsSystem(scene, context);
            JointSystem jointSystem(scene, context);
            MechanismSystem mechanismSystem(scene, &context);

            Entity armEntity = scene.CreateEntity();
            TransformComponent transform;
            scene.AddComponent(armEntity, transform);

            constexpr double armMass = 0.4; // kg
            constexpr double armLength = 0.25; // m
            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = static_cast<float>(armMass);
            rb.gravityFactor = 1.0f;
            rb.allowSleeping = false;
            scene.AddComponent(armEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.025f, static_cast<float>(armLength * 0.5), 0.025f);
            scene.AddComponent(armEntity, col);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Hinge;
            joint.parentEntity = invalidEntity;
            joint.childEntity = armEntity;
            joint.SetWorldSpace(
                glm::vec3(0.0f, 1.0f, 0.0f),
                glm::vec3(0.0f, 0.0f, 1.0f)
            );
            joint.hasLimits = true;
            joint.minLimit = -3.14159f;
            joint.maxLimit = 3.14159f;
            scene.AddComponent(jointEntity, joint);

            Entity mechEntity = scene.CreateEntity();
            ArmComponent arm;
            arm.name = "ServoArm";
            arm.actuatorType = ActuatorType::Servo;
            arm.servoModel = ServoModel::GoBildaTorque(ServoMode::Positional);
            arm.pwmChannel = 0;
            arm.linkMassKg = armMass;
            arm.centerOfMassDistanceM = armLength * 0.5;
            arm.jointEntity = jointEntity;
            arm.linkEntity = armEntity;
            arm.minAngleRad = -2.61799;
            arm.maxAngleRad = 2.61799;

            // Command target angle: 0.785 rad (45 degrees)
            constexpr double targetAngle = 0.785398;
            arm.servoModel.SetTargetAngle(targetAngle);
            scene.AddComponent(mechEntity, arm);

            physicsSystem.FixedUpdate(0.0);
            jointSystem.FixedUpdate(0.0);

            // Advance 200 ticks (1.0 second)
            for (int i = 0; i < 200; ++i)
            {
                mechanismSystem.FixedUpdate(dt);
                jointSystem.FixedUpdate(dt);
                physicsSystem.FixedUpdate(dt);
                jointSystem.ReadbackTelemetry();
            }

            auto* jComp = scene.GetComponent<JointComponent>(jointEntity);
            TEST_EXPECT_TRUE(ctx, jComp != nullptr);
            if (jComp)
            {
                // Verify arm converged close to target angle against gravity
                TEST_EXPECT_NEAR(ctx, jComp->currentAngle, targetAngle, 0.08);
                TEST_EXPECT_TRUE(ctx, arm.statorCurrentA >= 0.0);
            }
        }

        // -------------------------------------------------------------
        // Test 2: Differential Wrist Dual-Servo Coupling
        // -------------------------------------------------------------
        {
            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;

            DifferentialMechanismSystem diffSystem(scene, nullptr);

            Entity jointEntityA = scene.CreateEntity();
            JointComponent jointA;
            jointA.type = JointType::Hinge;
            scene.AddComponent(jointEntityA, jointA);

            Entity jointEntityB = scene.CreateEntity();
            JointComponent jointB;
            jointB.type = JointType::Hinge;
            scene.AddComponent(jointEntityB, jointB);

            Entity diffEntity = scene.CreateEntity();
            DifferentialComponent diff;
            diff.name = "ServoWrist";
            diff.actuatorType = ActuatorType::Servo;
            diff.servoModelA = ServoModel::GoBildaTorque(ServoMode::Positional);
            diff.servoModelB = ServoModel::GoBildaTorque(ServoMode::Positional);
            diff.pwmChannelA = 0;
            diff.pwmChannelB = 1;
            diff.jointEntityA = jointEntityA;
            diff.jointEntityB = jointEntityB;
            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, 1.0, 1.0, false);
            diff.effectiveInertia = glm::dmat2(0.02, 0.0, 0.0, 0.02);

            // Pure pitch command: both servos commanded to +0.5 rad
            constexpr double pitchTarget = 0.5;
            diff.servoModelA.SetTargetAngle(pitchTarget);
            diff.servoModelB.SetTargetAngle(pitchTarget);
            scene.AddComponent(diffEntity, diff);

            for (int i = 0; i < 150; ++i)
            {
                diffSystem.FixedUpdate(dt);
                auto* jA = scene.GetComponent<JointComponent>(jointEntityA);
                auto* jB = scene.GetComponent<JointComponent>(jointEntityB);
                if (jA && jB)
                {
                    jA->currentAngle += jA->targetVelocity * static_cast<float>(dt);
                    jB->currentAngle += jB->targetVelocity * static_cast<float>(dt);
                    jA->currentVelocity = jA->targetVelocity;
                    jB->currentVelocity = jB->targetVelocity;
                }
            }

            auto* finalJA = scene.GetComponent<JointComponent>(jointEntityA);
            auto* finalJB = scene.GetComponent<JointComponent>(jointEntityB);
            TEST_EXPECT_TRUE(ctx, finalJA != nullptr && finalJB != nullptr);
            if (finalJA && finalJB)
            {
                // Joint A (pitch) should have moved towards target
                TEST_EXPECT_TRUE(ctx, finalJA->currentAngle > 0.3);
                // Joint B (roll) should remain near zero in symmetric actuation
                TEST_EXPECT_NEAR(ctx, finalJB->currentAngle, 0.0, 0.05);
            }
        }

        // -------------------------------------------------------------
        // Test 3: Elevator Servo Actuation
        // -------------------------------------------------------------
        {
            Scene scene;
            constexpr double dt = PhysicsConfig::fixedDeltaTime;
            ElevatorSystem elevatorSystem(scene, nullptr);

            Entity jointEntity = scene.CreateEntity();
            JointComponent joint;
            joint.type = JointType::Slider;
            joint.currentPosition = 0.0f;
            joint.currentVelocity = 0.0f;
            scene.AddComponent(jointEntity, joint);

            Entity elevEntity = scene.CreateEntity();
            ElevatorComponent elev;
            elev.name = "ServoElevator";
            elev.actuatorType = ActuatorType::Servo;
            elev.servoModel = ServoModel::GoBildaTorque(ServoMode::Positional);
            elev.drumRadiusM = 0.02; // 2 cm
            elev.gearRatio = 1.0;
            elev.efficiency = 0.95;
            elev.riggingMode = ElevatorRiggingMode::SingleStage;

            ElevatorStageInfo stage;
            stage.jointEntity = jointEntity;
            stage.stageMassKg = 0.5;
            stage.maxTravelM = 0.5;
            elev.stages.push_back(stage);

            // Command linear target height: theta = (x / rDrum) * G
            // Target height = 0.05 m -> thetaTarget = 0.05 / 0.02 = 2.5 rad
            elev.servoModel.SetTargetAngle(2.5);
            scene.AddComponent(elevEntity, elev);

            for (int i = 0; i < 100; ++i)
            {
                elevatorSystem.FixedUpdate(dt);
                auto* jc = scene.GetComponent<JointComponent>(jointEntity);
                auto* eComp = scene.GetComponent<ElevatorComponent>(elevEntity);
                if (jc && eComp)
                {
                    double acc = eComp->appliedLinearForceN / eComp->lumpedEffectiveMassKg;
                    jc->currentVelocity += static_cast<float>(acc * dt);
                    jc->currentPosition += static_cast<float>(jc->currentVelocity * dt);
                }
            }

            auto* finalJc = scene.GetComponent<JointComponent>(jointEntity);
            auto* finalEComp = scene.GetComponent<ElevatorComponent>(elevEntity);
            TEST_EXPECT_TRUE(ctx, finalJc != nullptr && finalEComp != nullptr);
            if (finalJc && finalEComp)
            {
                TEST_EXPECT_TRUE(ctx, finalJc->currentPosition > 0.02f);
                TEST_EXPECT_TRUE(ctx, finalEComp->statorCurrentA >= 0.0);
            }
        }

        // -------------------------------------------------------------
        // Test 4: Swerve Module Servo Steer Azimuth Tracking
        // -------------------------------------------------------------
        {
            ModulePlacement placement;
            placement.name = "FrontLeft";
            placement.moduleIndex = 0;
            placement.steerPwmChannel = 0;

            SteerModuleParams steerParams;
            steerParams.actuatorType = ModuleActuatorType::Servo;
            steerParams.servo = ServoModel::GoBildaTorque(ServoMode::Positional);
            steerParams.steerHousingInertia = 0.00036;
            steerParams.coulombFrictionTorque = 0.02;

            DriveModuleParams driveParams;
            TireParams tireParams;

            SwerveModuleModel module(placement, steerParams, driveParams, tireParams);

            Control::MotorCommand steerCmd;
            steerCmd.mode = Control::MotorControlMode::PositionVoltage;
            steerCmd.demand = 1.5707963; // 90 degrees in radians

            constexpr double dt = 0.005;
            for (int i = 0; i < 100; ++i) // 0.5 s
            {
                module.StepSteer(steerCmd, 12.0, 0.0, dt);
            }

            const auto& state = module.GetState();
            TEST_EXPECT_NEAR(ctx, state.steerAngle, 1.5707963, 0.05);
            TEST_EXPECT_NEAR(ctx, state.unwrappedSteerAngle, 1.5707963, 0.05);
            TEST_EXPECT_TRUE(ctx, std::isfinite(state.steerStatorCurrent));
        }

        // -------------------------------------------------------------
        // Test 5: Electrical Regulated Rail Overload and Droop
        // -------------------------------------------------------------
        {
            Scene scene;
            ElectricalSystem electricalSystem(scene, nullptr);
            electricalSystem.SetRegulatedRail(6.0, 2.2, 0.88);

            Entity s1 = scene.CreateEntity();
            ServoComponent sc1;
            sc1.currentDraw = 2.0; // 2.0A
            scene.AddComponent(s1, sc1);

            Entity s2 = scene.CreateEntity();
            ServoComponent sc2;
            sc2.currentDraw = 3.0; // 3.0A -> total 5.0A > 2.2A limit
            scene.AddComponent(s2, sc2);

            electricalSystem.FixedUpdate(0.005);

            TEST_EXPECT_TRUE(ctx, electricalSystem.IsRegulatedRailBrownedOut());
            TEST_EXPECT_NEAR(ctx, electricalSystem.GetRegulatedRailCurrent(), 5.0, 0.01);
            // Expected droop voltage: 6.0 * (2.2 / 5.0) = 2.64V
            double expectedDroop = 6.0 * (2.2 / 5.0);
            TEST_EXPECT_NEAR(ctx, electricalSystem.GetRegulatedRailVoltage(), expectedDroop, 0.05);

            // Recovery test: reduce current load below limit
            auto* sComp1 = scene.GetComponent<ServoComponent>(s1);
            auto* sComp2 = scene.GetComponent<ServoComponent>(s2);
            if (sComp1 && sComp2)
            {
                sComp1->currentDraw = 0.8;
                sComp2->currentDraw = 0.6; // Total 1.4A < 2.2A
            }

            electricalSystem.FixedUpdate(0.005);

            TEST_EXPECT_FALSE(ctx, electricalSystem.IsRegulatedRailBrownedOut());
            TEST_EXPECT_NEAR(ctx, electricalSystem.GetRegulatedRailVoltage(), 6.0, 0.01);
            TEST_EXPECT_NEAR(ctx, electricalSystem.GetRegulatedRailCurrent(), 1.4, 0.01);
        }

        std::cout << "[DONE] ServoMechanismTest\n";
    }
}
