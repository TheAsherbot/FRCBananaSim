/**
 * @file PipelineIntegrationTest.cpp
 * @brief Implementation of unit and integration tests for Block 6 Day 5 Pipeline Integration & Beam-Break Synchronization.
 */

#include "PipelineIntegrationTest.h"
#include "Interaction/IntakeSurfaceModel.h"
#include "Interaction/ShooterSqueezeModel.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Systems/FlightAerodynamicsSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Systems/SensorSystem.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/RobotAssembler.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"

#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Interaction;
    using namespace Interaction::Systems;
    using namespace Interaction::Components;
    using namespace Robot;
    using namespace Robot::Sensors::Components;
    using namespace Robot::Sensors::Systems;
    using namespace Robot::Mechanisms::Components;
    using namespace Robot::Mechanisms::Systems;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;

    void RunPipelineIntegrationTest(TestContext& ctx)
    {
        std::cout << "[RUN ] PipelineIntegrationTest (Block 6 Day 5)\n";
        constexpr double dt = 0.005; // 200 Hz

        // =========================================================================
        // 1. Stage 3c, 3d, 4b and 6b Pipeline Registration & Ordering (PI1)
        // =========================================================================
        {
            std::cout << "  -> PI1: Pipeline Stage Execution & Order Determinism\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);
            robot.flywheelSystem = std::make_unique<FlywheelSystem>(scene);
            robot.flightAerodynamicsSystem = std::make_unique<FlightAerodynamicsSystem>(scene, nullptr);

            // Execute 20 pipeline steps on an empty scene: verifies zero null pointer dereferences,
            // clean stage transitions, and deterministic stepping at 200 Hz.
            for (int step = 0; step < 20; ++step)
            {
                RobotAssembler::StepPipeline(robot, dt);
            }

            TEST_EXPECT_EQ(ctx, robot.sensorSystem->GetOutputs().digitalInputs, 0u);
        }

        // =========================================================================
        // 2. Intake-to-Storage Capture & ContactZone Latching (PI2)
        // =========================================================================
        {
            std::cout << "  -> PI2: Intake Compliant Drag, Kinematic Capture & Stored Latching\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.flywheelSystem = std::make_unique<FlywheelSystem>(scene);
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);

            // 1. Chassis Root Link
            Entity chassis = scene.CreateEntity();
            {
                TransformComponent tc;
                tc.position = glm::vec3(0.0f, 0.20f, 0.0f);
                scene.AddComponent(chassis, tc);

                RigidBodyComponent rb;
                rb.motionType = BodyMotionType::Dynamic;
                rb.mass = 50.0f;
                scene.AddComponent(chassis, rb);
            }
            robot.chassisEntity = chassis;
            robot.rootLinkEntity = chassis;

            // 2. Intake Roller Flywheel Mechanism
            Entity roller = scene.CreateEntity();
            {
                FlywheelComponent fc;
                fc.name = "intake_roller";
                fc.spinAxis = { 1.0f, 0.0f, 0.0f };
                fc.rollerRadiusM = 0.0508;
                fc.flywheelInertiaKgm2 = 0.002;
                fc.motorCount = 1;
                fc.motorModel = Actuation::DCMotorModel::KrakenX60(1);
                fc.commandedVoltage = 10.0;
                scene.AddComponent(roller, fc);
            }
            robot.mechanismEntities["intake_roller"] = roller;

            // 3. Intake Contact Zone
            Entity zoneEntity = scene.CreateEntity();
            {
                ContactZoneComponent cz;
                cz.name = "intake_zone";
                cz.role = "intake";
                cz.parentLink = chassis;
                cz.localOrigin = { 0.0f, 0.05f, 0.45f };
                cz.dimensions = { 0.50f, 0.20f, 0.35f };
                cz.tangentAxis = { 0.0f, 0.0f, 1.0f }; // Pulls inward along +Z
                cz.compressionAxis = { 0.0f, 1.0f, 0.0f };
                cz.nominalGapM = 0.050f;
                cz.isEngaged = false;
                scene.AddComponent(zoneEntity, cz);
            }
            robot.contactZoneEntities["intake_zone"] = zoneEntity;

            // Spin up intake roller over 15 steps
            for (int i = 0; i < 15; ++i)
            {
                RobotAssembler::StepPipeline(robot, dt);
            }

            auto* fc = scene.GetComponent<FlywheelComponent>(roller);
            TEST_EXPECT_TRUE(ctx, fc->angularVelocityRadPerSec > 40.0);

            // 4. Spawn Game Piece in FREE state entering intake zone
            Entity piece = scene.CreateEntity();
            {
                TransformComponent tc;
                tc.position = glm::vec3(0.0f, 0.25f, 0.35f); // Overlaps intake zone
                scene.AddComponent(piece, tc);

                GamePieceComponent gp;
                gp.state = GamePieceState::Free;
                gp.spec.massKg = 0.235;
                gp.spec.outerRadiusM = 0.1778;
                gp.spec.surfaceFriction = 0.70;
                scene.AddComponent(piece, gp);

                RigidBodyComponent rb;
                rb.motionType = BodyMotionType::Dynamic;
                rb.mass = 0.235f;
                scene.AddComponent(piece, rb);
            }

            // Step 1: Detects overlap -> Transitions to INTAKING
            RobotAssembler::StepPipeline(robot, dt);
            auto* gp = scene.GetComponent<GamePieceComponent>(piece);
            TEST_EXPECT_EQ(ctx, gp->state, GamePieceState::Intaking);

            // Step 2: In Intaking, traction pulls piece forward, applying reaction torque to roller
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_TRUE(ctx, gp->linearVelocity.z > 0.0f);

            // Move piece past centerline threshold (local Z = 0.45 + link Z 0.0 = 0.45m)
            auto* tcPiece = scene.GetComponent<TransformComponent>(piece);
            tcPiece->position = glm::vec3(0.0f, 0.25f, 0.50f);

            // Step 3: Centerline crossed -> Kinematic Capture Handoff to STORED
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_EQ(ctx, gp->state, GamePieceState::Stored);

            auto* cz = scene.GetComponent<ContactZoneComponent>(zoneEntity);
            TEST_EXPECT_TRUE(ctx, cz->isEngaged);

            auto* rbPiece = scene.GetComponent<RigidBodyComponent>(piece);
            TEST_EXPECT_EQ(ctx, rbPiece->motionType, BodyMotionType::Kinematic);
        }

        // =========================================================================
        // 3. Beam-Break Synchronization & Debounce Window in SimOutputs (PI3)
        // =========================================================================
        {
            std::cout << "  -> PI3: Beam-Break Synchronization & Debounce Latching\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);

            // Contact Zone Entity
            Entity czEntity = scene.CreateEntity();
            {
                ContactZoneComponent cz;
                cz.name = "hopper_zone";
                cz.role = "intake";
                cz.isEngaged = false;
                scene.AddComponent(czEntity, cz);
            }

            // Beam-Break Sensor on DIO Channel 3 with 10 ms (2 ticks at 200 Hz) debounce
            Entity bbEntity = scene.CreateEntity();
            {
                BeamBreakComponent bb;
                bb.name = "hopper_beam_break";
                bb.sensorEntity = czEntity;
                bb.dioChannel = 3;
                bb.debounceTimeS = 0.010; // 10 ms debounce filter
                bb.normallyClosed = false;
                scene.AddComponent(bbEntity, bb);
            }

            // Step 1: Zone clear -> DIO 3 remains 0
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_FALSE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);

            // Trip Contact Zone (Game piece captured to Stored)
            auto* cz = scene.GetComponent<ContactZoneComponent>(czEntity);
            cz->isEngaged = true;

            // Step 2: 1 tick elapsed (5 ms < 10 ms debounce threshold) -> DIO 3 still 0
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_FALSE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);

            // Step 3: 2 ticks elapsed (10 ms >= 10 ms debounce threshold) -> DIO 3 asserts HIGH
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_TRUE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);

            // Clear Contact Zone
            cz->isEngaged = false;

            // Step 4: 1 tick of clearing (5 ms < 10 ms) -> DIO 3 remains HIGH during debounce
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_TRUE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);

            // Step 5: 2 ticks of clearing (10 ms >= 10 ms) -> DIO 3 returns to LOW (0)
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_FALSE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);
        }

        // =========================================================================
        // 4. Feeder Handoff (STORED -> ENGAGED) & Zone Transfer (PI4)
        // =========================================================================
        {
            std::cout << "  -> PI4: Feeder Handoff (STORED -> ENGAGED) & Dual Zone Transfer\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);

            // Intake Zone (DIO 3)
            Entity intakeZone = scene.CreateEntity();
            {
                ContactZoneComponent cz;
                cz.name = "intake_zone";
                cz.role = "intake";
                cz.isEngaged = true;
                scene.AddComponent(intakeZone, cz);
            }
            robot.contactZoneEntities["intake_zone"] = intakeZone;

            Entity intakeBB = scene.CreateEntity();
            {
                BeamBreakComponent bb;
                bb.sensorEntity = intakeZone;
                bb.dioChannel = 3;
                bb.debounceTimeS = 0.005;
                scene.AddComponent(intakeBB, bb);
            }

            // Shooter Zone (DIO 2)
            Entity shooterZone = scene.CreateEntity();
            {
                ContactZoneComponent cz;
                cz.name = "shooter_zone";
                cz.role = "shooter";
                cz.isEngaged = false;
                scene.AddComponent(shooterZone, cz);
            }
            robot.contactZoneEntities["shooter_zone"] = shooterZone;

            Entity shooterBB = scene.CreateEntity();
            {
                BeamBreakComponent bb;
                bb.sensorEntity = shooterZone;
                bb.dioChannel = 2;
                bb.debounceTimeS = 0.005;
                scene.AddComponent(shooterBB, bb);
            }

            // Stored Game Piece in Hopper
            Entity piece = scene.CreateEntity();
            {
                GamePieceComponent gp;
                gp.state = GamePieceState::Stored;
                gp.boundContactZone = intakeZone;
                scene.AddComponent(piece, gp);
            }

            // Step pipeline: Intake beam is tripped, Shooter beam is clear
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_TRUE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);
            TEST_EXPECT_FALSE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 2)) != 0);

            // Command Feeder: STORED -> ENGAGED
            RobotGamePieceSystem::TriggerFeedToShooter(scene, robot);

            auto* gp = scene.GetComponent<GamePieceComponent>(piece);
            TEST_EXPECT_EQ(ctx, gp->state, GamePieceState::Engaged);
            TEST_EXPECT_EQ(ctx, gp->boundContactZone, shooterZone);

            auto* czIn = scene.GetComponent<ContactZoneComponent>(intakeZone);
            auto* czShoot = scene.GetComponent<ContactZoneComponent>(shooterZone);
            TEST_EXPECT_FALSE(ctx, czIn->isEngaged);
            TEST_EXPECT_TRUE(ctx, czShoot->isEngaged);

            // Step pipeline: Sensors update with debounce
            RobotAssembler::StepPipeline(robot, dt);
            TEST_EXPECT_FALSE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 3)) != 0);
            TEST_EXPECT_TRUE(ctx, (robot.sensorSystem->GetOutputs().digitalInputs & (1u << 2)) != 0);
        }

        // =========================================================================
        // 5. Complete 6-State Lifecycle Integration (PI5)
        // =========================================================================
        {
            std::cout << "  -> PI5: Complete 6-State Lifecycle Integration\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.flywheelSystem = std::make_unique<FlywheelSystem>(scene);
            robot.flightAerodynamicsSystem = std::make_unique<FlightAerodynamicsSystem>(scene, nullptr);
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);

            // 1. Shooter Link Body
            Entity shooterLink = scene.CreateEntity();
            {
                TransformComponent tc;
                tc.position = glm::vec3(0.0f, 0.80f, 0.0f);
                scene.AddComponent(shooterLink, tc);

                RigidBodyComponent rb;
                rb.motionType = BodyMotionType::Dynamic;
                rb.mass = 25.0f;
                scene.AddComponent(shooterLink, rb);
            }
            robot.chassisEntity = shooterLink;
            robot.rootLinkEntity = shooterLink;

            // 2. Flywheel Mechanism
            Entity flywheel = scene.CreateEntity();
            {
                FlywheelComponent fc;
                fc.name = "shooter_flywheel";
                fc.spinAxis = { 1.0f, 0.0f, 0.0f };
                fc.rollerRadiusM = 0.0508;
                fc.flywheelInertiaKgm2 = 0.003;
                fc.motorCount = 2;
                fc.motorModel = Actuation::DCMotorModel::KrakenX60(2);
                fc.commandedVoltage = 12.0;
                fc.angularVelocityRadPerSec = 350.0; // Pre-spun to ~3340 RPM operating speed
                scene.AddComponent(flywheel, fc);
            }
            robot.mechanismEntities["shooter_flywheel"] = flywheel;

            // 3. Shooter Contact Zone
            Entity shooterZone = scene.CreateEntity();
            {
                ContactZoneComponent cz;
                cz.name = "shooter_contact_zone";
                cz.role = "shooter";
                cz.parentLink = shooterLink;
                cz.localOrigin = { 0.0f, 0.0f, 0.0f };
                cz.dimensions = { 0.40f, 0.040f, 0.18f };
                cz.tangentAxis = { 0.0f, 0.5f, 0.866f }; // 30 deg launch pitch
                cz.compressionAxis = { 0.0f, 0.866f, -0.5f };
                cz.nominalGapM = 0.040f;
                cz.isEngaged = false;
                scene.AddComponent(shooterZone, cz);
            }
            robot.contactZoneEntities["shooter_contact_zone"] = shooterZone;

            // Stabilize flywheels for 10 steps
            for (int i = 0; i < 10; ++i)
            {
                RobotAssembler::StepPipeline(robot, dt);
            }

            auto* fc = scene.GetComponent<FlywheelComponent>(flywheel);
            double preShotRPM = fc->angularVelocityRadPerSec * (60.0 / (2.0 * 3.14159265358979323846));
            TEST_EXPECT_TRUE(ctx, preShotRPM > 2000.0);

            // 4. Game Piece: Start in ENGAGED state in throat
            Entity piece = scene.CreateEntity();
            {
                TransformComponent tc;
                tc.position = glm::vec3(0.0f, 0.80f, 0.0f);
                scene.AddComponent(piece, tc);

                GamePieceComponent gp;
                gp.state = GamePieceState::Engaged;
                gp.spec.massKg = 0.235;
                gp.spec.thicknessM = 0.0508;
                gp.spec.outerRadiusM = 0.1778;
                gp.spec.complianceK = 180000.0;
                gp.boundContactZone = shooterZone;
                gp.capturingRobot = shooterLink;
                gp.transitProgressM = 0.0;
                gp.transitVelocityMPerS = 2.5;
                scene.AddComponent(piece, gp);

                RigidBodyComponent rb;
                rb.motionType = BodyMotionType::Kinematic;
                rb.mass = 0.235f;
                scene.AddComponent(piece, rb);
            }

            // Step compression squeeze until muzzle exit is reached
            bool launched = false;
            for (int i = 0; i < 50; ++i)
            {
                RobotAssembler::StepPipeline(robot, dt);
                auto* gp = scene.GetComponent<GamePieceComponent>(piece);
                if (gp->state == GamePieceState::Launched)
                {
                    launched = true;
                    break;
                }
            }

            TEST_EXPECT_TRUE(ctx, launched);

            auto* gp = scene.GetComponent<GamePieceComponent>(piece);
            TEST_EXPECT_EQ(ctx, gp->state, GamePieceState::Launched);
            TEST_EXPECT_TRUE(ctx, gp->linearVelocity.z > 5.0f); // High exit speed

            // Verify linear chassis recoil impulse
            auto* rbLink = scene.GetComponent<RigidBodyComponent>(shooterLink);
            TEST_EXPECT_TRUE(ctx, rbLink->linearVelocity.z < 0.0f); // Recoil opposes launch (+Z launch -> -Z recoil)

            // Step flight aerodynamics until piece lands on floor
            for (int i = 0; i < 400; ++i)
            {
                RobotAssembler::StepPipeline(robot, dt);
                if (gp->state == GamePieceState::Free)
                {
                    break;
                }
            }

            // Piece landed and transitioned back to Free
            TEST_EXPECT_EQ(ctx, gp->state, GamePieceState::Free);
            TEST_EXPECT_TRUE(ctx, !std::isnan(gp->linearVelocity.x));
            TEST_EXPECT_TRUE(ctx, !std::isnan(gp->linearVelocity.y));
            TEST_EXPECT_TRUE(ctx, !std::isnan(gp->linearVelocity.z));
        }

        std::cout << "[PASS] PipelineIntegrationTest Complete\n";
    }
}
