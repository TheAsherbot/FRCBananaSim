/**
 * @file IntakeCaptureTest.cpp
 * @brief Implementation of unit tests for Block 6 Day 4 Intake Traction and Kinematic Capture.
 */

#include "IntakeCaptureTest.h"
#include "Interaction/IntakeSurfaceModel.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
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
    using namespace Robot::Mechanisms::Components;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;

    void RunIntakeCaptureTest(TestContext& ctx)
    {
        std::cout << "[RUN ] IntakeCaptureTest (Block 6 Day 4)\n";
        constexpr double tolerance = 1e-4;

        // =========================================================================
        // 1. Zone Overlap Detection Geometry (IC1)
        // =========================================================================
        {
            std::cout << "  -> IC1: Oriented Bounding Volume Overlap Detection\n";

            glm::vec3 zonePos(0.0f, 0.25f, 0.40f);
            glm::quat zoneRot(1.0f, 0.0f, 0.0f, 0.0f); // Identity
            glm::vec3 zoneDims(0.40f, 0.20f, 0.30f);   // 400x200x300 mm
            double pieceRadius = 0.1778; // 2024 Note radius

            // 1. Inside zone
            glm::vec3 pieceInside(0.0f, 0.25f, 0.40f);
            bool inside = IntakeSurfaceModel::TestZoneOverlap(pieceInside, pieceRadius, zonePos, zoneRot, zoneDims);
            TEST_EXPECT_TRUE(ctx, inside);

            // 2. Far outside zone
            glm::vec3 pieceFar(2.0f, 0.25f, 0.40f);
            bool outside = IntakeSurfaceModel::TestZoneOverlap(pieceFar, pieceRadius, zonePos, zoneRot, zoneDims);
            TEST_EXPECT_FALSE(ctx, outside);

            // 3. Touching boundary within piece radius margin
            // Zone half-extents in X: 0.20m. Boundary margin: 0.20 + 0.1778 = 0.3778m
            glm::vec3 pieceTouching(0.35f, 0.25f, 0.40f);
            bool touching = IntakeSurfaceModel::TestZoneOverlap(pieceTouching, pieceRadius, zonePos, zoneRot, zoneDims);
            TEST_EXPECT_TRUE(ctx, touching);

            glm::vec3 pieceBeyondMargin(0.40f, 0.25f, 0.40f);
            bool beyond = IntakeSurfaceModel::TestZoneOverlap(pieceBeyondMargin, pieceRadius, zonePos, zoneRot, zoneDims);
            TEST_EXPECT_FALSE(ctx, beyond);
        }

        // =========================================================================
        // 2. Surface Velocity & Viscous Traction Dynamics (IC2)
        // =========================================================================
        {
            std::cout << "  -> IC2: Surface Velocity & Compliant Traction Drag Dynamics\n";

            IntakeParameters params;
            params.rollerRadiusM = 0.0508;
            params.frictionMu = 0.70;
            params.normalForceN = 45.0;
            params.viscousDampingNsPerM = 8.0;
            params.maxGripForceN = 120.0;
            params.centerlineCaptureThresholdM = 0.0;

            glm::vec3 tanWorld(0.0f, 0.0f, 1.0f); // Inward along +Z

            // 1. Surface velocity calculation with moving chassis
            double omega = 100.0; // rad/s (~955 RPM)
            glm::vec3 vParent(0.0f, 0.0f, 1.5f); // Robot driving forward at 1.5 m/s
            glm::vec3 vRoller = IntakeSurfaceModel::CalculateRollerSurfaceVelocity(omega, params.rollerRadiusM, tanWorld, vParent);
            // v_roller = (100 * 0.0508) + 1.5 = 5.08 + 1.5 = 6.58 m/s along +Z
            TEST_EXPECT_NEAR(ctx, vRoller.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, vRoller.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, vRoller.z, 6.58, tolerance);

            // 2. Relative velocity & traction calculation (Unclamped)
            glm::vec3 pieceVel(0.0f, 0.0f, 0.0f); // Piece stationary on ground
            glm::vec3 rollerVel(0.0f, 0.0f, 5.0f); // Roller surface at 5.0 m/s
            auto traction = IntakeSurfaceModel::CalculateIntakeTraction(params, rollerVel, pieceVel, tanWorld, -0.05);

            // F_coulomb = 0.70 * 45 = 31.5 N
            // F_viscous = 8.0 * 5.0 = 40.0 N
            // F_total = 31.5 + 40.0 = 71.5 N along +Z
            TEST_EXPECT_NEAR(ctx, traction.intakeForceN.z, 71.5, tolerance);
            // Load torque: tau = 71.5 * 0.0508 = 3.6322 N*m
            TEST_EXPECT_NEAR(ctx, traction.loadTorqueNm, 3.6322, tolerance);
            TEST_EXPECT_FALSE(ctx, traction.isPastCenterline);

            // 3. High slip saturation clamp at 120 N
            glm::vec3 rollerVelHigh(0.0f, 0.0f, 20.0f);
            auto tractionClamped = IntakeSurfaceModel::CalculateIntakeTraction(params, rollerVelHigh, pieceVel, tanWorld, -0.05);
            // Raw force: 31.5 + (8.0 * 20.0) = 191.5 N -> Clamped to 120.0 N
            TEST_EXPECT_NEAR(ctx, tractionClamped.intakeForceN.z, 120.0, tolerance);
            // Clamped load torque: 120.0 * 0.0508 = 6.096 N*m
            TEST_EXPECT_NEAR(ctx, tractionClamped.loadTorqueNm, 6.096, tolerance);
        }

        // =========================================================================
        // 3. Intake Pull Dynamics & Reaction Torque on Motor (IC3)
        // =========================================================================
        {
            std::cout << "  -> IC3: Active Intake Pull & Motor Reaction Torque Coupling\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            // Chassis link
            Entity chassis = scene.CreateEntity();
            robot.chassisEntity = chassis;
            TransformComponent tcChassis;
            tcChassis.position = glm::vec3(0.0f, 0.2f, 0.0f);
            scene.AddComponent(chassis, tcChassis);

            // Active Intake Roller Mechanism
            Entity intakeMech = scene.CreateEntity();
            robot.mechanismEntities["intake_roller"] = intakeMech;

            FlywheelComponent fwIntake;
            fwIntake.name = "intake_roller";
            fwIntake.rollerRadiusM = 0.0508;
            fwIntake.angularVelocityRadPerSec = 200.0; // Active spinning (~1910 RPM)
            fwIntake.externalLoadTorqueNm = 0.0;
            scene.AddComponent(intakeMech, fwIntake);

            // Intake Contact Zone
            Entity czEntity = scene.CreateEntity();
            robot.contactZoneEntities["intake_zone"] = czEntity;

            ContactZoneComponent cz;
            cz.name = "intake_zone";
            cz.role = "intake";
            cz.parentLink = chassis;
            cz.tangentAxis = glm::vec3(0.0f, 0.0f, 1.0f); // Inward along +Z
            cz.dimensions = glm::vec3(0.40f, 0.20f, 0.30f);
            cz.localOrigin = glm::vec3(0.0f, 0.0f, 0.20f);
            scene.AddComponent(czEntity, cz);

            // Game Piece overlapping intake zone in Free state
            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Free;
            piece.spec.massKg = 0.235;
            piece.linearVelocity = glm::vec3(0.0f);
            scene.AddComponent(pieceEntity, piece);

            TransformComponent tcPiece;
            tcPiece.position = glm::vec3(0.0f, 0.2f, 0.15f); // Inside intake zone, in front of centerline
            scene.AddComponent(pieceEntity, tcPiece);

            // Step 1: UpdateIntake detects overlap and transitions to Intaking
            RobotGamePieceSystem::UpdateIntake(scene, robot, 0.005);

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* postFw = scene.GetComponent<FlywheelComponent>(intakeMech);

            TEST_EXPECT_TRUE(ctx, postPiece != nullptr);
            TEST_EXPECT_TRUE(ctx, postFw != nullptr);

            // Piece transitioned to Intaking
            TEST_EXPECT_EQ(ctx, postPiece->state, GamePieceState::Intaking);
            TEST_EXPECT_EQ(ctx, postPiece->boundContactZone, czEntity);

            // Piece received inward forward acceleration
            TEST_EXPECT_TRUE(ctx, postPiece->linearVelocity.z > 0.0f);

            // Intake motor received equal-and-opposite reaction braking torque
            TEST_EXPECT_TRUE(ctx, postFw->externalLoadTorqueNm > 0.0);
        }

        // =========================================================================
        // 4. Centerline Crossing & Kinematic Capture Handoff to STORED (IC4)
        // =========================================================================
        {
            std::cout << "  -> IC4: Centerline Crossing & Kinematic Capture to STORED\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            Entity chassis = scene.CreateEntity();
            robot.chassisEntity = chassis;
            TransformComponent tcChassis;
            scene.AddComponent(chassis, tcChassis);

            Entity intakeMech = scene.CreateEntity();
            robot.mechanismEntities["intake_roller"] = intakeMech;
            FlywheelComponent fwIntake;
            fwIntake.name = "intake_roller";
            fwIntake.rollerRadiusM = 0.0508;
            fwIntake.angularVelocityRadPerSec = 200.0;
            scene.AddComponent(intakeMech, fwIntake);

            Entity czEntity = scene.CreateEntity();
            robot.contactZoneEntities["intake_zone"] = czEntity;
            ContactZoneComponent cz;
            cz.name = "intake_zone";
            cz.role = "intake";
            cz.parentLink = chassis;
            cz.tangentAxis = glm::vec3(0.0f, 0.0f, 1.0f);
            cz.dimensions = glm::vec3(0.40f, 0.20f, 0.30f);
            cz.localOrigin = glm::vec3(0.0f, 0.0f, 0.20f); // Centerline at z = 0.20
            scene.AddComponent(czEntity, cz);

            // Piece in Intaking state, placed past roller centerline (z = 0.25 > 0.20)
            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Intaking;
            piece.boundContactZone = czEntity;
            scene.AddComponent(pieceEntity, piece);

            TransformComponent tcPiece;
            tcPiece.position = glm::vec3(0.0f, 0.0f, 0.25f); // Past centerline!
            scene.AddComponent(pieceEntity, tcPiece);

            RigidBodyComponent rbPiece;
            rbPiece.motionType = BodyMotionType::Dynamic;
            scene.AddComponent(pieceEntity, rbPiece);

            RobotGamePieceSystem::UpdateIntake(scene, robot, 0.005);

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* postCZ = scene.GetComponent<ContactZoneComponent>(czEntity);
            auto* postRB = scene.GetComponent<RigidBodyComponent>(pieceEntity);

            // Must transition to Stored
            TEST_EXPECT_EQ(ctx, postPiece->state, GamePieceState::Stored);
            // Contact zone engaged asserted (trips beam-break!)
            TEST_EXPECT_TRUE(ctx, postCZ->isEngaged);
            // Rigid body switched to Kinematic
            TEST_EXPECT_EQ(ctx, postRB->motionType, BodyMotionType::Kinematic);
            // Relative velocity zeroed
            TEST_EXPECT_NEAR(ctx, postPiece->linearVelocity.z, 0.0, tolerance);
        }

        // =========================================================================
        // 5. Feed Handoff from STORED to ENGAGED (IC5)
        // =========================================================================
        {
            std::cout << "  -> IC5: Shooter Feed Handoff (STORED -> ENGAGED)\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            Entity intakeCZ = scene.CreateEntity();
            robot.contactZoneEntities["intake_zone"] = intakeCZ;
            ContactZoneComponent czIn;
            czIn.role = "intake";
            czIn.isEngaged = true; // Tripped while holding stored piece
            scene.AddComponent(intakeCZ, czIn);

            Entity shooterCZ = scene.CreateEntity();
            robot.contactZoneEntities["shooter_zone"] = shooterCZ;
            ContactZoneComponent czShoot;
            czShoot.role = "shooter";
            czShoot.isEngaged = false;
            scene.AddComponent(shooterCZ, czShoot);

            // Piece in Stored state
            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Stored;
            piece.boundContactZone = intakeCZ;
            scene.AddComponent(pieceEntity, piece);

            // Call TriggerFeedToShooter
            RobotGamePieceSystem::TriggerFeedToShooter(scene, robot);

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* postCZIn = scene.GetComponent<ContactZoneComponent>(intakeCZ);
            auto* postCZShoot = scene.GetComponent<ContactZoneComponent>(shooterCZ);

            // Piece transitioned to Engaged
            TEST_EXPECT_EQ(ctx, postPiece->state, GamePieceState::Engaged);
            // Bound to shooter zone
            TEST_EXPECT_EQ(ctx, postPiece->boundContactZone, shooterCZ);
            // Transit progress initialized to 0.0
            TEST_EXPECT_NEAR(ctx, postPiece->transitProgressM, 0.0, tolerance);
            // Positive feed velocity into shooter nip
            TEST_EXPECT_TRUE(ctx, postPiece->transitVelocityMPerS > 1.0);
            // Shooter zone asserted engaged
            TEST_EXPECT_TRUE(ctx, postCZShoot->isEngaged);
            // Intake zone cleared
            TEST_EXPECT_FALSE(ctx, postCZIn->isEngaged);
        }

        std::cout << "[PASS] IntakeCaptureTest Complete\n";
    }
}
