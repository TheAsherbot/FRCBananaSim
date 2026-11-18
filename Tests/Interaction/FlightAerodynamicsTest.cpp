/**
 * @file FlightAerodynamicsTest.cpp
 * @brief Implementation of unit tests for Block 6 Day 3 Recoil and Flight Aerodynamics.
 */

#include "FlightAerodynamicsTest.h"
#include "Interaction/Systems/FlightAerodynamicsSystem.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/RobotAssembler.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"

#include <iostream>
#include <cmath>
#include <limits>

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

    void RunFlightAerodynamicsTest(TestContext& ctx)
    {
        std::cout << "[RUN ] FlightAerodynamicsTest (Block 6 Day 3)\n";
        constexpr double tolerance = 1e-4;

        // =========================================================================
        // 1. Analytical Aerodynamics: Quadratic Drag & Magnus Lift (FA1)
        // =========================================================================
        {
            std::cout << "  -> FA1: Analytical Quadratic Air Drag & Magnus Rotational Lift\n";

            constexpr double rhoAir = 1.225; // kg/m^3
            constexpr double Cd = 0.45;
            constexpr double area = 0.045;   // m^2
            constexpr double ClAlpha = 1.20;
            constexpr double radius = 0.1778; // m

            // 1. Zero velocity yields zero drag
            glm::vec3 dragZero = FlightAerodynamicsSystem::CalculateDragForce(rhoAir, Cd, area, glm::vec3(0.0f));
            TEST_EXPECT_NEAR(ctx, dragZero.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, dragZero.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, dragZero.z, 0.0, tolerance);

            // 2. Drag opposes motion along +Z
            glm::vec3 v1(0.0f, 0.0f, 15.0f);
            glm::vec3 drag1 = FlightAerodynamicsSystem::CalculateDragForce(rhoAir, Cd, area, v1);
            TEST_EXPECT_NEAR(ctx, drag1.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, drag1.y, 0.0, tolerance);
            // Expected: -0.5 * 1.225 * 0.45 * 0.045 * 15.0^2 = -2.790703 N
            TEST_EXPECT_NEAR(ctx, drag1.z, -2.7907, 1e-3);

            // 3. Speed doubling quadruples drag force (v^2 scaling)
            glm::vec3 v2(0.0f, 0.0f, 30.0f);
            glm::vec3 drag2 = FlightAerodynamicsSystem::CalculateDragForce(rhoAir, Cd, area, v2);
            TEST_EXPECT_NEAR(ctx, drag2.z, -11.1628, 1e-3);
            double ratio = drag2.z / drag1.z;
            TEST_EXPECT_NEAR(ctx, ratio, 4.0, 1e-3);

            // 4. Zero spin or zero velocity yields zero Magnus lift
            glm::vec3 magnusZeroSpin = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, glm::vec3(0.0f), v1);
            TEST_EXPECT_NEAR(ctx, magnusZeroSpin.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, magnusZeroSpin.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, magnusZeroSpin.z, 0.0, tolerance);

            glm::vec3 magnusZeroVel = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, glm::vec3(100.0f, 0.0f, 0.0f), glm::vec3(0.0f));
            TEST_EXPECT_NEAR(ctx, magnusZeroVel.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, magnusZeroVel.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, magnusZeroVel.z, 0.0, tolerance);

            // 5. Topspin generates downward Magnus force
            // omega = (+100, 0, 0), v = (0, 0, 15) -> omega x v = (0, -1500, 0)
            glm::vec3 omegaTop(100.0f, 0.0f, 0.0f);
            glm::vec3 magnusTop = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, omegaTop, v1);
            TEST_EXPECT_NEAR(ctx, magnusTop.x, 0.0, tolerance);
            TEST_EXPECT_TRUE(ctx, magnusTop.y < 0.0f);
            // S_m = 0.5 * 1.225 * 1.20 * 0.045 * 0.1778 = 0.0058807 kg
            // F_m = 0.0058807 * (-1500.0) = -8.8211 N
            TEST_EXPECT_NEAR(ctx, magnusTop.y, -8.8211, 1e-3);
            TEST_EXPECT_NEAR(ctx, magnusTop.z, 0.0, tolerance);

            // 6. Backspin generates upward Magnus lift
            glm::vec3 omegaBack(-100.0f, 0.0f, 0.0f);
            glm::vec3 magnusBack = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, omegaBack, v1);
            TEST_EXPECT_NEAR(ctx, magnusBack.x, 0.0, tolerance);
            TEST_EXPECT_TRUE(ctx, magnusBack.y > 0.0f);
            TEST_EXPECT_NEAR(ctx, magnusBack.y, 8.8211, 1e-3);
        }

        // =========================================================================
        // 2. Linear Chassis Recoil & Linear Momentum Conservation (FA2)
        // =========================================================================
        {
            std::cout << "  -> FA2: Linear Chassis Recoil & Momentum Conservation at Rest\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            // 1. Shooter link rigid body initially at rest
            Entity shooterLink = scene.CreateEntity();
            robot.linkEntities["shooter_link"] = shooterLink;
            robot.chassisEntity = shooterLink;

            RigidBodyComponent rbShooter;
            rbShooter.mass = 15.0f; // 15 kg shooter chassis
            rbShooter.linearVelocity = glm::vec3(0.0f);
            rbShooter.angularVelocity = glm::vec3(0.0f);
            rbShooter.collisionGroupID = 1;
            rbShooter.collisionSubGroupID = 1;
            scene.AddComponent(shooterLink, rbShooter);

            TransformComponent tcShooter;
            tcShooter.position = glm::vec3(0.0f, 0.5f, 0.0f);
            tcShooter.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            scene.AddComponent(shooterLink, tcShooter);

            // 2. Contact zone attached to shooter link
            Entity czEntity = scene.CreateEntity();
            robot.contactZoneEntities["shooter_zone"] = czEntity;

            ContactZoneComponent cz;
            cz.name = "shooter_zone";
            cz.role = "shooter";
            cz.parentLink = shooterLink;
            cz.tangentAxis = glm::vec3(0.0f, 0.0f, 1.0f);     // Forward along +Z
            cz.compressionAxis = glm::vec3(0.0f, 1.0f, 0.0f); // Up along +Y
            cz.dimensions = glm::vec3(0.10f, 0.10f, 0.18f);   // 180 mm contact length
            cz.localOrigin = glm::vec3(0.0f, 0.0f, 0.0f);
            cz.isEngaged = true;
            scene.AddComponent(czEntity, cz);

            // 3. Projectile reaching muzzle at 20 m/s
            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Engaged;
            piece.spec.massKg = 0.235; // 235 grams Note
            piece.spec.outerRadiusM = 0.1778;
            piece.spec.thicknessM = 0.0508;
            piece.transitProgressM = 0.18; // At muzzle threshold
            piece.transitVelocityMPerS = 20.0;
            piece.boundContactZone = czEntity;
            piece.capturingRobot = shooterLink;
            scene.AddComponent(pieceEntity, piece);

            TransformComponent tcPiece;
            tcPiece.position = glm::vec3(0.0f, 0.5f, 0.0f);
            scene.AddComponent(pieceEntity, tcPiece);

            RigidBodyComponent rbPiece;
            rbPiece.mass = 0.235f;
            scene.AddComponent(pieceEntity, rbPiece);

            // Execute PostFlywheelUpdate
            RobotGamePieceSystem::PostFlywheelUpdate(scene, robot, 0.005);

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* postShooter = scene.GetComponent<RigidBodyComponent>(shooterLink);
            auto* postCZ = scene.GetComponent<ContactZoneComponent>(czEntity);

            TEST_EXPECT_TRUE(ctx, postPiece != nullptr);
            TEST_EXPECT_TRUE(ctx, postShooter != nullptr);
            TEST_EXPECT_TRUE(ctx, postCZ != nullptr);

            // 4. Projectile transitioned to Launched with exit velocity 20 m/s along +Z
            TEST_EXPECT_EQ(ctx, postPiece->state, GamePieceState::Launched);
            TEST_EXPECT_NEAR(ctx, postPiece->linearVelocity.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, postPiece->linearVelocity.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, postPiece->linearVelocity.z, 20.0, tolerance);
            TEST_EXPECT_EQ(ctx, postPiece->boundContactZone, invalidEntity);
            TEST_EXPECT_EQ(ctx, postPiece->capturingRobot, invalidEntity);
            TEST_EXPECT_FALSE(ctx, postCZ->isEngaged);
            TEST_EXPECT_EQ(ctx, postPiece->muzzleCollisionMaskTicks, 3u);

            // 5. Shooter chassis received recoil impulse: J = -m_piece * v_piece = -0.235 * 20 = -4.70 N*s
            // Delta v_shooter = -4.70 / 15.0 = -0.313333 m/s
            TEST_EXPECT_NEAR(ctx, postShooter->linearVelocity.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, postShooter->linearVelocity.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, postShooter->linearVelocity.z, -0.313333, 1e-4);

            // 6. Net linear momentum strictly conserved: P_total = m_shooter*v_shooter + m_piece*v_piece = 0
            double totalMomentum = postShooter->mass * postShooter->linearVelocity.z +
                                   postPiece->spec.massKg * postPiece->linearVelocity.z;
            TEST_EXPECT_NEAR(ctx, totalMomentum, 0.0, 1e-4);
        }

        // =========================================================================
        // 3. Recoil with Moving Chassis - Galilean Invariance (FA3)
        // =========================================================================
        {
            std::cout << "  -> FA3: Recoil Under Moving Chassis (Galilean Invariance)\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            // Chassis moving forward at 2.5 m/s
            Entity shooterLink = scene.CreateEntity();
            robot.chassisEntity = shooterLink;

            RigidBodyComponent rbShooter;
            rbShooter.mass = 15.0f;
            rbShooter.linearVelocity = glm::vec3(0.0f, 0.0f, 2.5f);
            scene.AddComponent(shooterLink, rbShooter);

            TransformComponent tcShooter;
            tcShooter.position = glm::vec3(0.0f, 0.5f, 0.0f);
            scene.AddComponent(shooterLink, tcShooter);

            Entity czEntity = scene.CreateEntity();
            ContactZoneComponent cz;
            cz.parentLink = shooterLink;
            cz.tangentAxis = glm::vec3(0.0f, 0.0f, 1.0f);
            cz.compressionAxis = glm::vec3(0.0f, 1.0f, 0.0f);
            cz.dimensions = glm::vec3(0.10f, 0.10f, 0.18f);
            cz.isEngaged = true;
            scene.AddComponent(czEntity, cz);

            // Projectile reaching muzzle with relative velocity 16.0 m/s
            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Engaged;
            piece.spec.massKg = 0.235;
            piece.transitProgressM = 0.18;
            piece.transitVelocityMPerS = 16.0;
            piece.boundContactZone = czEntity;
            scene.AddComponent(pieceEntity, piece);

            RigidBodyComponent rbPiece;
            rbPiece.mass = 0.235f;
            scene.AddComponent(pieceEntity, rbPiece);

            // Momentum before launch: (15.0 + 0.235) * 2.5 = 38.0875 kg*m/s
            double momentumBefore = (rbShooter.mass + piece.spec.massKg) * 2.5;

            RobotGamePieceSystem::PostFlywheelUpdate(scene, robot, 0.005);

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* postShooter = scene.GetComponent<RigidBodyComponent>(shooterLink);

            // Absolute exit velocity = v_robot (2.5) + v_rel (16.0) = 18.5 m/s
            TEST_EXPECT_NEAR(ctx, postPiece->linearVelocity.z, 18.5, tolerance);

            // Recoil impulse J = -0.235 * 16.0 = -3.76 N*s
            // v_shooter = 2.5 + (-3.76 / 15.0) = 2.5 - 0.250667 = 2.249333 m/s
            TEST_EXPECT_NEAR(ctx, postShooter->linearVelocity.z, 2.249333, 1e-4);

            // Momentum after launch: 15.0 * 2.249333 + 0.235 * 18.5 = 38.0875 kg*m/s
            double momentumAfter = postShooter->mass * postShooter->linearVelocity.z +
                                  postPiece->spec.massKg * postPiece->linearVelocity.z;
            TEST_EXPECT_NEAR(ctx, momentumAfter, momentumBefore, 1e-4);
        }

        // =========================================================================
        // 4. Muzzle Collision Masking Countdown (FA4)
        // =========================================================================
        {
            std::cout << "  -> FA4: 3-Tick Muzzle Collision Masking Countdown\n";

            Scene scene;
            FlightAerodynamicsSystem aeroSystem(scene, nullptr);

            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Launched;
            piece.linearVelocity = glm::vec3(0.0f, 0.0f, 15.0f);
            piece.muzzleCollisionMaskTicks = 3;
            scene.AddComponent(pieceEntity, piece);

            RigidBodyComponent rbPiece;
            rbPiece.collisionGroupID = 1; // Robot collision group
            rbPiece.collisionSubGroupID = 30;
            scene.AddComponent(pieceEntity, rbPiece);

            TransformComponent tcPiece;
            tcPiece.position = glm::vec3(0.0f, 1.0f, 0.0f);
            scene.AddComponent(pieceEntity, tcPiece);

            // Step 1: tick decrements to 2, collision group remains masked (1)
            aeroSystem.FixedUpdate(0.005);
            auto* p1 = scene.GetComponent<GamePieceComponent>(pieceEntity);
            auto* r1 = scene.GetComponent<RigidBodyComponent>(pieceEntity);
            TEST_EXPECT_EQ(ctx, p1->muzzleCollisionMaskTicks, 2u);
            TEST_EXPECT_EQ(ctx, r1->collisionGroupID, 1u);

            // Step 2: tick decrements to 1, collision group remains masked (1)
            aeroSystem.FixedUpdate(0.005);
            TEST_EXPECT_EQ(ctx, p1->muzzleCollisionMaskTicks, 1u);
            TEST_EXPECT_EQ(ctx, r1->collisionGroupID, 1u);

            // Step 3: tick hits 0 -> collision group reset to 0 (unmasked world collisions restored)
            aeroSystem.FixedUpdate(0.005);
            TEST_EXPECT_EQ(ctx, p1->muzzleCollisionMaskTicks, 0u);
            TEST_EXPECT_EQ(ctx, r1->collisionGroupID, 0u);
            TEST_EXPECT_EQ(ctx, r1->collisionSubGroupID, 0u);
        }

        // =========================================================================
        // 5. Free Flight Rest Detection & Carpet Landing (FA5)
        // =========================================================================
        {
            std::cout << "  -> FA5: Ground Landing Rest Detection & State Transition\n";

            Scene scene;
            FlightAerodynamicsSystem aeroSystem(scene, nullptr);

            Entity pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Launched;
            piece.spec.thicknessM = 0.0508;
            piece.spec.massKg = 0.235;
            // Placed at carpet altitude (y <= thickness * 1.5 = 0.076 m) and nearly stationary
            piece.linearVelocity = glm::vec3(0.0f, 0.0f, 0.03f); // < 0.1 m/s
            piece.stateTimer = 0.0;
            scene.AddComponent(pieceEntity, piece);

            TransformComponent tcPiece;
            tcPiece.position = glm::vec3(0.0f, 0.025f, 5.0f);
            scene.AddComponent(pieceEntity, tcPiece);

            // Step system until stateTimer exceeds 0.15s (35 steps * 0.005 = 0.175s)
            for (int i = 0; i < 35; ++i)
            {
                aeroSystem.FixedUpdate(0.005);
            }

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            TEST_EXPECT_TRUE(ctx, postPiece != nullptr);
            // Must transition to Free state once at rest on carpet
            TEST_EXPECT_EQ(ctx, postPiece->state, GamePieceState::Free);
            TEST_EXPECT_NEAR(ctx, postPiece->stateTimer, 0.0, tolerance);
        }

        // =========================================================================
        // 6. Numerical Robustness & Extreme Boundaries (FA6)
        // =========================================================================
        {
            std::cout << "  -> FA6: Numerical Robustness & Extreme Boundaries\n";

            constexpr double rhoAir = 1.225;
            constexpr double Cd = 0.45;
            constexpr double area = 0.045;
            constexpr double ClAlpha = 1.20;
            constexpr double radius = 0.1778;

            // 1. Extreme hypersonic velocity (200 m/s)
            glm::vec3 vHyper(0.0f, 0.0f, 200.0f);
            glm::vec3 dragHyper = FlightAerodynamicsSystem::CalculateDragForce(rhoAir, Cd, area, vHyper);
            TEST_EXPECT_FALSE(ctx, std::isnan(dragHyper.z));
            TEST_EXPECT_FALSE(ctx, std::isinf(dragHyper.z));
            TEST_EXPECT_TRUE(ctx, dragHyper.z < 0.0f);

            // 2. Extreme hyper-spin (50,000 rad/s)
            glm::vec3 omegaHyper(50000.0f, 0.0f, 0.0f);
            glm::vec3 magnusHyper = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, omegaHyper, vHyper);
            TEST_EXPECT_FALSE(ctx, std::isnan(magnusHyper.y));
            TEST_EXPECT_FALSE(ctx, std::isinf(magnusHyper.y));
            TEST_EXPECT_TRUE(ctx, magnusHyper.y < 0.0f);

            // 3. NaN sanitization: function returns zero rather than NaN
            glm::vec3 vNaN(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
            glm::vec3 dragNaN = FlightAerodynamicsSystem::CalculateDragForce(rhoAir, Cd, area, vNaN);
            TEST_EXPECT_NEAR(ctx, dragNaN.x, 0.0, tolerance);
            TEST_EXPECT_FALSE(ctx, std::isnan(dragNaN.x));

            glm::vec3 magnusNaN = FlightAerodynamicsSystem::CalculateMagnusForce(rhoAir, ClAlpha, area, radius, vNaN, vHyper);
            TEST_EXPECT_NEAR(ctx, magnusNaN.x, 0.0, tolerance);
            TEST_EXPECT_FALSE(ctx, std::isnan(magnusNaN.x));
        }

        std::cout << "[PASS] FlightAerodynamicsTest Complete\n";
    }
}
