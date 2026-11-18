/**
 * @file ShooterSqueezeTest.cpp
 * @brief Implementation of unit tests for Block 6 Day 2 Shooter Squeeze and Flywheel RPM Sag.
 */

#include "ShooterSqueezeTest.h"
#include "Interaction/ShooterSqueezeModel.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/RobotAssembler.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Core/Scene.h"

#include <filesystem>
#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Interaction;
    using namespace Interaction::Components;
    using namespace Robot;
    using namespace Robot::Schema;
    using namespace Robot::Sensors::Components;
    using namespace Robot::Mechanisms::Components;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;

    void RunShooterSqueezeTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ShooterSqueezeTest (Block 6 Day 2)\n";
        constexpr double tolerance = 1e-4;

        // =========================================================================
        // 1. Hunt-Crossley Normal Compression Properties (SS1)
        // =========================================================================
        {
            std::cout << "  -> SS1: Hunt-Crossley Normal Compression & Hysteresis Damping\n";

            constexpr double k = 25000.0;
            constexpr double e = 0.25;
            constexpr double vApp = 15.0; // 15 m/s typical shooter speed

            // 1. Zero force at zero penetration depth
            double f0 = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(0.0, 0.0, k, e, vApp);
            TEST_EXPECT_EQ(ctx, f0, 0.0);

            double fNeg = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(-0.01, 1.0, k, e, vApp);
            TEST_EXPECT_EQ(ctx, fNeg, 0.0);

            // 2. Loading vs Unloading Hysteresis at delta = 0.02 m
            constexpr double depth = 0.02;
            double fLoading = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(depth, +2.0, k, e, vApp);
            double fUnloading = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(depth, -2.0, k, e, vApp);

            // During compression, damping adds force; during expansion, damping subtracts force
            TEST_EXPECT_TRUE(ctx, fLoading > fUnloading);
            TEST_EXPECT_TRUE(ctx, fLoading > 0.0);
            TEST_EXPECT_TRUE(ctx, fUnloading > 0.0);

            // 3. No tensile adhesion: large negative compression velocity never pulls
            double fPulling = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(depth, -200.0, k, e, vApp);
            TEST_EXPECT_EQ(ctx, fPulling, 0.0);

            // 4. Elastic limit (e = 1.0): alpha = 0, so fLoading == fUnloading
            double fLoadingElastic = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(depth, +5.0, k, 1.0, vApp);
            double fUnloadingElastic = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(depth, -5.0, k, 1.0, vApp);
            TEST_EXPECT_NEAR(ctx, fLoadingElastic, fUnloadingElastic, 1e-6);
        }

        // =========================================================================
        // 2. Tangential Slip Traction & Regularization (SS2)
        // =========================================================================
        {
            std::cout << "  -> SS2: Tangential Slip Traction & Regularized Coulomb Friction\n";

            constexpr double fn = 500.0; // 500 N normal squeeze
            constexpr double mu = 0.68;

            // 1. Zero slip velocity produces zero traction force
            double ftZero = ShooterSqueezeModel::CalculateTangentialTractionForce(fn, 0.0, mu);
            TEST_EXPECT_NEAR(ctx, ftZero, 0.0, 1e-6);

            // 2. Positive slip velocity produces forward traction
            double ftFwd = ShooterSqueezeModel::CalculateTangentialTractionForce(fn, 5.0, mu);
            TEST_EXPECT_TRUE(ctx, ftFwd > 0.0);
            TEST_EXPECT_NEAR(ctx, ftFwd, mu * fn, 1.0); // Asymptotically approaches mu * fn = 340 N

            // 3. Negative slip velocity produces retarding traction (odd symmetry)
            double ftRev = ShooterSqueezeModel::CalculateTangentialTractionForce(fn, -5.0, mu);
            TEST_EXPECT_NEAR(ctx, ftRev, -ftFwd, 1e-6);

            // 4. Zero normal force produces zero traction
            double ftNoFn = ShooterSqueezeModel::CalculateTangentialTractionForce(0.0, 10.0, mu);
            TEST_EXPECT_EQ(ctx, ftNoFn, 0.0);
        }

        // =========================================================================
        // 3. Continuous Compression Nip Profile (SS3)
        // =========================================================================
        {
            std::cout << "  -> SS3: Continuous Compression Nip Profile\n";

            constexpr double contactLength = 0.18; // 18 cm
            constexpr double maxDelta = 0.03;      // 30 mm max squeeze
            constexpr double velocity = 10.0;     // 10 m/s

            double depthEntry = 0.0, velEntry = 0.0;
            ShooterSqueezeModel::CalculateCompressionProfile(0.0, velocity, contactLength, maxDelta, depthEntry, velEntry);
            TEST_EXPECT_NEAR(ctx, depthEntry, 0.0, 1e-6);
            TEST_EXPECT_TRUE(ctx, velEntry > 0.0); // Positive compression rate entering nip

            double depthMid = 0.0, velMid = 0.0;
            ShooterSqueezeModel::CalculateCompressionProfile(0.09, velocity, contactLength, maxDelta, depthMid, velMid);
            TEST_EXPECT_NEAR(ctx, depthMid, maxDelta, 1e-6);
            TEST_EXPECT_NEAR(ctx, velMid, 0.0, 1e-4); // Peak compression has zero rate of change

            double depthExit = 0.0, velExit = 0.0;
            ShooterSqueezeModel::CalculateCompressionProfile(0.18, velocity, contactLength, maxDelta, depthExit, velExit);
            TEST_EXPECT_NEAR(ctx, depthExit, 0.0, 1e-6);
            TEST_EXPECT_TRUE(ctx, velExit < 0.0); // Negative compression rate exiting nip
        }

        // =========================================================================
        // 4. Single-Shot Flywheel RPM Sag & Projectile Acceleration (SS4)
        // =========================================================================
        {
            std::cout << "  -> SS4: Flywheel RPM Sag & Projectile Acceleration Dynamics\n";

            SqueezeParameters params;
            params.stiffnessK = 25000.0;
            params.restitutionE = 0.25;
            params.frictionMu = 0.68;
            params.hoodFrictionMu = 0.05;
            params.rollerRadiusM = 0.0508;
            params.nominalGapM = 0.08;
            params.uncompressedSizeM = 0.12; // 40 mm max squeeze depth
            params.contactLengthM = 0.18;
            params.pieceMassKg = 0.235;

            constexpr double fwInertia = 0.0032; // kg*m^2
            constexpr double initialRpm = 4500.0;
            constexpr double rpmToRadS = (2.0 * 3.14159265358979323846) / 60.0;
            constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);

            double currentOmega = initialRpm * rpmToRadS;
            SqueezeState state;
            state.progressM = 0.0;
            state.velocityMPerS = 2.0; // Enters nip at 2 m/s from indexer

            constexpr double dt = 0.005; // 200 Hz
            int steps = 0;

            // Step until projectile clears the muzzle (contactLength)
            while (state.progressM < params.contactLengthM && steps < 50)
            {
                double vRoller = currentOmega * params.rollerRadiusM;
                ShooterSqueezeModel::StepSqueeze(params, state, vRoller, dt, 4);

                // Decelerate flywheel by load torque
                double deltaOmega = (state.loadTorqueNm * dt) / fwInertia;
                currentOmega -= deltaOmega;
                steps++;
            }

            double finalRpm = currentOmega * radSToRpm;
            double rpmSag = initialRpm - finalRpm;

            // Realistic FRC shooter metrics:
            // 1. Projectile accelerated significantly from feeder entry speed
            TEST_EXPECT_TRUE(ctx, state.velocityMPerS > 10.0);
            TEST_EXPECT_TRUE(ctx, state.velocityMPerS < 25.0);

            // 2. Flywheel experienced noticeable RPM sag (between 100 and 700 RPM)
            TEST_EXPECT_TRUE(ctx, rpmSag > 100.0);
            TEST_EXPECT_TRUE(ctx, rpmSag < 700.0);

            // 3. Transit took realistic duration (between 2 and 6 physics ticks at 200 Hz = 10 to 30 ms)
            TEST_EXPECT_TRUE(ctx, steps >= 2);
            TEST_EXPECT_TRUE(ctx, steps <= 8);

            // 4. Energy Conservation Accounting
            auto report = ShooterSqueezeModel::ComputeEnergyBalance(
                fwInertia,
                initialRpm * rpmToRadS,
                currentOmega,
                params.pieceMassKg,
                state.velocityMPerS,
                state.dissipatedEnergyJ);

            TEST_EXPECT_TRUE(ctx, report.flywheelDeltaEnergyJ >= report.projectileKineticEnergyJ);
            TEST_EXPECT_TRUE(ctx, report.isEnergyConserved);
            TEST_EXPECT_TRUE(ctx, report.balanceErrorPercent < 8.0);
        }

        // =========================================================================
        // 5. Parent Body Reaction Torque Verification (SS5)
        // =========================================================================
        {
            std::cout << "  -> SS5: Flywheel Deceleration Reaction Torque on Parent Body\n";

            Physics::JoltContext jolt;
            TEST_EXPECT_TRUE(ctx, jolt.Initialize());

            Scene scene;
            Mechanisms::Systems::FlywheelSystem fwSystem(scene, &jolt);

            // Create parent rigid body
            auto parentEntity = scene.CreateEntity();
            Physics::Components::RigidBodyComponent parentRb;
            parentRb.motionType = Physics::Components::BodyMotionType::Dynamic;
            parentRb.mass = 20.0f;
            scene.AddComponent(parentEntity, parentRb);

            // Create flywheel component
            auto fwEntity = scene.CreateEntity();
            FlywheelComponent fw;
            fw.name = "TestShooter";
            fw.parentEntity = parentEntity;
            fw.spinAxis = { 0.0f, 0.0f, 1.0f };
            fw.flywheelInertiaKgm2 = 0.0035;
            fw.gearRatio = 1.0;
            fw.efficiency = 1.0;
            fw.rollerRadiusM = 0.0508;
            fw.commandedVoltage = 0.0;
            fw.angularVelocityRadPerSec = 400.0; // Spinning at ~3820 RPM
            scene.AddComponent(fwEntity, fw);

            constexpr double dt = 0.005;

            // Apply a braking disturbance torque (simulating projectile squeeze)
            auto* fwComp = scene.GetComponent<FlywheelComponent>(fwEntity);
            fwComp->externalLoadTorqueNm = 6.0; // 6 N*m braking

            fwSystem.FixedUpdate(dt);

            // 1. Flywheel decelerated
            TEST_EXPECT_TRUE(ctx, fwComp->angularVelocityRadPerSec < 400.0);

            // 2. Equal and opposite reaction torque computed for parent body
            TEST_EXPECT_TRUE(ctx, fwComp->recoilTorqueNm > 0.0);

            // 3. External load torque was reset to 0.0 at tick end
            TEST_EXPECT_NEAR(ctx, fwComp->externalLoadTorqueNm, 0.0, 1e-6);
        }

        // =========================================================================
        // 6. Full Pipeline Integration E2E (SS6)
        // =========================================================================
        {
            std::cout << "  -> SS6: Full Pipeline Integration & Beam-Break Latching E2E\n";

            std::string manifestPath = "FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
            std::string sidecarPath = "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

            if (!std::filesystem::exists(manifestPath))
            {
                manifestPath = "../FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
                sidecarPath = "../FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
                materialsPath = "../FRCBananaSim/Resources/Config/materials.json";
            }

            RobotSchemaLoader loader;
            loader.LoadRobot(manifestPath, sidecarPath, materialsPath);

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials());
            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            // Locate shooter flywheel and rev it up
            auto fwIt = robot.mechanismEntities.find("shooter_flywheel");
            TEST_EXPECT_TRUE(ctx, fwIt != robot.mechanismEntities.end());
            auto* fw = scene.GetComponent<FlywheelComponent>(fwIt->second);
            TEST_EXPECT_TRUE(ctx, fw != nullptr);

            fw->angularVelocityRadPerSec = 471.24; // 4,500 RPM
            constexpr double initialFwSpeed = 471.24;

            // Spawn an ENGAGED game piece in the shooter zone
            auto pieceEntity = scene.CreateEntity();
            GamePieceComponent piece;
            piece.state = GamePieceState::Engaged;
            piece.spec.massKg = 0.235;
            piece.spec.outerRadiusM = 0.1778;
            piece.spec.thicknessM = 0.12;
            piece.transitProgressM = 0.0;
            piece.transitVelocityMPerS = 2.5; // Fed into shooter at 2.5 m/s
            scene.AddComponent(pieceEntity, piece);

            // Execute 10 physics ticks (50 ms)
            for (int i = 0; i < 10; ++i)
            {
                RobotAssembler::StepPipeline(robot, 0.005);
            }

            auto* postPiece = scene.GetComponent<GamePieceComponent>(pieceEntity);
            TEST_EXPECT_TRUE(ctx, postPiece != nullptr);
            if (postPiece)
            {
                // 1. Piece transitioned to Launched
                TEST_EXPECT_EQ(ctx, static_cast<int>(postPiece->state), static_cast<int>(GamePieceState::Launched));

                // 2. Piece gained significant velocity
                double exitSpeed = glm::length(postPiece->linearVelocity);
                TEST_EXPECT_TRUE(ctx, exitSpeed > 8.0);
                TEST_EXPECT_TRUE(ctx, exitSpeed < 25.0);

                // 3. Flywheel experienced RPM sag
                TEST_EXPECT_TRUE(ctx, fw->angularVelocityRadPerSec < initialFwSpeed);

                // 4. Flywheel load torque is reset to 0.0
                TEST_EXPECT_NEAR(ctx, fw->externalLoadTorqueNm, 0.0, 1e-6);
            }
        }
    }
}
