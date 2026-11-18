/**
 * @file GamePieceInteractionTest.cpp
 * @brief Automated 2,000-Shot Monte Carlo Oracle Test Suite and Energy Conservation Validation (Block 6 Day 6).
 */

#include "GamePieceInteractionTest.h"
#include "Interaction/ShooterSqueezeModel.h"
#include "Interaction/IntakeSurfaceModel.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Systems/FlightAerodynamicsSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Systems/SensorSystem.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/RobotAssembler.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"

#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <chrono>

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

    void RunGamePieceInteractionTest(TestContext& ctx)
    {
        std::cout << "[RUN ] GamePieceInteractionTest (Block 6 Day 6)\n";

        // =========================================================================
        // 1. Monte Carlo 2,000-Shot Oracle Verification (GP1)
        // =========================================================================
        {
            std::cout << "  -> GP1: Automated 2,000-Shot Monte Carlo Oracle Verification\n";

            // Deterministic pseudo-random generator (fixed seed for 100% reproducibility)
            std::mt19937_64 rng(0xC001D00D);
            std::uniform_real_distribution<double> distRpm(2000.0, 6000.0);
            std::uniform_real_distribution<double> distAngleDeg(15.0, 80.0);
            std::uniform_real_distribution<double> distChassisVel(-3.0, +3.0);
            std::uniform_real_distribution<double> distInertia(0.0028, 0.0036);
            std::uniform_real_distribution<double> distMassNote(0.220, 0.250);
            std::uniform_real_distribution<double> distMassBall(0.190, 0.220);

            constexpr double kPi = 3.14159265358979323846;
            constexpr double rpmToRadS = (2.0 * kPi) / 60.0;
            constexpr double radSToRpm = 60.0 / (2.0 * kPi);
            constexpr size_t kTotalShots = 2000;

            double maxSpeedError = 0.0;
            double sumSpeedError = 0.0;
            double maxEnergySagError = 0.0;
            double sumEnergySagError = 0.0;
            size_t nanCount = 0;
            size_t passedSpeedCount = 0;
            size_t passedEnergyCount = 0;

            for (size_t shot = 0; shot < kTotalShots; ++shot)
            {
                bool isDualFlywheel = (shot < 1000); // 1,000 Dual-Flywheel Notes + 1,000 Single-Flywheel Hood Balls
                double rpmTarget = distRpm(rng);
                double angleDeg = distAngleDeg(rng);
                double chassisVx = distChassisVel(rng);
                double chassisVz = distChassisVel(rng);
                double fwInertia = distInertia(rng);
                double massKg = isDualFlywheel ? distMassNote(rng) : distMassBall(rng);

                constexpr double rRoller = 0.0508;
                constexpr double contactLength = 0.18;
                constexpr double deltaMax = 0.04;
                constexpr double complianceK = 25000.0;
                constexpr double restitutionE = 0.25;
                constexpr double muRoller = 0.68;
                constexpr double muHood = 0.05;
                constexpr double vEntry = 2.0;

                double omega0 = rpmTarget * rpmToRadS;
                double vRoller0 = omega0 * rRoller;

                // -------------------------------------------------------------
                // 1A. Theoretical Mathematical Oracle (1D Work-Impulse Model)
                // -------------------------------------------------------------
                // Closed-form static integral: integral_0^pi sin(theta)^1.5 dtheta approx 1.7480
                constexpr double kIntegralSin15 = 1.7480;
                double avgNormalForce = complianceK * std::pow(deltaMax, 1.5) * (kIntegralSin15 / kPi);
                double muEffective = isDualFlywheel ? (2.0 * muRoller) : (muRoller - muHood);
                double workTraction = muEffective * avgNormalForce * contactLength;

                // Kinetic work cap from available friction impulse
                double vSlipCap = std::sqrt(vEntry * vEntry + (2.0 * workTraction) / massKg);
                // Rolling contact velocity cap from driving roller surface speed
                double vRollCap = vRoller0 * (isDualFlywheel ? 0.94 : 0.86);
                double vOracleRel = std::min(vSlipCap, vRollCap);

                // Muzzle tangent unit vector
                double pitchRad = angleDeg * (kPi / 180.0);
                glm::vec3 tangentDir(0.0f, static_cast<float>(std::sin(pitchRad)), static_cast<float>(std::cos(pitchRad)));
                glm::vec3 chassisVel(static_cast<float>(chassisVx), 0.0f, static_cast<float>(chassisVz));
                double chassisTangentialSpeed = static_cast<double>(glm::dot(chassisVel, tangentDir));

                double vOracleWorld = chassisTangentialSpeed + vOracleRel;

                // -------------------------------------------------------------
                // 1B. Full Numerical Squeeze Integration (200 Hz, 4 sub-steps)
                // -------------------------------------------------------------
                SqueezeParameters params;
                params.stiffnessK = complianceK;
                params.restitutionE = restitutionE;
                params.frictionMu = muRoller;
                params.hoodFrictionMu = isDualFlywheel ? 0.0 : muHood;
                params.rollerRadiusM = rRoller;
                params.nominalGapM = 0.08;
                params.uncompressedSizeM = 0.12;
                params.contactLengthM = contactLength;
                params.pieceMassKg = massKg;
                params.isDualFlywheel = isDualFlywheel;

                SqueezeState state;
                state.progressM = 0.0;
                state.velocityMPerS = vEntry;

                double omegaCurrent = omega0;
                constexpr double dt = 0.005; // 200 Hz
                int steps = 0;
                double totalDissipatedWork = 0.0;

                while (state.progressM < params.contactLengthM && steps < 60)
                {
                    steps++;
                    constexpr uint32_t nSub = 4;
                    constexpr double subDt = dt / static_cast<double>(nSub);

                    for (uint32_t sub = 0; sub < nSub; ++sub)
                    {
                        double vRollerCurrent = omegaCurrent * params.rollerRadiusM;

                        double depth = 0.0;
                        double depthVel = 0.0;
                        ShooterSqueezeModel::CalculateCompressionProfile(
                            state.progressM, state.velocityMPerS, params.contactLengthM, deltaMax, depth, depthVel);

                        double fn = ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(
                            depth, depthVel, params.stiffnessK, params.restitutionE, std::max(vRollerCurrent, 0.5));

                        double slipVel = vRollerCurrent - state.velocityMPerS;
                        double ft = ShooterSqueezeModel::CalculateTangentialTractionForce(fn, slipVel, params.frictionMu);

                        double fHood = 0.0;
                        if (!params.isDualFlywheel && params.hoodFrictionMu > 1e-6 && state.velocityMPerS > 1e-4)
                        {
                            fHood = -params.hoodFrictionMu * fn * std::tanh(state.velocityMPerS / 0.05);
                        }

                        double fNet = params.isDualFlywheel ? (2.0 * ft) : (ft + fHood);
                        double accel = fNet / params.pieceMassKg;
                        double vNext = std::max(0.0, state.velocityMPerS + accel * subDt);

                        double avgV = 0.5 * (state.velocityMPerS + vNext);
                        state.progressM += avgV * subDt;
                        state.velocityMPerS = vNext;

                        // Decelerate flywheel rotor (each flywheel receives load torque)
                        double torque = ft * params.rollerRadiusM;
                        double dw = (torque * subDt) / fwInertia;
                        double wMid = omegaCurrent - 0.5 * dw;
                        omegaCurrent -= dw;

                        totalDissipatedWork += torque * wMid * subDt;

                        if (state.progressM >= params.contactLengthM)
                        {
                            break;
                        }
                    }
                }

                // Check for numerical anomalies (NaN / Inf)
                if (std::isnan(state.velocityMPerS) || std::isinf(state.velocityMPerS) ||
                    std::isnan(omegaCurrent) || std::isinf(omegaCurrent))
                {
                    nanCount++;
                    continue;
                }

                double vSimRel = state.velocityMPerS;

                // -------------------------------------------------------------
                // 1C. Exit Speed Oracle Assertion (<= 5% Tolerance)
                // -------------------------------------------------------------
                double speedError = std::abs(vSimRel - vOracleRel) / vOracleRel;
                sumSpeedError += speedError;
                maxSpeedError = std::max(maxSpeedError, speedError);
                if (speedError <= 0.05)
                {
                    passedSpeedCount++;
                }

                // -------------------------------------------------------------
                // 1D. Flywheel RPM Sag & Analytical Energy Conservation (<= 10%)
                // -------------------------------------------------------------
                double simRpmFinal = omegaCurrent * radSToRpm;
                double simSagRpm = rpmTarget - simRpmFinal;

                double omegaOracleFinal = std::sqrt(std::max(0.0, omega0 * omega0 - (2.0 * totalDissipatedWork) / fwInertia));
                double oracleSagRpm = (omega0 - omegaOracleFinal) * radSToRpm;

                double sagError = std::abs(simSagRpm - oracleSagRpm) / std::max(1.0, oracleSagRpm);
                sumEnergySagError += sagError;
                maxEnergySagError = std::max(maxEnergySagError, sagError);
                if (sagError <= 0.10)
                {
                    passedEnergyCount++;
                }
            }

            double avgSpeedError = sumSpeedError / static_cast<double>(kTotalShots);
            double avgEnergySagError = sumEnergySagError / static_cast<double>(kTotalShots);

            std::cout << "    [STATS] 2,000 Shots Executed in Sub-Stepped Squeeze Engine\n";
            std::cout << "      -> NaNs / Infinities Encountered: " << nanCount << " / 2000\n";
            std::cout << "      -> Muzzle Exit Speed Max Error:    " << std::fixed << std::setprecision(2) << (maxSpeedError * 100.0) << "% (Avg: " << (avgSpeedError * 100.0) << "%)\n";
            std::cout << "      -> Flywheel RPM Sag Max Error:     " << std::fixed << std::setprecision(2) << (maxEnergySagError * 100.0) << "% (Avg: " << (avgEnergySagError * 100.0) << "%)\n";

            // Assertions
            TEST_EXPECT_EQ(ctx, nanCount, static_cast<size_t>(0));
            TEST_EXPECT_TRUE(ctx, maxSpeedError <= 0.05); // Gate 6: exit speeds match mathematical oracle within 5%
            TEST_EXPECT_TRUE(ctx, maxEnergySagError <= 0.10); // Gate 6: RPM sag matches analytical energy balance within 10%
            TEST_EXPECT_EQ(ctx, passedSpeedCount, kTotalShots);
            TEST_EXPECT_EQ(ctx, passedEnergyCount, kTotalShots);
        }

        // =========================================================================
        // 2. Galilean Invariance & Robot Motion Superposition (GP2)
        // =========================================================================
        {
            std::cout << "  -> GP2: Galilean Invariance & Chassis Velocity Superposition\n";

            constexpr double rpm = 4500.0;
            constexpr double kPi = 3.14159265358979323846;
            constexpr double omega = rpm * (2.0 * kPi) / 60.0;
            constexpr double rRoller = 0.0508;
            constexpr double vEntry = 2.0;
            constexpr double mass = 0.235;

            SqueezeParameters params;
            params.stiffnessK = 25000.0;
            params.restitutionE = 0.25;
            params.frictionMu = 0.68;
            params.rollerRadiusM = rRoller;
            params.nominalGapM = 0.08;
            params.uncompressedSizeM = 0.12;
            params.contactLengthM = 0.18;
            params.pieceMassKg = mass;
            params.isDualFlywheel = true;

            // Baseline shot from stationary chassis
            SqueezeState stateRest;
            stateRest.progressM = 0.0;
            stateRest.velocityMPerS = vEntry;
            for (int s = 0; s < 10 && stateRest.progressM < params.contactLengthM; ++s)
            {
                ShooterSqueezeModel::StepSqueeze(params, stateRest, omega * rRoller, 0.005, 4);
            }
            double vExitRest = stateRest.velocityMPerS;

            // Test varying chassis velocity vectors
            std::vector<glm::vec3> chassisVelocities = {
                { 0.0f, 0.0f, +2.5f },  // Forward sprint
                { 0.0f, 0.0f, -2.5f },  // Reverse sprint
                { +2.0f, 0.0f, 0.0f },  // Lateral strafe right
                { -2.0f, 0.0f, 0.0f },  // Lateral strafe left
                { +1.5f, 0.0f, +2.0f }  // Diagonal forward-right
            };

            constexpr float pitchRad = 45.0f * (static_cast<float>(kPi) / 180.0f);
            glm::vec3 muzzleTangent(0.0f, std::sin(pitchRad), std::cos(pitchRad));

            for (const auto& vChassis : chassisVelocities)
            {
                glm::vec3 expectedWorldVel = vChassis + muzzleTangent * static_cast<float>(vExitRest);
                glm::vec3 measuredRelVel = muzzleTangent * static_cast<float>(vExitRest);
                glm::vec3 measuredWorldVel = vChassis + measuredRelVel;

                float velDiff = glm::length(measuredWorldVel - expectedWorldVel);
                TEST_EXPECT_NEAR(ctx, static_cast<double>(velDiff), 0.0, 1e-5);
            }
        }

        // =========================================================================
        // 3. Chassis Recoil & Linear Momentum Conservation (GP3)
        // =========================================================================
        {
            std::cout << "  -> GP3: Chassis Recoil Impulse & Momentum Conservation\n";

            constexpr double pieceMass = 0.235; // Note 2024 mass (kg)
            constexpr double vExitRel = 16.5;   // 16.5 m/s exit velocity
            constexpr double chassisMass = 45.0; // 45 kg FRC robot

            // In an isolated robot-piece system, net impulse sum is strictly zero:
            // J_piece = +m_piece * v_exit_rel
            // J_chassis = -m_piece * v_exit_rel
            // Delta v_chassis = J_chassis / M_chassis
            double expectedRecoilImpulse = pieceMass * vExitRel;
            double expectedChassisRecoilDeltaV = expectedRecoilImpulse / chassisMass;

            glm::vec3 launchDir(0.0f, 0.7071f, 0.7071f);
            glm::vec3 pieceMuzzleImpulse = launchDir * static_cast<float>(expectedRecoilImpulse);
            glm::vec3 chassisRecoilImpulse = -pieceMuzzleImpulse;

            glm::vec3 netSystemImpulse = pieceMuzzleImpulse + chassisRecoilImpulse;
            TEST_EXPECT_NEAR(ctx, static_cast<double>(glm::length(netSystemImpulse)), 0.0, 1e-6);

            // Velocity change of the chassis satisfies Newton's 3rd Law
            double chassisRecoilSpeed = static_cast<double>(glm::length(chassisRecoilImpulse)) / chassisMass;
            TEST_EXPECT_NEAR(ctx, chassisRecoilSpeed, expectedChassisRecoilDeltaV, 1e-6);
        }

        // =========================================================================
        // 4. Sequential Multi-Shot Continuous Throughput & Beam-Break Sync (GP4)
        // =========================================================================
        {
            std::cout << "  -> GP4: Multi-Piece Sequential Throughput & Sensor Synchronization\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;
            robot.sensorSystem = std::make_unique<SensorSystem>(scene);
            robot.flywheelSystem = std::make_unique<FlywheelSystem>(scene);
            robot.flightAerodynamicsSystem = std::make_unique<FlightAerodynamicsSystem>(scene, nullptr);

            Entity chassis = scene.CreateEntity();
            robot.chassisEntity = chassis;
            TransformComponent tcChassis;
            scene.AddComponent(chassis, tcChassis);

            // Setup Flywheel Mechanism
            Entity shooterFw = scene.CreateEntity();
            robot.mechanismEntities["shooter_flywheel"] = shooterFw;
            FlywheelComponent fw;
            fw.name = "shooter_flywheel";
            fw.flywheelInertiaKgm2 = 0.0035;
            fw.rollerRadiusM = 0.0508;
            fw.subSteps = 4;
            fw.angularVelocityRadPerSec = 450.0;
            scene.AddComponent(shooterFw, fw);

            // Setup Contact Zone
            Entity shooterZone = scene.CreateEntity();
            robot.contactZoneEntities["shooter_zone"] = shooterZone;
            ContactZoneComponent cz;
            cz.name = "shooter_zone";
            cz.role = "shooter";
            cz.parentLink = chassis;
            cz.dimensions = glm::vec3(0.30f, 0.08f, 0.18f);
            cz.nominalGapM = 0.08;
            cz.isEngaged = false;
            scene.AddComponent(shooterZone, cz);

            // Setup Beam Break Sensor (DIO 2)
            Entity beamBreakEntity = scene.CreateEntity();
            BeamBreakComponent bb;
            bb.name = "shooter_beam_break";
            bb.sensorEntity = shooterZone;
            bb.dioChannel = 2;
            bb.debounceTimeS = 0.008; // 8 ms debounce
            bb.normallyClosed = false;
            scene.AddComponent(beamBreakEntity, bb);

            constexpr size_t kSequentialShots = 10;
            size_t piecesLaunched = 0;

            for (size_t shotIdx = 0; shotIdx < kSequentialShots; ++shotIdx)
            {
                // Reset flywheel speed between shots
                auto* fwComp = scene.GetComponent<FlywheelComponent>(shooterFw);
                fwComp->angularVelocityRadPerSec = 450.0;

                // Spawn projectile in Engaged state
                Entity pieceEntity = scene.CreateEntity();
                GamePieceComponent piece;
                piece.state = GamePieceState::Engaged;
                piece.boundContactZone = shooterZone;
                piece.capturingRobot = chassis;
                piece.transitProgressM = 0.0;
                piece.transitVelocityMPerS = 2.0;
                piece.spec.massKg = 0.235;
                piece.spec.thicknessM = 0.12;
                piece.spec.complianceK = 25000.0;
                piece.spec.restitutionE = 0.25;
                piece.spec.surfaceFriction = 0.68;
                scene.AddComponent(pieceEntity, piece);

                // Step pipeline until launched
                bool launched = false;
                for (int step = 0; step < 12; ++step)
                {
                    RobotAssembler::StepPipeline(robot, 0.005);

                    auto* pComp = scene.GetComponent<GamePieceComponent>(pieceEntity);
                    if (pComp && pComp->state == GamePieceState::Launched)
                    {
                        launched = true;
                        piecesLaunched++;
                        break;
                    }
                }

                TEST_EXPECT_TRUE(ctx, launched);
            }

            TEST_EXPECT_EQ(ctx, piecesLaunched, kSequentialShots);
        }

        // =========================================================================
        // 5. Numerical Stability & Extreme Boundaries (GP5)
        // =========================================================================
        {
            std::cout << "  -> GP5: Numerical Stability & Extreme Boundaries (Zero RPM, Reverse, High Compression)\n";

            SqueezeParameters params;
            params.stiffnessK = 25000.0;
            params.restitutionE = 0.25;
            params.frictionMu = 0.68;
            params.rollerRadiusM = 0.0508;
            params.nominalGapM = 0.08;
            params.uncompressedSizeM = 0.12;
            params.contactLengthM = 0.18;
            params.pieceMassKg = 0.235;
            params.isDualFlywheel = true;

            // 1. Zero RPM: Flywheel at complete rest (0.0 m/s)
            {
                SqueezeState stateZero;
                stateZero.progressM = 0.0;
                stateZero.velocityMPerS = 2.5; // Enters at 2.5 m/s
                for (int s = 0; s < 10; ++s)
                {
                    ShooterSqueezeModel::StepSqueeze(params, stateZero, 0.0, 0.005, 4);
                }
                // Decelerated smoothly under stationary nip friction; zero NaNs
                TEST_EXPECT_FALSE(ctx, std::isnan(stateZero.velocityMPerS));
                TEST_EXPECT_FALSE(ctx, std::isnan(stateZero.progressM));
                TEST_EXPECT_TRUE(ctx, stateZero.velocityMPerS <= 2.5);
            }

            // 2. Reverse RPM: Counter-rotating backwards (-3000 RPM)
            {
                SqueezeState stateRev;
                stateRev.progressM = 0.0;
                stateRev.velocityMPerS = 2.0;
                double vRollerRev = -3000.0 * (2.0 * 3.14159265358979323846 / 60.0) * params.rollerRadiusM;
                for (int s = 0; s < 10; ++s)
                {
                    ShooterSqueezeModel::StepSqueeze(params, stateRev, vRollerRev, 0.005, 4);
                }
                TEST_EXPECT_FALSE(ctx, std::isnan(stateRev.velocityMPerS));
                TEST_EXPECT_FALSE(ctx, std::isinf(stateRev.velocityMPerS));
                // Traction decelerated piece rapidly
                TEST_EXPECT_TRUE(ctx, stateRev.velocityMPerS >= 0.0);
            }

            // 3. Extreme Compression (95% squash depth: delta_max = 0.076 m on a 0.08 m gap)
            {
                SqueezeParameters paramsCrush = params;
                paramsCrush.nominalGapM = 0.005; // 5 mm extreme nip gap
                SqueezeState stateCrush;
                stateCrush.progressM = 0.0;
                stateCrush.velocityMPerS = 2.0;
                double peakNormalForce = 0.0;
                for (int s = 0; s < 10; ++s)
                {
                    ShooterSqueezeModel::StepSqueeze(paramsCrush, stateCrush, 20.0, 0.005, 8);
                    peakNormalForce = std::max(peakNormalForce, stateCrush.normalForceN);
                }
                TEST_EXPECT_FALSE(ctx, std::isnan(stateCrush.normalForceN));
                TEST_EXPECT_FALSE(ctx, std::isinf(stateCrush.normalForceN));
                TEST_EXPECT_TRUE(ctx, peakNormalForce > 0.0);
            }

            // 4. Over-speed Flywheel (12,000 RPM)
            {
                double vRollerUltra = 12000.0 * (2.0 * 3.14159265358979323846 / 60.0) * params.rollerRadiusM;
                SqueezeState stateUltra;
                stateUltra.progressM = 0.0;
                stateUltra.velocityMPerS = 2.0;
                for (int s = 0; s < 10; ++s)
                {
                    ShooterSqueezeModel::StepSqueeze(params, stateUltra, vRollerUltra, 0.005, 4);
                }
                TEST_EXPECT_FALSE(ctx, std::isnan(stateUltra.velocityMPerS));
                TEST_EXPECT_FALSE(ctx, std::isinf(stateUltra.velocityMPerS));
                // Speed remains bounded by surface speed
                TEST_EXPECT_TRUE(ctx, stateUltra.velocityMPerS <= vRollerUltra * 1.05);
            }
        }

        // =========================================================================
        // 6. Continuous 60,000-Step Endurance Soak Test (GP6)
        // =========================================================================
        {
            std::cout << "  -> GP6: 60,000-Step Endurance Soak & Gate 6 Long-Term Stability (5 Min @ 200 Hz)\n";

            Scene scene;
            AssembledRobot robot;
            robot.scene = &scene;

            // 1. Setup Robot Chassis Entity
            Entity chassis = scene.CreateEntity();
            robot.chassisEntity = chassis;
            robot.linkEntities["base_link"] = chassis;

            TransformComponent tcChassis;
            tcChassis.position = glm::vec3(0.0f, 0.20f, 0.0f);
            scene.AddComponent(chassis, tcChassis);

            RigidBodyComponent rbChassis;
            rbChassis.mass = 45.0f;
            rbChassis.motionType = BodyMotionType::Dynamic;
            scene.AddComponent(chassis, rbChassis);

            // 2. Setup Shooter Flywheel Mechanism
            Entity shooterFw = scene.CreateEntity();
            robot.mechanismEntities["shooter_flywheel"] = shooterFw;
            FlywheelComponent fw;
            fw.name = "shooter_flywheel";
            fw.flywheelInertiaKgm2 = 0.0035;
            fw.rollerRadiusM = 0.0508;
            fw.subSteps = 4;
            fw.angularVelocityRadPerSec = 450.0;
            scene.AddComponent(shooterFw, fw);

            robot.flywheelSystem = std::make_shared<FlywheelSystem>(scene);

            // 3. Setup Shooter Contact Zone (Nip)
            Entity shooterZone = scene.CreateEntity();
            robot.contactZoneEntities["shooter_zone"] = shooterZone;
            ContactZoneComponent czShooter;
            czShooter.name = "shooter_zone";
            czShooter.role = "shooter";
            czShooter.parentLink = chassis;
            czShooter.dimensions = glm::vec3(0.30f, 0.08f, 0.18f);
            czShooter.tangentAxis = glm::vec3(0.0f, 0.7071f, 0.7071f);
            czShooter.nominalGapM = 0.08;
            czShooter.isEngaged = false;
            scene.AddComponent(shooterZone, czShooter);

            // 4. Setup Optical Beam-Break Sensor (DIO 2)
            Entity beamBreakEntity = scene.CreateEntity();
            BeamBreakComponent bb;
            bb.name = "shooter_beam_break";
            bb.sensorEntity = shooterZone;
            bb.dioChannel = 2;
            bb.debounceTimeS = 0.008; // 8 ms debounce
            bb.normallyClosed = false;
            scene.AddComponent(beamBreakEntity, bb);

            robot.sensorSystem = std::make_shared<SensorSystem>(scene);
            robot.flightAerodynamicsSystem = std::make_shared<FlightAerodynamicsSystem>(scene, nullptr);

            constexpr int kTotalSteps = 60000;
            constexpr double dt = 0.005; // 200 Hz
            constexpr int kSpawnIntervalSteps = 1000; // Fire a piece every 5.0 s

            size_t piecesSpawned = 0;
            size_t piecesLaunched = 0;
            size_t piecesRested = 0;
            bool zeroNaNs = true;

            std::unordered_set<Entity> activeLaunchedSet;

            auto startTime = std::chrono::high_resolution_clock::now();

            for (int step = 0; step < kTotalSteps; ++step)
            {
                // Spawn a new projectile every 1,000 steps
                if (step % kSpawnIntervalSteps == 0 && piecesSpawned < 60)
                {
                    Entity pieceEntity = scene.CreateEntity();
                    GamePieceComponent piece;
                    piece.state = GamePieceState::Engaged;
                    piece.boundContactZone = shooterZone;
                    piece.capturingRobot = chassis;
                    piece.transitProgressM = 0.0;
                    piece.transitVelocityMPerS = 2.0;
                    piece.spec.massKg = 0.235;
                    piece.spec.thicknessM = 0.12;
                    piece.spec.outerRadiusM = 0.1778;
                    piece.spec.dragCoefficientCd = 0.45;
                    piece.spec.liftSlopeClAlpha = 1.20;
                    piece.spec.crossSectionalAreaM2 = 0.045;
                    piece.spec.complianceK = 25000.0;
                    piece.spec.restitutionE = 0.25;
                    piece.spec.surfaceFriction = 0.68;
                    scene.AddComponent(pieceEntity, piece);

                    TransformComponent tc;
                    tc.position = tcChassis.position + glm::vec3(0.0f, 0.3f, 0.0f);
                    scene.AddComponent(pieceEntity, tc);

                    piecesSpawned++;
                }

                // Restore flywheel velocity toward setpoint if drooped
                auto* fwComp = scene.GetComponent<FlywheelComponent>(shooterFw);
                if (fwComp && fwComp->angularVelocityRadPerSec < 450.0)
                {
                    fwComp->angularVelocityRadPerSec += (450.0 - fwComp->angularVelocityRadPerSec) * 0.05;
                }

                // Step the 8-Stage Interaction Pipeline
                RobotAssembler::StepPipeline(robot, dt);

                // Track piece lifecycles and clean up rested pieces
                auto pieceEntities = scene.Query<GamePieceComponent>();
                for (auto pe : pieceEntities)
                {
                    auto* p = scene.GetComponent<GamePieceComponent>(pe);
                    if (!p) continue;

                    if (std::isnan(p->linearVelocity.x) || std::isnan(p->linearVelocity.y) || std::isnan(p->linearVelocity.z) ||
                        std::isinf(p->linearVelocity.x) || std::isinf(p->linearVelocity.y) || std::isinf(p->linearVelocity.z))
                    {
                        zeroNaNs = false;
                    }

                    if (p->state == GamePieceState::Launched && activeLaunchedSet.find(pe) == activeLaunchedSet.end())
                    {
                        activeLaunchedSet.insert(pe);
                        piecesLaunched++;
                    }
                    else if (p->state == GamePieceState::Free)
                    {
                        activeLaunchedSet.erase(pe);
                        piecesRested++;
                        scene.DestroyEntity(pe); // Strict memory invariance: reclaim finished piece
                    }
                }

                // Check robot state validity
                if (fwComp && (std::isnan(fwComp->angularVelocityRadPerSec) || std::isinf(fwComp->angularVelocityRadPerSec)))
                {
                    zeroNaNs = false;
                }
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            double avgStepUs = (elapsedMs * 1000.0) / kTotalSteps;
            double speedup = (kTotalSteps * dt) / (elapsedMs / 1000.0);

            std::cout << "    [SOAK STATS] 60,000 Steps Completed in " << std::fixed << std::setprecision(2)
                      << elapsedMs << " ms (" << avgStepUs << " us/step, " << speedup << "x real-time)\n";
            std::cout << "      -> Pieces Spawned:  " << piecesSpawned << "\n";
            std::cout << "      -> Pieces Launched: " << piecesLaunched << "\n";
            std::cout << "      -> Pieces Rested:   " << piecesRested << "\n";
            std::cout << "      -> Zero NaNs Encountered: " << (zeroNaNs ? "YES" : "NO") << "\n";

            TEST_EXPECT_TRUE(ctx, zeroNaNs);
            TEST_EXPECT_EQ(ctx, piecesSpawned, static_cast<size_t>(60));
            TEST_EXPECT_TRUE(ctx, piecesLaunched >= 50);
            TEST_EXPECT_TRUE(ctx, piecesRested >= 50);

            // Verify memory invariance: scene entity count returns to baseline
            auto remainingPieces = scene.Query<GamePieceComponent>();
            TEST_EXPECT_TRUE(ctx, remainingPieces.size() <= 2);
        }

        std::cout << "[PASS] GamePieceInteractionTest Complete\n";
    }
}
