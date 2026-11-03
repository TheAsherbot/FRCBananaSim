/**
 * @file FullRobotEnduranceTest.cpp
 * @brief Headless 5-minute (60,000 steps at 200 Hz) endurance test for the full Schema v1 Reference 2026 robot.
 */

#include "FullRobotEnduranceTest.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/RobotAssembler.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Components/FourBarComponent.h"
#include "Control/SimControlBoundary.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace FRCBananaSim::Tests
{
    using namespace FRCBananaSim::Core::ECS::Core;
    using namespace FRCBananaSim::Core::ECS::Components;
    using namespace FRCBananaSim::Physics::Components;
    using namespace FRCBananaSim::Robot::Mechanisms::Components;
    using namespace FRCBananaSim::Control;
    using namespace FRCBananaSim::Robot;

    void RunFullRobotEnduranceTest(TestContext& ctx)
    {
        std::cout << "\n========================================\n";
        std::cout << " Test Suite: Full Robot Endurance (FE1)\n";
        std::cout << "========================================\n";

        // Locate Schema v1 reference robot resource paths
        std::string manifestPath = "FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
        std::string sidecarPath = "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
        std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

        if (!std::filesystem::exists(manifestPath))
        {
            manifestPath = "../FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
            sidecarPath = "../FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            materialsPath = "../FRCBananaSim/Resources/Config/materials.json";
        }

        std::cout << "  -> FE1: Full 2026 Reference Robot 5-Minute Endurance (60,000 Steps at 200 Hz)\n";

        Physics::JoltContext jolt;
        TEST_EXPECT_TRUE(ctx, jolt.Initialize());

        Scene scene;
        constexpr double dt = 0.005; // 200 Hz
        scene.SetFixedDeltaTime(dt);

        // 1. Static Ground Carpet
        Entity groundEntity = scene.CreateEntity();
        {
            TransformComponent transform;
            transform.position = glm::vec3(0.0f, -0.05f, 0.0f);
            transform.scale = glm::vec3(28.0f, 0.10f, 18.0f);
            scene.AddComponent(groundEntity, transform);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.85f;
            rb.restitution = 0.05f;
            scene.AddComponent(groundEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(14.0f, 0.05f, 9.0f);
            scene.AddComponent(groundEntity, col);
        }

        // 2. Assemble Full Reference Robot
        RobotAssembler assembler(scene, &jolt);
        const glm::vec3 robotSpawnPos(0.0f, 0.18f, 0.0f);
        auto robot = assembler.AssembleFromFiles(manifestPath, sidecarPath, materialsPath, robotSpawnPos);
        TEST_EXPECT_TRUE(ctx, robot.isSuccess);

        Physics::Systems::JointSystem jointSystem(scene, jolt);

        if (robot.physicsSystem)
        {
            robot.physicsSystem->FixedUpdate(0.0);
        }
        jointSystem.FixedUpdate(0.0);
        jointSystem.ReadbackTelemetry();

        // 3. Control Boundary & Mechanism References
        SimInputs robotInputs;
        robotInputs.robotEnabled = true;
        SimOutputs robotOutputs;
        if (robot.electricalSystem) robot.electricalSystem->SetOutputs(&robotOutputs);
        if (robot.sensorSystem) robot.sensorSystem->SetOutputs(&robotOutputs);
        if (robot.swerveSystem) robot.swerveSystem->SetInputs(robotInputs);

        auto* elevComp = scene.GetComponent<ElevatorComponent>(robot.mechanismEntities["elevator_drive"]);
        auto* shoulderComp = scene.GetComponent<ArmComponent>(robot.mechanismEntities["arm_shoulder"]);
        auto* wristComp = scene.GetComponent<ArmComponent>(robot.mechanismEntities["arm_wrist"]);
        auto* fourBarArm = scene.GetComponent<ArmComponent>(robot.mechanismEntities["intake_fourbar_deploy"]);
        auto* flywheelComp = scene.GetComponent<FlywheelComponent>(robot.mechanismEntities["shooter_flywheel"]);

        auto fourBars = scene.Query<FourBarComponent>();
        auto* fourBarComp = fourBars.empty() ? nullptr : scene.GetComponent<FourBarComponent>(fourBars[0]);
        auto* crankJc = scene.GetComponent<JointComponent>(robot.jointEntities["fourbar_crank_hinge"]);
        auto* folJc = scene.GetComponent<JointComponent>(robot.jointEntities["fourbar_follower_hinge"]);

        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
        TEST_EXPECT_TRUE(ctx, chassisRb != nullptr);

        // 4. Invariant Tracking Variables
        constexpr int kTotalSteps = 60000; // 5 simulated minutes = 300.0 seconds
        bool nanOrInfDetected = false;
        double maxFourBarDrift = 0.0;
        double minBatteryVoltage = 12.60;
        double maxTotalCurrent = 0.0;
        bool brownoutDetected = false;
        double totalElectricalEnergyJoules = 0.0;
        double totalPhysicsDurationUs = 0.0;

        constexpr double kP = 18.0;
        constexpr double kD = 0.6;
        constexpr double kInvSqrt2 = 0.7071067811865475;
        constexpr double modRx[4] = { +kInvSqrt2, +kInvSqrt2, -kInvSqrt2, -kInvSqrt2 };
        constexpr double modRy[4] = { +kInvSqrt2, -kInvSqrt2, +kInvSqrt2, -kInvSqrt2 };

        std::cout << "  Starting 60,000-step continuous dynamic execution loop...\n" << std::flush;

        // 5. Execution Loop (60,000 steps at 200 Hz)
        for (int step = 0; step < kTotalSteps; ++step)
        {
            double simTime = step * dt;
            double cycleTime = std::fmod(simTime, 18.0); // 18-second realistic match cycle

            // Coordinated Match Profile Generation
            double cmdVx = 0.0;
            double cmdVy = 0.0;
            double cmdOmega = 0.0;

            double targetElevH = 0.0;
            double targetShoulderRad = 0.0;
            double targetWristRad = 0.0;
            double targetIntakeRad = 0.0;
            double flywheelVolts = 0.0;

            if (cycleTime < 5.0)
            {
                // Phase 1 (0..5s): Intake & Maneuver (smooth starts from 0 velocity)
                double pFrac = cycleTime / 5.0;
                cmdVx = 1.0 * std::sin(pFrac * 3.14159265);
                cmdVy = 0.6 * std::sin(pFrac * 2.0 * 3.14159265);
                targetIntakeRad = 1.00 * std::sin(pFrac * 1.570796326); // Deploy intake
                targetElevH = 0.0;
                targetShoulderRad = 0.0;
                targetWristRad = 0.0;
                flywheelVolts = 0.0;
            }
            else if (cycleTime < 10.0)
            {
                // Phase 2 (5..10s): Transit & Lift
                double pFrac = (cycleTime - 5.0) / 5.0;
                cmdVx = -0.8 * std::sin(pFrac * 3.14159265);
                cmdVy = 0.5 * std::sin(pFrac * 3.14159265);
                cmdOmega = 0.3 * std::sin(pFrac * 3.14159265);
                targetIntakeRad = 1.00 * (1.0 - std::sin(pFrac * 1.570796326)); // Fold intake back
                targetElevH = 0.45 * std::sin(pFrac * 1.570796326); // Lift to 0.45 m
                targetShoulderRad = 0.50 * std::sin(pFrac * 1.570796326); // Pivot shoulder
                targetWristRad = 0.20 * std::sin(pFrac * 1.570796326); // Tilt wrist
                flywheelVolts = 0.0;
            }
            else if (cycleTime < 14.0)
            {
                // Phase 3 (10..14s): Spin-Up & Score
                cmdVx = 0.0;
                cmdVy = 0.0;
                cmdOmega = 0.0; // Rest steady on target
                targetIntakeRad = 0.0;
                targetElevH = 0.45;
                targetShoulderRad = 0.50;
                targetWristRad = 0.20;
                flywheelVolts = 6.0; // Spin up flywheel at 6V
            }
            else
            {
                // Phase 4 (14..18s): Return Transit & Reset
                double pFrac = (cycleTime - 14.0) / 4.0;
                cmdVx = -0.9 * std::sin(pFrac * 3.14159265);
                cmdVy = -0.5 * std::sin(pFrac * 3.14159265);
                flywheelVolts = 0.0; // Coast
                targetIntakeRad = 0.0;
                targetElevH = 0.45 * (1.0 - std::sin(pFrac * 1.570796326)); // Return to 0
                targetShoulderRad = 0.50 * (1.0 - std::sin(pFrac * 1.570796326));
                targetWristRad = 0.20 * (1.0 - std::sin(pFrac * 1.570796326));
            }

            // A. Swerve Kinematics & Closed-Loop Module Control
            double modVx[4] = { 0.0, 0.0, 0.0, 0.0 };
            double modVy[4] = { 0.0, 0.0, 0.0, 0.0 };
            double modMag[4] = { 0.0, 0.0, 0.0, 0.0 };
            double maxModMag = 0.0;

            for (uint32_t i = 0; i < 4; ++i)
            {
                modVx[i] = cmdVx - (cmdOmega * modRy[i]);
                modVy[i] = cmdVy + (cmdOmega * modRx[i]);
                modMag[i] = std::hypot(modVx[i], modVy[i]);
                maxModMag = std::max(maxModMag, modMag[i]);
            }

            double desaturateScale = (maxModMag > 4.0) ? (4.0 / maxModMag) : 1.0;

            for (uint32_t i = 0; i < 4; ++i)
            {
                double curAngle = robotOutputs.encoderPositions[i];
                double curVel = robotOutputs.encoderVelocities[i];

                double targetAngle = (modMag[i] > 1e-3) ? std::atan2(modVy[i], modVx[i]) : curAngle;
                double driveVolts = (modMag[i] > 1e-3) ? (modMag[i] * desaturateScale) : 0.0;

                double err = targetAngle - curAngle;

                while (err > 3.141592653589793) err -= 6.283185307179586;
                while (err < -3.141592653589793) err += 6.283185307179586;

                if (std::abs(driveVolts) > 1e-3)
                {
                    if (err > 1.5707963267948966)
                    {
                        err -= 3.141592653589793;
                        driveVolts = -driveVolts;
                    }
                    else if (err < -1.5707963267948966)
                    {
                        err += 3.141592653589793;
                        driveVolts = -driveVolts;
                    }
                    driveVolts *= std::cos(err);
                }

                // Suppress drive power while module is aligning to prevent sideways tire scrub stall
                if (std::abs(err) > 0.6)
                {
                    driveVolts = 0.0;
                }

                double steerV = std::clamp((err * kP) - (curVel * kD), -4.0, 4.0);
                driveVolts = std::clamp(driveVolts, -4.0, 4.0);

                robotInputs.motorCommands[i].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[i].demand = steerV;
                robotInputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[4 + i].demand = driveVolts;
            }

            // B. Elevator Control (Gentle lift/lower clamped to +/- 4.0V)
            if (elevComp)
            {
                double err = targetElevH - elevComp->totalCarriageHeightM;
                double v = std::clamp((err * 12.0) - (elevComp->totalCarriageVelocityMps * 2.0), -4.0, 4.0);
                elevComp->commandedVoltage = v;
                robotInputs.motorCommands[8].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[8].demand = v;
            }

            // C. Arm Shoulder Control (Clamped to +/- 3.5V)
            if (shoulderComp)
            {
                double err = targetShoulderRad - shoulderComp->currentAngleRad;
                double v = std::clamp((err * 10.0) - (shoulderComp->currentVelocityRadPerSec * 1.0), -3.5, 3.5);
                shoulderComp->commandedVoltage = v;
                robotInputs.motorCommands[9].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[9].demand = v;
            }

            // D. Arm Wrist Control (Clamped to +/- 2.5V)
            if (wristComp)
            {
                double err = targetWristRad - wristComp->currentAngleRad;
                double v = std::clamp((err * 8.0) - (wristComp->currentVelocityRadPerSec * 0.8), -2.5, 2.5);
                wristComp->commandedVoltage = v;
                robotInputs.motorCommands[10].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[10].demand = v;
            }

            // E. Intake 4-Bar Control (Clamped to +/- 3.0V)
            if (fourBarArm)
            {
                double err = targetIntakeRad - fourBarArm->currentAngleRad;
                double v = std::clamp((err * 10.0) - (fourBarArm->currentVelocityRadPerSec * 0.8), -3.0, 3.0);
                fourBarArm->commandedVoltage = v;
                robotInputs.motorCommands[11].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[11].demand = v;
            }

            // F. Shooter Flywheel Control (6.0V during Phase 3, 0.0V coast otherwise)
            if (flywheelComp)
            {
                flywheelComp->commandedVoltage = flywheelVolts;
                robotInputs.motorCommands[12].mode = MotorControlMode::Voltage;
                robotInputs.motorCommands[12].demand = flywheelVolts;
            }

            // G. Step Pipeline with High-Resolution Timing
            auto stepStart = std::chrono::high_resolution_clock::now();

            if (robot.swerveSystem)
            {
                robot.swerveSystem->SetInputs(robotInputs);
            }

            jointSystem.FixedUpdate(dt);
            RobotAssembler::StepPipeline(robot, dt);
            jointSystem.ReadbackTelemetry();

            auto stepEnd = std::chrono::high_resolution_clock::now();
            totalPhysicsDurationUs += std::chrono::duration<double, std::micro>(stepEnd - stepStart).count();

            // H. Ingest Telemetry Outputs
            robotOutputs = robot.sensorSystem->GetOutputs();
            if (robot.swerveSystem)
            {
                const auto& swerveOut = robot.swerveSystem->GetOutputs();
                for (uint32_t i = 0; i < 8; ++i)
                {
                    robotOutputs.encoderPositions[i] = swerveOut.encoderPositions[i];
                    robotOutputs.encoderVelocities[i] = swerveOut.encoderVelocities[i];
                    robotOutputs.rotorPositionsRot[i] = swerveOut.rotorPositionsRot[i];
                    robotOutputs.rotorVelocitiesRps[i] = swerveOut.rotorVelocitiesRps[i];
                    robotOutputs.motorCurrents[i] = swerveOut.motorCurrents[i];
                    robotOutputs.motorSupplyCurrents[i] = swerveOut.motorSupplyCurrents[i];
                }
            }
            robotOutputs.batteryTerminalVoltage = robot.electricalSystem->GetTerminalVoltage();
            robotOutputs.totalBusCurrent = robot.electricalSystem->GetTotalCurrent();
            robotOutputs.isBrownedOut = robot.electricalSystem->IsBrownedOut();

            // I. Strict Invariant Checks (Zero NaNs & Infs)
            if (!std::isfinite(robotOutputs.batteryTerminalVoltage) ||
                !std::isfinite(robotOutputs.totalBusCurrent) ||
                !std::isfinite(robotOutputs.gyroYaw) ||
                !std::isfinite(robotOutputs.gyroPitch) ||
                !std::isfinite(robotOutputs.gyroRoll))
            {
                nanOrInfDetected = true;
            }

            if (chassisRb && !chassisRb->bodyID.IsInvalid())
            {
                JPH::Vec3 vel = jolt.GetBodyInterface().GetLinearVelocity(chassisRb->bodyID);
                if (!std::isfinite(vel.GetX()) || !std::isfinite(vel.GetY()) || !std::isfinite(vel.GetZ()))
                {
                    nanOrInfDetected = true;
                }
            }

            // Energy accounting: integrate electrical energy delivered
            totalElectricalEnergyJoules += robotOutputs.batteryTerminalVoltage * robotOutputs.totalBusCurrent * dt;
            minBatteryVoltage = std::min(minBatteryVoltage, robotOutputs.batteryTerminalVoltage);
            maxTotalCurrent = std::max(maxTotalCurrent, robotOutputs.totalBusCurrent);
            if (robotOutputs.isBrownedOut)
            {
                brownoutDetected = true;
            }

            // Four-bar loop closure drift tracking
            if (fourBarComp)
            {
                maxFourBarDrift = std::max(maxFourBarDrift, fourBarComp->loopClosureErrorM);
            }
            if (crankJc && folJc)
            {
                double angularDelta = std::abs(static_cast<double>(crankJc->currentAngle) - static_cast<double>(folJc->currentAngle));
                maxFourBarDrift = std::max(maxFourBarDrift, angularDelta * 0.024);
            }

            // Periodic Progress Reporting (Every 12,000 steps = 1 simulated minute)
            if ((step + 1) % 12000 == 0)
            {
                int minute = (step + 1) / 12000;
                double avgUs = totalPhysicsDurationUs / (step + 1);
                std::cout << "  [Sim Minute " << minute << " / 5] Step " << (step + 1)
                          << " (" << ((step + 1) * dt) << "s): V_batt=" << robotOutputs.batteryTerminalVoltage
                          << "V, I_bus=" << robotOutputs.totalBusCurrent
                          << "A, 4-bar drift=" << (maxFourBarDrift * 1000.0)
                          << " mm, avg physics=" << avgUs << " us/step" << std::endl;
            }
        }

        // 6. Quantitative Gate 2a Invariant Assertions
        double meanStepUs = totalPhysicsDurationUs / static_cast<double>(kTotalSteps);

        std::cout << "\n  -> FE1 5-Minute Endurance Test Complete:\n";
        std::cout << "     Total Steps:             " << kTotalSteps << " (300.0 simulated seconds)\n";
        std::cout << "     Mean Physics Step Time:  " << meanStepUs << " us/step\n";
        std::cout << "     Min Battery Voltage:     " << minBatteryVoltage << " V\n";
        std::cout << "     Max Bus Current:         " << maxTotalCurrent << " A\n";
        std::cout << "     Max 4-Bar Loop Drift:    " << (maxFourBarDrift * 1000.0) << " mm\n";
        std::cout << "     Total Energy Consumed:   " << (totalElectricalEnergyJoules / 1000.0) << " kJ\n" << std::endl;

        // Assertion 1: Zero NaNs or infinities across all 60,000 steps
        TEST_EXPECT_FALSE(ctx, nanOrInfDetected);

        // Assertion 2: Four-bar linkage loop drift strictly < 1.5 mm
        TEST_EXPECT_TRUE(ctx, maxFourBarDrift < 0.0015);

        // Assertion 3: Battery electrical stability (no brownouts, voltage >= 9.0V)
        TEST_EXPECT_FALSE(ctx, brownoutDetected);
        TEST_EXPECT_TRUE(ctx, minBatteryVoltage >= 9.0);
        TEST_EXPECT_TRUE(ctx, totalElectricalEnergyJoules > 0.0);
        TEST_EXPECT_TRUE(ctx, std::isfinite(totalElectricalEnergyJoules));

        // Assertion 4: Average step time benchmark
#ifdef NDEBUG
        // Strict Fleet budget in Release mode: < 175 us/step
        TEST_EXPECT_TRUE(ctx, meanStepUs < 175.0);
#else
        // Debug build threshold: ensures stable throughput without hang
        TEST_EXPECT_TRUE(ctx, meanStepUs < 3500.0);
#endif
    }
}