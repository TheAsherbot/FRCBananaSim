/**
 * @file RobotAssemblerTest.cpp
 * @brief Implementation of test suite for Schema v1 programmatic robot assembly and 8-stage pipeline stepping.
 */

#include "RobotAssemblerTest.h"
#include "TestFramework.h"
#include "Robot/RobotAssembler.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Components/CouplingComponent.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Control/SimControlBoundary.h"

#include <filesystem>
#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Robot;
    using namespace Robot::Schema;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Physics::Components;

    void RunRobotAssemblerTest(TestContext& ctx)
    {
        std::cout << "[RUN ] RobotAssemblerTest\n";
        constexpr double tolerance = 1e-4;

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

        // =============================================================
        // 1. Programmatic Assembly from Schema v1 Objects (RA1)
        // =============================================================
        {
            std::cout << "  -> RA1: Reference2026 Full Hierarchy Assembly\n";

            RobotSchemaLoader loader;
            bool loadSuccess = loader.LoadRobot(manifestPath, sidecarPath, materialsPath);
            TEST_EXPECT_TRUE(ctx, loadSuccess);
            TEST_EXPECT_FALSE(ctx, loader.HasErrors());

            Physics::JoltContext jolt;
            TEST_EXPECT_TRUE(ctx, jolt.Initialize());

            Scene scene;
            RobotAssembler assembler(scene, &jolt);

            auto robot = assembler.Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials());

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);
            TEST_EXPECT_EQ(ctx, robot.robotName, "FRC_Reference_Robot_2026");
            TEST_EXPECT_TRUE(ctx, robot.rootLinkEntity != invalidEntity);
            TEST_EXPECT_TRUE(ctx, robot.chassisEntity == robot.rootLinkEntity);
            TEST_EXPECT_TRUE(ctx, robot.batteryEntity != invalidEntity);

            // Entity collection counts matching Reference2026 specification
            TEST_EXPECT_EQ(ctx, robot.linkEntities.size(), static_cast<size_t>(9));
            TEST_EXPECT_EQ(ctx, robot.jointEntities.size(), static_cast<size_t>(7));
            TEST_EXPECT_EQ(ctx, robot.mechanismEntities.size(), static_cast<size_t>(5));
            TEST_EXPECT_EQ(ctx, robot.sensorEntities.size(), static_cast<size_t>(7));
            TEST_EXPECT_EQ(ctx, robot.contactZoneEntities.size(), static_cast<size_t>(2));

            // Core subsystems allocated
            TEST_EXPECT_TRUE(ctx, robot.swerveSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.dispatcher != nullptr);
        }

        // =============================================================
        // 2. Subsystem Components & Entity Hierarchy Integrity (RA2)
        // =============================================================
        {
            std::cout << "  -> RA2: Subsystem Components & Entity Hierarchy Integrity\n";

            RobotSchemaLoader loader;
            loader.LoadRobot(manifestPath, sidecarPath, materialsPath);

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials());

            // 1. Battery Component Verification
            auto* bat = scene.GetComponent<Actuation::Components::BatteryComponent>(robot.batteryEntity);
            TEST_EXPECT_TRUE(ctx, bat != nullptr);
            if (bat)
            {
                TEST_EXPECT_NEAR(ctx, bat->terminalVoltage, 12.60, tolerance);
                TEST_EXPECT_NEAR(ctx, bat->totalCurrent, 1.50, tolerance);
                TEST_EXPECT_FALSE(ctx, bat->isBrownoutCritical);
            }

            // 2. CompositeMassComponent on Chassis
            auto* compMass = scene.GetComponent<Robot::Mechanisms::Components::CompositeMassComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, compMass != nullptr);
            if (compMass)
            {
                TEST_EXPECT_TRUE(ctx, compMass->chassisEntity == robot.chassisEntity);
                // 9 total links - 1 chassis root = 8 dynamic mechanism links
                TEST_EXPECT_EQ(ctx, compMass->mechanismLinks.size(), static_cast<size_t>(8));
            }

            // 3. Chassis (base_link) Physical Properties
            auto baseIt = robot.linkEntities.find("base_link");
            TEST_EXPECT_TRUE(ctx, baseIt != robot.linkEntities.end());
            if (baseIt != robot.linkEntities.end())
            {
                auto* baseRb = scene.GetComponent<RigidBodyComponent>(baseIt->second);
                TEST_EXPECT_TRUE(ctx, baseRb != nullptr);
                if (baseRb)
                {
                    TEST_EXPECT_FALSE(ctx, baseRb->allowSleeping);
                    TEST_EXPECT_NEAR(ctx, baseRb->mass, 38.50f, static_cast<float>(tolerance));
                    TEST_EXPECT_TRUE(ctx, baseRb->hasCustomInertia);
                }

                auto* baseCol = scene.GetComponent<ColliderComponent>(baseIt->second);
                TEST_EXPECT_TRUE(ctx, baseCol != nullptr);
                if (baseCol)
                {
                    TEST_EXPECT_EQ(ctx, static_cast<int>(baseCol->shape), static_cast<int>(ColliderShape::Box));
                }
            }

            // 4. Elevator Mechanism Component & Stage Mapping
            auto elevIt = robot.mechanismEntities.find("elevator_drive");
            TEST_EXPECT_TRUE(ctx, elevIt != robot.mechanismEntities.end());
            if (elevIt != robot.mechanismEntities.end())
            {
                auto* elev = scene.GetComponent<Robot::Mechanisms::Components::ElevatorComponent>(elevIt->second);
                TEST_EXPECT_TRUE(ctx, elev != nullptr);
                if (elev)
                {
                    TEST_EXPECT_NEAR(ctx, elev->gearRatio, 9.0, tolerance);
                    TEST_EXPECT_NEAR(ctx, elev->drumRadiusM, 0.01905, 1e-5);
                    TEST_EXPECT_NEAR(ctx, elev->springPreloadForceN, 45.0, tolerance);
                    TEST_EXPECT_EQ(ctx, elev->motorCount, static_cast<uint32_t>(2));
                    TEST_EXPECT_EQ(ctx, elev->stages.size(), static_cast<size_t>(1));
                }
            }

            // 5. Arm Mechanism Component
            auto armIt = robot.mechanismEntities.find("arm_shoulder");
            TEST_EXPECT_TRUE(ctx, armIt != robot.mechanismEntities.end());
            if (armIt != robot.mechanismEntities.end())
            {
                auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(armIt->second);
                TEST_EXPECT_TRUE(ctx, arm != nullptr);
                if (arm)
                {
                    TEST_EXPECT_NEAR(ctx, arm->gearRatio, 64.0, tolerance);
                    TEST_EXPECT_NEAR(ctx, arm->statorCurrentLimitA, 60.0, tolerance);
                }
            }

            // 6. Flywheel Mechanism Component
            auto fwIt = robot.mechanismEntities.find("shooter_flywheel");
            TEST_EXPECT_TRUE(ctx, fwIt != robot.mechanismEntities.end());
            if (fwIt != robot.mechanismEntities.end())
            {
                auto* fw = scene.GetComponent<Robot::Mechanisms::Components::FlywheelComponent>(fwIt->second);
                TEST_EXPECT_TRUE(ctx, fw != nullptr);
                if (fw)
                {
                    TEST_EXPECT_NEAR(ctx, fw->gearRatio, 0.5, tolerance);
                    TEST_EXPECT_NEAR(ctx, fw->flywheelInertiaKgm2, 0.0032, 1e-5);
                    TEST_EXPECT_NEAR(ctx, fw->statorCurrentLimitA, 80.0, tolerance);
                }
            }

            // 7. Prismatic Slider Joint
            auto jIt = robot.jointEntities.find("elevator_stage1_slider");
            TEST_EXPECT_TRUE(ctx, jIt != robot.jointEntities.end());
            if (jIt != robot.jointEntities.end())
            {
                auto* jc = scene.GetComponent<JointComponent>(jIt->second);
                TEST_EXPECT_TRUE(ctx, jc != nullptr);
                if (jc)
                {
                    TEST_EXPECT_EQ(ctx, static_cast<int>(jc->type), static_cast<int>(JointType::Slider));
                    TEST_EXPECT_TRUE(ctx, jc->hasLimits);
                    TEST_EXPECT_NEAR(ctx, jc->maxLimit, 0.60f, static_cast<float>(tolerance));
                }
            }
        }

        // =============================================================
        // 3. Official 8-Stage Execution Pipeline Stepping (RA3)
        // =============================================================
        {
            std::cout << "  -> RA3: 8-Stage Execution Pipeline Stepping (200 Hz)\n";

            RobotSchemaLoader loader;
            loader.LoadRobot(manifestPath, sidecarPath, materialsPath);

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials());

            // Verify all 8 pipeline systems are allocated
            TEST_EXPECT_TRUE(ctx, robot.electricalSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.elevatorSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.mechanismSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.flywheelSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.compositeMassSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.swerveSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.physicsSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.sensorSystem != nullptr);

            // Bind sensory feedback destination buffer
            Control::SimOutputs outputs;
            robot.electricalSystem->SetOutputs(&outputs);
            robot.sensorSystem->SetOutputs(&outputs);

            // Execute 20 fixed-step iterations (100 ms at 200 Hz, dt = 5 ms)
            for (int i = 0; i < 20; ++i)
            {
                RobotAssembler::StepPipeline(robot, 0.005);
            }

            // Assert telemetry outputs are healthy and free of NaNs
            TEST_EXPECT_FALSE(ctx, std::isnan(outputs.batteryTerminalVoltage));
            TEST_EXPECT_FALSE(ctx, std::isnan(outputs.totalBusCurrent));
            TEST_EXPECT_FALSE(ctx, std::isnan(outputs.gyroYaw));
            TEST_EXPECT_FALSE(ctx, std::isnan(outputs.gyroPitch));
            TEST_EXPECT_FALSE(ctx, std::isnan(outputs.gyroRoll));

            // Bus voltage stays nominal under quiescent idle load
            TEST_EXPECT_NEAR(ctx, outputs.batteryTerminalVoltage, 12.60, 0.5);
            TEST_EXPECT_TRUE(ctx, outputs.totalBusCurrent >= 1.0);
            TEST_EXPECT_FALSE(ctx, outputs.isBrownedOut);
        }

        // =============================================================
        // 4. File-Based Assembly & Graceful Error Handling (RA4)
        // =============================================================
        {
            std::cout << "  -> RA4: AssembleFromFiles & Error Diagnostic Handling\n";

            Physics::JoltContext jolt;
            jolt.Initialize();

            // 1. Success on valid files
            Scene scene1;
            RobotAssembler assembler1(scene1, &jolt);
            auto robot1 = assembler1.AssembleFromFiles(manifestPath, sidecarPath, materialsPath);
            TEST_EXPECT_TRUE(ctx, robot1.isSuccess);
            TEST_EXPECT_EQ(ctx, robot1.robotName, "FRC_Reference_Robot_2026");

            // 2. Graceful failure on missing or invalid files without crashing
            Scene scene2;
            RobotAssembler assembler2(scene2, &jolt);
            auto robot2 = assembler2.AssembleFromFiles("nonexistent/invalid_manifest.json");
            TEST_EXPECT_FALSE(ctx, robot2.isSuccess);
            TEST_EXPECT_FALSE(ctx, robot2.errorMessage.empty());
        }

        // =============================================================
        // 5. Dynamic Simulation Stability Verification (RA5)
        // =============================================================
        {
            std::cout << "  -> RA5: Dynamic Simulation Stability (200 Steps / 1.0 s)\n";

            Physics::JoltContext jolt;
            TEST_EXPECT_TRUE(ctx, jolt.Initialize());

            Scene scene;
            scene.SetFixedDeltaTime(0.005);

            // Ground carpet
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

            Control::SimInputs robotInputs;
            Control::SimOutputs robotOutputs;
            if (robot.electricalSystem) robot.electricalSystem->SetOutputs(&robotOutputs);
            if (robot.sensorSystem) robot.sensorSystem->SetOutputs(&robotOutputs);
            if (robot.swerveSystem) robot.swerveSystem->SetInputs(robotInputs);

            for (int step = 0; step < 200; ++step)
            {
                jointSystem.FixedUpdate(0.005);
                RobotAssembler::StepPipeline(robot, 0.005);
                jointSystem.ReadbackTelemetry();
            }

            // Verify chassis (base_link) remains stable and settled without exploding
            auto* baseRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, baseRb != nullptr);
            if (baseRb && !baseRb->bodyID.IsInvalid())
            {
                JPH::RVec3 pos = jolt.GetBodyInterface().GetPosition(baseRb->bodyID);
                JPH::Vec3 vel = jolt.GetBodyInterface().GetLinearVelocity(baseRb->bodyID);

                // Elevation must remain near resting ground level (0.10m - 0.60m)
                TEST_EXPECT_TRUE(ctx, pos.GetY() >= 0.10 && pos.GetY() <= 0.60);
                // Linear velocity must be stable and small (< 1.5 m/s)
                TEST_EXPECT_TRUE(ctx, vel.Length() < 1.5f);
            }

            // Verify all mechanism links have finite, small velocities
            for (const auto& [name, entity] : robot.linkEntities)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(entity);
                if (rb && !rb->bodyID.IsInvalid())
                {
                    JPH::Vec3 vel = jolt.GetBodyInterface().GetLinearVelocity(rb->bodyID);
                    TEST_EXPECT_TRUE(ctx, std::isfinite(vel.GetX()));
                    TEST_EXPECT_TRUE(ctx, std::isfinite(vel.GetY()));
                    TEST_EXPECT_TRUE(ctx, std::isfinite(vel.GetZ()));
                    TEST_EXPECT_TRUE(ctx, vel.Length() < 3.0f);
                }
            }
        }

        // =============================================================
        // 6. Complete System Stability: Voltage, Wheels, & Mesh Alignment (RA6)
        // =============================================================
        {
            std::cout << "  -> RA6: Multi-Stage Stability (Voltage Hold, Wheel Brake Stick, Mesh Alignment)\n";

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;

            // Carpet Ground
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

            Control::SimInputs robotInputs;
            Control::SimOutputs robotOutputs;
            if (robot.electricalSystem) robot.electricalSystem->SetOutputs(&robotOutputs);
            if (robot.sensorSystem) robot.sensorSystem->SetOutputs(&robotOutputs);
            if (robot.swerveSystem) robot.swerveSystem->SetInputs(robotInputs);

            // Command holding torque to intake 4-bar (stalled at -6V, typical idle state)
            robotInputs.motorCommands[11].mode = Control::MotorControlMode::Voltage;
            robotInputs.motorCommands[11].demand = -6.0;

            // Ensure drivetrain is in Brake mode with 0V demand
            for (int i = 0; i < 8; ++i)
            {
                robotInputs.motorCommands[i].mode = Control::MotorControlMode::Voltage;
                robotInputs.motorCommands[i].demand = 0.0;
                robotInputs.motorCommands[i].neutralMode = Control::NeutralMode::Brake;
            }

            bool brownoutDetected = false;
            double minVoltage = 12.60;
            double maxCurrent = 0.0;

            // Simulate 400 steps (2.0 seconds at 200 Hz)
            for (int step = 0; step < 400; ++step)
            {
                if (robot.swerveSystem) robot.swerveSystem->SetInputs(robotInputs);

                auto armMIt = robot.mechanismEntities.find("intake_fourbar_deploy");
                if (armMIt != robot.mechanismEntities.end())
                {
                    auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(armMIt->second);
                    if (arm) arm->commandedVoltage = robotInputs.motorCommands[11].demand;
                }

                jointSystem.FixedUpdate(0.005);
                RobotAssembler::StepPipeline(robot, 0.005);
                jointSystem.ReadbackTelemetry();

                robotOutputs = robot.sensorSystem->GetOutputs();
                if (robot.swerveSystem)
                {
                    const auto& swerveOut = robot.swerveSystem->GetOutputs();
                    for (uint32_t i = 0; i < 8; ++i)
                    {
                        robotOutputs.encoderPositions[i] = swerveOut.encoderPositions[i];
                        robotOutputs.encoderVelocities[i] = swerveOut.encoderVelocities[i];
                    }
                }
                robotOutputs.batteryTerminalVoltage = robot.electricalSystem->GetTerminalVoltage();
                robotOutputs.totalBusCurrent = robot.electricalSystem->GetTotalCurrent();
                robotOutputs.isBrownedOut = robot.electricalSystem->IsBrownedOut();

                if (robotOutputs.isBrownedOut)
                {
                    brownoutDetected = true;
                }
                minVoltage = std::min(minVoltage, robotOutputs.batteryTerminalVoltage);
                maxCurrent = std::max(maxCurrent, robotOutputs.totalBusCurrent);
            }

            // 1. Verify Battery Voltage Stability: Must remain steady above critical cutoff (6.3V)
            TEST_EXPECT_FALSE(ctx, brownoutDetected);
            TEST_EXPECT_TRUE(ctx, minVoltage >= 11.0); // Normal sag around ~11.8V - 12.5V, never 4.6V or 7V
            TEST_EXPECT_TRUE(ctx, maxCurrent < 100.0); // Never 500A-800A

            // 2. Verify Swerve Wheel Brake Lock & Level Posture: All 4 wheels on ground with zero spinning velocity
            if (robot.swerveSystem && robot.swerveSystem->GetModel())
            {
                auto* model = robot.swerveSystem->GetModel();
                for (uint32_t i = 0; i < 4; ++i)
                {
                    const auto& modState = model->GetModule(i).GetState();
                    TEST_EXPECT_NEAR(ctx, modState.wheelVelocity, 0.0, 0.05);
                    TEST_EXPECT_NEAR(ctx, modState.linearVelocity, 0.0, 0.01);
                    TEST_EXPECT_TRUE(ctx, modState.contactState.normalForce > 100.0);
                }
            }

            // Verify chassis pitch and roll are level (< 0.5 deg)
            TEST_EXPECT_NEAR(ctx, glm::degrees(robotOutputs.gyroPitch), 0.0, 0.5);
            TEST_EXPECT_NEAR(ctx, glm::degrees(robotOutputs.gyroRoll), 0.0, 0.5);

            // 3. Verify Chassis Rest Stability (No jumping or flying away)
            auto* baseRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            if (baseRb && !baseRb->bodyID.IsInvalid())
            {
                JPH::RVec3 pos = jolt.GetBodyInterface().GetPosition(baseRb->bodyID);
                JPH::Vec3 vel = jolt.GetBodyInterface().GetLinearVelocity(baseRb->bodyID);
                TEST_EXPECT_TRUE(ctx, pos.GetY() >= 0.25 && pos.GetY() <= 0.32);
                TEST_EXPECT_TRUE(ctx, vel.Length() < 0.05f);
            }

            // 4. Verify Intake Coupler & Arm Links Have Zero Swinging Velocity
            for (const auto& [name, entity] : robot.linkEntities)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(entity);
                if (rb && !rb->bodyID.IsInvalid())
                {
                    JPH::Vec3 linVel = jolt.GetBodyInterface().GetLinearVelocity(rb->bodyID);
                    JPH::Vec3 angVel = jolt.GetBodyInterface().GetAngularVelocity(rb->bodyID);
                    if (linVel.Length() >= 0.05f || angVel.Length() >= 0.05f)
                    {
                        std::cout << "  [DIAG] Link '" << name << "' linVel=" << linVel.Length()
                            << " (" << linVel.GetX() << "," << linVel.GetY() << "," << linVel.GetZ() << ")"
                            << " angVel=" << angVel.Length()
                            << " (" << angVel.GetX() << "," << angVel.GetY() << "," << angVel.GetZ() << ")\n";
                    }
                    TEST_EXPECT_TRUE(ctx, linVel.Length() < 0.05f);
                    TEST_EXPECT_TRUE(ctx, angVel.Length() < 0.05f);
                }
            }

            // 5. Verify Flywheel Visual Attachment to Wrist & Independent Wrist Motion at High Shoulder Angle
            auto* shoulderArm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(robot.mechanismEntities["arm_shoulder"]);
            auto* wristArm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(robot.mechanismEntities["arm_wrist"]);
            TEST_EXPECT_TRUE(ctx, shoulderArm != nullptr);
            TEST_EXPECT_TRUE(ctx, wristArm != nullptr);

            if (shoulderArm && wristArm)
            {
                // Raise shoulder to upper limit (+12V for 150 steps)
                shoulderArm->commandedVoltage = 12.0;
                wristArm->commandedVoltage = 6.0;
                for (int step = 0; step < 150; ++step)
                {
                    jointSystem.FixedUpdate(0.005);
                    RobotAssembler::StepPipeline(robot, 0.005);
                    jointSystem.ReadbackTelemetry();
                }

                TEST_EXPECT_TRUE(ctx, shoulderArm->currentAngleRad > 1.5);
                TEST_EXPECT_TRUE(ctx, wristArm->currentAngleRad > 0.8);

                // Reverse wrist to negative limit (-6V for 150 steps) while shoulder remains at upper limit
                shoulderArm->commandedVoltage = 0.0;
                wristArm->commandedVoltage = -6.0;
                for (int step = 0; step < 150; ++step)
                {
                    jointSystem.FixedUpdate(0.005);
                    RobotAssembler::StepPipeline(robot, 0.005);
                    jointSystem.ReadbackTelemetry();
                }

                TEST_EXPECT_TRUE(ctx, shoulderArm->currentAngleRad > 1.5);
                TEST_EXPECT_TRUE(ctx, wristArm->currentAngleRad < -0.8);

                auto* wristT = scene.GetComponent<TransformComponent>(robot.linkEntities["arm_wrist_link"]);
                auto* fwT = scene.GetComponent<TransformComponent>(robot.linkEntities["shooter_flywheel_visual"]);
                TEST_EXPECT_TRUE(ctx, wristT != nullptr);
                TEST_EXPECT_TRUE(ctx, fwT != nullptr);
                if (wristT && fwT)
                {
                    glm::vec3 expectedFwPos = wristT->position + (wristT->rotation * glm::vec3(0.0f, 0.06f, 0.06f));
                    TEST_EXPECT_TRUE(ctx, glm::length(fwT->position - expectedFwPos) < 0.01f);
                }
            }

            // 6. Verify Four-Bar Colliders, Deploy, and Fold-Back Lockstep (Zero Back-Bar Lag/Jump)
            auto* crankCol = scene.GetComponent<ColliderComponent>(robot.linkEntities["intake_crank_link"]);
            auto* folCol = scene.GetComponent<ColliderComponent>(robot.linkEntities["intake_follower_link"]);
            TEST_EXPECT_TRUE(ctx, crankCol != nullptr);
            TEST_EXPECT_TRUE(ctx, folCol != nullptr);

            auto* intakeArm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(robot.mechanismEntities["intake_fourbar_deploy"]);
            auto* crankJc = scene.GetComponent<JointComponent>(robot.jointEntities["fourbar_crank_hinge"]);
            auto* folJc = scene.GetComponent<JointComponent>(robot.jointEntities["fourbar_follower_hinge"]);
            TEST_EXPECT_TRUE(ctx, intakeArm != nullptr);
            TEST_EXPECT_TRUE(ctx, crankJc != nullptr);
            TEST_EXPECT_TRUE(ctx, folJc != nullptr);

            if (intakeArm && crankJc && folJc)
            {
                double maxDeployMismatch = 0.0;
                double maxFoldMismatch = 0.0;

                // Deploy 4-bar (+6V for 100 steps)
                intakeArm->commandedVoltage = 6.0;
                for (int step = 0; step < 100; ++step)
                {
                    jointSystem.FixedUpdate(0.005);
                    RobotAssembler::StepPipeline(robot, 0.005);
                    jointSystem.ReadbackTelemetry();

                    double diff = std::abs(static_cast<double>(crankJc->currentAngle) - static_cast<double>(folJc->currentAngle));
                    maxDeployMismatch = std::max(maxDeployMismatch, diff);
                }
                TEST_EXPECT_TRUE(ctx, crankJc->currentAngle > 1.2f);
                TEST_EXPECT_TRUE(ctx, folJc->currentAngle > 1.2f);
                TEST_EXPECT_TRUE(ctx, maxDeployMismatch < 0.03);

                // Fold 4-bar back up to 0.0 rad using PD controller for 150 steps
                for (int step = 0; step < 150; ++step)
                {
                    double err = 0.0 - intakeArm->currentAngleRad;
                    double pdVolts = std::clamp((err * 18.0) - (intakeArm->currentVelocityRadPerSec * 1.2), -6.0, 6.0);
                    intakeArm->commandedVoltage = (std::abs(err) > 0.008 || std::abs(intakeArm->currentVelocityRadPerSec) > 0.05) ? pdVolts : 0.0;

                    jointSystem.FixedUpdate(0.005);
                    RobotAssembler::StepPipeline(robot, 0.005);
                    jointSystem.ReadbackTelemetry();

                    double diff = std::abs(static_cast<double>(crankJc->currentAngle) - static_cast<double>(folJc->currentAngle));
                    maxFoldMismatch = std::max(maxFoldMismatch, diff);
                }
                TEST_EXPECT_NEAR(ctx, crankJc->currentAngle, 0.0f, 0.02f);
                TEST_EXPECT_NEAR(ctx, folJc->currentAngle, 0.0f, 0.02f);
                TEST_EXPECT_TRUE(ctx, maxFoldMismatch < 0.03);
            }
        }
    }
}
