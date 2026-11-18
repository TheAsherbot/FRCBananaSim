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
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Interaction/GamePieceSpec.h"
#include "Interaction/Components/GamePieceComponent.h"

#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
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
            TEST_EXPECT_EQ(ctx, robot.mechanismEntities.size(), static_cast<size_t>(6));
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
                    TEST_EXPECT_NEAR(ctx, fw->rollerRadiusM, 0.0508, tolerance);
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

            // 8. Contact Zones & Full Geometric Retention (Block 6 Day 1)
            auto czIt = robot.contactZoneEntities.find("intake_capture_zone");
            TEST_EXPECT_TRUE(ctx, czIt != robot.contactZoneEntities.end());
            if (czIt != robot.contactZoneEntities.end())
            {
                auto* cz = scene.GetComponent<Robot::Sensors::Components::ContactZoneComponent>(czIt->second);
                TEST_EXPECT_TRUE(ctx, cz != nullptr);
                if (cz)
                {
                    TEST_EXPECT_EQ(ctx, cz->name, "intake_capture_zone");
                    TEST_EXPECT_EQ(ctx, cz->role, "intake");
                    TEST_EXPECT_TRUE(ctx, glm::length(cz->dimensions) > 0.1f);
                    TEST_EXPECT_NEAR(ctx, glm::length(cz->tangentAxis), 1.0, tolerance);
                    TEST_EXPECT_NEAR(ctx, glm::length(cz->compressionAxis), 1.0, tolerance);
                    TEST_EXPECT_NEAR(ctx, cz->nominalGapM, 0.11, tolerance);
                }
            }

            // 9. Beam-Break Sensor Contact Zone Binding (Block 6 Day 1 Fix)
            auto bbIt = robot.sensorEntities.find("intake_beam_break");
            TEST_EXPECT_TRUE(ctx, bbIt != robot.sensorEntities.end());
            if (bbIt != robot.sensorEntities.end())
            {
                auto* bb = scene.GetComponent<Robot::Sensors::Components::BeamBreakComponent>(bbIt->second);
                TEST_EXPECT_TRUE(ctx, bb != nullptr);
                if (bb)
                {
                    TEST_EXPECT_TRUE(ctx, bb->sensorEntity != Core::ECS::Core::invalidEntity);
                    TEST_EXPECT_TRUE(ctx, bb->sensorEntity == robot.contactZoneEntities["intake_capture_zone"]);
                    auto* linkedCz = scene.GetComponent<Robot::Sensors::Components::ContactZoneComponent>(bb->sensorEntity);
                    TEST_EXPECT_TRUE(ctx, linkedCz != nullptr);
                }
            }

            // 10. GamePiece Data Structures Validation (Block 6 Day 1)
            {
                Interaction::GamePieceSpec spec;
                TEST_EXPECT_EQ(ctx, spec.name, "Note2024");
                TEST_EXPECT_NEAR(ctx, spec.massKg, 0.235, tolerance);
                TEST_EXPECT_NEAR(ctx, spec.outerRadiusM, 0.1778, tolerance);
                TEST_EXPECT_NEAR(ctx, spec.thicknessM, 0.0508, tolerance);
                TEST_EXPECT_NEAR(ctx, spec.complianceK, 25000.0, tolerance);

                Interaction::Components::GamePieceComponent gp;
                TEST_EXPECT_EQ(ctx, static_cast<int>(gp.state), static_cast<int>(Interaction::Components::GamePieceState::Free));
                TEST_EXPECT_TRUE(ctx, gp.capturingRobot == Core::ECS::Core::invalidEntity);
                TEST_EXPECT_TRUE(ctx, gp.boundContactZone == Core::ECS::Core::invalidEntity);
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

            // Flywheel external disturbance torque reset verification (Block 6 Day 1)
            auto fwMechIt = robot.mechanismEntities.find("shooter_flywheel");
            if (fwMechIt != robot.mechanismEntities.end())
            {
                auto* fw = scene.GetComponent<Robot::Mechanisms::Components::FlywheelComponent>(fwMechIt->second);
                if (fw)
                {
                    fw->externalLoadTorqueNm = 4.5;
                    RobotAssembler::StepPipeline(robot, 0.005);
                    TEST_EXPECT_NEAR(ctx, fw->externalLoadTorqueNm, 0.0, 1e-6);
                }
            }

            // Beam-Break contact zone engagement through pipeline stepping (Block 6 Day 1)
            auto czEntityIt = robot.contactZoneEntities.find("intake_capture_zone");
            if (czEntityIt != robot.contactZoneEntities.end())
            {
                auto* cz = scene.GetComponent<Robot::Sensors::Components::ContactZoneComponent>(czEntityIt->second);
                if (cz)
                {
                    cz->isEngaged = true;
                    // Step 3 ticks (15 ms) to exceed the 10 ms debounce threshold
                    for (int i = 0; i < 3; ++i)
                    {
                        RobotAssembler::StepPipeline(robot, 0.005);
                    }
                    // intake_beam_break is configured on dioChannel 2
                    TEST_EXPECT_TRUE(ctx, (outputs.digitalInputs & (1u << 2)) != 0);

                    // Disengage and verify falling edge clears
                    cz->isEngaged = false;
                    for (int i = 0; i < 3; ++i)
                    {
                        RobotAssembler::StepPipeline(robot, 0.005);
                    }
                    TEST_EXPECT_TRUE(ctx, (outputs.digitalInputs & (1u << 2)) == 0);
                }
            }
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
                JPH::RVec3 comPos = jolt.GetBodyInterface().GetCenterOfMassPosition(baseRb->bodyID);
                JPH::Vec3 vel = jolt.GetBodyInterface().GetLinearVelocity(baseRb->bodyID);
                TEST_EXPECT_TRUE(ctx, comPos.GetY() >= 0.25 && comPos.GetY() <= 0.32);
                TEST_EXPECT_TRUE(ctx, pos.GetY() >= 0.18 && pos.GetY() <= 0.22);
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

        // =============================================================
        // 7. Multi-Collider Compound Jolt Spawning (Q1 / RA7)
        // =============================================================
        {
            std::cout << "  -> RA7: Multi-Collider Compound Jolt Spawning (Q1)\n";

            RobotManifest customManifest;
            customManifest.schemaVersion = "1.0.0";
            customManifest.robotName = "MultiColliderBot";

            LinkDefinition link;
            link.name = "base_link";
            link.isRoot = true;
            link.massKg = 10.0;
            link.centerOfMassM = glm::dvec3(0.0, 0.0, 0.0);
            link.inertiaTensor.ixx = 1.0;
            link.inertiaTensor.iyy = 1.0;
            link.inertiaTensor.izz = 1.0;

            // 3 colliders: Box, Sphere, Cylinder
            ColliderDefinition col0;
            col0.name = "box_proxy";
            col0.type = "box";
            col0.originM = glm::dvec3(0.1, 0.0, 0.0);
            col0.dimensionsM = glm::dvec3(0.2, 0.2, 0.2);
            link.colliders.push_back(col0);

            ColliderDefinition col1;
            col1.name = "sphere_proxy";
            col1.type = "sphere";
            col1.originM = glm::dvec3(-0.1, 0.0, 0.0);
            col1.dimensionsM = glm::dvec3(0.05, 0.0, 0.0);
            link.colliders.push_back(col1);

            ColliderDefinition col2;
            col2.name = "cylinder_proxy";
            col2.type = "cylinder";
            col2.originM = glm::dvec3(0.0, 0.1, 0.0);
            col2.dimensionsM = glm::dvec3(0.04, 0.10, 0.0);
            link.colliders.push_back(col2);

            customManifest.links.push_back(link);

            RobotSidecar customSidecar;
            MaterialLibrary materials;

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(customManifest, customSidecar, materials);

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);
            TEST_EXPECT_TRUE(ctx, robot.chassisEntity != invalidEntity);

            auto* colComp = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, colComp != nullptr);
            if (colComp)
            {
                TEST_EXPECT_TRUE(ctx, colComp->IsCompound());
                TEST_EXPECT_EQ(ctx, colComp->subColliders.size(), static_cast<size_t>(2));
                TEST_EXPECT_EQ(ctx, static_cast<int>(colComp->shape), static_cast<int>(ColliderShape::Box));
                TEST_EXPECT_EQ(ctx, static_cast<int>(colComp->subColliders[0].shape), static_cast<int>(ColliderShape::Sphere));
                TEST_EXPECT_EQ(ctx, static_cast<int>(colComp->subColliders[1].shape), static_cast<int>(ColliderShape::Cylinder));
            }

            // Step physics once to create Jolt bodies
            if (robot.physicsSystem)
            {
                robot.physicsSystem->FixedUpdate(0.005);
            }

            auto* rb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, rb != nullptr);
            if (rb && !rb->bodyID.IsInvalid())
            {
                const JPH::BodyLockInterface& lock = jolt.GetPhysicsSystem().GetBodyLockInterfaceNoLock();
                const JPH::Body* body = lock.TryGetBody(rb->bodyID);
                TEST_EXPECT_TRUE(ctx, body != nullptr);
                if (body)
                {
                    const JPH::Shape* shape = body->GetShape();
                    TEST_EXPECT_TRUE(ctx, shape != nullptr);
                    if (shape)
                    {
                        const JPH::CompoundShape* compShape = nullptr;
                        if (shape->GetSubType() == JPH::EShapeSubType::StaticCompound)
                        {
                            compShape = static_cast<const JPH::CompoundShape*>(shape);
                        }
                        else if (shape->GetSubType() == JPH::EShapeSubType::OffsetCenterOfMass)
                        {
                            const auto* offsetShape = static_cast<const JPH::OffsetCenterOfMassShape*>(shape);
                            if (offsetShape->GetInnerShape()->GetSubType() == JPH::EShapeSubType::StaticCompound)
                            {
                                compShape = static_cast<const JPH::CompoundShape*>(offsetShape->GetInnerShape());
                            }
                        }
                        TEST_EXPECT_TRUE(ctx, compShape != nullptr);
                        if (compShape)
                        {
                            TEST_EXPECT_EQ(ctx, compShape->GetNumSubShapes(), static_cast<uint32_t>(3));
                        }
                    }
                }
            }
        }

        // =============================================================
        // 8. Decoupled Link Center of Mass (Q2 / RA8)
        // =============================================================
        {
            std::cout << "  -> RA8: Decoupled Link Center of Mass (Q2)\n";

            RobotManifest customManifest;
            customManifest.schemaVersion = "1.0.0";
            customManifest.robotName = "DecoupledComBot";

            LinkDefinition link;
            link.name = "base_link";
            link.isRoot = true;
            link.massKg = 15.0;
            // CoM offset in NWU frame
            link.centerOfMassM = glm::dvec3(0.12, 0.04, 0.25);
            link.inertiaTensor.ixx = 2.0;
            link.inertiaTensor.iyy = 2.0;
            link.inertiaTensor.izz = 2.0;

            // Collider origin at (0,0,0) in NWU frame (decoupled from CoM!)
            ColliderDefinition col;
            col.name = "chassis_box";
            col.type = "box";
            col.originM = glm::dvec3(0.0, 0.0, 0.0);
            col.dimensionsM = glm::dvec3(0.6, 0.6, 0.2);
            link.colliders.push_back(col);

            customManifest.links.push_back(link);

            RobotSidecar customSidecar;
            MaterialLibrary materials;

            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(customManifest, customSidecar, materials, glm::vec3(0.0f, 1.0f, 0.0f));

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            auto* rb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, rb != nullptr);
            if (rb)
            {
                TEST_EXPECT_TRUE(ctx, rb->hasCustomCenterOfMass);
                glm::vec3 expectedComEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(link.centerOfMassM));
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.x, expectedComEng.x, 1e-5f);
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.y, expectedComEng.y, 1e-5f);
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.z, expectedComEng.z, 1e-5f);
            }

            // Step physics to spawn body
            if (robot.physicsSystem)
            {
                robot.physicsSystem->FixedUpdate(0.005);
            }

            if (rb && !rb->bodyID.IsInvalid())
            {
                JPH::RVec3 bodyPos = jolt.GetBodyInterface().GetPosition(rb->bodyID);
                JPH::RVec3 comPos = jolt.GetBodyInterface().GetCenterOfMassPosition(rb->bodyID);

                // Body position should be the spawn origin (0.0, 1.0, 0.0)
                TEST_EXPECT_NEAR(ctx, static_cast<float>(bodyPos.GetX()), 0.0f, 1e-4f);
                TEST_EXPECT_NEAR(ctx, static_cast<float>(bodyPos.GetY()), 1.0f, 1e-4f);
                TEST_EXPECT_NEAR(ctx, static_cast<float>(bodyPos.GetZ()), 0.0f, 1e-4f);

                // Body Center of Mass should be spawn origin + expectedComEng
                glm::vec3 expectedComEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(link.centerOfMassM));
                TEST_EXPECT_NEAR(ctx, static_cast<float>(comPos.GetX()), expectedComEng.x, 1e-4f);
                TEST_EXPECT_NEAR(ctx, static_cast<float>(comPos.GetY()), 1.0f + expectedComEng.y, 1e-4f);
                TEST_EXPECT_NEAR(ctx, static_cast<float>(comPos.GetZ()), expectedComEng.z, 1e-4f);

                // The delta between CoM and Body origin matches centerOfMassOffset
                JPH::Vec3 delta = JPH::Vec3(comPos - bodyPos);
                TEST_EXPECT_NEAR(ctx, delta.GetX(), expectedComEng.x, 1e-4f);
                TEST_EXPECT_NEAR(ctx, delta.GetY(), expectedComEng.y, 1e-4f);
                TEST_EXPECT_NEAR(ctx, delta.GetZ(), expectedComEng.z, 1e-4f);
            }
        }

        // =============================================================
        // 9. Dynamic Flywheel Mount Offset (Q4 / RA9)
        // =============================================================
        {
            std::cout << "  -> RA9: Dynamic Flywheel Mount Offset (Q4)\n";

            RobotManifest customManifest;
            customManifest.schemaVersion = "1.0.0";
            customManifest.robotName = "FlywheelOffsetBot";

            LinkDefinition parentLink;
            parentLink.name = "arm_link";
            parentLink.isRoot = true;
            parentLink.massKg = 5.0;
            parentLink.centerOfMassM = glm::dvec3(0.0, 0.0, 0.0);
            parentLink.inertiaTensor.ixx = 1.0;
            parentLink.inertiaTensor.iyy = 1.0;
            parentLink.inertiaTensor.izz = 1.0;
            customManifest.links.push_back(parentLink);

            LinkDefinition visualLink;
            visualLink.name = "flywheel_mesh";
            visualLink.isRoot = false;
            visualLink.massKg = 1.0;
            visualLink.centerOfMassM = glm::dvec3(0.0, 0.0, 0.0);
            visualLink.inertiaTensor.ixx = 0.1;
            visualLink.inertiaTensor.iyy = 0.1;
            visualLink.inertiaTensor.izz = 0.1;
            customManifest.links.push_back(visualLink);

            RobotSidecar customSidecar;
            MechanismConfig fwMech;
            fwMech.name = "shooter_flywheel";
            fwMech.role = "flywheel";
            fwMech.parentLink = "arm_link";
            fwMech.visualLink = "flywheel_mesh";
            fwMech.gearRatio = 1.0;
            fwMech.mountOffsetM = glm::dvec3(0.20, -0.10, 0.35); // NWU
            customSidecar.mechanisms.push_back(fwMech);

            MaterialLibrary materials;
            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(customManifest, customSidecar, materials);

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            auto fwEntityIt = robot.mechanismEntities.find("shooter_flywheel");
            TEST_EXPECT_TRUE(ctx, fwEntityIt != robot.mechanismEntities.end());
            if (fwEntityIt != robot.mechanismEntities.end())
            {
                auto* fw = scene.GetComponent<Robot::Mechanisms::Components::FlywheelComponent>(fwEntityIt->second);
                TEST_EXPECT_TRUE(ctx, fw != nullptr);
                if (fw)
                {
                    glm::vec3 expectedEngOffset = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(fwMech.mountOffsetM));
                    TEST_EXPECT_NEAR(ctx, fw->visualLocalOffset.x, expectedEngOffset.x, 1e-5f);
                    TEST_EXPECT_NEAR(ctx, fw->visualLocalOffset.y, expectedEngOffset.y, 1e-5f);
                    TEST_EXPECT_NEAR(ctx, fw->visualLocalOffset.z, expectedEngOffset.z, 1e-5f);
                }
            }

            auto visEntityIt = robot.linkEntities.find("flywheel_mesh");
            TEST_EXPECT_TRUE(ctx, visEntityIt != robot.linkEntities.end());
            if (visEntityIt != robot.linkEntities.end())
            {
                auto* visT = scene.GetComponent<TransformComponent>(visEntityIt->second);
                auto* parentT = scene.GetComponent<TransformComponent>(robot.linkEntities["arm_link"]);
                TEST_EXPECT_TRUE(ctx, visT != nullptr);
                TEST_EXPECT_TRUE(ctx, parentT != nullptr);
                if (visT && parentT)
                {
                    glm::vec3 expectedEngOffset = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(fwMech.mountOffsetM));
                    glm::vec3 expectedPos = parentT->position + (parentT->rotation * expectedEngOffset);
                    TEST_EXPECT_NEAR(ctx, visT->position.x, expectedPos.x, 1e-4f);
                    TEST_EXPECT_NEAR(ctx, visT->position.y, expectedPos.y, 1e-4f);
                    TEST_EXPECT_NEAR(ctx, visT->position.z, expectedPos.z, 1e-4f);
                }
            }
        }

        // =============================================================
        // 10. Mass Reconciliation and Ballast Injection (RA10)
        // =============================================================
        {
            std::cout << "  -> RA10: Mass Reconciliation and Ballast Injection\n";

            RobotManifest customManifest;
            customManifest.schemaVersion = "1.0.0";
            customManifest.robotName = "BallastBot";

            LinkDefinition link;
            link.name = "base_link";
            link.isRoot = true;
            link.massKg = 30.0;
            link.centerOfMassM = glm::dvec3(0.0, 0.0, 0.10);
            link.inertiaTensor.ixx = 1.0;
            link.inertiaTensor.iyy = 2.0;
            link.inertiaTensor.izz = 3.0;

            ColliderDefinition col;
            col.name = "box";
            col.type = "box";
            col.originM = glm::dvec3(0.0, 0.0, 0.10);
            col.dimensionsM = glm::dvec3(0.8, 0.8, 0.2);
            link.colliders.push_back(col);
            customManifest.links.push_back(link);

            RobotSidecar customSidecar;
            customSidecar.massReconciliation.enabled = true;
            customSidecar.massReconciliation.linkMassMultiplier = 1.10; // Scaled base mass: 30 * 1.10 = 33 kg
            customSidecar.massReconciliation.unmodeledBallast.massKg = 5.0; // Ballast mass: 5 kg -> Total = 38 kg
            customSidecar.massReconciliation.unmodeledBallast.targetLink = "base_link";
            customSidecar.massReconciliation.unmodeledBallast.offsetM = glm::dvec3(0.10, 0.0, 0.05);

            MaterialLibrary materials;
            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(customManifest, customSidecar, materials);

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            auto* rb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, rb != nullptr);
            if (rb)
            {
                // Verify total mass: 33 kg + 5 kg = 38 kg
                TEST_EXPECT_NEAR(ctx, rb->mass, 38.0f, 1e-4f);

                // Verify reconciled center of mass:
                // c_new = (33 * [0, 0, 0.10] + 5 * [0.10, 0.0, 0.05]) / 38
                // c_new = [0.50 / 38, 0.0, 3.55 / 38]
                glm::dvec3 expectedComNWU(0.50 / 38.0, 0.0, 3.55 / 38.0);
                glm::vec3 expectedComEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(expectedComNWU));
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.x, expectedComEng.x, 1e-4f);
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.y, expectedComEng.y, 1e-4f);
                TEST_EXPECT_NEAR(ctx, rb->centerOfMassOffset.z, expectedComEng.z, 1e-4f);

                // Verify custom inertia is present and strictly positive definite
                TEST_EXPECT_TRUE(ctx, rb->hasCustomInertia);
                TEST_EXPECT_TRUE(ctx, rb->inertiaTensor[0][0] > 2.0f); // Engine X is NWU Y
                TEST_EXPECT_TRUE(ctx, rb->inertiaTensor[1][1] > 3.0f); // Engine Y is NWU Z
                TEST_EXPECT_TRUE(ctx, rb->inertiaTensor[2][2] > 1.0f); // Engine Z is NWU X

                // Verify no extra collider was spawned for ballast
                auto* colComp = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
                TEST_EXPECT_TRUE(ctx, colComp != nullptr);
                if (colComp)
                {
                    TEST_EXPECT_FALSE(ctx, colComp->IsCompound());
                    TEST_EXPECT_TRUE(ctx, colComp->subColliders.empty());
                }
            }
        }

        // =============================================================
        // 11. Virtual Bumper Generation & Mass Injection (RA11)
        // =============================================================
        {
            std::cout << "  -> RA11: Virtual Bumper Generation & Mass Injection\n";

            RobotManifest customManifest;
            customManifest.schemaVersion = "1.0.0";
            customManifest.robotName = "BumperBot";

            LinkDefinition link;
            link.name = "base_link";
            link.isRoot = true;
            link.massKg = 30.0;
            link.centerOfMassM = glm::dvec3(0.0, 0.0, 0.10);
            link.inertiaTensor.ixx = 1.0;
            link.inertiaTensor.iyy = 2.0;
            link.inertiaTensor.izz = 3.0;
            // No colliders in CAD!
            customManifest.links.push_back(link);

            RobotSidecar customSidecar;
            customSidecar.bumpers.enabled = true;
            customSidecar.bumpers.targetLink = "base_link";
            customSidecar.bumpers.shape = "perimeter";
            customSidecar.bumpers.lengthM = 0.85;
            customSidecar.bumpers.widthM = 0.85;
            customSidecar.bumpers.heightM = 0.127;
            customSidecar.bumpers.thicknessM = 0.0825;
            customSidecar.bumpers.offsetM = glm::dvec3(0.0, 0.0, 0.09);
            customSidecar.bumpers.massKg = 6.0;

            MaterialLibrary materials;
            Physics::JoltContext jolt;
            jolt.Initialize();

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(customManifest, customSidecar, materials);

            TEST_EXPECT_TRUE(ctx, robot.isSuccess);

            auto* rb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, rb != nullptr);
            if (rb)
            {
                // Verify chassis mass includes bumpers: 30 kg + 6 kg = 36 kg
                TEST_EXPECT_NEAR(ctx, rb->mass, 36.0f, 1e-4f);
            }

            auto* colComp = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
            TEST_EXPECT_TRUE(ctx, colComp != nullptr);
            if (colComp)
            {
                // 4-piece perimeter bumper generates primary + 3 sub-colliders = 4 boxes
                TEST_EXPECT_TRUE(ctx, colComp->IsCompound());
                TEST_EXPECT_EQ(ctx, colComp->subColliders.size(), static_cast<size_t>(3));
            }
        }

        // =============================================================
        // 13. Robot Comp 2 Assembly Verification (RA13)
        // =============================================================
        {
            std::cout << "  -> RA13: Robot Comp 2 Full Assembly Verification\n";

            std::string comp2Manifest = "FRCBananaSim/Resources/Robots/Robot Comp 2/manifest.json";
            std::string comp2Sidecar = "FRCBananaSim/Resources/Robots/Robot Comp 2/sidecar.json";
            std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

            if (!std::filesystem::exists(comp2Manifest))
            {
                comp2Manifest = "../FRCBananaSim/Resources/Robots/Robot Comp 2/manifest.json";
                comp2Sidecar = "../FRCBananaSim/Resources/Robots/Robot Comp 2/sidecar.json";
                materialsPath = "../FRCBananaSim/Resources/Config/materials.json";
            }

            RobotSchemaLoader loader;
            bool loadOk = loader.LoadRobot(comp2Manifest, comp2Sidecar, materialsPath);
            TEST_EXPECT_TRUE(ctx, loadOk);
            if (!loadOk || loader.HasErrors())
            {
                for (const auto& err : loader.GetErrors())
                {
                    std::cout << "  [Loader Error] " << err << "\n";
                }
            }

            Physics::JoltContext jolt;
            TEST_EXPECT_TRUE(ctx, jolt.Initialize());

            Scene scene;
            RobotAssembler assembler(scene, &jolt);
            auto robot = assembler.Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials());

            if (!robot.isSuccess)
            {
                std::cout << "  [Assembly Error] " << robot.errorMessage << "\n";
            }
            TEST_EXPECT_TRUE(ctx, robot.isSuccess);
            TEST_EXPECT_EQ(ctx, robot.robotName, "Robot Comp 2");
            TEST_EXPECT_TRUE(ctx, robot.chassisEntity != invalidEntity);
            TEST_EXPECT_TRUE(ctx, robot.swerveSystem != nullptr);
            TEST_EXPECT_TRUE(ctx, robot.dispatcher != nullptr);

            std::cout << "  -> Comp 2 Assembly Stats:\n";
            std::cout << "     Links: " << robot.linkEntities.size() << "\n";
            std::cout << "     Joints: " << robot.jointEntities.size() << "\n";
            std::cout << "     Mechanisms: " << robot.mechanismEntities.size() << "\n";
            std::cout << "     Sensors: " << robot.sensorEntities.size() << "\n";
            std::cout << "     ContactZones: " << robot.contactZoneEntities.size() << "\n";
        }
    }
}
