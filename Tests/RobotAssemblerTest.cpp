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
    }
}