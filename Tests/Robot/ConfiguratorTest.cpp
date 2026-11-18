/**
 * @file ConfiguratorTest.cpp
 * @brief Implementation of test suite for SidecarConfigurator document model, auto-generator, and linter.
 */

#include "ConfiguratorTest.h"
#include "TestFramework.h"
#include "Tools/Configurator/ConfiguratorState.h"
#include "Tools/Configurator/Panels/ConfiguratorWidgets.h"
#include "Tools/Configurator/Panels/CADViewportPanel.h"
#include "Robot/Schema/RobotSchemaLoader.h"

#include <filesystem>
#include <iostream>

namespace FRCBananaSim::Tests
{
    using namespace Tools::Configurator;
    using namespace Robot::Schema;

    void RunConfiguratorTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ConfiguratorTest\n";

        std::string refPath = "FRCBananaSim/Resources/Robots/Reference2026";
        if (!std::filesystem::exists(refPath))
        {
            refPath = "../FRCBananaSim/Resources/Robots/Reference2026";
        }

        // =============================================================
        // 1. CF1: CAD Folder Ingestion & Model Binding
        // =============================================================
        {
            std::cout << "  -> CF1: CAD Folder Ingestion & Model Binding\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);
            TEST_EXPECT_TRUE(ctx, state.isCadLoaded);
            TEST_EXPECT_TRUE(ctx, state.isSidecarLoaded);

            TEST_EXPECT_EQ(ctx, state.manifest.links.size(), static_cast<size_t>(9));
            TEST_EXPECT_EQ(ctx, state.manifest.joints.size(), static_cast<size_t>(7));
            TEST_EXPECT_EQ(ctx, state.manifest.contactZones.size(), static_cast<size_t>(2));

            // Reference2026 has zero linter errors
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // TestAssembly2026 Ingestion (Verifies small component inertia tensors like 'wheels')
            std::string testPath = "FRCBananaSim/Resources/Robots/TestAssembly2026";
            if (!std::filesystem::exists(testPath))
            {
                testPath = "../FRCBananaSim/Resources/Robots/TestAssembly2026";
            }
            if (std::filesystem::exists(testPath))
            {
                ConfiguratorState testState;
                bool testOk = testState.LoadCADFolder(testPath);
                TEST_EXPECT_TRUE(ctx, testOk);
                TEST_EXPECT_TRUE(ctx, testState.isCadLoaded);
                TEST_EXPECT_TRUE(ctx, testState.manifest.FindLink("wheels") != nullptr);

                // Auto-generate clean starter sidecar specifically matching TestAssembly2026
                testState.AutoGenerateStarterSidecar();
                testState.SaveSidecar(testPath + "/sidecar.json");

                // Reload and verify 0 linter errors
                ConfiguratorState verifyState;
                bool verifyOk = verifyState.LoadCADFolder(testPath);
                TEST_EXPECT_TRUE(ctx, verifyOk);
                TEST_EXPECT_TRUE(ctx, verifyState.linterErrors.empty());
            }
        }

        // =============================================================
        // 2. CF2: Auto-Generate Starter Sidecar Bootstrap
        // =============================================================
        {
            std::cout << "  -> CF2: Auto-Generate Starter Sidecar Bootstrap\n";

            ConfiguratorState state;
            std::string manifestFile = refPath + "/manifest.json";
            bool ok = state.LoadManifest(manifestFile);
            TEST_EXPECT_TRUE(ctx, ok);
            TEST_EXPECT_TRUE(ctx, state.isCadLoaded);
            TEST_EXPECT_FALSE(ctx, state.isSidecarLoaded);

            state.AutoGenerateStarterSidecar();
            TEST_EXPECT_TRUE(ctx, state.isSidecarLoaded);

            // Verify Swerve Generation
            TEST_EXPECT_EQ(ctx, state.sidecar.swerve.modules.size(), static_cast<size_t>(4));
            TEST_EXPECT_EQ(ctx, state.sidecar.swerve.driveMotorPreset, "KrakenX60");
            TEST_EXPECT_EQ(ctx, state.sidecar.swerve.steerMotorPreset, "KrakenX44");

            // Verify Mechanism Generation
            TEST_EXPECT_TRUE(ctx, !state.sidecar.mechanisms.empty());
            bool foundArm = false;
            bool foundElev = false;
            for (const auto& mech : state.sidecar.mechanisms)
            {
                if (mech.role == "arm" || mech.role == "wrist") foundArm = true;
                if (mech.role == "elevator") foundElev = true;
            }
            TEST_EXPECT_TRUE(ctx, foundArm);
            TEST_EXPECT_TRUE(ctx, foundElev);

            // Verify IMU Sensor Generation
            TEST_EXPECT_TRUE(ctx, !state.sidecar.sensors.empty());
            TEST_EXPECT_EQ(ctx, state.sidecar.sensors[0].type, "imu_pigeon2");

            // Auto-generated sidecar must pass live linter with 0 errors
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());
        }

        // =============================================================
        // 3. CF3: Live Schema Linter Diagnostics
        // =============================================================
        {
            std::cout << "  -> CF3: Live Schema Linter Diagnostics\n";

            ConfiguratorState state;
            state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // A. Trigger CAN ID Collision
            uint32_t origCanId = state.sidecar.sensors[0].canId;
            state.sidecar.sensors[0].canId = state.sidecar.swerve.modules[0].cancoderId;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundCanCollision = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("CAN ID Collision") != std::string::npos) foundCanCollision = true;
            }
            TEST_EXPECT_TRUE(ctx, foundCanCollision);

            // Revert CAN collision
            state.sidecar.sensors[0].canId = origCanId;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // B. Trigger Orphan Joint Reference
            state.sidecar.mechanisms[0].targetJoint = "non_existent_joint_xyz";
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundOrphan = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Orphan Joint Reference") != std::string::npos) foundOrphan = true;
            }
            TEST_EXPECT_TRUE(ctx, foundOrphan);

            // Revert orphan joint
            state.sidecar.mechanisms[0].targetJoint = state.manifest.joints[0].name;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // C. Trigger Differential Wrist Non-Revolute Joint Error
            MechanismConfig diffMech;
            diffMech.name = "invalid_diff";
            diffMech.role = "differential";
            diffMech.targetJoints = { "elevator_stage1_slider", "arm_shoulder_hinge" }; // Prismatic + Revolute
            state.sidecar.mechanisms.push_back(diffMech);

            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundDiffError = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("must be revolute") != std::string::npos) foundDiffError = true;
            }
            TEST_EXPECT_TRUE(ctx, foundDiffError);
        }

        // =============================================================
        // 4. CF4: Save and Reload Round-Trip Serialization
        // =============================================================
        {
            std::cout << "  -> CF4: Save and Reload Round-Trip Serialization\n";

            ConfiguratorState state;
            state.LoadCADFolder(refPath);
            state.AutoGenerateStarterSidecar();

            std::string tempOut = "test_starter_sidecar.json";
            bool saved = state.SaveSidecar(tempOut);
            TEST_EXPECT_TRUE(ctx, saved);
            TEST_EXPECT_TRUE(ctx, std::filesystem::exists(tempOut));

            // Reload via standard loader
            RobotSchemaLoader loader;
            bool reloaded = loader.LoadSidecarFromFile(tempOut);
            TEST_EXPECT_TRUE(ctx, reloaded);

            const auto& loadedSidecar = loader.GetSidecar();
            TEST_EXPECT_EQ(ctx, loadedSidecar.swerve.modules.size(), static_cast<size_t>(4));
            TEST_EXPECT_EQ(ctx, loadedSidecar.mechanisms.size(), state.sidecar.mechanisms.size());
            TEST_EXPECT_EQ(ctx, loadedSidecar.sensors.size(), state.sidecar.sensors.size());

            // Cleanup
            std::filesystem::remove(tempOut);
        }

        // =============================================================
        // 5. CF5: Load and Validate TestAssembly2026 (User Robot)
        // =============================================================
        {
            std::cout << "  -> CF5: Load and Validate TestAssembly2026 (User Robot)\n";

            std::string testPath = "FRCBananaSim/Resources/Robots/TestAssembly2026";
            if (!std::filesystem::exists(testPath))
            {
                testPath = "../FRCBananaSim/Resources/Robots/TestAssembly2026";
            }

            ConfiguratorState state;
            bool loaded = state.LoadCADFolder(testPath);
            TEST_EXPECT_TRUE(ctx, loaded);
            TEST_EXPECT_TRUE(ctx, state.isCadLoaded);
            TEST_EXPECT_TRUE(ctx, state.isSidecarLoaded);
            TEST_EXPECT_TRUE(ctx, state.manifest.links.size() >= 14);
            TEST_EXPECT_TRUE(ctx, state.manifest.joints.size() >= 13);

            state.RunLinter();
            if (!state.linterErrors.empty())
            {
                std::cout << "Linter errors for TestAssembly2026:\n";
                for (const auto& err : state.linterErrors)
                {
                    std::cout << "  - " << err << "\n";
                }
            }
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());
        }

        // =============================================================
        // 5b. CF5b: Load and Validate Robot Comp 2
        // =============================================================
        {
            std::cout << "  -> CF5b: Load and Validate Robot Comp 2\n";

            std::string comp2Path = "FRCBananaSim/Resources/Robots/Robot Comp 2";
            if (!std::filesystem::exists(comp2Path))
            {
                comp2Path = "../FRCBananaSim/Resources/Robots/Robot Comp 2";
            }

            if (std::filesystem::exists(comp2Path))
            {
                ConfiguratorState state;
                bool loaded = state.LoadCADFolder(comp2Path);
                TEST_EXPECT_TRUE(ctx, loaded);
                TEST_EXPECT_TRUE(ctx, state.isCadLoaded);
                TEST_EXPECT_TRUE(ctx, state.isSidecarLoaded);

                state.RunLinter();
                if (!state.linterErrors.empty())
                {
                    std::cout << "Linter errors for Robot Comp 2:\n";
                    for (const auto& err : state.linterErrors)
                    {
                        std::cout << "  - " << err << "\n";
                    }
                }
                TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());
            }
        }

        // =============================================================
        // 6. CF6: Swerve Drivetrain Configuration & Caliper Calculator
        // =============================================================
        {
            std::cout << "  -> CF6: Swerve Drivetrain Configuration & Caliper Calculator\n";

            ConfiguratorState state;
            state.LoadCADFolder(refPath);
            state.AutoGenerateStarterSidecar();

            // Calibrate swerve to SDS MK4i L2
            state.sidecar.swerve.driveGearRatio = 6.75;
            state.sidecar.swerve.steerGearRatio = 21.428;
            state.sidecar.swerve.driveMotorPreset = "Falcon500";
            state.sidecar.swerve.steerMotorPreset = "KrakenX44";

            // Caliper wear measurement: 3.92 inches measured on worn tread
            double caliperInches = 3.92;
            state.sidecar.swerve.wheelRadiusM = (caliperInches * 0.0254) / 2.0;
            TEST_EXPECT_NEAR(ctx, state.sidecar.swerve.wheelRadiusM, 0.049784, 1e-4);

            // Re-run linter, must be valid
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Introduce deliberate CAN collision on swerve modules
            state.sidecar.swerve.modules[0].driveCanId = state.sidecar.swerve.modules[1].driveCanId;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());

            // Revert CAN collision
            state.sidecar.swerve.modules[0].driveCanId = 1;
            state.sidecar.swerve.modules[1].driveCanId = 3;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());
        }

        // =============================================================
        // 7. CF7: Mass Reconciliation & Ballast Calibration
        // =============================================================
        {
            std::cout << "  -> CF7: Mass Reconciliation & Ballast Calibration\n";

            ConfiguratorState state;
            state.LoadCADFolder(refPath);
            state.AutoGenerateStarterSidecar();

            // Enable mass reconciliation
            state.sidecar.massReconciliation.enabled = true;
            state.sidecar.massReconciliation.linkMassMultiplier = 1.05;
            state.sidecar.massReconciliation.unmodeledBallast.targetLink = "base_link";
            state.sidecar.massReconciliation.unmodeledBallast.massKg = 3.25;
            state.sidecar.massReconciliation.unmodeledBallast.offsetM = { 0.05, 0.0, -0.02 };

            // Save to temporary file
            std::string tempSidecar = "test_reconciled_sidecar.json";
            bool saved = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saved);

            // Reload via RobotSchemaLoader
            RobotSchemaLoader loader;
            bool reloaded = loader.LoadSidecarFromFile(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloaded);

            const auto& sidecar = loader.GetSidecar();
            TEST_EXPECT_TRUE(ctx, sidecar.massReconciliation.enabled);
            TEST_EXPECT_NEAR(ctx, sidecar.massReconciliation.linkMassMultiplier, 1.05, 1e-6);
            TEST_EXPECT_EQ(ctx, sidecar.massReconciliation.unmodeledBallast.targetLink, "base_link");
            TEST_EXPECT_NEAR(ctx, sidecar.massReconciliation.unmodeledBallast.massKg, 3.25, 1e-6);
            TEST_EXPECT_NEAR(ctx, sidecar.massReconciliation.unmodeledBallast.offsetM.x, 0.05, 1e-6);

            // Test linter error on invalid ballast target link
            state.sidecar.massReconciliation.unmodeledBallast.targetLink = "non_existent_link";
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());

            // Cleanup
            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 8. CF8: Mechanism Configuration, Multi-Controller & Round-Trip
        // =============================================================
        {
            std::cout << "  -> CF8: Mechanism Configuration, Multi-Controller & Round-Trip\n";

            ConfiguratorState state;
            state.LoadCADFolder(refPath);
            state.AutoGenerateStarterSidecar();

            // Clear auto-generated mechanisms to test manual authoring
            state.sidecar.mechanisms.clear();

            // 1. Arm with multi-controller
            MechanismConfig arm;
            arm.name = "arm";
            arm.role = "arm";
            arm.targetJoint = "arm_shoulder_hinge";
            arm.gearRatio = 64.0;
            arm.efficiency = 0.95;
            arm.coulombFriction = 0.15;
            arm.viscousDamping = 0.05;
            arm.motorPreset = "KrakenX60";
            arm.statorCurrentLimitA = 80.0;
            arm.supplyCurrentLimitA = 40.0;
            arm.controllers = {
                { "CTRE_TalonFX", 11, "rio", false, "leader" },
                { "CTRE_TalonFX", 12, "rio", true, "follower" }
            };
            state.sidecar.mechanisms.push_back(arm);

            // 2. Add dynamic flywheel mechanism (Q4 Fix)
            MechanismConfig flywheel;
            flywheel.name = "shooter_flywheel";
            flywheel.role = "flywheel";
            flywheel.parentLink = "wrist_link";
            flywheel.visualLink = "wrist_link";
            flywheel.flywheelInertiaKgm2 = 0.0035;
            flywheel.spinAxis = { 0.0, 1.0, 0.0 };
            flywheel.mountOffsetM = { 0.04, 0.02, -0.01 };
            flywheel.controllers = {
                { "CTRE_TalonFX", 15, "rio", false, "leader" }
            };
            state.sidecar.mechanisms.push_back(flywheel);

            // 3. Add elevator with spool and spring assist
            MechanismConfig elevator;
            elevator.name = "climber_elevator";
            elevator.role = "elevator";
            elevator.targetJoint = "elevator_stage1_slider";
            elevator.gearRatio = 12.0;
            elevator.drumRadiusM = 0.022;
            elevator.springAssist.type = "constant_force";
            elevator.springAssist.forceN = 55.0;
            elevator.controllers = {
                { "REV_SparkMax", 16, "rio", false, "leader" }
            };
            state.sidecar.mechanisms.push_back(elevator);

            // Run linter - must be valid
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Deliberate orphan joint error check
            MechanismConfig orphanMech;
            orphanMech.name = "orphan_arm";
            orphanMech.role = "arm";
            orphanMech.targetJoint = "non_existent_hinge";
            state.sidecar.mechanisms.push_back(orphanMech);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());

            // Remove orphan
            state.sidecar.mechanisms.pop_back();
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Save to temp file
            std::string tempSidecar = "test_mechanisms_sidecar.json";
            bool saved = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saved);

            // Reload via RobotSchemaLoader
            RobotSchemaLoader loader;
            bool reloaded = loader.LoadSidecarFromFile(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloaded);

            const auto* loadedArm = loader.GetSidecar().FindMechanism("arm");
            TEST_EXPECT_TRUE(ctx, loadedArm != nullptr);
            if (loadedArm)
            {
                TEST_EXPECT_NEAR(ctx, loadedArm->gearRatio, 64.0, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedArm->efficiency, 0.95, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedArm->coulombFriction, 0.15, 1e-6);
                TEST_EXPECT_EQ(ctx, loadedArm->controllers.size(), 2);
                if (loadedArm->controllers.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedArm->controllers[0].canId, 11);
                    TEST_EXPECT_EQ(ctx, loadedArm->controllers[1].canId, 12);
                    TEST_EXPECT_TRUE(ctx, loadedArm->controllers[1].inverted);
                }
            }

            const auto* loadedFlywheel = loader.GetSidecar().FindMechanism("shooter_flywheel");
            TEST_EXPECT_TRUE(ctx, loadedFlywheel != nullptr);
            if (loadedFlywheel)
            {
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->flywheelInertiaKgm2, 0.0035, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.x, 0.04, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.y, 0.02, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.z, -0.01, 1e-6);
            }

            const auto* loadedElevator = loader.GetSidecar().FindMechanism("climber_elevator");
            TEST_EXPECT_TRUE(ctx, loadedElevator != nullptr);
            if (loadedElevator)
            {
                TEST_EXPECT_NEAR(ctx, loadedElevator->drumRadiusM, 0.022, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedElevator->springAssist.forceN, 55.0, 1e-6);
            }

            // Cleanup
            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 9. CF9: Recent Robots History MRU & Persistence
        // =============================================================
        {
            std::cout << "  -> CF9: Recent Robots History MRU & Persistence\n";

            // Path equivalence tests
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent("C:/Robots/Alpha", "C:\\Robots\\Alpha"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent("C:/Robots/Alpha/", "C:/Robots/Alpha"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent("C:/Robots/alpha", "C:/Robots/ALPHA"));
            TEST_EXPECT_FALSE(ctx, ConfiguratorState::ArePathsEquivalent("C:/Robots/Alpha", "C:/Robots/Beta"));

            std::string tempSettings = "test_configurator_settings.json";
            std::filesystem::remove(tempSettings);

            // Initially empty
            auto recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(0));

            // Add 1st robot
            ConfiguratorState::AddRecentRobot("C:/Robots/Robot1", tempSettings);
            recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(1));
            TEST_EXPECT_EQ(ctx, recents[0], std::string("C:/Robots/Robot1"));

            // Add 2nd robot
            ConfiguratorState::AddRecentRobot("C:/Robots/Robot2", tempSettings);
            recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(2));
            TEST_EXPECT_EQ(ctx, recents[0], std::string("C:/Robots/Robot2"));
            TEST_EXPECT_EQ(ctx, recents[1], std::string("C:/Robots/Robot1"));

            // Add 3rd robot
            ConfiguratorState::AddRecentRobot("C:/Robots/Robot3", tempSettings);
            recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(3));
            TEST_EXPECT_EQ(ctx, recents[0], std::string("C:/Robots/Robot3"));
            TEST_EXPECT_EQ(ctx, recents[1], std::string("C:/Robots/Robot2"));
            TEST_EXPECT_EQ(ctx, recents[2], std::string("C:/Robots/Robot1"));

            // Re-add 2nd robot (with backslashes) -> should move to front without duplicating
            ConfiguratorState::AddRecentRobot("C:\\Robots\\Robot2", tempSettings);
            recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(3));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[0], "C:/Robots/Robot2"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[1], "C:/Robots/Robot3"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[2], "C:/Robots/Robot1"));

            // Add 4th robot -> should truncate to 3, dropping Robot1
            ConfiguratorState::AddRecentRobot("C:/Robots/Robot4", tempSettings);
            recents = ConfiguratorState::GetRecentRobots(tempSettings);
            TEST_EXPECT_EQ(ctx, recents.size(), static_cast<size_t>(3));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[0], "C:/Robots/Robot4"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[1], "C:/Robots/Robot2"));
            TEST_EXPECT_TRUE(ctx, ConfiguratorState::ArePathsEquivalent(recents[2], "C:/Robots/Robot3"));

            // Cleanup
            std::filesystem::remove(tempSettings);
        }

        // =============================================================
        // 10. CF10: 2-DOF Differential Wrists, Couplings & Schema Serialization
        // =============================================================
        {
            std::cout << "  -> CF10: 2-DOF Differential Wrists, Couplings & Schema Serialization\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);

            // 1. Differential Mechanism Validation
            MechanismConfig diffWrist;
            diffWrist.name = "diff_wrist";
            diffWrist.role = "differential";
            diffWrist.differentialType = "symmetric_bevel";
            diffWrist.gearRatios = { 45.0, 35.0 };
            diffWrist.coulombFrictions = { 0.12, 0.10 };
            diffWrist.viscousDampings = { 0.04, 0.03 };
            diffWrist.motorPresets = { "KrakenX60", "Falcon500" };
            diffWrist.actuatorTypes = { "can_motor", "can_motor" };
            diffWrist.statorCurrentLimits = { 65.0, 55.0 };
            diffWrist.supplyCurrentLimits = { 40.0, 35.0 };

            // Deliberate invalid test: targetJoints with a prismatic joint
            diffWrist.targetJoints = { "elevator_stage1_slider", "arm_wrist_hinge" };
            state.sidecar.mechanisms.push_back(diffWrist);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must fail because elevator_stage1_slider is not revolute

            // Fix with valid revolute joints
            state.sidecar.mechanisms.back().targetJoints = { "arm_shoulder_hinge", "arm_wrist_hinge" };
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Add dual controllers
            ControllerConfig ctrlA;
            ctrlA.canId = 15;
            ctrlA.vendor = "CTRE_TalonFX";
            ctrlA.canBus = "rio";
            ctrlA.role = "leader";
            ControllerConfig ctrlB;
            ctrlB.canId = 16;
            ctrlB.vendor = "CTRE_TalonFX";
            ctrlB.canBus = "rio";
            ctrlB.role = "follower";
            ctrlB.inverted = true;
            state.sidecar.mechanisms.back().controllers = { ctrlA, ctrlB };

            // 2. Power Coupling
            state.sidecar.couplings.clear(); // Clear initial couplings to test dedicated authoring and reload
            CouplingConfig pCoup;
            pCoup.name = "intake_sync";
            pCoup.type = "power_coupling";
            pCoup.solverMode = "analytic_coupled";
            pCoup.powerCoupling.transmissionType = "belt";
            pCoup.powerCoupling.masterJoint = "arm_shoulder_hinge";
            pCoup.powerCoupling.slaveJoint = "arm_shoulder_hinge"; // Deliberate self-coupling error
            state.sidecar.couplings.push_back(pCoup);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());

            // Fix slave joint
            state.sidecar.couplings.back().powerCoupling.slaveJoint = "arm_wrist_hinge";
            state.sidecar.couplings.back().powerCoupling.ratio = 2.5;
            state.sidecar.couplings.back().powerCoupling.inverted = true;
            state.sidecar.couplings.back().powerCoupling.efficiency = 0.97;
            state.sidecar.couplings.back().powerCoupling.backlashRad = 0.005;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 3. Elevator Cascade Coupling
            CouplingConfig cCoup;
            cCoup.name = "rigged_elevator";
            cCoup.type = "elevator_cascade";
            cCoup.solverMode = "analytic_coupled";

            // Deliberate invalid test: non-prismatic joint in cascade
            StageCouplingConfig badStage;
            badStage.joint = "arm_shoulder_hinge";
            badStage.multiplier = 1.0;
            cCoup.cascadeStages.push_back(badStage);
            state.sidecar.couplings.push_back(cCoup);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());

            // Fix with valid prismatic joints
            state.sidecar.couplings.back().cascadeStages.clear();
            StageCouplingConfig stage1{ "elevator_stage1_slider", 1.0 };
            StageCouplingConfig stage2{ "elevator_carriage_slider", 2.0 };
            state.sidecar.couplings.back().cascadeStages = { stage1, stage2 };
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 4. Four-Bar Linkage Loop
            CouplingConfig fbCoup;
            fbCoup.name = "intake_fourbar";
            fbCoup.type = "four_bar_loop";
            fbCoup.solverMode = "analytic_coupled";
            fbCoup.fourBar.crankJoint = "fourbar_crank_hinge";
            fbCoup.fourBar.couplerJoint = "fourbar_coupler_hinge";
            fbCoup.fourBar.followerJoint = "fourbar_follower_hinge";
            fbCoup.fourBar.closureParentLink = "base_link";
            fbCoup.fourBar.closureChildLink = "intake_coupler_link";
            fbCoup.fourBar.closureParentAnchorM = { 0.15, 0.0, 0.25 };
            fbCoup.fourBar.closureChildAnchorM = { 0.05, 0.0, 0.0 };
            fbCoup.fourBar.closureAxis = { 0.0, 1.0, 0.0 };
            state.sidecar.couplings.push_back(fbCoup);
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 5. Round-Trip Serialization & Verification
            std::string tempSidecar = "test_cf10_sidecar.json";
            bool saved = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saved);

            RobotSchemaLoader loader;
            bool reloaded = loader.LoadSidecarFromFile(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloaded);

            // Verify Differential
            const auto* loadedDiff = loader.GetSidecar().FindMechanism("diff_wrist");
            TEST_EXPECT_TRUE(ctx, loadedDiff != nullptr);
            if (loadedDiff)
            {
                TEST_EXPECT_EQ(ctx, loadedDiff->role, std::string("differential"));
                TEST_EXPECT_EQ(ctx, loadedDiff->differentialType, std::string("symmetric_bevel"));
                TEST_EXPECT_EQ(ctx, loadedDiff->targetJoints.size(), static_cast<size_t>(2));
                if (loadedDiff->targetJoints.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedDiff->targetJoints[0], std::string("arm_shoulder_hinge"));
                    TEST_EXPECT_EQ(ctx, loadedDiff->targetJoints[1], std::string("arm_wrist_hinge"));
                }
                TEST_EXPECT_EQ(ctx, loadedDiff->gearRatios.size(), static_cast<size_t>(2));
                if (loadedDiff->gearRatios.size() == 2)
                {
                    TEST_EXPECT_NEAR(ctx, loadedDiff->gearRatios[0], 45.0, 1e-6);
                    TEST_EXPECT_NEAR(ctx, loadedDiff->gearRatios[1], 35.0, 1e-6);
                }
                TEST_EXPECT_EQ(ctx, loadedDiff->controllers.size(), static_cast<size_t>(2));
                if (loadedDiff->controllers.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedDiff->controllers[0].canId, 15);
                    TEST_EXPECT_EQ(ctx, loadedDiff->controllers[1].canId, 16);
                    TEST_EXPECT_TRUE(ctx, loadedDiff->controllers[1].inverted);
                }
                TEST_EXPECT_EQ(ctx, loadedDiff->motorPresets.size(), static_cast<size_t>(2));
                if (loadedDiff->motorPresets.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedDiff->motorPresets[0], std::string("KrakenX60"));
                    TEST_EXPECT_EQ(ctx, loadedDiff->motorPresets[1], std::string("Falcon500"));
                }
                TEST_EXPECT_EQ(ctx, loadedDiff->statorCurrentLimits.size(), static_cast<size_t>(2));
                if (loadedDiff->statorCurrentLimits.size() == 2)
                {
                    TEST_EXPECT_NEAR(ctx, loadedDiff->statorCurrentLimits[0], 65.0, 1e-6);
                    TEST_EXPECT_NEAR(ctx, loadedDiff->statorCurrentLimits[1], 55.0, 1e-6);
                }
                TEST_EXPECT_EQ(ctx, loadedDiff->supplyCurrentLimits.size(), static_cast<size_t>(2));
                if (loadedDiff->supplyCurrentLimits.size() == 2)
                {
                    TEST_EXPECT_NEAR(ctx, loadedDiff->supplyCurrentLimits[0], 40.0, 1e-6);
                    TEST_EXPECT_NEAR(ctx, loadedDiff->supplyCurrentLimits[1], 35.0, 1e-6);
                }
            }

            // Verify Couplings
            TEST_EXPECT_EQ(ctx, loader.GetSidecar().couplings.size(), static_cast<size_t>(3));
            const auto* loadedPower = loader.GetSidecar().FindCoupling("intake_sync");
            TEST_EXPECT_TRUE(ctx, loadedPower != nullptr);
            if (loadedPower)
            {
                TEST_EXPECT_EQ(ctx, loadedPower->type, std::string("power_coupling"));
                TEST_EXPECT_EQ(ctx, loadedPower->powerCoupling.masterJoint, std::string("arm_shoulder_hinge"));
                TEST_EXPECT_EQ(ctx, loadedPower->powerCoupling.slaveJoint, std::string("arm_wrist_hinge"));
                TEST_EXPECT_NEAR(ctx, loadedPower->powerCoupling.ratio, 2.5, 1e-6);
                TEST_EXPECT_TRUE(ctx, loadedPower->powerCoupling.inverted);
                TEST_EXPECT_NEAR(ctx, loadedPower->powerCoupling.efficiency, 0.97, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedPower->powerCoupling.backlashRad, 0.005, 1e-6);
            }

            const auto* loadedCascade = loader.GetSidecar().FindCoupling("rigged_elevator");
            TEST_EXPECT_TRUE(ctx, loadedCascade != nullptr);
            if (loadedCascade)
            {
                TEST_EXPECT_EQ(ctx, loadedCascade->type, std::string("elevator_cascade"));
                TEST_EXPECT_EQ(ctx, loadedCascade->cascadeStages.size(), static_cast<size_t>(2));
                if (loadedCascade->cascadeStages.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedCascade->cascadeStages[0].joint, std::string("elevator_stage1_slider"));
                    TEST_EXPECT_NEAR(ctx, loadedCascade->cascadeStages[0].multiplier, 1.0, 1e-6);
                    TEST_EXPECT_EQ(ctx, loadedCascade->cascadeStages[1].joint, std::string("elevator_carriage_slider"));
                    TEST_EXPECT_NEAR(ctx, loadedCascade->cascadeStages[1].multiplier, 2.0, 1e-6);
                }
            }

            const auto* loadedFB = loader.GetSidecar().FindCoupling("intake_fourbar");
            TEST_EXPECT_TRUE(ctx, loadedFB != nullptr);
            if (loadedFB)
            {
                TEST_EXPECT_EQ(ctx, loadedFB->type, std::string("four_bar_loop"));
                TEST_EXPECT_EQ(ctx, loadedFB->fourBar.crankJoint, std::string("fourbar_crank_hinge"));
                TEST_EXPECT_EQ(ctx, loadedFB->fourBar.couplerJoint, std::string("fourbar_coupler_hinge"));
                TEST_EXPECT_EQ(ctx, loadedFB->fourBar.followerJoint, std::string("fourbar_follower_hinge"));
                TEST_EXPECT_EQ(ctx, loadedFB->fourBar.closureParentLink, std::string("base_link"));
                TEST_EXPECT_EQ(ctx, loadedFB->fourBar.closureChildLink, std::string("intake_coupler_link"));
                TEST_EXPECT_NEAR(ctx, loadedFB->fourBar.closureParentAnchorM.x, 0.15, 1e-6);
                TEST_EXPECT_NEAR(ctx, loadedFB->fourBar.closureAxis.y, 1.0, 1e-6);
            }

            // Cleanup
            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 11. CF11: Sensor Suite Panel, Multi-Bus IMU/DIO/Analog & Linter Diagnostics
        // =============================================================
        {
            std::cout << "  -> CF11: Sensor Suite Panel, Multi-Bus IMU/DIO/Analog & Linter Diagnostics\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);

            state.sidecar.sensors.clear();

            // 1. CANcoder with deliberate orphan joint
            SensorConfig badCancoder;
            badCancoder.name = "wrist_encoder";
            badCancoder.type = "cancoder";
            badCancoder.canId = 22;
            badCancoder.bus = "rio";
            badCancoder.targetJoint = "non_existent_joint_xyz";
            state.sidecar.sensors.push_back(badCancoder);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must fail orphan joint check

            // Fix target joint to valid revolute joint
            state.sidecar.sensors.back().targetJoint = "arm_wrist_hinge";
            state.sidecar.sensors.back().sensorToMechanismRatio = 1.5;
            state.sidecar.sensors.back().offsetRad = 0.25;
            state.sidecar.sensors.back().inverted = true;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 2. Optical Beam Break with deliberate orphan contact zone
            SensorConfig badBeamBreak;
            badBeamBreak.name = "intake_sensor";
            badBeamBreak.type = "beam_break";
            badBeamBreak.dioChannel = 0;
            badBeamBreak.contactZone = "invalid_zone_404";
            state.sidecar.sensors.push_back(badBeamBreak);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must fail orphan contact zone check

            // Fix contact zone (Reference2026 defines intake_capture_zone)
            state.sidecar.sensors.back().contactZone = "intake_capture_zone";
            state.sidecar.sensors.back().normallyClosed = true;
            state.sidecar.sensors.back().debounceTimeS = 0.015;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 3. Limit Switch with deliberate DIO collision (both using DIO 0)
            SensorConfig badLimit;
            badLimit.name = "elevator_top_limit";
            badLimit.type = "limit_switch";
            badLimit.dioChannel = 0; // Collision with intake_sensor DIO 0!
            badLimit.targetJoint = "elevator_stage1_slider";
            badLimit.limitDirection = "forward";
            badLimit.triggerMin = 0.45;
            badLimit.triggerMax = 0.50;
            state.sidecar.sensors.push_back(badLimit);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must catch DIO collision

            // Fix DIO channel to unique channel 1
            state.sidecar.sensors.back().dioChannel = 1;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 4. Analog Input with deliberate duplicate analog channel
            SensorConfig pot1;
            pot1.name = "pot_shoulder";
            pot1.type = "analog_input";
            pot1.analogChannel = 0;
            pot1.targetJoint = "arm_shoulder_hinge";
            pot1.triggerMin = 0.0;
            pot1.triggerMax = 2.5;
            state.sidecar.sensors.push_back(pot1);

            SensorConfig badPot2;
            badPot2.name = "pot_duplicate";
            badPot2.type = "analog_input";
            badPot2.analogChannel = 0; // Collision on analog channel 0!
            badPot2.targetJoint = "arm_shoulder_hinge";
            state.sidecar.sensors.push_back(badPot2);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must catch Analog collision

            // Remove duplicate pot
            state.sidecar.sensors.pop_back();
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 5. Add Pigeon 2.0 IMU
            SensorConfig pige;
            pige.name = "chassis_imu";
            pige.type = "imu_pigeon2";
            pige.canId = 25;
            pige.bus = "rio";
            pige.parentLink = "base_link";
            pige.mountOffsetM = { 0.05, -0.02, 0.12 };
            pige.gyroNoiseStdDevRadS = 0.002;
            pige.gyroBiasRandomWalkRadS2 = 0.00005;
            pige.accelNoiseStdDevMS2 = 0.015;
            state.sidecar.sensors.push_back(pige);
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 6. REV Through Bore Encoder Validation (Dual-channel quadrature and collision)
            SensorConfig revEnc;
            revEnc.name = "arm_through_bore";
            revEnc.type = "rev_through_bore";
            revEnc.dioChannel = 2;
            revEnc.secondaryDioChannel = 2; // Deliberate Channel A == Channel B collision!
            revEnc.customStringParams["encoderMode"] = "quadrature";
            revEnc.targetJoint = "arm_shoulder_hinge";
            revEnc.sensorToMechanismRatio = 1.0;
            revEnc.cpr = 8192;
            revEnc.offsetRad = 0.0;
            state.sidecar.sensors.push_back(revEnc);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must detect Channel A == Channel B collision!

            // Fix Channel B to unique DIO channel 3
            state.sidecar.sensors.back().secondaryDioChannel = 3;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 7. AndyMark LIDAR-3 / CAN Lidar Validation
            SensorConfig lidar;
            lidar.name = "intake_lidar";
            lidar.type = "andymark_lidar3";
            lidar.bus = "rio";
            lidar.canId = 35;
            lidar.parentLink = "base_link";
            lidar.triggerMin = 5.0;  // Deliberate min > max error!
            lidar.triggerMax = 4.0;
            lidar.mountOffsetM = { 0.10, 0.05, 0.20 };
            lidar.customNumericParams["noiseStdDevM"] = 0.005;
            state.sidecar.sensors.push_back(lidar);
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty()); // Must catch Min Range >= Max Range error!

            // Fix LIDAR min range to 0.03m
            state.sidecar.sensors.back().triggerMin = 0.03;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 8. Round-trip serialization & reload
            std::string tempSidecar = "test_cf11_sidecar.json";
            bool saved = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saved);

            RobotSchemaLoader loader;
            bool reloaded = loader.LoadSidecarFromFile(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloaded);

            const auto& loadedSensors = loader.GetSidecar().sensors;
            TEST_EXPECT_EQ(ctx, loadedSensors.size(), static_cast<size_t>(7));

            // Verify CANcoder
            const auto* sCancoder = loader.GetSidecar().FindSensor("wrist_encoder");
            TEST_EXPECT_TRUE(ctx, sCancoder != nullptr);
            if (sCancoder)
            {
                TEST_EXPECT_EQ(ctx, sCancoder->type, std::string("cancoder"));
                TEST_EXPECT_EQ(ctx, sCancoder->canId, 22);
                TEST_EXPECT_EQ(ctx, sCancoder->bus, std::string("rio"));
                TEST_EXPECT_EQ(ctx, sCancoder->targetJoint, std::string("arm_wrist_hinge"));
                TEST_EXPECT_NEAR(ctx, sCancoder->sensorToMechanismRatio, 1.5, 1e-6);
                TEST_EXPECT_NEAR(ctx, sCancoder->offsetRad, 0.25, 1e-6);
                TEST_EXPECT_TRUE(ctx, sCancoder->inverted);
            }

            // Verify Beam Break
            const auto* sBeam = loader.GetSidecar().FindSensor("intake_sensor");
            TEST_EXPECT_TRUE(ctx, sBeam != nullptr);
            if (sBeam)
            {
                TEST_EXPECT_EQ(ctx, sBeam->type, std::string("beam_break"));
                TEST_EXPECT_EQ(ctx, sBeam->dioChannel, 0);
                TEST_EXPECT_EQ(ctx, sBeam->contactZone, std::string("intake_capture_zone"));
                TEST_EXPECT_TRUE(ctx, sBeam->normallyClosed);
                TEST_EXPECT_NEAR(ctx, sBeam->debounceTimeS, 0.015, 1e-6);
            }

            // Verify Limit Switch
            const auto* sLimit = loader.GetSidecar().FindSensor("elevator_top_limit");
            TEST_EXPECT_TRUE(ctx, sLimit != nullptr);
            if (sLimit)
            {
                TEST_EXPECT_EQ(ctx, sLimit->type, std::string("limit_switch"));
                TEST_EXPECT_EQ(ctx, sLimit->dioChannel, 1);
                TEST_EXPECT_EQ(ctx, sLimit->limitDirection, std::string("forward"));
                TEST_EXPECT_EQ(ctx, sLimit->targetJoint, std::string("elevator_stage1_slider"));
                TEST_EXPECT_NEAR(ctx, sLimit->triggerMin, 0.45, 1e-6);
                TEST_EXPECT_NEAR(ctx, sLimit->triggerMax, 0.50, 1e-6);
            }

            // Verify Analog Input
            const auto* sPot = loader.GetSidecar().FindSensor("pot_shoulder");
            TEST_EXPECT_TRUE(ctx, sPot != nullptr);
            if (sPot)
            {
                TEST_EXPECT_EQ(ctx, sPot->type, std::string("analog_input"));
                TEST_EXPECT_EQ(ctx, sPot->analogChannel, 0);
                TEST_EXPECT_EQ(ctx, sPot->targetJoint, std::string("arm_shoulder_hinge"));
                TEST_EXPECT_NEAR(ctx, sPot->triggerMin, 0.0, 1e-6);
                TEST_EXPECT_NEAR(ctx, sPot->triggerMax, 2.5, 1e-6);
            }

            // Verify IMU
            const auto* sImu = loader.GetSidecar().FindSensor("chassis_imu");
            TEST_EXPECT_TRUE(ctx, sImu != nullptr);
            if (sImu)
            {
                TEST_EXPECT_EQ(ctx, sImu->type, std::string("imu_pigeon2"));
                TEST_EXPECT_EQ(ctx, sImu->canId, 25);
                TEST_EXPECT_EQ(ctx, sImu->bus, std::string("rio"));
                TEST_EXPECT_EQ(ctx, sImu->parentLink, std::string("base_link"));
                TEST_EXPECT_NEAR(ctx, sImu->mountOffsetM.x, 0.05, 1e-6);
                TEST_EXPECT_NEAR(ctx, sImu->mountOffsetM.y, -0.02, 1e-6);
                TEST_EXPECT_NEAR(ctx, sImu->mountOffsetM.z, 0.12, 1e-6);
                TEST_EXPECT_NEAR(ctx, sImu->gyroNoiseStdDevRadS, 0.002, 1e-6);
                TEST_EXPECT_NEAR(ctx, sImu->gyroBiasRandomWalkRadS2, 0.00005, 1e-6);
                TEST_EXPECT_NEAR(ctx, sImu->accelNoiseStdDevMS2, 0.015, 1e-6);
            }

            // Verify REV Through Bore Encoder
            const auto* sRev = loader.GetSidecar().FindSensor("arm_through_bore");
            TEST_EXPECT_TRUE(ctx, sRev != nullptr);
            if (sRev)
            {
                TEST_EXPECT_EQ(ctx, sRev->type, std::string("rev_through_bore"));
                TEST_EXPECT_EQ(ctx, sRev->dioChannel, 2);
                TEST_EXPECT_EQ(ctx, sRev->secondaryDioChannel, 3);
                TEST_EXPECT_EQ(ctx, sRev->targetJoint, std::string("arm_shoulder_hinge"));
                TEST_EXPECT_EQ(ctx, sRev->cpr, 8192);
                TEST_EXPECT_NEAR(ctx, sRev->sensorToMechanismRatio, 1.0, 1e-6);
            }

            // Verify AndyMark LIDAR 3
            const auto* sLidar = loader.GetSidecar().FindSensor("intake_lidar");
            TEST_EXPECT_TRUE(ctx, sLidar != nullptr);
            if (sLidar)
            {
                TEST_EXPECT_EQ(ctx, sLidar->type, std::string("andymark_lidar3"));
                TEST_EXPECT_EQ(ctx, sLidar->canId, 35);
                TEST_EXPECT_EQ(ctx, sLidar->bus, std::string("rio"));
                TEST_EXPECT_EQ(ctx, sLidar->parentLink, std::string("base_link"));
                TEST_EXPECT_NEAR(ctx, sLidar->triggerMin, 0.03, 1e-6);
                TEST_EXPECT_NEAR(ctx, sLidar->triggerMax, 4.0, 1e-6);
                TEST_EXPECT_NEAR(ctx, sLidar->mountOffsetM.x, 0.10, 1e-6);
                TEST_EXPECT_NEAR(ctx, sLidar->mountOffsetM.y, 0.05, 1e-6);
                TEST_EXPECT_NEAR(ctx, sLidar->mountOffsetM.z, 0.20, 1e-6);
            }

            // Cleanup
            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 9. CF9: Intelligent Unit Parsing & Auto-Conversion
        // =============================================================
        {
            std::cout << "  -> CF9: Intelligent Unit Parsing & Auto-Conversion\n";

            float val = 0.0f;

            // 1. Length in meters: user types inches with quote "
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("24\"", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.6096, 1e-4);

            // 2. Length in meters: user types "in" or "inch"
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("24 in", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.6096, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("24inch", "Trackwidth (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.6096, 1e-4);

            // 3. Length in meters: user types millimeters and centimeters
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("600 mm", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.6000, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("60 cm", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.6000, 1e-4);

            // 4. Length in meters: user types feet ' or ft
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("2.5 ft", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.7620, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("2.5'", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.7620, 1e-4);

            // 5. Length in meters: raw value or m
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("0.884 m", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.8840, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("0.884", "Wheelbase (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.8840, 1e-4);

            // 6. Target Inches: user types millimeters
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("100 mm", "Caliper Diam (in)", val));
            TEST_EXPECT_NEAR(ctx, val, 3.9370, 1e-3);

            // 7. Target Millimeters: user types inches
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("4\"", "Caliper Diam (mm)", val));
            TEST_EXPECT_NEAR(ctx, val, 101.60, 1e-2);

            // 8. Mass in kg: user types lbs or #
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("120 lbs", "Bench Scale Mass (kg)", val));
            TEST_EXPECT_NEAR(ctx, val, 54.431, 1e-3);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("120#", "Ballast Mass (kg)", val));
            TEST_EXPECT_NEAR(ctx, val, 54.431, 1e-3);

            // 9. Angle in radians: user types deg or °
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("90 deg", "Magnet Offset (rad)##CANcoder", val));
            TEST_EXPECT_NEAR(ctx, val, 1.570796, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("180°", "Magnet Offset (rad)", val));
            TEST_EXPECT_NEAR(ctx, val, 3.141592, 1e-4);

            // 10. Angle in degrees: user types rad
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("3.14159 rad", "Travel Range (deg)", val));
            TEST_EXPECT_NEAR(ctx, val, 180.0, 1e-2);

            // 11. Servo Speed (s/60 deg) <-> RPM
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("100 rpm", "Speed (s/60 deg)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.100, 1e-4);
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("0.100 s/60", "Free Speed (RPM)", val));
            TEST_EXPECT_NEAR(ctx, val, 100.0, 1e-2);

            // 12. Torque in N*m: user types kg*cm
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("20 kg*cm", "Stall Torque (N*m)", val));
            TEST_EXPECT_NEAR(ctx, val, 1.9613, 1e-3);

            // 13. Expressions / Fractions: 12/60
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("12 / 60", "Drive Gear Reduction", val));
            TEST_EXPECT_NEAR(ctx, val, 0.200, 1e-4);

            // 14. Fraction with unit: 24/2" -> 12" -> 0.3048 m
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("24/2\"", "Trackwidth (m)", val));
            TEST_EXPECT_NEAR(ctx, val, 0.3048, 1e-4);

            // 15. Gearbox reduction fraction on ratio field: 1/9
            TEST_EXPECT_TRUE(ctx, ParseUnitExpression("1/9", "Velocity Ratio (slave / master)##CoupRatio", val));
            TEST_EXPECT_NEAR(ctx, val, 1.0 / 9.0, 1e-4);
        }

        // =============================================================
        // 10. CF10: Joint Limit Overrides and Linter Validation
        // =============================================================
        {
            std::cout << "  -> CF10: Joint Limit Overrides and Linter Validation\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);

            // Initially Reference2026 has zero errors
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 1. Add valid override for an existing joint
            JointLimitOverride validOv;
            validOv.hasLimits = true;
            validOv.lower = -1.57;
            validOv.upper = 1.57;
            validOv.unit = "rad";
            state.sidecar.jointOverrides["arm_shoulder_hinge"] = validOv;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Verify effective limits reflects the override
            auto eff = state.sidecar.GetEffectiveLimits("arm_shoulder_hinge", state.manifest);
            TEST_EXPECT_TRUE(ctx, eff.hasLimits);
            TEST_EXPECT_NEAR(ctx, eff.lower, -1.57, 1e-4);
            TEST_EXPECT_NEAR(ctx, eff.upper, 1.57, 1e-4);

            // 2. Add invalid limits: lower > upper
            JointLimitOverride invalidOv;
            invalidOv.hasLimits = true;
            invalidOv.lower = 2.5;
            invalidOv.upper = 0.5;
            state.sidecar.jointOverrides["arm_shoulder_hinge"] = invalidOv;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundLimitError = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Joint Limits Error") != std::string::npos) foundLimitError = true;
            }
            TEST_EXPECT_TRUE(ctx, foundLimitError);

            // Revert back to valid
            state.sidecar.jointOverrides["arm_shoulder_hinge"] = validOv;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 3. Add orphan joint override
            JointLimitOverride orphanOv;
            orphanOv.hasLimits = true;
            state.sidecar.jointOverrides["non_existent_joint_xyz"] = orphanOv;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundOrphanError = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Orphan Joint Override") != std::string::npos) foundOrphanError = true;
            }
            TEST_EXPECT_TRUE(ctx, foundOrphanError);

            // Remove orphan override
            state.sidecar.jointOverrides.erase("non_existent_joint_xyz");
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 4. Test Save and Reload of sidecar with jointOverrides
            std::string tempSidecar = (std::filesystem::temp_directory_path() / "test_sidecar_limits.json").string();
            bool saveOk = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saveOk);

            ConfiguratorState reloadState;
            reloadState.manifest = state.manifest;
            reloadState.isCadLoaded = true;
            bool reloadOk = reloadState.LoadSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloadOk);
            TEST_EXPECT_TRUE(ctx, reloadState.sidecar.FindJointOverride("arm_shoulder_hinge") != nullptr);
            const auto* reloadedArmOv = reloadState.sidecar.FindJointOverride("arm_shoulder_hinge");
            TEST_EXPECT_TRUE(ctx, reloadedArmOv->hasLimits);
            TEST_EXPECT_NEAR(ctx, reloadedArmOv->lower, -1.57, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadedArmOv->upper, 1.57, 1e-4);

            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 11. CF12: Bumper Configuration and Linter Validation
        // =============================================================
        {
            std::cout << "  -> CF12: Bumper Configuration and Linter Validation\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);

            // Initially Reference2026 has zero errors
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 1. Configure valid perimeter bumpers
            state.sidecar.bumpers.enabled = true;
            state.sidecar.bumpers.targetLink = "base_link";
            state.sidecar.bumpers.shape = "perimeter";
            state.sidecar.bumpers.lengthM = 0.85;
            state.sidecar.bumpers.widthM = 0.85;
            state.sidecar.bumpers.heightM = 0.127;
            state.sidecar.bumpers.thicknessM = 0.0825;
            state.sidecar.bumpers.massKg = 6.0;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 2. Invalid target link
            state.sidecar.bumpers.targetLink = "missing_chassis_link";
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundTargetError = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Bumper Target Link Error") != std::string::npos) foundTargetError = true;
            }
            TEST_EXPECT_TRUE(ctx, foundTargetError);

            // Revert target link
            state.sidecar.bumpers.targetLink = "base_link";
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 3. Non-positive dimensions error
            state.sidecar.bumpers.lengthM = -0.5;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            state.sidecar.bumpers.lengthM = 0.85;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 4. Excessive wall thickness error
            state.sidecar.bumpers.thicknessM = 0.50; // 0.50 * 2 = 1.0 > 0.85
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            state.sidecar.bumpers.thicknessM = 0.0825;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 5. Overweight warning (> 15 lbs / 6.804 kg)
            state.sidecar.bumpers.massKg = 8.0;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty()); // No hard error
            bool foundWeightWarn = false;
            for (const auto& w : state.linterWarnings)
            {
                if (w.find("Bumper Rule Warning") != std::string::npos) foundWeightWarn = true;
            }
            TEST_EXPECT_TRUE(ctx, foundWeightWarn);
            state.sidecar.bumpers.massKg = 6.0;
            state.RunLinter();

            // 6. Test Save and Reload of sidecar with bumpers
            std::string tempSidecar = (std::filesystem::temp_directory_path() / "test_sidecar_bumpers.json").string();
            bool saveOk = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saveOk);

            ConfiguratorState reloadState;
            reloadState.manifest = state.manifest;
            reloadState.isCadLoaded = true;
            bool reloadOk = reloadState.LoadSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloadOk);
            TEST_EXPECT_TRUE(ctx, reloadState.sidecar.bumpers.enabled);
            TEST_EXPECT_EQ(ctx, reloadState.sidecar.bumpers.targetLink, "base_link");
            TEST_EXPECT_EQ(ctx, reloadState.sidecar.bumpers.shape, "perimeter");
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.bumpers.lengthM, 0.85, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.bumpers.widthM, 0.85, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.bumpers.heightM, 0.127, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.bumpers.thicknessM, 0.0825, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.bumpers.massKg, 6.0, 1e-4);

            std::filesystem::remove(tempSidecar);
        }

        // =============================================================
        // 11. CF11: Contact Zones Document Model & Linter Diagnostics
        // =============================================================
        {
            std::cout << "  -> CF11: Contact Zones Document Model & Linter Diagnostics\n";

            ConfiguratorState state;
            bool ok = state.LoadCADFolder(refPath);
            TEST_EXPECT_TRUE(ctx, ok);
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Initially Reference2026 has 2 CAD contact zones in manifest:
            // "zone_intake_roller" and "zone_shooter_nip"
            auto initialZones = state.GetEffectiveContactZones();
            TEST_EXPECT_EQ(ctx, initialZones.size(), static_cast<size_t>(2));

            // 1. Add valid sidecar contact zone
            ContactZoneDefinition newZone;
            newZone.name = "indexer_stage1";
            newZone.parentLink = "base_link";
            newZone.role = "intake";
            newZone.dimensionsM = { 0.2, 0.2, 0.1 };
            newZone.tangentAxis = { 0.0, 1.0, 0.0 };
            newZone.compressionAxis = { 0.0, 0.0, 1.0 };
            newZone.nominalGapM = 0.038;
            state.sidecar.contactZones.push_back(newZone);

            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());
            TEST_EXPECT_EQ(ctx, state.GetEffectiveContactZones().size(), static_cast<size_t>(3));

            // 2. Non-existent parent link error
            state.sidecar.contactZones.back().parentLink = "non_existent_link_xyz";
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundParentErr = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Contact Zone Error: Zone 'indexer_stage1' references non-existent parentLink") != std::string::npos)
                {
                    foundParentErr = true;
                }
            }
            TEST_EXPECT_TRUE(ctx, foundParentErr);
            state.sidecar.contactZones.back().parentLink = "base_link";
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 3. Non-positive size error
            state.sidecar.contactZones.back().dimensionsM.x = 0.0;
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundSizeErr = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("dimensions must be strictly positive") != std::string::npos)
                {
                    foundSizeErr = true;
                }
            }
            TEST_EXPECT_TRUE(ctx, foundSizeErr);
            state.sidecar.contactZones.back().dimensionsM.x = 0.2;
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 4. Non-orthogonal tangent and compression axes warning
            state.sidecar.contactZones.back().tangentAxis = { 1.0, 0.0, 0.0 };
            state.sidecar.contactZones.back().compressionAxis = { 1.0, 0.0, 0.0 }; // Collinear!
            state.RunLinter();
            bool foundOrthWarn = false;
            for (const auto& w : state.linterWarnings)
            {
                if (w.find("tangentAxis and compressionAxis are not perpendicular") != std::string::npos)
                {
                    foundOrthWarn = true;
                }
            }
            TEST_EXPECT_TRUE(ctx, foundOrthWarn);
            state.sidecar.contactZones.back().compressionAxis = { 0.0, 0.0, 1.0 };
            state.RunLinter();

            // 5. Beam break sensor binding to sidecar contact zone
            SensorConfig beamBreak;
            beamBreak.name = "indexer_entry_break";
            beamBreak.type = "beam_break";
            beamBreak.dioChannel = 15; // unused channel
            beamBreak.contactZone = "indexer_stage1";
            state.sidecar.sensors.push_back(beamBreak);

            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // Point beam break to non-existent zone
            state.sidecar.sensors.back().contactZone = "unknown_zone";
            state.RunLinter();
            TEST_EXPECT_FALSE(ctx, state.linterErrors.empty());
            bool foundBreakZoneErr = false;
            for (const auto& err : state.linterErrors)
            {
                if (err.find("Orphan Contact Zone Reference: Sensor 'indexer_entry_break' references non-existent contact zone 'unknown_zone'") != std::string::npos)
                {
                    foundBreakZoneErr = true;
                }
            }
            TEST_EXPECT_TRUE(ctx, foundBreakZoneErr);
            state.sidecar.sensors.back().contactZone = "indexer_stage1";
            state.RunLinter();
            TEST_EXPECT_TRUE(ctx, state.linterErrors.empty());

            // 6. Save and reload sidecar with contact zones
            std::string tempSidecar = (std::filesystem::temp_directory_path() / "test_sidecar_contactzones.json").string();
            bool saveOk = state.SaveSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, saveOk);

            ConfiguratorState reloadState;
            reloadState.manifest = state.manifest;
            reloadState.isCadLoaded = true;
            bool reloadOk = reloadState.LoadSidecar(tempSidecar);
            TEST_EXPECT_TRUE(ctx, reloadOk);
            TEST_EXPECT_EQ(ctx, reloadState.sidecar.contactZones.size(), static_cast<size_t>(1));
            TEST_EXPECT_EQ(ctx, reloadState.sidecar.contactZones[0].name, "indexer_stage1");
            TEST_EXPECT_EQ(ctx, reloadState.sidecar.contactZones[0].parentLink, "base_link");
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.contactZones[0].dimensionsM.x, 0.2, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.contactZones[0].dimensionsM.y, 0.2, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.contactZones[0].dimensionsM.z, 0.1, 1e-4);
            TEST_EXPECT_NEAR(ctx, reloadState.sidecar.contactZones[0].nominalGapM, 0.038, 1e-4);

            std::filesystem::remove(tempSidecar);

            // 7. Verify deletion of contact zones down to empty without crash
            ConfiguratorState emptyZoneState;
            emptyZoneState.manifest.robotName = "TestEmpty";
            Robot::Schema::ContactZoneDefinition czOnly;
            czOnly.name = "single_custom_zone";
            czOnly.parentLink = "base_link";
            czOnly.role = "intake";
            czOnly.dimensionsM = { 0.1, 0.1, 0.1 };
            emptyZoneState.sidecar.contactZones.push_back(czOnly);
            emptyZoneState.selectedContactZone = czOnly.name;

            TEST_EXPECT_EQ(ctx, emptyZoneState.GetEffectiveContactZones().size(), static_cast<size_t>(1));

            // Simulate deletion of the only contact zone
            auto it = std::remove_if(emptyZoneState.sidecar.contactZones.begin(), emptyZoneState.sidecar.contactZones.end(),
                [&](const auto& z) { return z.name == czOnly.name; });
            emptyZoneState.sidecar.contactZones.erase(it, emptyZoneState.sidecar.contactZones.end());
            auto remaining = emptyZoneState.GetEffectiveContactZones();
            emptyZoneState.selectedContactZone = remaining.empty() ? "" : remaining[0].name;

            TEST_EXPECT_TRUE(ctx, emptyZoneState.selectedContactZone.empty());
            TEST_EXPECT_TRUE(ctx, emptyZoneState.GetEffectiveContactZones().empty());

            // Ensure FindContactZone on empty/deleted name is safely nullptr and doesn't crash
            TEST_EXPECT_TRUE(ctx, emptyZoneState.sidecar.FindContactZone(emptyZoneState.selectedContactZone) == nullptr);
            TEST_EXPECT_TRUE(ctx, emptyZoneState.manifest.FindContactZone(emptyZoneState.selectedContactZone) == nullptr);
            emptyZoneState.RunLinter();
            TEST_EXPECT_TRUE(ctx, emptyZoneState.linterErrors.empty());
        }

        // =============================================================
        // 12. CF12: Interactive CAD Viewport Canonical Views, Orthographic Projection & Camera Preferences Persistence
        // =============================================================
        {
            std::cout << "  -> CF12: Interactive CAD Viewport Canonical Views, Ortho Projection & Camera Preferences\n";

            bool createdImGui = false;
            if (ImGui::GetCurrentContext() == nullptr)
            {
                ImGui::CreateContext();
                createdImGui = true;
            }

            CADViewportPanel viewport;

            // 1. Initial Default State
            viewport.ResetCamera();
            TEST_EXPECT_FALSE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetDistance(), 2.2f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 215.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 25.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetTarget().y, 0.25f, 1e-3f);
            TEST_EXPECT_TRUE(ctx, viewport.GetCurrentViewName().find("Perspective") != std::string::npos);

            // 2. Canonical Views & Auto-Ortho Snapping
            // Right (+X)
            viewport.SetCanonicalView(CanonicalView::Right, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 90.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 0.0f, 1e-3f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Right (+X) [Orthographic]");

            // Left (-X)
            viewport.SetCanonicalView(CanonicalView::Left, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 270.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 0.0f, 1e-3f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Left (-X) [Orthographic]");

            // Top (+Y)
            viewport.SetCanonicalView(CanonicalView::Top, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 180.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 89.99f, 1e-2f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Top (+Y) [Orthographic]");

            // Bottom (-Y)
            viewport.SetCanonicalView(CanonicalView::Bottom, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 180.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), -89.99f, 1e-2f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Bottom (-Y) [Orthographic]");

            // Front (+Z)
            viewport.SetCanonicalView(CanonicalView::Front, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 0.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 0.0f, 1e-3f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Front (+Z) [Orthographic]");

            // Back (-Z)
            viewport.SetCanonicalView(CanonicalView::Back, /*makeOrtho=*/true);
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, viewport.GetYaw(), 180.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, viewport.GetPitch(), 0.0f, 1e-3f);
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Back (-Z) [Orthographic]");

            // 3. Orthographic Toggle
            viewport.ToggleOrthographic();
            TEST_EXPECT_FALSE(ctx, viewport.IsOrthographic());
            TEST_EXPECT_EQ(ctx, viewport.GetCurrentViewName(), "Back (-Z) [Perspective]");
            viewport.ToggleOrthographic();
            TEST_EXPECT_TRUE(ctx, viewport.IsOrthographic());

            // 4. Persistence of Preferences to JSON
            std::string tempSettings = (std::filesystem::temp_directory_path() / "test_configurator_cam_settings.json").string();
            viewport.SetPreset(CameraPreset::Blender);
            viewport.GetCameraConfig().invertScrollZoom = false;
            viewport.GetCameraConfig().invertOrbitX = true;
            viewport.GetCameraConfig().invertOrbitY = true;
            viewport.GetCameraConfig().orbitSensitivity = 0.55f;
            viewport.GetCameraConfig().panSensitivity = 0.0042f;
            viewport.GetCameraConfig().zoomSensitivity = 0.35f;
            viewport.GetCameraConfig().fovDegrees = 60.0f;
            viewport.SetYaw(135.0f);
            viewport.SetPitch(40.0f);
            viewport.SetDistance(3.75f);
            viewport.SetTarget(glm::vec3(0.12f, 0.45f, -0.22f));

            viewport.SaveSettings(tempSettings);
            TEST_EXPECT_TRUE(ctx, std::filesystem::exists(tempSettings));

            // Reload into fresh panel
            CADViewportPanel loadedPanel;
            loadedPanel.LoadSettings(tempSettings);

            TEST_EXPECT_TRUE(ctx, loadedPanel.GetCameraConfig().preset == CameraPreset::Blender);
            TEST_EXPECT_FALSE(ctx, loadedPanel.GetCameraConfig().invertScrollZoom);
            TEST_EXPECT_TRUE(ctx, loadedPanel.GetCameraConfig().invertOrbitX);
            TEST_EXPECT_TRUE(ctx, loadedPanel.GetCameraConfig().invertOrbitY);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetCameraConfig().orbitSensitivity, 0.55f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetCameraConfig().panSensitivity, 0.0042f, 1e-4f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetCameraConfig().zoomSensitivity, 0.35f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetCameraConfig().fovDegrees, 60.0f, 1e-3f);
            TEST_EXPECT_TRUE(ctx, loadedPanel.IsOrthographic());
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetYaw(), 135.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetPitch(), 40.0f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetDistance(), 3.75f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetTarget().x, 0.12f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetTarget().y, 0.45f, 1e-3f);
            TEST_EXPECT_NEAR(ctx, loadedPanel.GetTarget().z, -0.22f, 1e-3f);

            // Cleanup
            std::filesystem::remove(tempSettings);
            if (createdImGui)
            {
                ImGui::DestroyContext();
            }
        }
    }
}
