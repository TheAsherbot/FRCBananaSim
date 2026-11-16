/**
 * @file SchemaSerializationTest.cpp
 * @brief Unit tests verifying RobotSidecarSerializer round-trip fidelity, differentials, servos, and mass reconciliation.
 */

#include "Robot/SchemaSerializationTest.h"
#include "TestFramework.h"
#include "Robot/Schema/RobotSidecarSerializer.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/RobotManifest.h"

#include <nlohmann/json.hpp>
#include <filesystem>
#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Robot::Schema;

    void RunSchemaSerializationTest(TestContext& ctx)
    {
        try
        {
            std::cout << "[RUN ] SchemaSerializationTest" << std::endl;
        constexpr double tolerance = 1e-5;

        // =============================================================
        // SS1: Reference2026 Full Sidecar Round-Trip
        // =============================================================
        {
            std::cout << "  -> SS1: Reference2026 sidecar.json Round-Trip Fidelity\n";

            std::string sidecarPath = "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            if (!std::filesystem::exists(sidecarPath))
            {
                sidecarPath = "../FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            }

            RobotSchemaLoader loaderA;
            bool loadA = loaderA.LoadSidecarFromFile(sidecarPath);
            TEST_EXPECT_TRUE(ctx, loadA);
            TEST_EXPECT_FALSE(ctx, loaderA.HasErrors());

            const auto& sidecarA = loaderA.GetSidecar();

            // Serialize sidecarA to JSON string
            std::string serializedStr = RobotSidecarSerializer::SerializeToString(sidecarA, 2);
            TEST_EXPECT_FALSE(ctx, serializedStr.empty());

            // Parse back into loaderB
            RobotSchemaLoader loaderB;
            nlohmann::json parsedJson = nlohmann::json::parse(serializedStr);
            bool loadB = loaderB.LoadSidecarFromJson(parsedJson);
            TEST_EXPECT_TRUE(ctx, loadB);
            TEST_EXPECT_FALSE(ctx, loaderB.HasErrors());

            const auto& sidecarB = loaderB.GetSidecar();

            // Verify top-level metadata
            TEST_EXPECT_EQ(ctx, sidecarB.schemaVersion, sidecarA.schemaVersion);
            TEST_EXPECT_EQ(ctx, sidecarB.targetManifest, sidecarA.targetManifest);

            // Verify electrical
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.openCircuitVoltageV, sidecarA.electrical.openCircuitVoltageV, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.internalResistanceOhm, sidecarA.electrical.internalResistanceOhm, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.quiescentCurrentA, sidecarA.electrical.quiescentCurrentA, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.brownoutWarningV, sidecarA.electrical.brownoutWarningV, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.regulatedRailVoltageV, sidecarA.electrical.regulatedRailVoltageV, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.regulatedRailCurrentLimitA, sidecarA.electrical.regulatedRailCurrentLimitA, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.electrical.regulatedRailEfficiency, sidecarA.electrical.regulatedRailEfficiency, tolerance);

            // Verify swerve
            TEST_EXPECT_EQ(ctx, sidecarB.swerve.chassisLink, sidecarA.swerve.chassisLink);
            TEST_EXPECT_EQ(ctx, sidecarB.swerve.driveMotorPreset, sidecarA.swerve.driveMotorPreset);
            TEST_EXPECT_EQ(ctx, sidecarB.swerve.steerMotorPreset, sidecarA.swerve.steerMotorPreset);
            TEST_EXPECT_NEAR(ctx, sidecarB.swerve.driveGearRatio, sidecarA.swerve.driveGearRatio, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.swerve.steerGearRatio, sidecarA.swerve.steerGearRatio, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecarB.swerve.wheelRadiusM, sidecarA.swerve.wheelRadiusM, tolerance);
            TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules.size(), sidecarA.swerve.modules.size());

            for (size_t i = 0; i < sidecarA.swerve.modules.size() && i < sidecarB.swerve.modules.size(); ++i)
            {
                TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules[i].name, sidecarA.swerve.modules[i].name);
                TEST_EXPECT_NEAR(ctx, sidecarB.swerve.modules[i].posM.x, sidecarA.swerve.modules[i].posM.x, tolerance);
                TEST_EXPECT_NEAR(ctx, sidecarB.swerve.modules[i].posM.y, sidecarA.swerve.modules[i].posM.y, tolerance);
                TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules[i].driveCanId, sidecarA.swerve.modules[i].driveCanId);
                TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules[i].steerCanId, sidecarA.swerve.modules[i].steerCanId);
                TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules[i].cancoderId, sidecarA.swerve.modules[i].cancoderId);
                TEST_EXPECT_EQ(ctx, sidecarB.swerve.modules[i].canBus, sidecarA.swerve.modules[i].canBus);
            }

            // Verify mechanisms
            TEST_EXPECT_EQ(ctx, sidecarB.mechanisms.size(), sidecarA.mechanisms.size());
            for (size_t i = 0; i < sidecarA.mechanisms.size() && i < sidecarB.mechanisms.size(); ++i)
            {
                const auto& mA = sidecarA.mechanisms[i];
                const auto& mB = sidecarB.mechanisms[i];

                TEST_EXPECT_EQ(ctx, mB.name, mA.name);
                TEST_EXPECT_EQ(ctx, mB.role, mA.role);
                TEST_EXPECT_EQ(ctx, mB.targetJoint, mA.targetJoint);
                TEST_EXPECT_EQ(ctx, mB.motorPreset, mA.motorPreset);
                TEST_EXPECT_NEAR(ctx, mB.gearRatio, mA.gearRatio, tolerance);
                TEST_EXPECT_NEAR(ctx, mB.efficiency, mA.efficiency, tolerance);
                TEST_EXPECT_NEAR(ctx, mB.statorCurrentLimitA, mA.statorCurrentLimitA, tolerance);
                TEST_EXPECT_NEAR(ctx, mB.supplyCurrentLimitA, mA.supplyCurrentLimitA, tolerance);
                TEST_EXPECT_NEAR(ctx, mB.coulombFriction, mA.coulombFriction, tolerance);
                TEST_EXPECT_NEAR(ctx, mB.viscousDamping, mA.viscousDamping, tolerance);
                TEST_EXPECT_EQ(ctx, mB.controllers.size(), mA.controllers.size());

                for (size_t c = 0; c < mA.controllers.size() && c < mB.controllers.size(); ++c)
                {
                    TEST_EXPECT_EQ(ctx, mB.controllers[c].vendor, mA.controllers[c].vendor);
                    TEST_EXPECT_EQ(ctx, mB.controllers[c].canId, mA.controllers[c].canId);
                    TEST_EXPECT_EQ(ctx, mB.controllers[c].canBus, mA.controllers[c].canBus);
                    TEST_EXPECT_EQ(ctx, mB.controllers[c].inverted, mA.controllers[c].inverted);
                    TEST_EXPECT_EQ(ctx, mB.controllers[c].role, mA.controllers[c].role);
                }
            }

            // Verify couplings
            TEST_EXPECT_EQ(ctx, sidecarB.couplings.size(), sidecarA.couplings.size());
            for (size_t i = 0; i < sidecarA.couplings.size() && i < sidecarB.couplings.size(); ++i)
            {
                const auto& cA = sidecarA.couplings[i];
                const auto& cB = sidecarB.couplings[i];
                TEST_EXPECT_EQ(ctx, cB.name, cA.name);
                TEST_EXPECT_EQ(ctx, cB.type, cA.type);
                TEST_EXPECT_EQ(ctx, cB.solverMode, cA.solverMode);

                if (cA.type == "power_coupling")
                {
                    TEST_EXPECT_EQ(ctx, cB.powerCoupling.transmissionType, cA.powerCoupling.transmissionType);
                    TEST_EXPECT_EQ(ctx, cB.powerCoupling.masterJoint, cA.powerCoupling.masterJoint);
                    TEST_EXPECT_EQ(ctx, cB.powerCoupling.slaveJoint, cA.powerCoupling.slaveJoint);
                    TEST_EXPECT_NEAR(ctx, cB.powerCoupling.ratio, cA.powerCoupling.ratio, tolerance);
                    TEST_EXPECT_NEAR(ctx, cB.powerCoupling.backlashRad, cA.powerCoupling.backlashRad, tolerance);
                }
                else if (cA.type == "elevator_cascade")
                {
                    TEST_EXPECT_EQ(ctx, cB.cascadeStages.size(), cA.cascadeStages.size());
                }
                else if (cA.type == "four_bar_loop")
                {
                    TEST_EXPECT_EQ(ctx, cB.fourBar.crankJoint, cA.fourBar.crankJoint);
                    TEST_EXPECT_EQ(ctx, cB.fourBar.couplerJoint, cA.fourBar.couplerJoint);
                    TEST_EXPECT_EQ(ctx, cB.fourBar.followerJoint, cA.fourBar.followerJoint);
                }
            }

            // Verify sensors
            TEST_EXPECT_EQ(ctx, sidecarB.sensors.size(), sidecarA.sensors.size());
            for (size_t i = 0; i < sidecarA.sensors.size() && i < sidecarB.sensors.size(); ++i)
            {
                const auto& sA = sidecarA.sensors[i];
                const auto& sB = sidecarB.sensors[i];
                TEST_EXPECT_EQ(ctx, sB.name, sA.name);
                TEST_EXPECT_EQ(ctx, sB.type, sA.type);
                TEST_EXPECT_EQ(ctx, sB.bus, sA.bus);
                TEST_EXPECT_EQ(ctx, sB.canId, sA.canId);
                TEST_EXPECT_EQ(ctx, sB.dioChannel, sA.dioChannel);
                TEST_EXPECT_EQ(ctx, sB.analogChannel, sA.analogChannel);
                TEST_EXPECT_EQ(ctx, sB.parentLink, sA.parentLink);
                TEST_EXPECT_EQ(ctx, sB.targetJoint, sA.targetJoint);
            }
        }

        // =============================================================
        // SS2: Mass Reconciliation Calibration Round-Trip
        // =============================================================
        {
            std::cout << "  -> SS2: Mass Reconciliation Calibration Serialization\n";

            RobotSidecar sidecar;
            sidecar.schemaVersion = "1.0.0";
            sidecar.targetManifest = "manifest.json";
            sidecar.massReconciliation.enabled = true;
            sidecar.massReconciliation.linkMassMultiplier = 1.042;
            sidecar.massReconciliation.unmodeledBallast.targetLink = "base_link";
            sidecar.massReconciliation.unmodeledBallast.massKg = 4.25;
            sidecar.massReconciliation.unmodeledBallast.offsetM = { 0.02, 0.0, -0.05 };

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            TEST_EXPECT_TRUE(ctx, j.contains("massReconciliation"));

            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto& loaded = loader.GetSidecar();
            TEST_EXPECT_TRUE(ctx, loaded.massReconciliation.enabled);
            TEST_EXPECT_NEAR(ctx, loaded.massReconciliation.linkMassMultiplier, 1.042, tolerance);
            TEST_EXPECT_EQ(ctx, loaded.massReconciliation.unmodeledBallast.targetLink, "base_link");
            TEST_EXPECT_NEAR(ctx, loaded.massReconciliation.unmodeledBallast.massKg, 4.25, tolerance);
            TEST_EXPECT_NEAR(ctx, loaded.massReconciliation.unmodeledBallast.offsetM.x, 0.02, tolerance);
            TEST_EXPECT_NEAR(ctx, loaded.massReconciliation.unmodeledBallast.offsetM.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, loaded.massReconciliation.unmodeledBallast.offsetM.z, -0.05, tolerance);
        }

        // =============================================================
        // SS3: Differential Wrist Mechanism Serialization
        // =============================================================
        {
            std::cout << "  -> SS3: 2-DOF Differential Wrist Mechanism Serialization\n";

            RobotSidecar sidecar;
            MechanismConfig diffMech;
            diffMech.name = "diff_wrist";
            diffMech.role = "differential";
            diffMech.differentialType = "symmetric_bevel";
            diffMech.targetJoints = { "wrist_pitch_hinge", "wrist_roll_hinge" };
            diffMech.gearRatios = { 15.0, 15.0 };
            diffMech.coulombFrictions = { 0.25, 0.25 };
            diffMech.viscousDampings = { 0.05, 0.05 };

            ControllerConfig ctrlA;
            ctrlA.vendor = "CTRE_TalonFX";
            ctrlA.canId = 18;
            ctrlA.canBus = "canivore";
            ctrlA.role = "leader";

            ControllerConfig ctrlB;
            ctrlB.vendor = "CTRE_TalonFX";
            ctrlB.canId = 19;
            ctrlB.canBus = "canivore";
            ctrlB.role = "leader";

            diffMech.controllers.push_back(ctrlA);
            diffMech.controllers.push_back(ctrlB);
            sidecar.mechanisms.push_back(diffMech);

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto* loadedMech = loader.GetSidecar().FindMechanism("diff_wrist");
            TEST_EXPECT_TRUE(ctx, loadedMech != nullptr);
            if (loadedMech)
            {
                TEST_EXPECT_EQ(ctx, loadedMech->role, "differential");
                TEST_EXPECT_EQ(ctx, loadedMech->differentialType, "symmetric_bevel");
                TEST_EXPECT_EQ(ctx, loadedMech->targetJoints.size(), 2);
                if (loadedMech->targetJoints.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedMech->targetJoints[0], "wrist_pitch_hinge");
                    TEST_EXPECT_EQ(ctx, loadedMech->targetJoints[1], "wrist_roll_hinge");
                }
                TEST_EXPECT_EQ(ctx, loadedMech->gearRatios.size(), 2);
                TEST_EXPECT_EQ(ctx, loadedMech->controllers.size(), 2);
                if (loadedMech->controllers.size() == 2)
                {
                    TEST_EXPECT_EQ(ctx, loadedMech->controllers[0].canId, 18);
                    TEST_EXPECT_EQ(ctx, loadedMech->controllers[1].canId, 19);
                }
            }
        }

        // =============================================================
        // SS4: Servo Mechanism & Mount Offset (Q4) Round-Trip
        // =============================================================
        {
            std::cout << "  -> SS4: Servo Actuator & Dynamic Flywheel Mount Offset (Q4) Serialization\n";

            RobotSidecar sidecar;

            // Servo Arm
            MechanismConfig servoArm;
            servoArm.name = "servo_indexer";
            servoArm.role = "arm";
            servoArm.targetJoint = "indexer_pivot";
            servoArm.actuatorType = "servo";
            servoArm.servo.preset = "GoBildaTorque";
            servoArm.servo.mode = "positional";
            servoArm.servo.pwmChannel = 5;
            servoArm.servo.supplyVoltageV = 6.0;
            servoArm.servo.travelRangeDeg = 300.0;
            sidecar.mechanisms.push_back(servoArm);

            // Flywheel with mountOffsetM (Q4 fix)
            MechanismConfig flywheel;
            flywheel.name = "top_roller";
            flywheel.role = "flywheel";
            flywheel.parentLink = "shooter_hood_link";
            flywheel.visualLink = "top_roller_visual";
            flywheel.flywheelInertiaKgm2 = 0.0025;
            flywheel.spinAxis = { 1.0, 0.0, 0.0 };
            flywheel.mountOffsetM = { 0.05, 0.02, -0.01 };
            sidecar.mechanisms.push_back(flywheel);

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto* loadedServo = loader.GetSidecar().FindMechanism("servo_indexer");
            TEST_EXPECT_TRUE(ctx, loadedServo != nullptr);
            if (loadedServo)
            {
                TEST_EXPECT_EQ(ctx, loadedServo->actuatorType, "servo");
                TEST_EXPECT_EQ(ctx, loadedServo->servo.preset, "GoBildaTorque");
                TEST_EXPECT_EQ(ctx, loadedServo->servo.pwmChannel, 5);
                TEST_EXPECT_NEAR(ctx, loadedServo->servo.travelRangeDeg, 300.0, tolerance);
            }

            const auto* loadedFlywheel = loader.GetSidecar().FindMechanism("top_roller");
            TEST_EXPECT_TRUE(ctx, loadedFlywheel != nullptr);
            if (loadedFlywheel)
            {
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.x, 0.05, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.y, 0.02, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedFlywheel->mountOffsetM.z, -0.01, tolerance);
            }
        }

        // =============================================================
        // SS5: Custom Motor & Servo Parametric Specs Round-Trip
        // =============================================================
        {
            std::cout << "  -> SS5: Custom Motor & Servo Parametric Specs Round-Trip\n";

            RobotSidecar sidecar;
            sidecar.swerve.driveActuatorType = "motor";
            sidecar.swerve.driveMotorPreset = "Custom";
            sidecar.swerve.customDriveMotor.nominalVoltageV = 14.8;
            sidecar.swerve.customDriveMotor.stallTorqueNm = 4.25;
            sidecar.swerve.customDriveMotor.freeSpeedRpm = 7200.0;
            sidecar.swerve.customDriveMotor.stallCurrentA = 310.0;
            sidecar.swerve.customDriveMotor.freeCurrentA = 1.85;

            sidecar.swerve.steerActuatorType = "servo";
            sidecar.swerve.steerServoPreset = "Custom";
            sidecar.swerve.customSteerServo.nominalVoltageV = 7.4;
            sidecar.swerve.customSteerServo.stallTorqueKgCm = 35.5;
            sidecar.swerve.customSteerServo.speedSecPer60Deg = 0.075;
            sidecar.swerve.customSteerServo.stallCurrentA = 3.8;
            sidecar.swerve.customSteerServo.freeCurrentA = 0.22;
            sidecar.swerve.customSteerServo.travelRangeDeg = 270.0;

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto& loadedSwerve = loader.GetSidecar().swerve;
            TEST_EXPECT_EQ(ctx, loadedSwerve.driveMotorPreset, "Custom");
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customDriveMotor.nominalVoltageV, 14.8, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customDriveMotor.stallTorqueNm, 4.25, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customDriveMotor.freeSpeedRpm, 7200.0, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customDriveMotor.stallCurrentA, 310.0, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customDriveMotor.freeCurrentA, 1.85, tolerance);

            TEST_EXPECT_EQ(ctx, loadedSwerve.steerServoPreset, "Custom");
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.nominalVoltageV, 7.4, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.stallTorqueKgCm, 35.5, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.speedSecPer60Deg, 0.075, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.stallCurrentA, 3.8, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.freeCurrentA, 0.22, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedSwerve.customSteerServo.travelRangeDeg, 270.0, tolerance);
        }

        // =============================================================
        // SS6: Joint Limit Overrides Round-Trip & Effective Limits
        // =============================================================
        {
            std::cout << "  -> SS6: Joint Limit Overrides Round-Trip & Effective Limits\n";

            RobotSidecar sidecar;

            JointLimitOverride armOv;
            armOv.hasLimits = true;
            armOv.lower = -1.570796;
            armOv.upper = 2.094395;
            armOv.unit = "rad";
            armOv.springFrequencyHz = 25.0;
            armOv.springDampingRatio = 0.85;
            sidecar.jointOverrides["arm_shoulder_hinge"] = armOv;

            JointLimitOverride elevOv;
            elevOv.hasLimits = true;
            elevOv.lower = 0.05;
            elevOv.upper = 1.25;
            elevOv.unit = "m";
            elevOv.springFrequencyHz = 35.0;
            elevOv.springDampingRatio = 0.95;
            sidecar.jointOverrides["elevator_carriage_slider"] = elevOv;

            JointLimitOverride contOv;
            contOv.hasLimits = false;
            contOv.lower = 0.0;
            contOv.upper = 0.0;
            contOv.unit = "rad";
            contOv.springFrequencyHz = 40.0;
            contOv.springDampingRatio = 1.0;
            sidecar.jointOverrides["intake_roller_joint"] = contOv;

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto& loadedSidecar = loader.GetSidecar();
            TEST_EXPECT_EQ(ctx, loadedSidecar.jointOverrides.size(), static_cast<size_t>(3));

            const auto* loadedArm = loadedSidecar.FindJointOverride("arm_shoulder_hinge");
            TEST_EXPECT_TRUE(ctx, loadedArm != nullptr);
            if (loadedArm)
            {
                TEST_EXPECT_TRUE(ctx, loadedArm->hasLimits);
                TEST_EXPECT_NEAR(ctx, loadedArm->lower, -1.570796, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedArm->upper, 2.094395, tolerance);
                TEST_EXPECT_EQ(ctx, loadedArm->unit, "rad");
                TEST_EXPECT_NEAR(ctx, loadedArm->springFrequencyHz, 25.0, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedArm->springDampingRatio, 0.85, tolerance);
            }

            const auto* loadedElev = loadedSidecar.FindJointOverride("elevator_carriage_slider");
            TEST_EXPECT_TRUE(ctx, loadedElev != nullptr);
            if (loadedElev)
            {
                TEST_EXPECT_TRUE(ctx, loadedElev->hasLimits);
                TEST_EXPECT_NEAR(ctx, loadedElev->lower, 0.05, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedElev->upper, 1.25, tolerance);
                TEST_EXPECT_EQ(ctx, loadedElev->unit, "m");
                TEST_EXPECT_NEAR(ctx, loadedElev->springFrequencyHz, 35.0, tolerance);
                TEST_EXPECT_NEAR(ctx, loadedElev->springDampingRatio, 0.95, tolerance);
            }

            const auto* loadedCont = loadedSidecar.FindJointOverride("intake_roller_joint");
            TEST_EXPECT_TRUE(ctx, loadedCont != nullptr);
            if (loadedCont)
            {
                TEST_EXPECT_FALSE(ctx, loadedCont->hasLimits);
                TEST_EXPECT_EQ(ctx, loadedCont->unit, "rad");
            }

            // Test GetEffectiveLimits fallback to manifest when joint is not overridden
            RobotManifest manifest;
            JointDefinition cadJoint;
            cadJoint.name = "default_cad_joint";
            cadJoint.limits.hasLimits = true;
            cadJoint.limits.lower = -0.5;
            cadJoint.limits.upper = 1.5;
            cadJoint.limits.unit = "rad";
            manifest.joints.push_back(cadJoint);

            // Override takes precedence
            auto effArm = loadedSidecar.GetEffectiveLimits("arm_shoulder_hinge", manifest);
            TEST_EXPECT_TRUE(ctx, effArm.hasLimits);
            TEST_EXPECT_NEAR(ctx, effArm.lower, -1.570796, tolerance);

            // Manifest CAD fallback works when no override
            auto effCad = loadedSidecar.GetEffectiveLimits("default_cad_joint", manifest);
            TEST_EXPECT_TRUE(ctx, effCad.hasLimits);
            TEST_EXPECT_NEAR(ctx, effCad.lower, -0.5, tolerance);
            TEST_EXPECT_NEAR(ctx, effCad.upper, 1.5, tolerance);

            // Unknown joint returns hasLimits = false
            auto effUnknown = loadedSidecar.GetEffectiveLimits("unknown_joint", manifest);
            TEST_EXPECT_FALSE(ctx, effUnknown.hasLimits);
        }

        // =============================================================
        // SS7: Bumper Configuration Serialization Round-Trip
        // =============================================================
        {
            std::cout << "  -> SS7: Bumper Configuration Serialization Round-Trip\n";

            RobotSidecar sidecar;
            sidecar.bumpers.enabled = true;
            sidecar.bumpers.targetLink = "chassis_base";
            sidecar.bumpers.shape = "u_shape";
            sidecar.bumpers.lengthM = 0.88;
            sidecar.bumpers.widthM = 0.86;
            sidecar.bumpers.heightM = 0.13;
            sidecar.bumpers.thicknessM = 0.0825;
            sidecar.bumpers.offsetM = { 0.02, -0.01, 0.095 };
            sidecar.bumpers.massKg = 6.25;
            sidecar.bumpers.material = "bumper_fabric";
            sidecar.bumpers.color = "blue";
            sidecar.bumpers.cutoutWidthM = 0.42;

            nlohmann::json j = RobotSidecarSerializer::SerializeToJson(sidecar);
            RobotSchemaLoader loader;
            bool success = loader.LoadSidecarFromJson(j);
            TEST_EXPECT_TRUE(ctx, success);

            const auto& loadedBumpers = loader.GetSidecar().bumpers;
            TEST_EXPECT_TRUE(ctx, loadedBumpers.enabled);
            TEST_EXPECT_EQ(ctx, loadedBumpers.targetLink, "chassis_base");
            TEST_EXPECT_EQ(ctx, loadedBumpers.shape, "u_shape");
            TEST_EXPECT_NEAR(ctx, loadedBumpers.lengthM, 0.88, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.widthM, 0.86, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.heightM, 0.13, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.thicknessM, 0.0825, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.offsetM.x, 0.02, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.offsetM.y, -0.01, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.offsetM.z, 0.095, tolerance);
            TEST_EXPECT_NEAR(ctx, loadedBumpers.massKg, 6.25, tolerance);
            TEST_EXPECT_EQ(ctx, loadedBumpers.material, "bumper_fabric");
            TEST_EXPECT_EQ(ctx, loadedBumpers.color, "blue");
            TEST_EXPECT_NEAR(ctx, loadedBumpers.cutoutWidthM, 0.42, tolerance);
        }
        std::cout << "[DONE] SchemaSerializationTest" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "EXCEPTION in SchemaSerializationTest: " << e.what() << std::endl;
            ++ctx.failed;
        }
        catch (...)
        {
            std::cerr << "UNKNOWN EXCEPTION in SchemaSerializationTest" << std::endl;
            ++ctx.failed;
        }
    }
}
