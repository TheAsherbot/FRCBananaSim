/**
 * @file RobotSchemaTest.cpp
 * @brief Test suite verifying Schema v1 JSON deserialization, physical validation, and coordinate transformation.
 */

#include "RobotSchemaTest.h"
#include "TestFramework.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Schema/RobotManifest.h"
#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/MaterialLibrary.h"

#include <nlohmann/json.hpp>
#include <glm/gtc/quaternion.hpp>
#include <filesystem>
#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Robot::Schema;

    void RunRobotSchemaTest(TestContext& ctx)
    {
        std::cout << "[RUN ] RobotSchemaTest\n";
        constexpr double tolerance = 1e-5;

        // =============================================================
        // TS1: Full Reference2026 Ingestion & Verification
        // =============================================================
        {
            std::cout << "  -> TS1: Reference2026 Ingestion & Structural Verification\n";

            std::string manifestPath = "FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
            std::string sidecarPath = "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

            // Fallback for build directory execution
            if (!std::filesystem::exists(manifestPath))
            {
                manifestPath = "../FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
                sidecarPath = "../FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
                materialsPath = "../FRCBananaSim/Resources/Config/materials.json";
            }

            RobotSchemaLoader loader;
            bool loadSuccess = loader.LoadRobot(manifestPath, sidecarPath, materialsPath);

            TEST_EXPECT_TRUE(ctx, loadSuccess);
            TEST_EXPECT_FALSE(ctx, loader.HasErrors());

            if (loader.HasErrors())
            {
                for (const auto& err : loader.GetErrors())
                {
                    std::cerr << "     [Schema Error] " << err << "\n";
                }
            }

            const auto& manifest = loader.GetManifest();
            TEST_EXPECT_EQ(ctx, manifest.schemaVersion, "1.0.0");
            TEST_EXPECT_EQ(ctx, manifest.robotName, "FRC_Reference_Robot_2026");
            TEST_EXPECT_EQ(ctx, manifest.units, "SI");
            TEST_EXPECT_EQ(ctx, manifest.coordinateSystem, "NWU");
            TEST_EXPECT_NEAR(ctx, manifest.massSummary.totalMassKg, 54.20, tolerance);

            // 9 kinematic links
            TEST_EXPECT_EQ(ctx, manifest.links.size(), static_cast<size_t>(9));

            const LinkDefinition* root = manifest.GetRootLink();
            TEST_EXPECT_TRUE(ctx, root != nullptr);
            if (root)
            {
                TEST_EXPECT_EQ(ctx, root->name, "base_link");
                TEST_EXPECT_NEAR(ctx, root->massKg, 38.50, tolerance);
            }

            // Verify all 9 links have mass > 0 and positive-definite inertia tensors
            for (const auto& link : manifest.links)
            {
                TEST_EXPECT_TRUE(ctx, link.massKg > 0.0);
                TEST_EXPECT_TRUE(ctx, link.inertiaTensor.IsPositiveDefinite());
            }

            // 7 kinematic joints
            TEST_EXPECT_EQ(ctx, manifest.joints.size(), static_cast<size_t>(7));
            const JointDefinition* elevSlider = manifest.FindJoint("elevator_stage1_slider");
            TEST_EXPECT_TRUE(ctx, elevSlider != nullptr);
            if (elevSlider)
            {
                TEST_EXPECT_EQ(ctx, elevSlider->type, "prismatic");
                TEST_EXPECT_EQ(ctx, elevSlider->parentLink, "base_link");
                TEST_EXPECT_EQ(ctx, elevSlider->childLink, "elevator_stage_1");
                TEST_EXPECT_TRUE(ctx, elevSlider->limits.hasLimits);
                TEST_EXPECT_NEAR(ctx, elevSlider->limits.upper, 0.60, tolerance);
                TEST_EXPECT_NEAR(ctx, elevSlider->limits.springFrequencyHz, 40.0, tolerance);
            }

            // 2 contact zones
            TEST_EXPECT_EQ(ctx, manifest.contactZones.size(), static_cast<size_t>(2));
            const ContactZoneDefinition* intakeZone = manifest.FindContactZone("intake_capture_zone");
            TEST_EXPECT_TRUE(ctx, intakeZone != nullptr);
            if (intakeZone)
            {
                TEST_EXPECT_EQ(ctx, intakeZone->role, "intake");
                TEST_EXPECT_EQ(ctx, intakeZone->parentLink, "intake_coupler_link");
            }

            // Sidecar verifications
            const auto& sidecar = loader.GetSidecar();
            TEST_EXPECT_EQ(ctx, sidecar.schemaVersion, "1.0.0");
            TEST_EXPECT_NEAR(ctx, sidecar.electrical.openCircuitVoltageV, 12.60, tolerance);
            TEST_EXPECT_NEAR(ctx, sidecar.electrical.internalResistanceOhm, 0.023, tolerance);

            // Swerve drivetrain
            TEST_EXPECT_EQ(ctx, sidecar.swerve.chassisLink, "base_link");
            TEST_EXPECT_EQ(ctx, sidecar.swerve.driveMotorPreset, "KrakenX60");
            TEST_EXPECT_EQ(ctx, sidecar.swerve.steerMotorPreset, "KrakenX44");
            TEST_EXPECT_EQ(ctx, sidecar.swerve.modules.size(), static_cast<size_t>(4));

            // Mechanisms (elevator, arm, wrist, 4-bar, flywheel)
            TEST_EXPECT_EQ(ctx, sidecar.mechanisms.size(), static_cast<size_t>(5));
            const MechanismConfig* elevMech = sidecar.FindMechanism("elevator_drive");
            TEST_EXPECT_TRUE(ctx, elevMech != nullptr);
            if (elevMech)
            {
                TEST_EXPECT_EQ(ctx, elevMech->role, "elevator");
                TEST_EXPECT_NEAR(ctx, elevMech->gearRatio, 9.0, tolerance);
                TEST_EXPECT_NEAR(ctx, elevMech->drumRadiusM, 0.01905, tolerance);
                TEST_EXPECT_NEAR(ctx, elevMech->springAssist.forceN, 45.0, tolerance);
                TEST_EXPECT_EQ(ctx, elevMech->controllers.size(), static_cast<size_t>(2));
            }

            // Couplings (cascade, four_bar, unified power_coupling)
            TEST_EXPECT_EQ(ctx, sidecar.couplings.size(), static_cast<size_t>(3));
            const CouplingConfig* powerCoup = sidecar.FindCoupling("shoulder_to_wrist_coupling");
            TEST_EXPECT_TRUE(ctx, powerCoup != nullptr);
            if (powerCoup)
            {
                TEST_EXPECT_EQ(ctx, powerCoup->type, "power_coupling");
                TEST_EXPECT_EQ(ctx, powerCoup->powerCoupling.transmissionType, "chain");
                TEST_EXPECT_EQ(ctx, powerCoup->powerCoupling.masterJoint, "arm_shoulder_hinge");
                TEST_EXPECT_EQ(ctx, powerCoup->powerCoupling.slaveJoint, "arm_wrist_hinge");
                TEST_EXPECT_TRUE(ctx, powerCoup->powerCoupling.inverted);
                TEST_EXPECT_NEAR(ctx, powerCoup->powerCoupling.efficiency, 0.92, tolerance);
                TEST_EXPECT_NEAR(ctx, powerCoup->powerCoupling.backlashRad, 0.005, tolerance);
            }

            // Sensors (Pigeon2, CANcoder, limits, beam break, analog potentiometer, laser CAN)
            TEST_EXPECT_EQ(ctx, sidecar.sensors.size(), static_cast<size_t>(7));
            const SensorConfig* pot = sidecar.FindSensor("arm_joint_potentiometer");
            TEST_EXPECT_TRUE(ctx, pot != nullptr);
            if (pot)
            {
                TEST_EXPECT_EQ(ctx, pot->bus, "analog");
                TEST_EXPECT_EQ(ctx, pot->analogChannel, 0u);
                TEST_EXPECT_TRUE(ctx, pot->customNumericParams.count("minVolts") > 0);
                TEST_EXPECT_NEAR(ctx, pot->customNumericParams.at("minVolts"), 0.5, tolerance);
            }

            // Material Library & Contact Matrix Verification
            const auto& matLib = loader.GetMaterials();
            TEST_EXPECT_TRUE(ctx, matLib.GetMaterialCount() >= 8);
            TEST_EXPECT_TRUE(ctx, matLib.GetMaterialPairCount() >= 6);

            // Colson rubber on bilinear carpet calibrated pair
            MaterialPairProperties carpetContact = matLib.ResolveContactProperties("colson_performa", "bilinear_carpet");
            TEST_EXPECT_NEAR(ctx, carpetContact.staticFriction, 1.25, tolerance);
            TEST_EXPECT_NEAR(ctx, carpetContact.dynamicFriction, 1.10, tolerance);
            TEST_EXPECT_NEAR(ctx, carpetContact.restitution, 0.02, tolerance);
            TEST_EXPECT_NEAR(ctx, carpetContact.rollingResistance, 0.015, tolerance);

            // Commutative pair symmetry test: (B, A) == (A, B)
            MaterialPairProperties carpetContactReversed = matLib.ResolveContactProperties("bilinear_carpet", "colson_performa");
            TEST_EXPECT_NEAR(ctx, carpetContactReversed.staticFriction, 1.25, tolerance);
            TEST_EXPECT_NEAR(ctx, carpetContactReversed.dynamicFriction, 1.10, tolerance);

            // Fallback geometric mean mixing test
            MaterialPairProperties fallbackContact = matLib.ResolveContactProperties("aluminum_6061", "steel_4140");
            double expectedMu = std::sqrt(0.45 * 0.50);
            double expectedE = std::sqrt(0.05 * 0.10);
            TEST_EXPECT_NEAR(ctx, fallbackContact.staticFriction, expectedMu, tolerance);
            TEST_EXPECT_NEAR(ctx, fallbackContact.restitution, expectedE, tolerance);
        }

        // =============================================================
        // TS2: Coordinate Transformation & Matrix Invariant Accuracy
        // =============================================================
        {
            std::cout << "  -> TS2: Coordinate Transform Accuracy (NWU <-> Engine Frame)\n";

            // 1. Canonical Basis Vectors
            // Forward: NWU (+X) -> Engine (+Z)
            glm::dvec3 fwdNWU(1.0, 0.0, 0.0);
            glm::dvec3 fwdEng = RobotSchemaLoader::ConvertNWUVectorToEngine(fwdNWU);
            TEST_EXPECT_NEAR(ctx, fwdEng.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, fwdEng.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, fwdEng.z, 1.0, tolerance);

            // Left: NWU (+Y) -> Engine (+X) [Engine +X is Left in right-handed (+X Left, +Y Up, +Z Fwd)]
            glm::dvec3 leftNWU(0.0, 1.0, 0.0);
            glm::dvec3 leftEng = RobotSchemaLoader::ConvertNWUVectorToEngine(leftNWU);
            TEST_EXPECT_NEAR(ctx, leftEng.x, 1.0, tolerance);
            TEST_EXPECT_NEAR(ctx, leftEng.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, leftEng.z, 0.0, tolerance);

            // Up: NWU (+Z) -> Engine (+Y)
            glm::dvec3 upNWU(0.0, 0.0, 1.0);
            glm::dvec3 upEng = RobotSchemaLoader::ConvertNWUVectorToEngine(upNWU);
            TEST_EXPECT_NEAR(ctx, upEng.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, upEng.y, 1.0, tolerance);
            TEST_EXPECT_NEAR(ctx, upEng.z, 0.0, tolerance);

            // 2. Vector Round-Trip Identity: ||v - T^-1(T(v))|| < 1e-7
            glm::dvec3 testVec(3.4215, -1.8904, 0.7742);
            glm::dvec3 engVec = RobotSchemaLoader::ConvertNWUVectorToEngine(testVec);
            glm::dvec3 roundTripVec = RobotSchemaLoader::ConvertEngineVectorToNWU(engVec);
            double vecDiff = glm::length(testVec - roundTripVec);
            TEST_EXPECT_TRUE(ctx, vecDiff < 1e-7);

            // 3. Matrix Orthogonality & Inverse: M * M^T == I
            glm::dmat3 M = RobotSchemaLoader::GetNWUToEngineMatrix();
            glm::dmat3 Minv = RobotSchemaLoader::GetEngineToNWUMatrix();
            glm::dmat3 identity = M * Minv;

            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    double expected = (r == c) ? 1.0 : 0.0;
                    TEST_EXPECT_NEAR(ctx, identity[c][r], expected, 1e-9);
                }
            }

            // 4. Inertia Tensor Transform Invariance
            // Construct a physical positive-definite inertia matrix in NWU
            glm::dmat3 iNWU(
                3.50, 0.15, -0.08,
                0.15, 2.80,  0.12,
               -0.08, 0.12,  4.20
            );

            glm::dmat3 iEngine = RobotSchemaLoader::ConvertNWUInertiaToEngine(iNWU);

            // Check Sylvester's criterion on converted engine inertia tensor
            InertiaTensor engCheck;
            engCheck.ixx = iEngine[0][0];
            engCheck.iyy = iEngine[1][1];
            engCheck.izz = iEngine[2][2];
            engCheck.ixy = iEngine[1][0];
            engCheck.ixz = iEngine[2][0];
            engCheck.iyz = iEngine[2][1];
            TEST_EXPECT_TRUE(ctx, engCheck.IsPositiveDefinite());

            // Check round-trip on full 3x3 inertia matrix
            glm::dmat3 roundTripInertia = RobotSchemaLoader::ConvertEngineInertiaToNWU(iEngine);
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    TEST_EXPECT_NEAR(ctx, roundTripInertia[c][r], iNWU[c][r], 1e-7);
                }
            }

            // 5. Quaternion Rotation Preservation
            // 90 deg CCW Yaw in NWU rotates Forward (+X) into Left (+Y)
            double halfPi = 1.5707963267948966;
            glm::dquat qYawNWU = glm::angleAxis(halfPi, glm::dvec3(0.0, 0.0, 1.0));
            glm::dvec3 rotatedNWU = glm::mat3_cast(qYawNWU) * glm::dvec3(1.0, 0.0, 0.0);
            TEST_EXPECT_NEAR(ctx, rotatedNWU.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, rotatedNWU.y, 1.0, tolerance);
            TEST_EXPECT_NEAR(ctx, rotatedNWU.z, 0.0, tolerance);

            // Convert to Engine frame: should rotate Engine Forward (+Z) into Engine Left (+X)
            glm::dquat qYawEng = RobotSchemaLoader::ConvertNWUQuatToEngine(qYawNWU);
            glm::dvec3 rotatedEng = glm::mat3_cast(qYawEng) * glm::dvec3(0.0, 0.0, 1.0);
            TEST_EXPECT_NEAR(ctx, rotatedEng.x, 1.0, tolerance);
            TEST_EXPECT_NEAR(ctx, rotatedEng.y, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, rotatedEng.z, 0.0, tolerance);

            // Convert back to NWU
            glm::dquat qRoundTripNWU = RobotSchemaLoader::ConvertEngineQuatToNWU(qYawEng);
            glm::dvec3 rotatedBack = glm::mat3_cast(qRoundTripNWU) * glm::dvec3(1.0, 0.0, 0.0);
            TEST_EXPECT_NEAR(ctx, rotatedBack.x, 0.0, tolerance);
            TEST_EXPECT_NEAR(ctx, rotatedBack.y, 1.0, tolerance);
        }

        // =============================================================
        // TS3: Structured Diagnostic Catching on Malformed JSON
        // =============================================================
        {
            std::cout << "  -> TS3: Error Catching & Validation Diagnostics\n";

            // 1. Zero or Negative Link Mass
            {
                RobotSchemaLoader loader;
                nlohmann::json badJson = {
                    { "schemaVersion", "1.0.0" },
                    { "links", nlohmann::json::array({
                        {
                            { "name", "bad_link" },
                            { "mass_kg", 0.0 }, // Invalid zero mass!
                            { "inertiaTensor_kgm2", { {"ixx", 1.0}, {"iyy", 1.0}, {"izz", 1.0} } }
                        }
                    })}
                };

                loader.LoadManifestFromJson(badJson);
                bool valid = loader.Validate();
                TEST_EXPECT_FALSE(ctx, valid);
                TEST_EXPECT_TRUE(ctx, loader.HasErrors());
            }

            // 2. Non-Positive-Definite Inertia Tensor (Negative Diagonal)
            {
                RobotSchemaLoader loader;
                nlohmann::json badJson = {
                    { "schemaVersion", "1.0.0" },
                    { "links", nlohmann::json::array({
                        {
                            { "name", "unphysical_inertia_link" },
                            { "mass_kg", 10.0 },
                            { "inertiaTensor_kgm2", { {"ixx", -2.0}, {"iyy", 1.0}, {"izz", 1.0} } }
                        }
                    })}
                };

                loader.LoadManifestFromJson(badJson);
                bool valid = loader.Validate();
                TEST_EXPECT_FALSE(ctx, valid);
                TEST_EXPECT_TRUE(ctx, loader.HasErrors());
            }

            // 3. Unmatched Joint Child Link
            {
                RobotSchemaLoader loader;
                nlohmann::json manifestJson = {
                    { "schemaVersion", "1.0.0" },
                    { "links", nlohmann::json::array({
                        {
                            { "name", "base_link" },
                            { "isRoot", true },
                            { "mass_kg", 20.0 },
                            { "inertiaTensor_kgm2", { {"ixx", 1.0}, {"iyy", 1.0}, {"izz", 1.0} } }
                        }
                    })},
                    { "joints", nlohmann::json::array({
                        {
                            { "name", "broken_joint" },
                            { "parentLink", "base_link" },
                            { "childLink", "phantom_link" } // phantom_link does not exist!
                        }
                    })}
                };

                loader.LoadManifestFromJson(manifestJson);
                bool valid = loader.Validate();
                TEST_EXPECT_FALSE(ctx, valid);
                TEST_EXPECT_TRUE(ctx, loader.HasErrors());
            }

            // 4. Unmatched PowerCoupling Master Joint in Sidecar
            {
                RobotSchemaLoader loader;
                nlohmann::json manifestJson = {
                    { "schemaVersion", "1.0.0" },
                    { "links", nlohmann::json::array({
                        {
                            { "name", "base_link" },
                            { "isRoot", true },
                            { "mass_kg", 20.0 },
                            { "inertiaTensor_kgm2", { {"ixx", 1.0}, {"iyy", 1.0}, {"izz", 1.0} } }
                        }
                    })}
                };

                nlohmann::json sidecarJson = {
                    { "schemaVersion", "1.0.0" },
                    { "targetManifest", "manifest.json" },
                    { "couplings", nlohmann::json::array({
                        {
                            { "name", "broken_power_coupling" },
                            { "type", "power_coupling" },
                            { "powerCoupling", {
                                { "masterJoint", "missing_master" },
                                { "slaveJoint", "missing_slave" }
                            }}
                        }
                    })}
                };

                loader.LoadManifestFromJson(manifestJson);
                loader.LoadSidecarFromJson(sidecarJson);
                bool valid = loader.Validate();
                TEST_EXPECT_FALSE(ctx, valid);
                TEST_EXPECT_TRUE(ctx, loader.HasErrors());
            }

            // 5. Unsupported Schema Version
            {
                RobotSchemaLoader loader;
                nlohmann::json badVersionJson = {
                    { "schemaVersion", "9.9.9" },
                    { "links", nlohmann::json::array({
                        {
                            { "name", "base_link" },
                            { "mass_kg", 10.0 },
                            { "inertiaTensor_kgm2", { {"ixx", 1.0}, {"iyy", 1.0}, {"izz", 1.0} } }
                        }
                    })}
                };

                loader.LoadManifestFromJson(badVersionJson);
                bool valid = loader.Validate();
                TEST_EXPECT_FALSE(ctx, valid);
                TEST_EXPECT_TRUE(ctx, loader.HasErrors());
            }
        }

        std::cout << "[DONE] RobotSchemaTest\n";
    }
}