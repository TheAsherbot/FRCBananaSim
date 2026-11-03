/**
 * @file RobotSchemaLoader.cpp
 * @brief Implementation of RobotSchemaLoader JSON deserialization, validation, and coordinate math.
 */

#include "Robot/Schema/RobotSchemaLoader.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cmath>

namespace FRCBananaSim::Robot::Schema
{
    static glm::dvec3 ParseOptionalVec3(const nlohmann::json& parent, const std::string& key,
        const glm::dvec3& defaultValue = { 0.0, 0.0, 0.0 })
    {
        if (parent.is_object() && parent.contains(key))
        {
            const auto& node = parent[key];
            if (node.is_array() && node.size() >= 3)
            {
                return glm::dvec3(
                    node[0].get<double>(),
                    node[1].get<double>(),
                    node[2].get<double>()
                );
            }
        }
        return defaultValue;
    }

    static glm::dquat ParseOptionalQuat(const nlohmann::json& parent, const std::string& key,
        const glm::dquat& defaultValue = { 1.0, 0.0, 0.0, 0.0 })
    {
        if (parent.is_object() && parent.contains(key))
        {
            const auto& node = parent[key];
            if (node.is_array() && node.size() >= 4)
            {
                return glm::dquat(
                    node[0].get<double>(),
                    node[1].get<double>(),
                    node[2].get<double>(),
                    node[3].get<double>()
                );
            }
        }
        return defaultValue;
    }

    static glm::dvec3 ParseVec3(const nlohmann::json& node, const glm::dvec3& defaultValue = { 0.0, 0.0, 0.0 })
    {
        if (node.is_array() && node.size() >= 3)
        {
            return glm::dvec3(
                node[0].get<double>(),
                node[1].get<double>(),
                node[2].get<double>()
            );
        }
        return defaultValue;
    }

    static glm::dquat ParseQuat(const nlohmann::json& node, const glm::dquat& defaultValue = { 1.0, 0.0, 0.0, 0.0 })
    {
        if (node.is_array() && node.size() >= 4)
        {
            return glm::dquat(
                node[0].get<double>(),
                node[1].get<double>(),
                node[2].get<double>(),
                node[3].get<double>()
            );
        }
        return defaultValue;
    }

    RobotSchemaLoader::RobotSchemaLoader()
    {}

    bool RobotSchemaLoader::LoadRobot(const std::string& manifestPath, const std::string& sidecarPath, const std::string& materialsPath)
    {
        ClearDiagnostics();

        if (!LoadManifestFromFile(manifestPath))
        {
            return false;
        }

        std::filesystem::path mPath(manifestPath);
        std::filesystem::path dir = mPath.has_parent_path() ? mPath.parent_path() : std::filesystem::current_path();

        // Infer sidecar path if omitted
        std::string resolvedSidecar = sidecarPath;
        if (resolvedSidecar.empty())
        {
            std::filesystem::path autoSidecar = dir / "sidecar.json";
            if (std::filesystem::exists(autoSidecar))
            {
                resolvedSidecar = autoSidecar.string();
            }
        }

        if (!resolvedSidecar.empty())
        {
            if (!LoadSidecarFromFile(resolvedSidecar))
            {
                return false;
            }
        }
        else
        {
            warnings.push_back("No sidecar.json specified or found in manifest directory: " + dir.string());
        }

        // Infer materials path if omitted
        std::string resolvedMaterials = materialsPath;
        if (resolvedMaterials.empty())
        {
            std::filesystem::path localMat = dir / "materials.json";
            std::filesystem::path configMat = "FRCBananaSim/Resources/Config/materials.json";

            if (std::filesystem::exists(localMat))
            {
                resolvedMaterials = localMat.string();
            }
            else if (std::filesystem::exists(configMat))
            {
                resolvedMaterials = configMat.string();
            }
        }

        if (!resolvedMaterials.empty())
        {
            LoadMaterialsFromFile(resolvedMaterials);
        }

        return Validate();
    }

    bool RobotSchemaLoader::LoadManifestFromFile(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            errors.push_back("Failed to open manifest file: " + filepath);
            return false;
        }

        try
        {
            nlohmann::json json;
            file >> json;
            return LoadManifestFromJson(json);
        }
        catch (const std::exception& e)
        {
            errors.push_back("Exception parsing manifest JSON (" + filepath + "): " + e.what());
            return false;
        }
    }

    bool RobotSchemaLoader::LoadManifestFromJson(const nlohmann::json& json)
    {
        manifest = RobotManifest{};

        manifest.schemaVersion = json.value("schemaVersion", "1.0.0");
        manifest.robotName = json.value("robotName", "");
        manifest.units = json.value("units", "SI");
        manifest.coordinateSystem = json.value("coordinateSystem", "NWU");

        if (json.contains("massSummary"))
        {
            const auto& ms = json["massSummary"];
            manifest.massSummary.totalMassKg = ms.value("totalMass_kg", 0.0);
            manifest.massSummary.centerOfMassM = ParseOptionalVec3(ms, "centerOfMass_m");
        }

        // Links
        if (json.contains("links") && json["links"].is_array())
        {
            for (const auto& lNode : json["links"])
            {
                LinkDefinition link;
                link.name = lNode.value("name", "");
                link.isRoot = lNode.value("isRoot", false);
                link.massKg = lNode.value("mass_kg", 0.0);
                link.centerOfMassM = ParseOptionalVec3(lNode, "centerOfMass_m");
                link.material = lNode.value("material", "");
                link.visualMesh = lNode.value("visualMesh", "");

                if (lNode.contains("inertiaTensor_kgm2"))
                {
                    const auto& it = lNode["inertiaTensor_kgm2"];
                    link.inertiaTensor.ixx = it.value("ixx", 0.0);
                    link.inertiaTensor.iyy = it.value("iyy", 0.0);
                    link.inertiaTensor.izz = it.value("izz", 0.0);
                    link.inertiaTensor.ixy = it.value("ixy", 0.0);
                    link.inertiaTensor.ixz = it.value("ixz", 0.0);
                    link.inertiaTensor.iyz = it.value("iyz", 0.0);
                }

                if (lNode.contains("colliders") && lNode["colliders"].is_array())
                {
                    for (const auto& cNode : lNode["colliders"])
                    {
                        ColliderDefinition col;
                        col.name = cNode.value("name", "");
                        col.type = cNode.value("type", "box");
                        col.originM = ParseOptionalVec3(cNode, "origin_m");
                        col.rotationQuat = ParseOptionalQuat(cNode, "rotation_quat");
                        col.dimensionsM = ParseOptionalVec3(cNode, "dimensions_m");
                        col.material = cNode.value("material", link.material);
                        link.colliders.push_back(col);
                    }
                }

                manifest.links.push_back(link);
            }
        }

        // Joints
        if (json.contains("joints") && json["joints"].is_array())
        {
            for (const auto& jNode : json["joints"])
            {
                JointDefinition joint;
                joint.name = jNode.value("name", "");
                joint.type = jNode.value("type", "revolute");
                joint.parentLink = jNode.value("parentLink", "");
                joint.childLink = jNode.value("childLink", "");
                joint.originM = ParseOptionalVec3(jNode, "origin_m");
                joint.axis = ParseOptionalVec3(jNode, "axis", glm::dvec3(0.0, 0.0, 1.0));

                if (jNode.contains("limits"))
                {
                    const auto& lim = jNode["limits"];
                    joint.limits.hasLimits = lim.value("hasLimits", false);
                    joint.limits.lower = lim.value("lower", 0.0);
                    joint.limits.upper = lim.value("upper", 0.0);
                    joint.limits.unit = lim.value("unit", "rad");
                    joint.limits.springFrequencyHz = lim.value("springFrequency_Hz", 40.0);
                    joint.limits.springDampingRatio = lim.value("springDampingRatio", 1.0);
                }

                manifest.joints.push_back(joint);
            }
        }

        // Contact Zones
        if (json.contains("contactZones") && json["contactZones"].is_array())
        {
            for (const auto& zNode : json["contactZones"])
            {
                ContactZoneDefinition zone;
                zone.name = zNode.value("name", "");
                zone.role = zNode.value("role", "intake");
                zone.parentLink = zNode.value("parentLink", "");
                zone.shape = zNode.value("shape", "box");
                zone.originM = ParseOptionalVec3(zNode, "origin_m");
                zone.rotationQuat = ParseOptionalQuat(zNode, "rotation_quat");
                zone.dimensionsM = ParseOptionalVec3(zNode, "dimensions_m");
                zone.tangentAxis = ParseOptionalVec3(zNode, "tangentAxis", glm::dvec3(1.0, 0.0, 0.0));
                zone.compressionAxis = ParseOptionalVec3(zNode, "compressionAxis", glm::dvec3(0.0, 0.0, 1.0));
                zone.nominalGapM = zNode.value("nominalGap_m", 0.0);
                manifest.contactZones.push_back(zone);
            }
        }

        return true;
    }

    bool RobotSchemaLoader::LoadSidecarFromFile(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            errors.push_back("Failed to open sidecar file: " + filepath);
            return false;
        }

        try
        {
            nlohmann::json json;
            file >> json;
            return LoadSidecarFromJson(json);
        }
        catch (const std::exception& e)
        {
            errors.push_back("Exception parsing sidecar JSON (" + filepath + "): " + e.what());
            return false;
        }
    }

    bool RobotSchemaLoader::LoadSidecarFromJson(const nlohmann::json& json)
    {
        sidecar = RobotSidecar{};

        sidecar.schemaVersion = json.value("schemaVersion", "1.0.0");
        sidecar.targetManifest = json.value("targetManifest", "manifest.json");

        // Electrical
        if (json.contains("electrical"))
        {
            const auto& el = json["electrical"];
            sidecar.electrical.openCircuitVoltageV = el.value("openCircuitVoltage_V", 12.6);
            sidecar.electrical.internalResistanceOhm = el.value("internalResistance_Ohm", 0.023);
            sidecar.electrical.quiescentCurrentA = el.value("quiescentCurrent_A", 1.5);
            sidecar.electrical.brownoutWarningV = el.value("brownoutWarning_V", 6.8);
            sidecar.electrical.brownoutShutoffV = el.value("brownoutShutoff_V", 6.3);
            sidecar.electrical.brownoutRecoveryV = el.value("brownoutRecovery_V", 7.5);
        }

        // Swerve
        if (json.contains("swerve"))
        {
            const auto& sw = json["swerve"];
            sidecar.swerve.chassisLink = sw.value("chassisLink", "base_link");
            sidecar.swerve.driveMotorPreset = sw.value("driveMotorPreset", "KrakenX60");
            sidecar.swerve.steerMotorPreset = sw.value("steerMotorPreset", "KrakenX44");
            sidecar.swerve.driveGearRatio = sw.value("driveGearRatio", 5.27);
            sidecar.swerve.steerGearRatio = sw.value("steerGearRatio", 26.0909);
            sidecar.swerve.driveEfficiency = sw.value("driveEfficiency", 0.96);
            sidecar.swerve.steerEfficiency = sw.value("steerEfficiency", 0.92);
            sidecar.swerve.wheelRadiusM = sw.value("wheelRadius_m", 0.0508);
            sidecar.swerve.driveStatorLimitA = sw.value("driveStatorLimit_A", 60.0);
            sidecar.swerve.driveSupplyLimitA = sw.value("driveSupplyLimit_A", 40.0);
            sidecar.swerve.steerStatorLimitA = sw.value("steerStatorLimit_A", 40.0);
            sidecar.swerve.steerSupplyLimitA = sw.value("steerSupplyLimit_A", 30.0);

            if (sw.contains("modules") && sw["modules"].is_array())
            {
                for (const auto& mNode : sw["modules"])
                {
                    SwerveModuleConfig mod;
                    mod.name = mNode.value("name", "");
                    mod.posM = ParseOptionalVec3(mNode, "pos_m");
                    mod.driveCanId = mNode.value("driveCanId", 0);
                    mod.steerCanId = mNode.value("steerCanId", 0);
                    mod.cancoderId = mNode.value("cancoderId", 0);
                    mod.canBus = mNode.value("canBus", "rio");
                    sidecar.swerve.modules.push_back(mod);
                }
            }
        }

        // Mechanisms
                // -------------------------------------------------------------
        // In FRCBananaSim/Source/Robot/Schema/RobotSchemaLoader.cpp (around line 355):
        // -------------------------------------------------------------
        if (json.contains("mechanisms") && json["mechanisms"].is_array())
        {
            for (const auto& mNode : json["mechanisms"])
            {
                MechanismConfig mech;
                mech.name = mNode.value("name", "");
                mech.role = mNode.value("role", "elevator");
                mech.targetJoint = mNode.value("targetJoint", "");
                mech.parentLink = mNode.value("parentLink", "");
                mech.motorPreset = mNode.value("motorPreset", "KrakenX60");
                mech.gearRatio = mNode.value("gearRatio", 1.0);
                mech.efficiency = mNode.value("efficiency", 1.0);
                mech.drumRadiusM = mNode.value("drumRadius_m", 0.0);
                mech.flywheelInertiaKgm2 = mNode.value("flywheelInertia_kgm2", 0.0);
                mech.spinAxis = ParseOptionalVec3(mNode, "spinAxis", glm::dvec3(0.0, 1.0, 0.0));
                mech.visualLink = mNode.value("visualLink", "");
                mech.statorCurrentLimitA = mNode.value("statorCurrentLimit_A", 60.0);
                mech.supplyCurrentLimitA = mNode.value("supplyCurrentLimit_A", 40.0);
                mech.coulombFriction = mNode.value("coulombFriction", 0.0);
                mech.viscousDamping = mNode.value("viscousDamping", 0.0);

                // ==================== [START NEW DIFFERENTIAL PARSING] ====================
                if (mNode.contains("targetJoints") && mNode["targetJoints"].is_array())
                {
                    for (const auto& tj : mNode["targetJoints"])
                    {
                        mech.targetJoints.push_back(tj.get<std::string>());
                    }
                }
                if (mNode.contains("differentialType") && mNode["differentialType"].is_string())
                {
                    mech.differentialType = mNode["differentialType"].get<std::string>();
                }
                if (mNode.contains("gearRatios") && mNode["gearRatios"].is_array())
                {
                    for (const auto& gr : mNode["gearRatios"])
                    {
                        mech.gearRatios.push_back(gr.get<double>());
                    }
                }
                if (mNode.contains("couplingMatrix") && mNode["couplingMatrix"].is_array())
                {
                    for (const auto& row : mNode["couplingMatrix"])
                    {
                        std::vector<double> r;
                        if (row.is_array())
                        {
                            for (const auto& val : row)
                            {
                                r.push_back(val.get<double>());
                            }
                        }
                        mech.couplingMatrix.push_back(r);
                    }
                }
                if (mNode.contains("coulombFriction") && mNode["coulombFriction"].is_array())
                {
                    for (const auto& cf : mNode["coulombFriction"])
                    {
                        mech.coulombFrictions.push_back(cf.get<double>());
                    }
                }
                if (mNode.contains("viscousDamping") && mNode["viscousDamping"].is_array())
                {
                    for (const auto& vd : mNode["viscousDamping"])
                    {
                        mech.viscousDampings.push_back(vd.get<double>());
                    }
                }
                // ===================== [END NEW DIFFERENTIAL PARSING] =====================

                if (mNode.contains("springAssist"))
                {
                    const auto& sa = mNode["springAssist"];
                    mech.springAssist.type = sa.value("type", "constant_force");
                    mech.springAssist.forceN = sa.value("force_N", 0.0);
                }

                if (mNode.contains("controllers") && mNode["controllers"].is_array())
                {
                    for (const auto& cNode : mNode["controllers"])
                    {
                        ControllerConfig ctrl;
                        ctrl.vendor = cNode.value("vendor", "CTRE_TalonFX");
                        ctrl.canId = cNode.value("canId", 0);
                        ctrl.canBus = cNode.value("canBus", "rio");
                        ctrl.inverted = cNode.value("inverted", false);
                        ctrl.role = cNode.value("role", "leader");
                        mech.controllers.push_back(ctrl);
                    }
                }

                sidecar.mechanisms.push_back(mech);
            }
        }

        // Couplings
        if (json.contains("couplings") && json["couplings"].is_array())
        {
            for (const auto& cNode : json["couplings"])
            {
                CouplingConfig coup;
                coup.name = cNode.value("name", "");
                coup.type = cNode.value("type", "power_coupling");
                coup.solverMode = cNode.value("solverMode", "analytic_coupled");

                if (coup.type == "power_coupling" && cNode.contains("powerCoupling"))
                {
                    const auto& pc = cNode["powerCoupling"];
                    coup.powerCoupling.transmissionType = pc.value("transmissionType", "gear");
                    coup.powerCoupling.masterJoint = pc.value("masterJoint", "");
                    coup.powerCoupling.slaveJoint = pc.value("slaveJoint", "");
                    coup.powerCoupling.ratio = pc.value("ratio", 1.0);
                    coup.powerCoupling.inverted = pc.value("inverted", false);
                    coup.powerCoupling.efficiency = pc.value("efficiency", 0.95);
                    coup.powerCoupling.backlashRad = pc.value("backlashRad", 0.0);
                }
                else if (coup.type == "elevator_cascade")
                {
                    auto stageArray = cNode.contains("cascadeStages") ? cNode["cascadeStages"] :
                        (cNode.contains("stages") ? cNode["stages"] : nlohmann::json::array());

                    for (const auto& sNode : stageArray)
                    {
                        StageCouplingConfig stage;
                        stage.joint = sNode.value("joint", "");
                        stage.multiplier = sNode.value("multiplier", 1.0);
                        coup.cascadeStages.push_back(stage);
                    }
                }
                else if (coup.type == "four_bar_loop")
                {
                    coup.fourBar.crankJoint = cNode.value("crankJoint", "");
                    coup.fourBar.couplerJoint = cNode.value("couplerJoint", "");
                    coup.fourBar.followerJoint = cNode.value("followerJoint", "");
                    coup.fourBar.closureParentLink = cNode.value("closureParentLink", "");
                    coup.fourBar.closureChildLink = cNode.value("closureChildLink", "");
                    coup.fourBar.closureParentAnchorM = ParseOptionalVec3(cNode, "closureParentAnchor_m");
                    coup.fourBar.closureChildAnchorM = ParseOptionalVec3(cNode, "closureChildAnchor_m");
                    coup.fourBar.closureAxis = ParseOptionalVec3(cNode, "closureAxis", glm::dvec3(0.0, 1.0, 0.0));
                }

                sidecar.couplings.push_back(coup);
            }
        }

        // Sensors
        if (json.contains("sensors") && json["sensors"].is_array())
        {
            for (const auto& sNode : json["sensors"])
            {
                SensorConfig sensor;
                sensor.name = sNode.value("name", "");
                sensor.type = sNode.value("type", "imu_pigeon2");
                sensor.bus = sNode.value("bus", "rio");
                sensor.canId = sNode.value("canId", 0);
                sensor.dioChannel = sNode.value("dioChannel", 0);
                sensor.analogChannel = sNode.value("analogChannel", 0);
                sensor.i2cAddress = sNode.value("i2cAddress", 0);
                sensor.attachToControllerCanId = sNode.value("attachToControllerCanId", 0);
                sensor.parentLink = sNode.value("parentLink", "");
                sensor.targetJoint = sNode.value("targetJoint", "");
                sensor.contactZone = sNode.value("contactZone", "");
                sensor.limitDirection = sNode.value("limitDirection", "forward");
                sensor.triggerMin = sNode.value("triggerMin", 0.0);
                sensor.triggerMax = sNode.value("triggerMax", 0.0);
                sensor.normallyClosed = sNode.value("normallyClosed", false);
                sensor.debounceTimeS = sNode.value("debounceTime_s", 0.010);
                sensor.inverted = sNode.value("inverted", false);
                sensor.mountOffsetM = ParseOptionalVec3(sNode, "mountOffset_m");
                sensor.mountRotationQuat = ParseOptionalQuat(sNode, "mountRotation_quat");
                sensor.gyroNoiseStdDevRadS = sNode.value("gyroNoiseStdDev_rad_s", 0.0);
                sensor.gyroBiasRandomWalkRadS2 = sNode.value("gyroBiasRandomWalk_rad_s2", 0.0);
                sensor.accelNoiseStdDevMS2 = sNode.value("accelNoiseStdDev_m_s2", 0.0);
                sensor.sensorToMechanismRatio = sNode.value("sensorToMechanismRatio", 1.0);
                sensor.cpr = sNode.value("cpr", 4096);
                sensor.offsetRad = sNode.value("offset_rad", 0.0);

                if (sNode.contains("customNumericParams") && sNode["customNumericParams"].is_object())
                {
                    for (auto it = sNode["customNumericParams"].begin(); it != sNode["customNumericParams"].end(); ++it)
                    {
                        if (it.value().is_number())
                        {
                            sensor.customNumericParams[it.key()] = it.value().get<double>();
                        }
                    }
                }

                if (sNode.contains("customStringParams") && sNode["customStringParams"].is_object())
                {
                    for (auto it = sNode["customStringParams"].begin(); it != sNode["customStringParams"].end(); ++it)
                    {
                        if (it.value().is_string())
                        {
                            sensor.customStringParams[it.key()] = it.value().get<std::string>();
                        }
                    }
                }

                sidecar.sensors.push_back(sensor);
            }
        }

        return true;
    }

    bool RobotSchemaLoader::LoadMaterialsFromFile(const std::string& filepath)
    {
        return materialLibrary.LoadFromFile(filepath);
    }

    bool RobotSchemaLoader::Validate()
    {
        bool isValid = true;

        // 1. Schema Version Enforcement
        if (manifest.schemaVersion != "1.0.0")
        {
            errors.push_back("Unsupported manifest schemaVersion: '" + manifest.schemaVersion + "' (Expected '1.0.0')");
            isValid = false;
        }

        if (!sidecar.targetManifest.empty() && sidecar.schemaVersion != "1.0.0")
        {
            errors.push_back("Unsupported sidecar schemaVersion: '" + sidecar.schemaVersion + "' (Expected '1.0.0')");
            isValid = false;
        }

        // 2. Link Physical Sanity
        if (manifest.links.empty())
        {
            errors.push_back("Manifest defines no links");
            isValid = false;
        }

        const LinkDefinition* root = manifest.GetRootLink();
        if (!root)
        {
            warnings.push_back("No link has isRoot set to true; defaulting to first link: " + (manifest.links.empty() ? "" : manifest.links[0].name));
        }

        for (const auto& link : manifest.links)
        {
            if (link.massKg <= 1e-6)
            {
                errors.push_back("Link '" + link.name + "' has invalid non-positive mass: " + std::to_string(link.massKg));
                isValid = false;
            }

            if (!link.inertiaTensor.IsPositiveDefinite())
            {
                errors.push_back("Link '" + link.name + "' inertia tensor is not strictly positive-definite");
                isValid = false;
            }

            if (!link.material.empty() && !materialLibrary.HasMaterial(link.material))
            {
                warnings.push_back("Link '" + link.name + "' references unknown material: '" + link.material + "'");
            }

            for (const auto& col : link.colliders)
            {
                if (!col.material.empty() && !materialLibrary.HasMaterial(col.material))
                {
                    warnings.push_back("Collider '" + col.name + "' references unknown material: '" + col.material + "'");
                }
            }
        }

        // 3. Joint Topology
        for (const auto& joint : manifest.joints)
        {
            if (!manifest.FindLink(joint.parentLink))
            {
                errors.push_back("Joint '" + joint.name + "' references missing parentLink: '" + joint.parentLink + "'");
                isValid = false;
            }

            if (!manifest.FindLink(joint.childLink))
            {
                errors.push_back("Joint '" + joint.name + "' references missing childLink: '" + joint.childLink + "'");
                isValid = false;
            }
        }

        // 4. Contact Zones
        for (const auto& zone : manifest.contactZones)
        {
            if (!manifest.FindLink(zone.parentLink))
            {
                errors.push_back("ContactZone '" + zone.name + "' references missing parentLink: '" + zone.parentLink + "'");
                isValid = false;
            }
        }

        // 5. Sidecar Swerve Chassis Link
        if (!sidecar.swerve.chassisLink.empty() && !manifest.FindLink(sidecar.swerve.chassisLink))
        {
            errors.push_back("Swerve config references missing chassisLink: '" + sidecar.swerve.chassisLink + "'");
            isValid = false;
        }

        // 6. Sidecar Mechanisms
        for (const auto& mech : sidecar.mechanisms)
        {
            if (!mech.targetJoint.empty() && !manifest.FindJoint(mech.targetJoint))
            {
                errors.push_back("Mechanism '" + mech.name + "' references missing targetJoint: '" + mech.targetJoint + "'");
                isValid = false;
            }

            if (!mech.parentLink.empty() && !manifest.FindLink(mech.parentLink))
            {
                errors.push_back("Mechanism '" + mech.name + "' references missing parentLink: '" + mech.parentLink + "'");
                isValid = false;
            }
        }

        // 7. Sidecar Couplings
        for (const auto& coup : sidecar.couplings)
        {
            if (coup.type == "power_coupling")
            {
                if (!manifest.FindJoint(coup.powerCoupling.masterJoint))
                {
                    errors.push_back("Coupling '" + coup.name + "' references missing masterJoint: '" + coup.powerCoupling.masterJoint + "'");
                    isValid = false;
                }
                if (!manifest.FindJoint(coup.powerCoupling.slaveJoint))
                {
                    errors.push_back("Coupling '" + coup.name + "' references missing slaveJoint: '" + coup.powerCoupling.slaveJoint + "'");
                    isValid = false;
                }
            }
            else if (coup.type == "elevator_cascade")
            {
                for (const auto& stage : coup.cascadeStages)
                {
                    if (!manifest.FindJoint(stage.joint))
                    {
                        errors.push_back("Cascade coupling '" + coup.name + "' references missing stage joint: '" + stage.joint + "'");
                        isValid = false;
                    }
                }
            }
            else if (coup.type == "four_bar_loop")
            {
                if (!manifest.FindJoint(coup.fourBar.crankJoint))
                {
                    errors.push_back("FourBar coupling '" + coup.name + "' references missing crankJoint: '" + coup.fourBar.crankJoint + "'");
                    isValid = false;
                }
                if (!manifest.FindJoint(coup.fourBar.couplerJoint))
                {
                    errors.push_back("FourBar coupling '" + coup.name + "' references missing couplerJoint: '" + coup.fourBar.couplerJoint + "'");
                    isValid = false;
                }
                if (!manifest.FindJoint(coup.fourBar.followerJoint))
                {
                    errors.push_back("FourBar coupling '" + coup.name + "' references missing followerJoint: '" + coup.fourBar.followerJoint + "'");
                    isValid = false;
                }
                if (!manifest.FindLink(coup.fourBar.closureParentLink))
                {
                    errors.push_back("FourBar coupling '" + coup.name + "' references missing closureParentLink: '" + coup.fourBar.closureParentLink + "'");
                    isValid = false;
                }
                if (!manifest.FindLink(coup.fourBar.closureChildLink))
                {
                    errors.push_back("FourBar coupling '" + coup.name + "' references missing closureChildLink: '" + coup.fourBar.closureChildLink + "'");
                    isValid = false;
                }
            }
        }

        // 8. Sidecar Sensors
        for (const auto& sensor : sidecar.sensors)
        {
            if (!sensor.parentLink.empty() && !manifest.FindLink(sensor.parentLink))
            {
                errors.push_back("Sensor '" + sensor.name + "' references missing parentLink: '" + sensor.parentLink + "'");
                isValid = false;
            }
            if (!sensor.targetJoint.empty() && !manifest.FindJoint(sensor.targetJoint))
            {
                errors.push_back("Sensor '" + sensor.name + "' references missing targetJoint: '" + sensor.targetJoint + "'");
                isValid = false;
            }
            if (!sensor.contactZone.empty() && !manifest.FindContactZone(sensor.contactZone))
            {
                errors.push_back("Sensor '" + sensor.name + "' references missing contactZone: '" + sensor.contactZone + "'");
                isValid = false;
            }
        }

        return isValid;
    }

    const RobotManifest& RobotSchemaLoader::GetManifest() const { return manifest; }
    const RobotSidecar& RobotSchemaLoader::GetSidecar() const { return sidecar; }
    const MaterialLibrary& RobotSchemaLoader::GetMaterials() const { return materialLibrary; }
    MaterialLibrary& RobotSchemaLoader::GetMaterials() { return materialLibrary; }

    bool RobotSchemaLoader::HasErrors() const { return !errors.empty(); }
    bool RobotSchemaLoader::HasWarnings() const { return !warnings.empty(); }
    const std::vector<std::string>& RobotSchemaLoader::GetErrors() const { return errors; }
    const std::vector<std::string>& RobotSchemaLoader::GetWarnings() const { return warnings; }

    void RobotSchemaLoader::ClearDiagnostics()
    {
        errors.clear();
        warnings.clear();
    }

    glm::dmat3 RobotSchemaLoader::GetNWUToEngineMatrix()
    {
        return glm::dmat3(
            0.0, 0.0, 1.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0
        );
    }

    glm::dmat3 RobotSchemaLoader::GetEngineToNWUMatrix()
    {
        return glm::dmat3(
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            1.0, 0.0, 0.0
        );
    }

    glm::dvec3 RobotSchemaLoader::ConvertNWUVectorToEngine(const glm::dvec3& vNWU)
    {
        return glm::dvec3(vNWU.y, vNWU.z, vNWU.x);
    }

    glm::dvec3 RobotSchemaLoader::ConvertEngineVectorToNWU(const glm::dvec3& vEngine)
    {
        return glm::dvec3(vEngine.z, vEngine.x, vEngine.y);
    }

    glm::vec3 RobotSchemaLoader::ConvertNWUVectorToEngine(const glm::vec3& vNWU)
    {
        return glm::vec3(vNWU.y, vNWU.z, vNWU.x);
    }

    glm::vec3 RobotSchemaLoader::ConvertEngineVectorToNWU(const glm::vec3& vEngine)
    {
        return glm::vec3(vEngine.z, vEngine.x, vEngine.y);
    }

    glm::dmat3 RobotSchemaLoader::ConvertNWUInertiaToEngine(const glm::dmat3& iNWU)
    {
        glm::dmat3 M = GetNWUToEngineMatrix();
        return M * iNWU * glm::transpose(M);
    }

    glm::dmat3 RobotSchemaLoader::ConvertEngineInertiaToNWU(const glm::dmat3& iEngine)
    {
        glm::dmat3 Minv = GetEngineToNWUMatrix();
        return Minv * iEngine * glm::transpose(Minv);
    }

    glm::dquat RobotSchemaLoader::ConvertNWUQuatToEngine(const glm::dquat& qNWU)
    {
        glm::dmat3 M = GetNWUToEngineMatrix();
        glm::dmat3 rNWU = glm::mat3_cast(qNWU);
        glm::dmat3 rEngine = M * rNWU * glm::transpose(M);
        return glm::normalize(glm::quat_cast(rEngine));
    }

    glm::dquat RobotSchemaLoader::ConvertEngineQuatToNWU(const glm::dquat& qEngine)
    {
        glm::dmat3 Minv = GetEngineToNWUMatrix();
        glm::dmat3 rEngine = glm::mat3_cast(qEngine);
        glm::dmat3 rNWU = Minv * rEngine * glm::transpose(Minv);
        return glm::normalize(glm::quat_cast(rNWU));
    }
}