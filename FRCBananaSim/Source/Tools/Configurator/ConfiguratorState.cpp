/**
 * @file ConfiguratorState.cpp
 * @brief Implementation of CAD ingestion, starter sidecar auto-generation, serialization, and linter.
 */

#include "Tools/Configurator/ConfiguratorState.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Schema/RobotSidecarSerializer.h"

#include <filesystem>
#include <algorithm>
#include <map>
#include <set>
#include <fstream>
#include <nlohmann/json.hpp>

namespace FRCBananaSim::Tools::Configurator
{
    using namespace Robot::Schema;

    ConfiguratorState::ConfiguratorState()
    {
        Clear();
    }

    void ConfiguratorState::Clear()
    {
        cadFolderPath.clear();
        manifestPath.clear();
        sidecarPath.clear();
        materialsPath.clear();

        manifest = RobotManifest();
        sidecar = RobotSidecar();
        materials = MaterialLibrary();

        isCadLoaded = false;
        isSidecarLoaded = false;

        selectedLink.clear();
        selectedJoint.clear();
        selectedMechanism.clear();

        statusMessage = "Ready. Open a CAD folder or load Reference2026 to begin.";
        linterErrors.clear();
        linterWarnings.clear();
    }

    bool ConfiguratorState::LoadCADFolder(const std::string& folderPath)
    {
        std::filesystem::path folder(folderPath);
        if (!std::filesystem::exists(folder) || !std::filesystem::is_directory(folder))
        {
            statusMessage = "Error: Folder path does not exist: " + folderPath;
            return false;
        }

        cadFolderPath = folder.string();
        std::filesystem::path mPath = folder / "manifest.json";
        std::filesystem::path sPath = folder / "sidecar.json";
        std::filesystem::path matPath = folder / "materials.json";

        if (!std::filesystem::exists(mPath))
        {
            statusMessage = "Error: Folder missing manifest.json: " + mPath.string();
            return false;
        }

        std::string sPathStr = std::filesystem::exists(sPath) ? sPath.string() : "";
        std::string matPathStr = std::filesystem::exists(matPath) ? matPath.string() : "";

        RobotSchemaLoader loader;
        if (!loader.LoadManifestFromFile(mPath.string()))
        {
            statusMessage = "Validation Error loading CAD manifest.json: ";
            if (loader.HasErrors())
            {
                statusMessage += loader.GetErrors()[0];
            }
            return false;
        }

        manifest = loader.GetManifest();
        manifestPath = mPath.string();
        isCadLoaded = true;

        if (!sPathStr.empty())
        {
            if (loader.LoadSidecarFromFile(sPathStr))
            {
                sidecar = loader.GetSidecar();
                sidecarPath = sPathStr;
                isSidecarLoaded = true;
                sidecarVersion++;
            }
            else
            {
                sidecar = RobotSidecar();
                sidecarPath = sPathStr;
                isSidecarLoaded = false;
                sidecarVersion++;
                statusMessage = "CAD loaded. Note: sidecar.json has syntax errors.";
            }
        }
        else
        {
            sidecar = RobotSidecar();
            sidecarPath = (folder / "sidecar.json").string();
            isSidecarLoaded = false;
            sidecarVersion++;
        }

        if (!matPathStr.empty())
        {
            materials.LoadFromFile(matPathStr);
            materialsPath = matPathStr;
        }

        if (!manifest.links.empty())
        {
            selectedLink = manifest.links[0].name;
        }
        if (!manifest.joints.empty())
        {
            selectedJoint = manifest.joints[0].name;
        }

        statusMessage = "Loaded CAD robot '" + manifest.robotName + "' with " +
            std::to_string(manifest.links.size()) + " links, " +
            std::to_string(manifest.joints.size()) + " joints.";

        RunLinter();
        return true;
    }

    bool ConfiguratorState::LoadManifest(const std::string& filePath)
    {
        RobotSchemaLoader loader;
        if (!loader.LoadManifestFromFile(filePath))
        {
            statusMessage = "Failed to parse manifest: " + filePath;
            return false;
        }

        manifest = loader.GetManifest();
        manifestPath = filePath;
        isCadLoaded = true;
        cadFolderPath = std::filesystem::path(filePath).parent_path().string();

        if (!manifest.links.empty())
        {
            selectedLink = manifest.links[0].name;
        }
        if (!manifest.joints.empty())
        {
            selectedJoint = manifest.joints[0].name;
        }

        statusMessage = "Loaded manifest '" + manifest.robotName + "'.";
        RunLinter();
        return true;
    }

    bool ConfiguratorState::LoadSidecar(const std::string& filePath)
    {
        RobotSchemaLoader loader;
        if (!loader.LoadSidecarFromFile(filePath))
        {
            statusMessage = "Failed to parse sidecar: " + filePath;
            return false;
        }

        sidecar = loader.GetSidecar();
        sidecarPath = filePath;
        isSidecarLoaded = true;
        sidecarVersion++;

        statusMessage = "Loaded sidecar: " + filePath;
        RunLinter();
        return true;
    }

    bool ConfiguratorState::SaveSidecar(const std::string& filePath)
    {
        std::string targetPath = filePath.empty() ? sidecarPath : filePath;
        if (targetPath.empty())
        {
            if (!cadFolderPath.empty())
            {
                targetPath = (std::filesystem::path(cadFolderPath) / "sidecar.json").string();
            }
            else
            {
                targetPath = "sidecar.json";
            }
        }

        sidecarPath = targetPath;
        bool ok = RobotSidecarSerializer::SerializeToFile(sidecar, targetPath);
        if (ok)
        {
            isSidecarLoaded = true;
            statusMessage = "Successfully saved sidecar to " + targetPath;
        }
        else
        {
            statusMessage = "Error: Failed to write sidecar file to " + targetPath;
        }

        return ok;
    }

    void ConfiguratorState::AutoGenerateStarterSidecar()
    {
        if (!isCadLoaded)
        {
            statusMessage = "Cannot auto-generate: No CAD manifest loaded!";
            return;
        }

        sidecar = RobotSidecar();

        // 1. Identify Root Link for Chassis
        std::string rootLinkName = "base_link";
        for (const auto& link : manifest.links)
        {
            if (link.isRoot)
            {
                rootLinkName = link.name;
                break;
            }
        }

        const auto* rootLink = manifest.FindLink(rootLinkName);
        double chassisMass = rootLink && rootLink->massKg > 0.0 ? rootLink->massKg : 35.0;

        // 2. Seed Swerve Drivetrain
        sidecar.swerve.chassisLink = rootLinkName;
        sidecar.swerve.driveMotorPreset = "KrakenX60";
        sidecar.swerve.steerMotorPreset = "KrakenX44";
        sidecar.swerve.driveGearRatio = 6.75;
        sidecar.swerve.steerGearRatio = 21.428;
        sidecar.swerve.wheelRadiusM = 0.0508; // 2 inch nominal
        sidecar.swerve.driveStatorLimitA = 60.0;
        sidecar.swerve.steerStatorLimitA = 40.0;

        constexpr double halfExt = 0.28; // Standard 22"x22" wheel placement
        sidecar.swerve.modules = {
            SwerveModuleConfig{ "FrontLeft",  glm::dvec3( halfExt,  halfExt, 0.0), 1, 2, 21, "rio" },
            SwerveModuleConfig{ "FrontRight", glm::dvec3( halfExt, -halfExt, 0.0), 3, 4, 22, "rio" },
            SwerveModuleConfig{ "BackLeft",   glm::dvec3(-halfExt,  halfExt, 0.0), 5, 6, 23, "rio" },
            SwerveModuleConfig{ "BackRight",  glm::dvec3(-halfExt, -halfExt, 0.0), 7, 8, 24, "rio" }
        };

        // 3. Seed 1-DOF Mechanisms from Non-Swerve Joints
        uint32_t nextCanId = 9;
        for (const auto& joint : manifest.joints)
        {
            std::string jNameLower = joint.name;
            std::transform(jNameLower.begin(), jNameLower.end(), jNameLower.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            // Ignore internal swerve steer joints if named swerve_*
            if (jNameLower.find("steer") != std::string::npos && jNameLower.find("swerve") != std::string::npos)
            {
                continue;
            }

            MechanismConfig mech;
            mech.targetJoint = joint.name;
            mech.parentLink = joint.parentLink;
            mech.name = joint.name;

            if (nextCanId >= 20 && nextCanId <= 25)
            {
                nextCanId = 26;
            }
            ControllerConfig ctrl;
            ctrl.canId = nextCanId++;
            ctrl.vendor = "CTRE_TalonFX";
            ctrl.canBus = "rio";
            ctrl.inverted = false;
            ctrl.role = "leader";

            if (joint.type == "prismatic")
            {
                mech.role = "elevator";
                mech.motorPreset = "KrakenX60";
                mech.gearRatio = 9.0;
                mech.efficiency = 0.92;
                mech.drumRadiusM = 0.01905;
                mech.springAssist.type = "constant_force";
                mech.springAssist.forceN = 45.0;
                mech.statorCurrentLimitA = 60.0;
                mech.controllers.push_back(ctrl);
                sidecar.mechanisms.push_back(mech);
            }
            else if (joint.type == "revolute" || joint.type == "continuous")
            {
                if (jNameLower.find("wrist") != std::string::npos)
                {
                    mech.role = "wrist";
                    mech.motorPreset = "Falcon500";
                    mech.gearRatio = 25.0;
                    mech.efficiency = 0.95;
                    mech.statorCurrentLimitA = 40.0;
                }
                else if (jNameLower.find("flywheel") != std::string::npos || jNameLower.find("shooter") != std::string::npos)
                {
                    mech.role = "flywheel";
                    mech.motorPreset = "KrakenX60";
                    mech.gearRatio = 0.5;
                    mech.efficiency = 0.95;
                    mech.flywheelInertiaKgm2 = 0.0032;
                    mech.statorCurrentLimitA = 80.0;
                }
                else if (jNameLower.find("turret") != std::string::npos || jNameLower.find("azimuth") != std::string::npos)
                {
                    mech.role = "turret";
                    mech.motorPreset = "KrakenX44";
                    mech.gearRatio = 40.0;
                    mech.efficiency = 0.95;
                    mech.statorCurrentLimitA = 40.0;
                }
                else if (jNameLower.find("intake") != std::string::npos)
                {
                    mech.role = "intake";
                    mech.motorPreset = "KrakenX60";
                    mech.gearRatio = 12.0;
                    mech.efficiency = 0.95;
                    mech.statorCurrentLimitA = 40.0;
                }
                else
                {
                    mech.role = "arm";
                    mech.motorPreset = "KrakenX60";
                    mech.gearRatio = 65.0;
                    mech.efficiency = 0.95;
                    mech.statorCurrentLimitA = 60.0;
                }

                mech.controllers.push_back(ctrl);
                sidecar.mechanisms.push_back(mech);
            }
        }

        // 4. Seed Standard Pigeon 2 IMU
        SensorConfig imu;
        imu.name = "imu_pigeon2";
        imu.type = "imu_pigeon2";
        imu.canId = 25;
        imu.bus = "rio";
        imu.parentLink = rootLinkName;
        sidecar.sensors.push_back(imu);

        // 5. Seed Parametric FRC Bumpers (enabled if no CAD bumpers present)
        bool hasCadBumpers = false;
        for (const auto& l : manifest.links)
        {
            for (const auto& col : l.colliders)
            {
                if (col.name.find("bumper") != std::string::npos || col.material == "bumper_fabric")
                {
                    hasCadBumpers = true;
                    break;
                }
            }
        }
        sidecar.bumpers.enabled = !hasCadBumpers;
        sidecar.bumpers.targetLink = rootLinkName;
        sidecar.bumpers.shape = "box";
        sidecar.bumpers.lengthM = 0.85;
        sidecar.bumpers.widthM = 0.85;
        sidecar.bumpers.heightM = 0.127;
        sidecar.bumpers.thicknessM = 0.0825;
        sidecar.bumpers.offsetM = glm::dvec3(0.0, 0.09, 0.0); // +Y Up: 0.09m elevation
        sidecar.bumpers.massKg = 6.0;
        sidecar.bumpers.material = "bumper_fabric";
        sidecar.bumpers.color = "red";
        sidecar.bumpers.cutoutWidthM = 0.40;

        isSidecarLoaded = true;
        sidecarVersion++;
        statusMessage = "Auto-generated starter sidecar with Swerve, " +
            std::to_string(sidecar.mechanisms.size()) + " mechanisms, and Pigeon2.";

        RunLinter();
    }

    void ConfiguratorState::RunLinter()
    {
        linterErrors.clear();
        linterWarnings.clear();

        if (!isCadLoaded)
        {
            return;
        }

        // 1. CAN ID Collisions
        std::map<std::pair<uint32_t, std::string>, std::string> canMap;

        auto checkCanId = [&](uint32_t id, const std::string& bus, const std::string& owner)
        {
            if (id == 0) return; // 0 represents unassigned
            auto key = std::make_pair(id, bus);
            auto it = canMap.find(key);
            if (it != canMap.end())
            {
                linterErrors.push_back("CAN ID Collision: ID " + std::to_string(id) + " on bus '" + bus +
                    "' is claimed by both '" + it->second + "' and '" + owner + "'.");
            }
            else
            {
                canMap[key] = owner;
            }
        };

        // Check swerve controllers and encoders
        for (const auto& mod : sidecar.swerve.modules)
        {
            checkCanId(mod.driveCanId, mod.canBus, "Swerve " + mod.name + " Drive");
            checkCanId(mod.steerCanId, mod.canBus, "Swerve " + mod.name + " Steer");
            checkCanId(mod.cancoderId, mod.canBus, "Swerve " + mod.name + " CANcoder");
        }

        // Check mechanism controllers
        for (const auto& mech : sidecar.mechanisms)
        {
            for (const auto& ctrl : mech.controllers)
            {
                checkCanId(ctrl.canId, ctrl.canBus, "Mechanism '" + mech.name + "'");
            }
        }

        // Check sensor CAN IDs
        for (const auto& sensor : sidecar.sensors)
        {
            checkCanId(sensor.canId, sensor.bus, "Sensor '" + sensor.name + "'");
        }

        // 2. PWM Channel Collisions
        std::map<uint32_t, std::string> pwmMap;
        for (const auto& mech : sidecar.mechanisms)
        {
            if (mech.actuatorType == "servo")
            {
                uint32_t ch = mech.servo.pwmChannel;
                auto it = pwmMap.find(ch);
                if (it != pwmMap.end())
                {
                    linterErrors.push_back("PWM Channel Collision: Channel " + std::to_string(ch) +
                        " is claimed by both '" + it->second + "' and '" + mech.name + "'.");
                }
                else
                {
                    pwmMap[ch] = mech.name;
                }
            }
        }

        // 3. Orphan Joint References
        for (const auto& mech : sidecar.mechanisms)
        {
            if (!mech.targetJoint.empty() && !manifest.FindJoint(mech.targetJoint))
            {
                linterErrors.push_back("Orphan Joint Reference: Mechanism '" + mech.name +
                    "' targets non-existent joint '" + mech.targetJoint + "'.");
            }

            if (mech.role == "differential")
            {
                if (mech.targetJoints.size() < 2 || mech.targetJoints[0].empty() || mech.targetJoints[1].empty())
                {
                    linterErrors.push_back("Differential Wrist '" + mech.name +
                        "' requires both targetJoints[0] (pitch) and targetJoints[1] (roll) to be specified.");
                }
                else
                {
                    const auto* pj = manifest.FindJoint(mech.targetJoints[0]);
                    const auto* rj = manifest.FindJoint(mech.targetJoints[1]);
                    if (!pj)
                    {
                        linterErrors.push_back("Differential Wrist '" + mech.name +
                            "' references missing pitchJoint '" + mech.targetJoints[0] + "'.");
                    }
                    if (!rj)
                    {
                        linterErrors.push_back("Differential Wrist '" + mech.name +
                            "' references missing rollJoint '" + mech.targetJoints[1] + "'.");
                    }

                    // Differentials strictly require two revolute joints
                    if (pj && pj->type != "revolute" && pj->type != "continuous")
                    {
                        linterErrors.push_back("Differential Wrist '" + mech.name +
                            "' pitchJoint '" + pj->name + "' must be revolute (got " + pj->type + ").");
                    }
                    if (rj && rj->type != "revolute" && rj->type != "continuous")
                    {
                        linterErrors.push_back("Differential Wrist '" + mech.name +
                            "' rollJoint '" + rj->name + "' must be revolute (got " + rj->type + ").");
                    }
                }
            }
        }

        // 4. Missing Joint Limits (Warning)
        for (const auto& mech : sidecar.mechanisms)
        {
            if (mech.role == "arm" || mech.role == "elevator" || mech.role == "wrist")
            {
                const auto* j = manifest.FindJoint(mech.targetJoint);
                if (j)
                {
                    auto eff = sidecar.GetEffectiveLimits(mech.targetJoint, manifest);
                    if (!eff.hasLimits)
                    {
                        linterWarnings.push_back("Joint '" + j->name +
                            "' targeted by mechanism '" + mech.name + "' has no physical limits configured in CAD or sidecar overrides.");
                    }
                }
            }
            else if (mech.role == "differential" && mech.targetJoints.size() >= 2)
            {
                for (size_t i = 0; i < 2; ++i)
                {
                    const auto& jName = mech.targetJoints[i];
                    if (!jName.empty() && manifest.FindJoint(jName))
                    {
                        auto eff = sidecar.GetEffectiveLimits(jName, manifest);
                        if (!eff.hasLimits)
                        {
                            linterWarnings.push_back("Joint '" + jName +
                                "' (output " + (i == 0 ? "A/pitch" : "B/roll") + ") targeted by differential '" +
                                mech.name + "' has no physical limits configured in CAD or sidecar overrides.");
                        }
                    }
                }
            }
        }

        // 4b. Joint Limit Overrides Validation
        for (const auto& [jointName, ov] : sidecar.jointOverrides)
        {
            if (!manifest.FindJoint(jointName))
            {
                linterErrors.push_back("Orphan Joint Override: Joint '" + jointName +
                    "' referenced in jointOverrides does not exist in CAD manifest.");
            }
            else if (ov.hasLimits && ov.lower > ov.upper)
            {
                linterErrors.push_back("Joint Limits Error: Joint '" + jointName +
                    "' lower limit (" + std::to_string(ov.lower) + ") cannot be greater than upper limit (" +
                    std::to_string(ov.upper) + ").");
            }
        }

        // 5. High Stator Current Warning (> 400A aggregate)
        double totalStatorLimitA = 0.0;
        totalStatorLimitA += (sidecar.swerve.driveStatorLimitA * 4.0);
        totalStatorLimitA += (sidecar.swerve.steerStatorLimitA * 4.0);
        for (const auto& mech : sidecar.mechanisms)
        {
            totalStatorLimitA += (mech.statorCurrentLimitA * std::max<size_t>(1, mech.controllers.size()));
        }

        if (totalStatorLimitA > 400.0)
        {
            linterWarnings.push_back("Aggregate stator current limits exceed 400 A (configured " +
                std::to_string(static_cast<int>(totalStatorLimitA)) + " A). Tune supply/stator limits to prevent brownout.");
        }

        // 6. Mass Reconciliation Target Link Validation
        if (sidecar.massReconciliation.enabled && sidecar.massReconciliation.unmodeledBallast.massKg > 0.0)
        {
            const auto& target = sidecar.massReconciliation.unmodeledBallast.targetLink;
            if (!target.empty() && !manifest.FindLink(target))
            {
                linterErrors.push_back("Mass Reconciliation unmodeledBallast references missing targetLink '" + target + "'.");
            }
        }

        // 7. Mechanical Couplings Validation
        for (const auto& coup : sidecar.couplings)
        {
            if (coup.type == "power_coupling")
            {
                if (coup.powerCoupling.masterJoint.empty())
                {
                    linterErrors.push_back("Power Coupling '" + coup.name + "' has no masterJoint specified.");
                }
                else if (!manifest.FindJoint(coup.powerCoupling.masterJoint))
                {
                    linterErrors.push_back("Power Coupling '" + coup.name + "' references missing masterJoint '" +
                        coup.powerCoupling.masterJoint + "'.");
                }

                if (coup.powerCoupling.slaveJoint.empty())
                {
                    linterErrors.push_back("Power Coupling '" + coup.name + "' has no slaveJoint specified.");
                }
                else if (!manifest.FindJoint(coup.powerCoupling.slaveJoint))
                {
                    linterErrors.push_back("Power Coupling '" + coup.name + "' references missing slaveJoint '" +
                        coup.powerCoupling.slaveJoint + "'.");
                }

                if (!coup.powerCoupling.masterJoint.empty() && coup.powerCoupling.masterJoint == coup.powerCoupling.slaveJoint)
                {
                    linterErrors.push_back("Power Coupling '" + coup.name + "' cannot couple joint '" +
                        coup.powerCoupling.masterJoint + "' to itself.");
                }
            }
            else if (coup.type == "elevator_cascade")
            {
                if (coup.cascadeStages.empty())
                {
                    linterWarnings.push_back("Elevator Cascade Coupling '" + coup.name + "' has no stages defined.");
                }
                for (size_t s = 0; s < coup.cascadeStages.size(); ++s)
                {
                    const auto& stage = coup.cascadeStages[s];
                    if (stage.joint.empty())
                    {
                        linterErrors.push_back("Elevator Cascade Coupling '" + coup.name + "' stage " +
                            std::to_string(s + 1) + " has no joint specified.");
                    }
                    else
                    {
                        const auto* j = manifest.FindJoint(stage.joint);
                        if (!j)
                        {
                            linterErrors.push_back("Elevator Cascade Coupling '" + coup.name + "' stage " +
                                std::to_string(s + 1) + " references missing joint '" + stage.joint + "'.");
                        }
                        else if (j->type != "prismatic")
                        {
                            linterErrors.push_back("Elevator Cascade Coupling '" + coup.name + "' stage joint '" +
                                stage.joint + "' must be prismatic (got " + j->type + ").");
                        }
                    }
                }
            }
            else if (coup.type == "four_bar_loop")
            {
                if (!coup.fourBar.crankJoint.empty() && !manifest.FindJoint(coup.fourBar.crankJoint))
                {
                    linterErrors.push_back("Four-Bar Coupling '" + coup.name + "' references missing crankJoint '" +
                        coup.fourBar.crankJoint + "'.");
                }
                if (!coup.fourBar.couplerJoint.empty() && !manifest.FindJoint(coup.fourBar.couplerJoint))
                {
                    linterErrors.push_back("Four-Bar Coupling '" + coup.name + "' references missing couplerJoint '" +
                        coup.fourBar.couplerJoint + "'.");
                }
                if (!coup.fourBar.followerJoint.empty() && !manifest.FindJoint(coup.fourBar.followerJoint))
                {
                    linterErrors.push_back("Four-Bar Coupling '" + coup.name + "' references missing followerJoint '" +
                        coup.fourBar.followerJoint + "'.");
                }
                if (!coup.fourBar.closureParentLink.empty() && !manifest.FindLink(coup.fourBar.closureParentLink))
                {
                    linterErrors.push_back("Four-Bar Coupling '" + coup.name + "' references missing closureParentLink '" +
                        coup.fourBar.closureParentLink + "'.");
                }
                if (!coup.fourBar.closureChildLink.empty() && !manifest.FindLink(coup.fourBar.closureChildLink))
                {
                    linterErrors.push_back("Four-Bar Coupling '" + coup.name + "' references missing closureChildLink '" +
                        coup.fourBar.closureChildLink + "'.");
                }
            }
        }

        // 8. Sensor Suite Validation
        std::map<uint32_t, std::string> dioMap;
        std::map<uint32_t, std::string> analogMap;

        std::set<uint32_t> activeControllerCanIds;
        for (const auto& mod : sidecar.swerve.modules)
        {
            if (mod.driveCanId > 0) activeControllerCanIds.insert(mod.driveCanId);
            if (mod.steerCanId > 0) activeControllerCanIds.insert(mod.steerCanId);
        }
        for (const auto& mech : sidecar.mechanisms)
        {
            for (const auto& ctrl : mech.controllers)
            {
                if (ctrl.canId > 0) activeControllerCanIds.insert(ctrl.canId);
            }
        }

        for (const auto& sensor : sidecar.sensors)
        {
            // Orphan Target Joint check
            if (!sensor.targetJoint.empty() && !manifest.FindJoint(sensor.targetJoint))
            {
                linterErrors.push_back("Orphan Joint Reference: Sensor '" + sensor.name +
                    "' targets non-existent joint '" + sensor.targetJoint + "'.");
            }

            // Orphan Parent Link check
            if (!sensor.parentLink.empty() && !manifest.FindLink(sensor.parentLink))
            {
                linterErrors.push_back("Orphan Link Reference: Sensor '" + sensor.name +
                    "' targets non-existent parent link '" + sensor.parentLink + "'.");
            }

            // Contact Zone check for beam breaks
            if (!sensor.contactZone.empty())
            {
                bool zoneFound = false;
                for (const auto& cz : manifest.contactZones)
                {
                    if (cz.name == sensor.contactZone)
                    {
                        zoneFound = true;
                        break;
                    }
                }
                if (!zoneFound)
                {
                    linterErrors.push_back("Orphan Contact Zone Reference: Sensor '" + sensor.name +
                        "' references non-existent contact zone '" + sensor.contactZone + "'.");
                }
            }

            // DIO Channel Validation
            bool checkPrimaryDio = false;
            if (sensor.type == "beam_break" ||
                (sensor.type == "limit_switch" && sensor.attachToControllerCanId == 0) ||
                (sensor.type == "rev_through_bore" && sensor.attachToControllerCanId == 0) ||
                (sensor.type == "andymark_lidar3" && sensor.bus == "dio_pwm"))
            {
                checkPrimaryDio = true;
            }

            if (checkPrimaryDio)
            {
                if (sensor.dioChannel > 31)
                {
                    linterErrors.push_back("DIO Channel Out of Range: Sensor '" + sensor.name +
                        "' specifies DIO channel " + std::to_string(sensor.dioChannel) + " (valid [0..31]).");
                }
                else
                {
                    auto it = dioMap.find(sensor.dioChannel);
                    if (it != dioMap.end())
                    {
                        linterErrors.push_back("DIO Channel Collision: Channel " + std::to_string(sensor.dioChannel) +
                            " is claimed by both '" + it->second + "' and '" + sensor.name + "'.");
                    }
                    else
                    {
                        dioMap[sensor.dioChannel] = sensor.name;
                    }
                }
            }

            // REV Through Bore Secondary DIO (Quadrature Channel B)
            if (sensor.type == "rev_through_bore" && sensor.attachToControllerCanId == 0)
            {
                bool isQuad = (sensor.customStringParams.count("encoderMode") && sensor.customStringParams.at("encoderMode") == "quadrature")
                    || (sensor.secondaryDioChannel > 0);
                if (isQuad)
                {
                    if (sensor.secondaryDioChannel > 31)
                    {
                        linterErrors.push_back("DIO Channel Out of Range: REV Through Bore '" + sensor.name +
                            "' specifies Channel B DIO " + std::to_string(sensor.secondaryDioChannel) + " (valid [0..31]).");
                    }
                    else if (sensor.secondaryDioChannel == sensor.dioChannel)
                    {
                        linterErrors.push_back("DIO Channel Collision: REV Through Bore '" + sensor.name +
                            "' Channel A and Channel B cannot share the same DIO channel (" + std::to_string(sensor.dioChannel) + ").");
                    }
                    else
                    {
                        auto it = dioMap.find(sensor.secondaryDioChannel);
                        if (it != dioMap.end())
                        {
                            linterErrors.push_back("DIO Channel Collision: Channel " + std::to_string(sensor.secondaryDioChannel) +
                                " is claimed by both '" + it->second + "' and '" + sensor.name + " (Channel B)'.");
                        }
                        else
                        {
                            dioMap[sensor.secondaryDioChannel] = sensor.name + " (Channel B)";
                        }
                    }
                }
            }

            // Limit switch & REV Through Bore hardware-attached controller validation
            if (sensor.type == "limit_switch" && sensor.attachToControllerCanId > 0)
            {
                if (activeControllerCanIds.find(sensor.attachToControllerCanId) == activeControllerCanIds.end())
                {
                    linterWarnings.push_back("Limit Switch '" + sensor.name +
                        "' is attached to motor controller CAN ID " + std::to_string(sensor.attachToControllerCanId) +
                        ", which is not assigned to any swerve module or mechanism.");
                }
            }
            else if (sensor.type == "rev_through_bore" && sensor.attachToControllerCanId > 0)
            {
                if (activeControllerCanIds.find(sensor.attachToControllerCanId) == activeControllerCanIds.end())
                {
                    linterWarnings.push_back("REV Through Bore Encoder '" + sensor.name +
                        "' is attached to SPARK data port CAN ID " + std::to_string(sensor.attachToControllerCanId) +
                        ", which is not assigned to any swerve module or mechanism.");
                }
            }

            // LIDAR Range Validation
            if ((sensor.type == "andymark_lidar3" || sensor.type == "laser_can") && sensor.triggerMax > 0.0)
            {
                if (sensor.triggerMin >= sensor.triggerMax)
                {
                    linterErrors.push_back("LIDAR Range Error: Sensor '" + sensor.name + "' Min Range (" +
                        std::to_string(sensor.triggerMin) + " m) must be strictly less than Max Range (" +
                        std::to_string(sensor.triggerMax) + " m).");
                }
            }

            // Analog Channel Validation
            if (sensor.type == "analog_input" || sensor.bus == "analog")
            {
                if (sensor.analogChannel > 7)
                {
                    linterErrors.push_back("Analog Channel Out of Range: Sensor '" + sensor.name +
                        "' specifies analog channel " + std::to_string(sensor.analogChannel) + " (valid [0..7]).");
                }
                else
                {
                    auto it = analogMap.find(sensor.analogChannel);
                    if (it != analogMap.end())
                    {
                        linterErrors.push_back("Analog Channel Collision: Channel " + std::to_string(sensor.analogChannel) +
                            " is claimed by both '" + it->second + "' and '" + sensor.name + "'.");
                    }
                    else
                    {
                        analogMap[sensor.analogChannel] = sensor.name;
                    }
                }
            }

            // Rotary encoder joint type checks
            if ((sensor.type == "cancoder" || sensor.type == "rev_through_bore") && !sensor.targetJoint.empty())
            {
                const auto* j = manifest.FindJoint(sensor.targetJoint);
                if (j && j->type == "prismatic")
                {
                    std::string label = (sensor.type == "cancoder") ? "CANcoder '" : "REV Through Bore Encoder '";
                    linterWarnings.push_back(label + sensor.name +
                        "' is bound to prismatic joint '" + sensor.targetJoint + "'. Rotary encoders measure angular revolute motion.");
                }
            }
        }

        // 9. Bumper Configuration Validation
        if (sidecar.bumpers.enabled)
        {
            if (!sidecar.bumpers.targetLink.empty() && !manifest.FindLink(sidecar.bumpers.targetLink))
            {
                linterErrors.push_back("Bumper Target Link Error: targetLink '" +
                    sidecar.bumpers.targetLink + "' does not exist in CAD manifest.");
            }

            if (sidecar.bumpers.lengthM <= 0.0 || sidecar.bumpers.widthM <= 0.0 || sidecar.bumpers.heightM <= 0.0)
            {
                linterErrors.push_back("Bumper Dimensions Error: Length, width, and height must be strictly positive.");
            }

            if (sidecar.bumpers.shape == "perimeter" || sidecar.bumpers.shape == "u_shape")
            {
                if (sidecar.bumpers.thicknessM <= 0.0 ||
                    sidecar.bumpers.thicknessM * 2.0 >= sidecar.bumpers.lengthM ||
                    sidecar.bumpers.thicknessM * 2.0 >= sidecar.bumpers.widthM)
                {
                    linterErrors.push_back("Bumper Thickness Error: Wall thickness must be positive and less than half of outer bumper dimensions.");
                }
            }

            if (sidecar.bumpers.shape == "u_shape" && sidecar.bumpers.cutoutWidthM <= 0.0)
            {
                linterWarnings.push_back("Bumper Cutout Warning: Front cutout width is zero or negative.");
            }

            if (sidecar.bumpers.massKg > 6.804) // > 15 lbs
            {
                linterWarnings.push_back("Bumper Rule Warning: Bumper mass exceeds standard FRC 15 lb limit (" +
                    std::to_string(static_cast<int>(sidecar.bumpers.massKg * 2.20462)) + " lbs).");
            }
            if (sidecar.bumpers.heightM < 0.10)
            {
                linterWarnings.push_back("Bumper Rule Warning: Bumper height (" +
                    std::to_string(static_cast<int>(sidecar.bumpers.heightM * 39.3701)) + " in) is less than standard FRC 5-inch requirement.");
            }
        }
    }

    bool ConfiguratorState::ArePathsEquivalent(const std::string& a, const std::string& b)
    {
        if (a == b)
        {
            return true;
        }

        try
        {
            if (std::filesystem::exists(a) && std::filesystem::exists(b))
            {
                return std::filesystem::equivalent(a, b);
            }
        }
        catch (...) {}

        // Fallback: normalize slashes and compare case-insensitively on Windows
        std::string na = a;
        std::string nb = b;
        for (char& c : na) if (c == '\\') c = '/';
        for (char& c : nb) if (c == '\\') c = '/';
        while (!na.empty() && na.back() == '/') na.pop_back();
        while (!nb.empty() && nb.back() == '/') nb.pop_back();

#ifdef _WIN32
        return _stricmp(na.c_str(), nb.c_str()) == 0;
#else
        return na == nb;
#endif
    }

    std::string ConfiguratorState::ResolveSettingsPath(const std::string& customPath)
    {
        if (!customPath.empty())
        {
            return customPath;
        }

        if (std::filesystem::exists("FRCBananaSim/Resources/Config"))
        {
            return "FRCBananaSim/Resources/Config/configurator_settings.json";
        }
        else if (std::filesystem::exists("../FRCBananaSim/Resources/Config"))
        {
            return "../FRCBananaSim/Resources/Config/configurator_settings.json";
        }
        return "configurator_settings.json";
    }

    std::vector<std::string> ConfiguratorState::GetRecentRobots(const std::string& settingsFilePath)
    {
        std::vector<std::string> recents;
        std::string configPath = ResolveSettingsPath(settingsFilePath);

        if (!std::filesystem::exists(configPath))
        {
            return recents;
        }

        try
        {
            std::ifstream f(configPath);
            if (f.is_open())
            {
                nlohmann::json j;
                f >> j;

                if (j.contains("recentRobots") && j["recentRobots"].is_array())
                {
                    for (const auto& item : j["recentRobots"])
                    {
                        if (item.is_string())
                        {
                            std::string path = item.get<std::string>();
                            if (!path.empty())
                            {
                                bool duplicate = false;
                                for (const auto& existing : recents)
                                {
                                    if (ArePathsEquivalent(existing, path))
                                    {
                                        duplicate = true;
                                        break;
                                    }
                                }
                                if (!duplicate)
                                {
                                    recents.push_back(path);
                                }
                                if (recents.size() >= 3)
                                {
                                    break;
                                }
                            }
                        }
                    }
                }

                // If recentRobots wasn't populated or was empty, check lastRobotPath as fallback
                if (recents.empty() && j.contains("lastRobotPath") && j["lastRobotPath"].is_string())
                {
                    std::string lastPath = j["lastRobotPath"].get<std::string>();
                    if (!lastPath.empty())
                    {
                        recents.push_back(lastPath);
                    }
                }
            }
        }
        catch (...) {}

        return recents;
    }

    void ConfiguratorState::AddRecentRobot(const std::string& folderPath, const std::string& settingsFilePath)
    {
        if (folderPath.empty())
        {
            return;
        }

        std::string configPath = ResolveSettingsPath(settingsFilePath);

        // Load existing recents
        std::vector<std::string> current = GetRecentRobots(configPath);

        // Build updated list with folderPath at index 0 (max 3 items)
        std::vector<std::string> updated;
        updated.push_back(folderPath);

        for (const auto& p : current)
        {
            if (!ArePathsEquivalent(p, folderPath))
            {
                updated.push_back(p);
            }
            if (updated.size() >= 3)
            {
                break;
            }
        }

        // Save back to JSON
        try
        {
            nlohmann::json j;
            if (std::filesystem::exists(configPath))
            {
                std::ifstream f(configPath);
                if (f.is_open())
                {
                    f >> j;
                }
            }

            j["recentRobots"] = updated;
            j["lastRobotPath"] = folderPath;

            std::ofstream f(configPath);
            if (f.is_open())
            {
                f << j.dump(2);
            }
        }
        catch (...) {}
    }
}
