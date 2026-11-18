/**
 * @file RobotSidecarSerializer.cpp
 * @brief Implementation of RobotSidecarSerializer C++ JSON serialization for Schema v1.0.0.
 */

#include "Robot/Schema/RobotSidecarSerializer.h"
#include <fstream>
#include <cmath>

namespace FRCBananaSim::Robot::Schema
{
    nlohmann::json RobotSidecarSerializer::SerializeToJson(const RobotSidecar& sidecar)
    {
        nlohmann::json root;

        root["schemaVersion"] = sidecar.schemaVersion.empty() ? "1.0.0" : sidecar.schemaVersion;
        root["targetManifest"] = sidecar.targetManifest.empty() ? "manifest.json" : sidecar.targetManifest;

        // 1. Electrical Specification
        {
            nlohmann::json el;
            el["openCircuitVoltage_V"] = sidecar.electrical.openCircuitVoltageV;
            el["internalResistance_Ohm"] = sidecar.electrical.internalResistanceOhm;
            el["quiescentCurrent_A"] = sidecar.electrical.quiescentCurrentA;
            el["brownoutWarning_V"] = sidecar.electrical.brownoutWarningV;
            el["brownoutShutoff_V"] = sidecar.electrical.brownoutShutoffV;
            el["brownoutRecovery_V"] = sidecar.electrical.brownoutRecoveryV;
            el["servoRailVoltage_V"] = sidecar.electrical.regulatedRailVoltageV;
            el["servoRailCurrentLimit_A"] = sidecar.electrical.regulatedRailCurrentLimitA;
            el["servoRailBuckEfficiency"] = sidecar.electrical.regulatedRailEfficiency;
            root["electrical"] = el;
        }

        // 2. Swerve Drivetrain Specification
        {
            nlohmann::json sw;
            sw["chassisLink"] = sidecar.swerve.chassisLink;
            sw["driveMotorPreset"] = sidecar.swerve.driveMotorPreset;
            sw["steerMotorPreset"] = sidecar.swerve.steerMotorPreset;
            sw["driveGearRatio"] = sidecar.swerve.driveGearRatio;
            sw["steerGearRatio"] = sidecar.swerve.steerGearRatio;
            sw["driveEfficiency"] = sidecar.swerve.driveEfficiency;
            sw["steerEfficiency"] = sidecar.swerve.steerEfficiency;
            sw["wheelRadius_m"] = sidecar.swerve.wheelRadiusM;
            sw["driveStatorLimit_A"] = sidecar.swerve.driveStatorLimitA;
            sw["driveSupplyLimit_A"] = sidecar.swerve.driveSupplyLimitA;
            sw["steerStatorLimit_A"] = sidecar.swerve.steerStatorLimitA;
            sw["steerSupplyLimit_A"] = sidecar.swerve.steerSupplyLimitA;

            if (sidecar.swerve.driveActuatorType != "motor")
            {
                sw["driveActuatorType"] = sidecar.swerve.driveActuatorType;
            }
            if (sidecar.swerve.steerActuatorType != "motor")
            {
                sw["steerActuatorType"] = sidecar.swerve.steerActuatorType;
            }
            if (!sidecar.swerve.driveServoPreset.empty() && sidecar.swerve.driveActuatorType == "servo")
            {
                sw["driveServoPreset"] = sidecar.swerve.driveServoPreset;
            }
            if (!sidecar.swerve.steerServoPreset.empty() && sidecar.swerve.steerActuatorType == "servo")
            {
                sw["steerServoPreset"] = sidecar.swerve.steerServoPreset;
            }

            if (sidecar.swerve.driveMotorPreset == "Custom")
            {
                nlohmann::json cm;
                cm["nominalVoltage_V"] = sidecar.swerve.customDriveMotor.nominalVoltageV;
                cm["stallTorque_Nm"] = sidecar.swerve.customDriveMotor.stallTorqueNm;
                cm["freeSpeed_rpm"] = sidecar.swerve.customDriveMotor.freeSpeedRpm;
                cm["stallCurrent_A"] = sidecar.swerve.customDriveMotor.stallCurrentA;
                cm["freeCurrent_A"] = sidecar.swerve.customDriveMotor.freeCurrentA;
                sw["customDriveMotor"] = cm;
            }
            if (sidecar.swerve.steerMotorPreset == "Custom")
            {
                nlohmann::json cm;
                cm["nominalVoltage_V"] = sidecar.swerve.customSteerMotor.nominalVoltageV;
                cm["stallTorque_Nm"] = sidecar.swerve.customSteerMotor.stallTorqueNm;
                cm["freeSpeed_rpm"] = sidecar.swerve.customSteerMotor.freeSpeedRpm;
                cm["stallCurrent_A"] = sidecar.swerve.customSteerMotor.stallCurrentA;
                cm["freeCurrent_A"] = sidecar.swerve.customSteerMotor.freeCurrentA;
                sw["customSteerMotor"] = cm;
            }
            if (sidecar.swerve.driveServoPreset == "Custom")
            {
                nlohmann::json cs;
                cs["nominalVoltage_V"] = sidecar.swerve.customDriveServo.nominalVoltageV;
                cs["stallTorque_kgcm"] = sidecar.swerve.customDriveServo.stallTorqueKgCm;
                cs["speed_secPer60Deg"] = sidecar.swerve.customDriveServo.speedSecPer60Deg;
                cs["stallCurrent_A"] = sidecar.swerve.customDriveServo.stallCurrentA;
                cs["freeCurrent_A"] = sidecar.swerve.customDriveServo.freeCurrentA;
                cs["travelRange_deg"] = sidecar.swerve.customDriveServo.travelRangeDeg;
                sw["customDriveServo"] = cs;
            }
            if (sidecar.swerve.steerServoPreset == "Custom")
            {
                nlohmann::json cs;
                cs["nominalVoltage_V"] = sidecar.swerve.customSteerServo.nominalVoltageV;
                cs["stallTorque_kgcm"] = sidecar.swerve.customSteerServo.stallTorqueKgCm;
                cs["speed_secPer60Deg"] = sidecar.swerve.customSteerServo.speedSecPer60Deg;
                cs["stallCurrent_A"] = sidecar.swerve.customSteerServo.stallCurrentA;
                cs["freeCurrent_A"] = sidecar.swerve.customSteerServo.freeCurrentA;
                cs["travelRange_deg"] = sidecar.swerve.customSteerServo.travelRangeDeg;
                sw["customSteerServo"] = cs;
            }

            nlohmann::json modArray = nlohmann::json::array();
            for (const auto& mod : sidecar.swerve.modules)
            {
                nlohmann::json m;
                m["name"] = mod.name;
                m["pos_m"] = { mod.posM.x, mod.posM.y, mod.posM.z };
                m["driveCanId"] = mod.driveCanId;
                m["steerCanId"] = mod.steerCanId;
                m["cancoderId"] = mod.cancoderId;
                m["canBus"] = mod.canBus.empty() ? "rio" : mod.canBus;

                if (sidecar.swerve.driveActuatorType == "servo" || mod.drivePwmChannel > 0)
                {
                    m["drivePwmChannel"] = mod.drivePwmChannel;
                }
                if (sidecar.swerve.steerActuatorType == "servo" || mod.steerPwmChannel > 0)
                {
                    m["steerPwmChannel"] = mod.steerPwmChannel;
                }

                modArray.push_back(m);
            }
            sw["modules"] = modArray;
            root["swerve"] = sw;
        }

        // 3. Mechanisms
        {
            nlohmann::json mechArray = nlohmann::json::array();
            for (const auto& mech : sidecar.mechanisms)
            {
                nlohmann::json m;
                m["name"] = mech.name;
                m["role"] = mech.role;

                if (mech.role == "differential")
                {
                    m["targetJoints"] = mech.targetJoints;
                    m["differentialType"] = mech.differentialType;
                    m["gearRatios"] = mech.gearRatios;

                    if (!mech.couplingMatrix.empty())
                    {
                        m["couplingMatrix"] = mech.couplingMatrix;
                    }
                    if (!mech.coulombFrictions.empty())
                    {
                        m["coulombFriction"] = mech.coulombFrictions;
                    }
                    if (!mech.viscousDampings.empty())
                    {
                        m["viscousDamping"] = mech.viscousDampings;
                    }

                    if (!mech.motorPresets.empty())
                    {
                        m["motorPresets"] = mech.motorPresets;
                    }
                    if (!mech.actuatorTypes.empty())
                    {
                        m["actuatorTypes"] = mech.actuatorTypes;
                    }
                    if (!mech.statorCurrentLimits.empty())
                    {
                        m["statorCurrentLimits"] = mech.statorCurrentLimits;
                    }
                    if (!mech.supplyCurrentLimits.empty())
                    {
                        m["supplyCurrentLimits"] = mech.supplyCurrentLimits;
                    }
                    if (mech.customMotors.size() >= 2)
                    {
                        nlohmann::json cmArray = nlohmann::json::array();
                        for (const auto& cm : mech.customMotors)
                        {
                            nlohmann::json cmNode;
                            cmNode["nominalVoltage_V"] = cm.nominalVoltageV;
                            cmNode["stallTorque_Nm"] = cm.stallTorqueNm;
                            cmNode["freeSpeed_rpm"] = cm.freeSpeedRpm;
                            cmNode["stallCurrent_A"] = cm.stallCurrentA;
                            cmNode["freeCurrent_A"] = cm.freeCurrentA;
                            cmArray.push_back(cmNode);
                        }
                        m["customMotors"] = cmArray;
                    }

                    if (mech.actuatorType == "servo" || (!mech.actuatorTypes.empty() && (mech.actuatorTypes[0] == "servo" || (mech.actuatorTypes.size() > 1 && mech.actuatorTypes[1] == "servo"))))
                    {
                        m["actuatorType"] = "servo";
                        nlohmann::json sArray = nlohmann::json::array();
                        for (const auto& sc : mech.servos)
                        {
                            nlohmann::json s;
                            s["preset"] = sc.preset;
                            s["mode"] = sc.mode;
                            s["pwmChannel"] = sc.pwmChannel;
                            s["supplyVoltage_V"] = sc.supplyVoltageV;
                            s["travelRange_deg"] = sc.travelRangeDeg;
                            sArray.push_back(s);
                        }
                        m["servos"] = sArray;
                    }
                }
                else
                {
                    if (!mech.targetJoint.empty())
                    {
                        m["targetJoint"] = mech.targetJoint;
                    }
                    if (!mech.parentLink.empty())
                    {
                        m["parentLink"] = mech.parentLink;
                    }

                    m["gearRatio"] = mech.gearRatio;
                    m["efficiency"] = mech.efficiency;
                    m["coulombFriction"] = mech.coulombFriction;
                    m["viscousDamping"] = mech.viscousDamping;

                    if (mech.role == "elevator")
                    {
                        m["drumRadius_m"] = mech.drumRadiusM;
                        if (mech.springAssist.forceN != 0.0 || !mech.springAssist.type.empty())
                        {
                            nlohmann::json sa;
                            sa["type"] = mech.springAssist.type.empty() ? "constant_force" : mech.springAssist.type;
                            sa["force_N"] = mech.springAssist.forceN;
                            m["springAssist"] = sa;
                        }
                    }
                    else if (mech.role == "flywheel" || mech.role == "intake")
                    {
                        m["flywheelInertia_kgm2"] = mech.flywheelInertiaKgm2;
                        m["rollerRadius_m"] = mech.rollerRadiusM;
                        m["spinAxis"] = { mech.spinAxis.x, mech.spinAxis.y, mech.spinAxis.z };
                        if (!mech.visualLink.empty())
                        {
                            m["visualLink"] = mech.visualLink;
                        }
                        if (std::abs(mech.mountOffsetM.x) > 1e-6 ||
                            std::abs(mech.mountOffsetM.y) > 1e-6 ||
                            std::abs(mech.mountOffsetM.z) > 1e-6)
                        {
                            m["mountOffset_m"] = { mech.mountOffsetM.x, mech.mountOffsetM.y, mech.mountOffsetM.z };
                        }
                    }

                    if (mech.actuatorType == "servo")
                    {
                        m["actuatorType"] = "servo";
                        nlohmann::json s;
                        s["preset"] = mech.servo.preset;
                        s["mode"] = mech.servo.mode;
                        s["pwmChannel"] = mech.servo.pwmChannel;
                        s["supplyVoltage_V"] = mech.servo.supplyVoltageV;
                        s["travelRange_deg"] = mech.servo.travelRangeDeg;
                        if (mech.servo.preset == "Custom")
                        {
                            nlohmann::json cs;
                            cs["nominalVoltage_V"] = mech.customServo.nominalVoltageV;
                            cs["stallTorque_kgcm"] = mech.customServo.stallTorqueKgCm;
                            cs["speed_secPer60Deg"] = mech.customServo.speedSecPer60Deg;
                            cs["stallCurrent_A"] = mech.customServo.stallCurrentA;
                            cs["freeCurrent_A"] = mech.customServo.freeCurrentA;
                            cs["travelRange_deg"] = mech.customServo.travelRangeDeg;
                            s["customServo"] = cs;
                        }
                        m["servo"] = s;
                    }
                }

                if (mech.actuatorType != "servo")
                {
                    m["motorPreset"] = mech.motorPreset;
                    if (mech.motorPreset == "Custom")
                    {
                        nlohmann::json cm;
                        cm["nominalVoltage_V"] = mech.customMotor.nominalVoltageV;
                        cm["stallTorque_Nm"] = mech.customMotor.stallTorqueNm;
                        cm["freeSpeed_rpm"] = mech.customMotor.freeSpeedRpm;
                        cm["stallCurrent_A"] = mech.customMotor.stallCurrentA;
                        cm["freeCurrent_A"] = mech.customMotor.freeCurrentA;
                        m["customMotor"] = cm;
                    }
                    m["statorCurrentLimit_A"] = mech.statorCurrentLimitA;
                    m["supplyCurrentLimit_A"] = mech.supplyCurrentLimitA;

                    if (!mech.controllers.empty())
                    {
                        nlohmann::json cArray = nlohmann::json::array();
                        for (const auto& ctrl : mech.controllers)
                        {
                            nlohmann::json c;
                            c["vendor"] = ctrl.vendor;
                            c["canId"] = ctrl.canId;
                            c["canBus"] = ctrl.canBus.empty() ? "rio" : ctrl.canBus;
                            c["inverted"] = ctrl.inverted;
                            c["role"] = ctrl.role.empty() ? "leader" : ctrl.role;
                            cArray.push_back(c);
                        }
                        m["controllers"] = cArray;
                    }
                }

                mechArray.push_back(m);
            }
            root["mechanisms"] = mechArray;
        }

        // 4. Couplings
        {
            nlohmann::json coupArray = nlohmann::json::array();
            for (const auto& coup : sidecar.couplings)
            {
                nlohmann::json c;
                c["name"] = coup.name;
                c["type"] = coup.type;
                c["solverMode"] = coup.solverMode.empty() ? "analytic_coupled" : coup.solverMode;

                if (coup.type == "power_coupling")
                {
                    nlohmann::json pc;
                    pc["transmissionType"] = coup.powerCoupling.transmissionType.empty() ? "gear" : coup.powerCoupling.transmissionType;
                    pc["masterJoint"] = coup.powerCoupling.masterJoint;
                    pc["slaveJoint"] = coup.powerCoupling.slaveJoint;
                    pc["ratio"] = coup.powerCoupling.ratio;
                    pc["inverted"] = coup.powerCoupling.inverted;
                    pc["efficiency"] = coup.powerCoupling.efficiency;
                    pc["backlashRad"] = coup.powerCoupling.backlashRad;
                    c["powerCoupling"] = pc;
                }
                else if (coup.type == "elevator_cascade")
                {
                    nlohmann::json csArray = nlohmann::json::array();
                    for (const auto& stage : coup.cascadeStages)
                    {
                        nlohmann::json cs;
                        cs["joint"] = stage.joint;
                        cs["multiplier"] = stage.multiplier;
                        csArray.push_back(cs);
                    }
                    c["cascadeStages"] = csArray;
                }
                else if (coup.type == "four_bar_loop")
                {
                    c["crankJoint"] = coup.fourBar.crankJoint;
                    c["couplerJoint"] = coup.fourBar.couplerJoint;
                    c["followerJoint"] = coup.fourBar.followerJoint;
                    c["closureParentLink"] = coup.fourBar.closureParentLink;
                    c["closureChildLink"] = coup.fourBar.closureChildLink;
                    c["closureParentAnchor_m"] = {
                        coup.fourBar.closureParentAnchorM.x,
                        coup.fourBar.closureParentAnchorM.y,
                        coup.fourBar.closureParentAnchorM.z
                    };
                    c["closureChildAnchor_m"] = {
                        coup.fourBar.closureChildAnchorM.x,
                        coup.fourBar.closureChildAnchorM.y,
                        coup.fourBar.closureChildAnchorM.z
                    };
                    c["closureAxis"] = {
                        coup.fourBar.closureAxis.x,
                        coup.fourBar.closureAxis.y,
                        coup.fourBar.closureAxis.z
                    };
                }

                coupArray.push_back(c);
            }
            root["couplings"] = coupArray;
        }

        // 5. Sensors
        {
            nlohmann::json sensorArray = nlohmann::json::array();
            for (const auto& sensor : sidecar.sensors)
            {
                nlohmann::json s;
                s["name"] = sensor.name;
                s["type"] = sensor.type;
                s["bus"] = sensor.bus.empty() ? "rio" : sensor.bus;

                if (sensor.canId > 0 || (sensor.bus != "analog" && sensor.bus != "rio" && sensor.type.find("imu") != std::string::npos) ||
                    sensor.type == "andymark_lidar3" || sensor.type == "laser_can")
                {
                    s["canId"] = sensor.canId;
                }

                bool isRevI2C = (sensor.type == "rev_through_bore" && sensor.bus.find("i2c") != std::string::npos);

                if (sensor.dioChannel > 0 || sensor.type == "limit_switch" || sensor.type == "beam_break" ||
                    (sensor.type == "rev_through_bore" && !isRevI2C && sensor.attachToControllerCanId == 0))
                {
                    s["dioChannel"] = sensor.dioChannel;
                }

                if (sensor.secondaryDioChannel > 0 ||
                    (sensor.type == "rev_through_bore" && !isRevI2C && sensor.attachToControllerCanId == 0))
                {
                    s["secondaryDioChannel"] = sensor.secondaryDioChannel;
                }

                if (sensor.i2cAddress > 0 || sensor.bus.find("i2c") != std::string::npos || sensor.type == "color_sensor")
                {
                    s["i2cAddress"] = sensor.i2cAddress;
                }

                if (sensor.analogChannel > 0 || sensor.bus == "analog" || sensor.type == "analog_input")
                {
                    s["analogChannel"] = sensor.analogChannel;
                }

                if (!sensor.parentLink.empty())
                {
                    s["parentLink"] = sensor.parentLink;
                }

                if (!sensor.targetJoint.empty())
                {
                    s["targetJoint"] = sensor.targetJoint;
                }

                if (!sensor.contactZone.empty())
                {
                    s["contactZone"] = sensor.contactZone;
                }

                if (sensor.attachToControllerCanId > 0)
                {
                    s["attachToControllerCanId"] = sensor.attachToControllerCanId;
                }

                if (!sensor.limitDirection.empty())
                {
                    s["limitDirection"] = sensor.limitDirection;
                }

                if (sensor.type == "limit_switch" || sensor.type == "beam_break")
                {
                    s["normallyClosed"] = sensor.normallyClosed;
                }

                if (sensor.type == "limit_switch" || sensor.type == "analog_input" ||
                    sensor.type == "andymark_lidar3" || sensor.type == "laser_can")
                {
                    s["triggerMin"] = sensor.triggerMin;
                    s["triggerMax"] = sensor.triggerMax;
                }

                if (sensor.debounceTimeS > 0.0)
                {
                    s["debounceTime_s"] = sensor.debounceTimeS;
                }

                if (sensor.inverted)
                {
                    s["inverted"] = sensor.inverted;
                }

                if (std::abs(sensor.mountOffsetM.x) > 1e-6 ||
                    std::abs(sensor.mountOffsetM.y) > 1e-6 ||
                    std::abs(sensor.mountOffsetM.z) > 1e-6)
                {
                    s["mountOffset_m"] = { sensor.mountOffsetM.x, sensor.mountOffsetM.y, sensor.mountOffsetM.z };
                }

                if (std::abs(sensor.mountRotationQuat.w - 1.0) > 1e-6 ||
                    std::abs(sensor.mountRotationQuat.x) > 1e-6 ||
                    std::abs(sensor.mountRotationQuat.y) > 1e-6 ||
                    std::abs(sensor.mountRotationQuat.z) > 1e-6)
                {
                    s["mountRotation_quat"] = {
                        sensor.mountRotationQuat.w,
                        sensor.mountRotationQuat.x,
                        sensor.mountRotationQuat.y,
                        sensor.mountRotationQuat.z
                    };
                }

                if (sensor.type.find("imu") != std::string::npos)
                {
                    if (sensor.gyroNoiseStdDevRadS > 0.0)
                    {
                        s["gyroNoiseStdDev_rad_s"] = sensor.gyroNoiseStdDevRadS;
                    }
                    if (sensor.gyroBiasRandomWalkRadS2 > 0.0)
                    {
                        s["gyroBiasRandomWalk_rad_s2"] = sensor.gyroBiasRandomWalkRadS2;
                    }
                    if (sensor.accelNoiseStdDevMS2 > 0.0)
                    {
                        s["accelNoiseStdDev_m_s2"] = sensor.accelNoiseStdDevMS2;
                    }
                }

                if (sensor.type == "cancoder" || sensor.type == "rev_through_bore")
                {
                    s["sensorToMechanismRatio"] = sensor.sensorToMechanismRatio;
                    s["cpr"] = sensor.cpr;
                    s["offset_rad"] = sensor.offsetRad;
                }

                if (!sensor.customNumericParams.empty())
                {
                    s["customNumericParams"] = sensor.customNumericParams;
                }

                if (!sensor.customStringParams.empty())
                {
                    s["customStringParams"] = sensor.customStringParams;
                }

                sensorArray.push_back(s);
            }
            root["sensors"] = sensorArray;
        }

        // 6. Mass Reconciliation Specification
        if (sidecar.massReconciliation.enabled)
        {
            nlohmann::json mr;
            mr["linkMassMultiplier"] = sidecar.massReconciliation.linkMassMultiplier;

            nlohmann::json ub;
            ub["targetLink"] = sidecar.massReconciliation.unmodeledBallast.targetLink.empty() ?
                "base_link" : sidecar.massReconciliation.unmodeledBallast.targetLink;
            ub["mass_kg"] = sidecar.massReconciliation.unmodeledBallast.massKg;
            ub["offset_m"] = {
                sidecar.massReconciliation.unmodeledBallast.offsetM.x,
                sidecar.massReconciliation.unmodeledBallast.offsetM.y,
                sidecar.massReconciliation.unmodeledBallast.offsetM.z
            };
            mr["unmodeledBallast"] = ub;

            if (!sidecar.massReconciliation.linkMassOverrides.empty())
            {
                nlohmann::json overridesObj = nlohmann::json::object();
                for (const auto& [linkName, mass] : sidecar.massReconciliation.linkMassOverrides)
                {
                    if (mass > 0.0)
                    {
                        overridesObj[linkName] = mass;
                    }
                }
                if (!overridesObj.empty())
                {
                    mr["linkMassOverrides"] = overridesObj;
                }
            }

            root["massReconciliation"] = mr;
        }

        // 7. Joint Overrides (Hardstop Limits & Compliance Springs)
        if (!sidecar.jointOverrides.empty())
        {
            nlohmann::json joJson = nlohmann::json::object();
            for (const auto& [jointName, ov] : sidecar.jointOverrides)
            {
                nlohmann::json item;
                item["hasLimits"] = ov.hasLimits;
                item["lower"] = ov.lower;
                item["upper"] = ov.upper;
                item["unit"] = ov.unit;
                item["springFrequency_Hz"] = ov.springFrequencyHz;
                item["springDampingRatio"] = ov.springDampingRatio;
                joJson[jointName] = item;
            }
            root["jointOverrides"] = joJson;
        }

        // 8. Bumpers (Parametric Collision Geometry & Mass)
        if (sidecar.bumpers.enabled)
        {
            nlohmann::json bJson;
            bJson["enabled"] = sidecar.bumpers.enabled;
            if (!sidecar.bumpers.targetLink.empty())
            {
                bJson["targetLink"] = sidecar.bumpers.targetLink;
            }
            bJson["shape"] = sidecar.bumpers.shape;
            bJson["length_m"] = sidecar.bumpers.lengthM;
            bJson["width_m"] = sidecar.bumpers.widthM;
            bJson["height_m"] = sidecar.bumpers.heightM;
            bJson["thickness_m"] = sidecar.bumpers.thicknessM;
            bJson["offset_m"] = {
                sidecar.bumpers.offsetM.x,
                sidecar.bumpers.offsetM.y,
                sidecar.bumpers.offsetM.z
            };
            bJson["mass_kg"] = sidecar.bumpers.massKg;
            bJson["material"] = sidecar.bumpers.material;
            bJson["color"] = sidecar.bumpers.color;
            if (sidecar.bumpers.shape == "u_shape")
            {
                bJson["cutoutWidth_m"] = sidecar.bumpers.cutoutWidthM;
            }
            root["bumpers"] = bJson;
        }

        // 9. Contact Zones (Intake, Shooter, Indexer Interaction Volumes)
        if (!sidecar.contactZones.empty())
        {
            nlohmann::json czArray = nlohmann::json::array();
            for (const auto& cz : sidecar.contactZones)
            {
                nlohmann::json czNode;
                czNode["name"] = cz.name;
                czNode["role"] = cz.role;
                czNode["parentLink"] = cz.parentLink;
                czNode["shape"] = cz.shape;
                czNode["origin_m"] = { cz.originM.x, cz.originM.y, cz.originM.z };
                czNode["rotation_quat"] = { cz.rotationQuat.w, cz.rotationQuat.x, cz.rotationQuat.y, cz.rotationQuat.z };
                czNode["dimensions_m"] = { cz.dimensionsM.x, cz.dimensionsM.y, cz.dimensionsM.z };
                czNode["tangentAxis"] = { cz.tangentAxis.x, cz.tangentAxis.y, cz.tangentAxis.z };
                czNode["compressionAxis"] = { cz.compressionAxis.x, cz.compressionAxis.y, cz.compressionAxis.z };
                czNode["nominalGap_m"] = cz.nominalGapM;
                czArray.push_back(czNode);
            }
            root["contactZones"] = czArray;
        }

        return root;
    }

    std::string RobotSidecarSerializer::SerializeToString(const RobotSidecar& sidecar, int indent)
    {
        nlohmann::json j = SerializeToJson(sidecar);
        return j.dump(indent);
    }

    bool RobotSidecarSerializer::SerializeToFile(const RobotSidecar& sidecar, const std::string& filepath, int indent)
    {
        std::ofstream outFile(filepath);
        if (!outFile.is_open())
        {
            return false;
        }

        outFile << SerializeToString(sidecar, indent) << "\n";
        return outFile.good();
    }
}
