/**
 * @file SwerveParams.cpp
 * @brief Implementation of swerve drivetrain parameter factories, JSON loaders, and frame converters.
 */

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    glm::mat3 ChassisParams::GetJoltInertiaTensor() const
    {
        // Coordinate frame conversion from NWU to Jolt (Engine):
        // NWU:  +X Forward, +Y Left,  +Z Up     -> Rotations: Roll around X, Pitch around Y, Yaw around Z
        // Jolt: +X Right,   +Y Up,    +Z Forward -> Rotations: Pitch around X, Yaw around Y, Roll around Z
        // Mapping principal axes:
        // Jolt X (Pitch) = NWU Y
        // Jolt Y (Yaw)   = NWU Z
        // Jolt Z (Roll)  = NWU X

        glm::mat3 joltTensor(0.0);
        joltTensor[0][0] = inertiaTensor[1][1]; // Pitch
        joltTensor[1][1] = inertiaTensor[2][2]; // Yaw
        joltTensor[2][2] = inertiaTensor[0][0]; // Roll

        // Off-diagonals (if any exist in arbitrary asymmetrical mass distributions)
        joltTensor[0][1] = inertiaTensor[1][2];
        joltTensor[1][0] = inertiaTensor[2][1];
        joltTensor[0][2] = inertiaTensor[1][0];
        joltTensor[2][0] = inertiaTensor[0][1];
        joltTensor[1][2] = inertiaTensor[2][0];
        joltTensor[2][1] = inertiaTensor[0][2];

        return joltTensor;
    }

    SwerveDrivetrainConfig SwerveDrivetrainConfig::CreateMK5nR3Reference()
    {
        SwerveDrivetrainConfig config;

        // 1. Chassis Physical Properties
        config.chassis.mass = 50.0;
        config.chassis.dimensions = glm::vec3(0.70, 0.70, 0.15); // 70cm x 70cm x 15cm
        config.chassis.centerOfMassOffset = glm::vec3(0.0, 0.0, 0.0);
        config.chassis.centerOfMassHeight = 0.20;

        // Inertia tensor for a rectangular slab: Ix = M/12*(y^2 + z^2), Iy = M/12*(x^2 + z^2), Iz = M/12*(x^2 + y^2)
        // In NWU: X=Forward, Y=Left, Z=Up
        double m = config.chassis.mass;
        double dx = config.chassis.dimensions.x;
        double dy = config.chassis.dimensions.y;
        double dz = config.chassis.dimensions.z;

        double ixx = (m / 12.0) * (dy * dy + dz * dz); // Roll axis (around Forward +X)
        double iyy = (m / 12.0) * (dx * dx + dz * dz); // Pitch axis (around Left +Y)
        double izz = (m / 12.0) * (dx * dx + dy * dy); // Yaw axis (around Up +Z)

        config.chassis.inertiaTensor = glm::mat3(0.0);
        config.chassis.inertiaTensor[0][0] = ixx;
        config.chassis.inertiaTensor[1][1] = iyy;
        config.chassis.inertiaTensor[2][2] = izz;

        // 2. Tire & Contact Properties
        config.tire.wheelRadius = 0.0508; // 4 inch diameter (2 inch radius)
        config.tire.wheelWidth = 0.0381;  // 1.5 inch
        config.tire.wheelMass = 0.65;
        config.tire.wheelAxleInertia = 0.00084;
        config.tire.radialStiffness = 1.2e5;       // 120 kN/m
        config.tire.radialDamping = 1200.0;        // 1200 N*s/m
        config.tire.preSlideDisplacement = 0.001;  // 1 mm
        config.tire.muStatic = 1.20;
        config.tire.muKinetic = 1.05;
        config.tire.stribeckVelocity = 0.15;
        config.tire.loadSensitivityBeta = 0.08;
        config.tire.rollingResistance = 0.015;
        config.tire.lateralFrictionRatio = 1.0;

        // 3. SDS MK5n R3 Actuation Setup
        // Steer ratio 287/11 = 26.0909:1 with Kraken X44
        config.steer.motor = Actuation::DCMotorModel::KrakenX44(26.0909, 0.92);
        config.steer.steerHousingInertia = 0.00036;
        config.steer.coulombFrictionTorque = 0.08;
        config.steer.viscousDamping = 0.002;

        // Drive ratio 5.27:1 with Kraken X60
        config.drive.motor = Actuation::DCMotorModel::KrakenX60(5.27, 0.96);
        config.drive.coulombFrictionTorque = 1.8;

        // 4. Module Placements in NWU (+X Forward, +Y Left):
        // Trackwidth = 0.56m, Wheelbase = 0.56m (Half-extents: 0.28m)
        constexpr double halfExtent = 0.28;

        // [0] FrontLeft: (+X, +Y)
        config.modules[0] = { "FrontLeft", 0, glm::vec2(+halfExtent, +halfExtent), 2, 1, 1 };
        // [1] FrontRight: (+X, -Y)
        config.modules[1] = { "FrontRight", 1, glm::vec2(+halfExtent, -halfExtent), 4, 3, 2 };
        // [2] BackLeft: (-X, +Y)
        config.modules[2] = { "BackLeft", 2, glm::vec2(-halfExtent, +halfExtent), 6, 5, 3 };
        // [3] BackRight: (-X, -Y)
        config.modules[3] = { "BackRight", 3, glm::vec2(-halfExtent, -halfExtent), 8, 7, 4 };

        return config;
    }

    static Actuation::DCMotorModel ParseMotor(const std::string& type, double ratio, double eff)
    {
        if (type == "KrakenX60")
        {
            return Actuation::DCMotorModel::KrakenX60(ratio, eff);
        }
        if (type == "KrakenX60FOC")
        {
            return Actuation::DCMotorModel::KrakenX60FOC(ratio, eff);
        }
        if (type == "KrakenX44")
        {
            return Actuation::DCMotorModel::KrakenX44(ratio, eff);
        }
        if (type == "KrakenX44FOC")
        {
            return Actuation::DCMotorModel::KrakenX44FOC(ratio, eff);
        }
        if (type == "Falcon500")
        {
            return Actuation::DCMotorModel::Falcon500(ratio, eff);
        }
        if (type == "NEO" || type == "NEO1")
        {
            return Actuation::DCMotorModel::NEO1(ratio, eff);
        }
        if (type == "NEO2")
        {
            return Actuation::DCMotorModel::NEO2(ratio, eff);
        }
        if (type == "NEOVortex")
        {
            return Actuation::DCMotorModel::NEOVortex(ratio, eff);
        }
        if (type == "NEO550")
        {
            return Actuation::DCMotorModel::NEO550(ratio, eff);
        }
        if (type == "Minion")
        {
            return Actuation::DCMotorModel::Minion(ratio, eff);
        }

        return Actuation::DCMotorModel::KrakenX60(ratio, eff);
    }

    SwerveDrivetrainConfig SwerveDrivetrainConfig::LoadFromJsonString(const std::string& jsonString)
    {
        SwerveDrivetrainConfig config = CreateMK5nR3Reference();

        nlohmann::json root;
        try
        {
            root = nlohmann::json::parse(jsonString);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[SwerveParams ERROR] JSON parsing exception: " << e.what() << std::endl;
            return config;
        }

        // 1. Chassis
        if (root.contains("chassis") && root["chassis"].is_object())
        {
            const auto& c = root["chassis"];
            config.chassis.mass = c.value("mass_kg", config.chassis.mass);
            config.chassis.dimensions.x = c.value("length_m", config.chassis.dimensions.x);
            config.chassis.dimensions.y = c.value("width_m", config.chassis.dimensions.y);
            config.chassis.dimensions.z = c.value("height_m", config.chassis.dimensions.z);
            config.chassis.centerOfMassHeight = c.value("cgHeight_m", config.chassis.centerOfMassHeight);

            if (c.contains("cgOffset_m") && c["cgOffset_m"].is_array() && c["cgOffset_m"].size() >= 2)
            {
                config.chassis.centerOfMassOffset.x = c["cgOffset_m"][0].get<double>();
                config.chassis.centerOfMassOffset.y = c["cgOffset_m"][1].get<double>();
            }

            if (c.contains("inertiaTensor_kgm2") && c["inertiaTensor_kgm2"].is_array())
            {
                const auto& it = c["inertiaTensor_kgm2"];
                if (it.size() == 3 && it[0].is_array())
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            config.chassis.inertiaTensor[j][i] = it[i][j].get<double>();
                        }
                    }
                }
                else if (it.size() == 3)
                {
                    config.chassis.inertiaTensor = glm::mat3(0.0);
                    config.chassis.inertiaTensor[0][0] = it[0].get<double>();
                    config.chassis.inertiaTensor[1][1] = it[1].get<double>();
                    config.chassis.inertiaTensor[2][2] = it[2].get<double>();
                }
            }
        }

        // 2. Tire & Contact Properties
        if (root.contains("wheel") && root["wheel"].is_object())
        {
            const auto& w = root["wheel"];
            config.tire.wheelRadius = w.value("radius_m", config.tire.wheelRadius);
            config.tire.wheelWidth = w.value("width_m", config.tire.wheelWidth);
            config.tire.wheelMass = w.value("mass_kg", config.tire.wheelMass);
            config.tire.wheelAxleInertia = w.value("axleInertia_kgm2", config.tire.wheelAxleInertia);
            config.tire.muStatic = w.value("friction", config.tire.muStatic);
        }

        if (root.contains("tire") && root["tire"].is_object())
        {
            const auto& t = root["tire"];
            config.tire.radialStiffness = t.value("radialStiffness_Npm", config.tire.radialStiffness);
            config.tire.radialDamping = t.value("radialDamping_Nspm", config.tire.radialDamping);
            config.tire.preSlideDisplacement = t.value("preSlide_m", config.tire.preSlideDisplacement);
            config.tire.muStatic = t.value("mu_static", config.tire.muStatic);
            config.tire.muKinetic = t.value("mu_kinetic", config.tire.muKinetic);
            config.tire.stribeckVelocity = t.value("stribeckVelocity_mps", config.tire.stribeckVelocity);
            config.tire.loadSensitivityBeta = t.value("loadSensitivityBeta", config.tire.loadSensitivityBeta);
            config.tire.rollingResistance = t.value("rollingResistance", config.tire.rollingResistance);
            config.tire.lateralFrictionRatio = t.value("lateralFrictionRatio", config.tire.lateralFrictionRatio);
        }

        // 3. Steer & Drive Electromechanics
        if (root.contains("motors") && root["motors"].is_object())
        {
            const auto& m = root["motors"];
            if (m.contains("steer") && m["steer"].is_object())
            {
                const auto& sm = m["steer"];
                std::string type = sm.value("type", "KrakenX44");
                double ratio = sm.value("gearRatio", 26.0909);
                double eff = sm.value("efficiency", 0.92);
                config.steer.motor = ParseMotor(type, ratio, eff);
            }
            if (m.contains("drive") && m["drive"].is_object())
            {
                const auto& dm = m["drive"];
                std::string type = dm.value("type", "KrakenX60");
                double ratio = dm.value("gearRatio", 5.27);
                double eff = dm.value("efficiency", 0.96);
                config.drive.motor = ParseMotor(type, ratio, eff);
            }
        }

        if (root.contains("steerHousing") && root["steerHousing"].is_object())
        {
            const auto& sh = root["steerHousing"];
            if (sh.contains("mass_kg") && sh.contains("radius_m"))
            {
                double mHousing = sh.value("mass_kg", 0.45);
                double rHousing = sh.value("radius_m", 0.04);
                config.steer.steerHousingInertia = 0.5 * mHousing * rHousing * rHousing;
            }
        }

        if (root.contains("steerModule") && root["steerModule"].is_object())
        {
            const auto& sm = root["steerModule"];
            config.steer.steerHousingInertia = sm.value("steerHousingInertia_kgm2", config.steer.steerHousingInertia);
            config.steer.coulombFrictionTorque = sm.value("coulombFriction_Nm", config.steer.coulombFrictionTorque);
            config.steer.viscousDamping = sm.value("viscousDamping_Nms", config.steer.viscousDamping);
        }

        if (root.contains("driveModule") && root["driveModule"].is_object())
        {
            const auto& dm = root["driveModule"];
            config.drive.coulombFrictionTorque = dm.value("coulombFriction_Nm", config.drive.coulombFrictionTorque);
        }

        // 4. Module Placements
        if (root.contains("modules") && root["modules"].is_array())
        {
            const auto& mods = root["modules"];
            for (size_t i = 0; i < mods.size() && i < 4; ++i)
            {
                const auto& mod = mods[i];
                config.modules[i].moduleIndex = static_cast<uint32_t>(i);
                config.modules[i].name = mod.value("name", config.modules[i].name);

                if (mod.contains("location_m") && mod["location_m"].is_array() && mod["location_m"].size() >= 2)
                {
                    // Note: Schema stores [X_engine, Z_engine] which maps to [Y_NWU_left, X_NWU_forward]
                    // If JSON stores [X, Y], locationOnChassis: X=Forward, Y=Left
                    float rawX = mod["location_m"][0].get<float>();
                    float rawY = mod["location_m"][1].get<float>();
                    // When positive forward/right is used in JSON:
                    config.modules[i].locationOnChassis = glm::vec2(rawY, -rawX);
                }

                config.modules[i].steerCANId = mod.value("steerCAN", config.modules[i].steerCANId);
                config.modules[i].driveCANId = mod.value("driveCAN", config.modules[i].driveCANId);
                config.modules[i].canCoderId = mod.value("canCoder", config.modules[i].canCoderId);
            }
        }

        return config;
    }

    SwerveDrivetrainConfig SwerveDrivetrainConfig::LoadFromJsonFile(const std::string& filePath)
    {
        std::ifstream file(filePath);
        if (!file.is_open())
        {
            file.open("../" + filePath);
        }
        if (!file.is_open())
        {
            file.open("../../" + filePath);
        }

        if (!file.is_open())
        {
            std::cerr << "[SwerveParams ERROR] Could not open config file: " << filePath << std::endl;
            return CreateMK5nR3Reference();
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return LoadFromJsonString(buffer.str());
    }
}