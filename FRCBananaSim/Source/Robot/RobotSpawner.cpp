/**
 * @file RobotSpawner.cpp
 * @brief Implementation of declarative JSON robot loader and physical robot spawner.
 */

#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/Units/Units.h"
#include "Telemetry/TelemetryMacros.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

namespace FRCBananaSim::Robot
{
    using namespace Drivetrain::Components;
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    static DCMotorModel CreateMotorModel(const std::string& type, double gearRatio, double efficiency)
    {
        if (type == "KrakenX60")
        {
            return DCMotorModel::KrakenX60(gearRatio, efficiency);
        }
        if (type == "KrakenX44")
        {
            return DCMotorModel::KrakenX44(gearRatio, efficiency);
        }
        if (type == "Falcon500")
        {
            return DCMotorModel::Falcon500(gearRatio, efficiency);
        }
        if (type == "NEO" || type == "NEO1")
        {
            return DCMotorModel::NEO1(gearRatio, efficiency);
        }
        if (type == "NEO2")
        {
            return DCMotorModel::NEO2(gearRatio, efficiency);
        }
        if (type == "NEOVortex")
        {
            return DCMotorModel::NEOVortex(gearRatio, efficiency);
        }
        if (type == "NEO550")
        {
            return DCMotorModel::NEO550(gearRatio, efficiency);
        }
        if (type == "Minion")
        {
            return DCMotorModel::Minion(gearRatio, efficiency);
        }
        if (type == "GoBilda5202")
        {
            return DCMotorModel::GoBilda5202(gearRatio, efficiency);
        }

        return DCMotorModel::Falcon500(gearRatio, efficiency);
    }

    SpawnedRobot RobotSpawner::SpawnFromJsonFile(Scene& scene, const std::string& jsonFilePath,
        const glm::vec3& spawnPosition, const glm::quat& spawnRotation, uint32_t robotGroupID)
    {
        std::ifstream file(jsonFilePath);
        if (!file.is_open())
        {
            // Probe relative path if running from build/ or sub-directory
            file.open("../" + jsonFilePath);
        }
        if (!file.is_open())
        {
            file.open("../../" + jsonFilePath);
        }

        if (!file.is_open())
        {
            std::cerr << "[RobotSpawner ERROR] Failed to open robot configuration file: " << jsonFilePath << std::endl;
            return SpawnedRobot{};
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return SpawnFromJsonString(scene, buffer.str(), spawnPosition, spawnRotation, robotGroupID);
    }

    SpawnedRobot RobotSpawner::SpawnFromJsonString(Scene& scene, const std::string& jsonString,
        const glm::vec3& spawnPosition, const glm::quat& spawnRotation, uint32_t robotGroupID)
    {
        SIM_PROFILE_SCOPE("RobotSpawner::SpawnFromJsonString");

        SpawnedRobot result;
        nlohmann::json root;

        try
        {
            root = nlohmann::json::parse(jsonString);
        }
        catch (const nlohmann::json::exception& e)
        {
            std::cerr << "[RobotSpawner ERROR] JSON parsing failure: " << e.what() << std::endl;
            return result;
        }

        // 1. Chassis Physical Parameters
        const auto& chassisNode = root.value("chassis", nlohmann::json::object());
        float chassisMass = chassisNode.value("mass_kg", 50.0f);
        float width = chassisNode.value("width_m", 0.70f);
        float height = chassisNode.value("height_m", 0.15f);
        float length = chassisNode.value("length_m", 0.70f);
        float wheelbase = chassisNode.value("wheelbase_m", 0.56f);
        float trackwidth = chassisNode.value("trackwidth_m", 0.56f);
        float chassisFriction = chassisNode.value("friction", 0.7f);

        // 2. Battery Parameters
        const auto& batteryNode = root.value("battery", nlohmann::json::object());
        double nominalV = batteryNode.value("nominalVoltage_V", 12.60);
        double internalR = batteryNode.value("internalResistance_ohm", 0.023);
        double quiescentI = batteryNode.value("quiescentCurrent_A", 1.5);
        double statorLimit = batteryNode.value("statorCurrentLimit_A", 60.0);
        double supplyLimit = batteryNode.value("supplyCurrentLimit_A", 40.0);
        double brownoutWarn = batteryNode.value("brownoutWarning_V", 6.8);
        double brownoutCutoff = batteryNode.value("brownoutCutoff_V", 6.3);
        double brownoutRecovery = batteryNode.value("brownoutRecovery_V", 7.5);

        // 3. Motor Configurations
        const auto& motorsNode = root.value("motors", nlohmann::json::object());
        const auto& steerMotorNode = motorsNode.value("steer", nlohmann::json::object());
        const auto& driveMotorNode = motorsNode.value("drive", nlohmann::json::object());

        std::string steerMotorType = steerMotorNode.value("type", "Falcon500");
        double steerGearRatio = steerMotorNode.value("gearRatio", 21.43);
        double steerEfficiency = steerMotorNode.value("efficiency", 0.92);

        std::string driveMotorType = driveMotorNode.value("type", "KrakenX60");
        double driveGearRatio = driveMotorNode.value("gearRatio", 6.12);
        double driveEfficiency = driveMotorNode.value("efficiency", 0.96);

        // 4. Steer Housing & Wheel Geometry
        const auto& steerNode = root.value("steerHousing", nlohmann::json::object());
        float steerMass = steerNode.value("mass_kg", 0.45f);
        float steerRadius = steerNode.value("radius_m", 0.04f);
        float steerHeight = steerNode.value("height_m", 0.06f);

        const auto& wheelNode = root.value("wheel", nlohmann::json::object());
        float wheelMass = wheelNode.value("mass_kg", 0.65f);
        double wheelRadius = wheelNode.value("radius_m", 0.0508);
        float wheelWidth = wheelNode.value("width_m", 0.04f);
        float wheelFriction = wheelNode.value("friction", 1.2f);
        float wheelRestitution = wheelNode.value("restitution", 0.05f);

        // A. Battery Entity
        Entity batteryEntity = scene.CreateEntity();
        BatteryComponent batteryComp;
        batteryComp.batteryModel = BatteryModel(nominalV, internalR, quiescentI, statorLimit,
            supplyLimit, brownoutWarn, brownoutCutoff, brownoutRecovery);
        scene.AddComponent(batteryEntity, batteryComp);
        result.batteryEntity = batteryEntity;

        // B. Chassis Root Entity
        Entity chassisEntity = scene.CreateEntity();
        TransformComponent chassisTransform;
        chassisTransform.position = spawnPosition;
        chassisTransform.rotation = spawnRotation;
        chassisTransform.scale = glm::vec3(width, height, length);
        scene.AddComponent(chassisEntity, chassisTransform);

        RigidBodyComponent chassisRb;
        chassisRb.motionType = BodyMotionType::Dynamic;
        chassisRb.mass = chassisMass;
        chassisRb.friction = chassisFriction;
        chassisRb.collisionGroupID = robotGroupID;
        chassisRb.collisionSubGroupID = 0; // SubGroupID 0 for chassis
        scene.AddComponent(chassisEntity, chassisRb);

        ColliderComponent chassisCol;
        chassisCol.shape = ColliderShape::Box;
        chassisCol.boxHalfExtents = glm::vec3(width * 0.5f, height * 0.5f, length * 0.5f);
        scene.AddComponent(chassisEntity, chassisCol);

        SwerveChassisComponent chassisComp;
        chassisComp.batteryEntity = batteryEntity;
        chassisComp.trackWidth = trackwidth;
        chassisComp.wheelBase = wheelbase;

        // C. Module Layout parsing
        std::array<glm::vec2, 4> defaultLocations = {
            glm::vec2(-trackwidth * 0.5f,  wheelbase * 0.5f), // FrontLeft  [-X, +Z]
            glm::vec2(trackwidth * 0.5f,  wheelbase * 0.5f), // FrontRight [+X, +Z]
            glm::vec2(-trackwidth * 0.5f, -wheelbase * 0.5f), // BackLeft   [-X, -Z]
            glm::vec2(trackwidth * 0.5f, -wheelbase * 0.5f)  // BackRight  [+X, -Z]
        };

        std::array<uint32_t, 4> defaultSteerCAN = { 1, 3, 5, 7 };
        std::array<uint32_t, 4> defaultDriveCAN = { 2, 4, 6, 8 };
        std::array<uint32_t, 4> defaultCanCoder = { 1, 2, 3, 4 };

        const auto& modulesArray = root.value("modules", nlohmann::json::array());

        for (uint32_t i = 0; i < 4; ++i)
        {
            glm::vec2 modLoc = defaultLocations[i];
            uint32_t steerCAN = defaultSteerCAN[i];
            uint32_t driveCAN = defaultDriveCAN[i];
            uint32_t canCoder = defaultCanCoder[i];

            if (i < modulesArray.size())
            {
                const auto& modJson = modulesArray[i];
                if (modJson.contains("location_m") && modJson["location_m"].is_array() && modJson["location_m"].size() >= 2)
                {
                    modLoc.x = modJson["location_m"][0].get<float>();
                    modLoc.y = modJson["location_m"][1].get<float>();
                }
                steerCAN = modJson.value("steerCAN", steerCAN);
                driveCAN = modJson.value("driveCAN", driveCAN);
                canCoder = modJson.value("canCoder", canCoder);
            }

            // World spatial placements adhering strictly to: +X Right, +Y Up, +Z Forward
            // Chassis sits on top; steer housing hangs below chassis; 10 mm air gap to wheel
            constexpr float moduleAirGap = 0.010f; // 10 mm clearance between steer housing and wheel
            float halfChassisH = height * 0.5f;
            float halfSteerH = steerHeight * 0.5f;

            glm::vec3 localSteerOffset(modLoc.x, -halfChassisH - halfSteerH, modLoc.y);
            glm::vec3 localWheelOffset(modLoc.x, -halfChassisH - steerHeight - moduleAirGap - static_cast<float>(wheelRadius), modLoc.y);

            glm::vec3 worldSteerPos = spawnPosition + spawnRotation * localSteerOffset;
            glm::vec3 worldWheelPos = spawnPosition + spawnRotation * localWheelOffset;
            glm::vec3 worldSteerPivot = spawnPosition + spawnRotation * glm::vec3(modLoc.x, -halfChassisH, modLoc.y);
            glm::vec3 worldDrivePivot = worldWheelPos;

            // 1. Steer Link Entity
            Entity steerEntity = scene.CreateEntity();
            TransformComponent steerTransform;
            steerTransform.position = worldSteerPos;
            steerTransform.rotation = spawnRotation;
            steerTransform.scale = glm::vec3(steerRadius * 2.0f, steerHeight, steerRadius * 2.0f);
            scene.AddComponent(steerEntity, steerTransform);

            RigidBodyComponent steerRb;
            steerRb.motionType = BodyMotionType::Dynamic;
            steerRb.mass = steerMass;
            steerRb.collisionGroupID = robotGroupID;
            steerRb.collisionSubGroupID = 1 + i; // SubGroupID 1..4
            scene.AddComponent(steerEntity, steerRb);

            ColliderComponent steerCol;
            steerCol.shape = ColliderShape::Cylinder;
            steerCol.radius = steerRadius;
            steerCol.halfHeight = steerHeight * 0.5f;
            scene.AddComponent(steerEntity, steerCol);

            // 2. Wheel Link Entity
            Entity wheelEntity = scene.CreateEntity();
            TransformComponent wheelTransform;
            wheelTransform.position = worldWheelPos;
            wheelTransform.rotation = spawnRotation;
            wheelTransform.scale = glm::vec3(wheelWidth, static_cast<float>(wheelRadius) * 2.0f, static_cast<float>(wheelRadius) * 2.0f);
            scene.AddComponent(wheelEntity, wheelTransform);

            RigidBodyComponent wheelRb;
            wheelRb.motionType = BodyMotionType::Dynamic;
            wheelRb.mass = wheelMass;
            wheelRb.friction = wheelFriction;
            wheelRb.restitution = wheelRestitution;
            wheelRb.collisionGroupID = robotGroupID;
            wheelRb.collisionSubGroupID = 5 + i; // SubGroupID 5..8
            scene.AddComponent(wheelEntity, wheelRb);

            // Wheel collision volume: Sphere matching wheel radius eliminates cylinder rim chatter
            ColliderComponent wheelCol;
            wheelCol.shape = ColliderShape::Sphere;
            wheelCol.radius = static_cast<float>(wheelRadius);
            scene.AddComponent(wheelEntity, wheelCol);

            // 3. Vertical Steer Joint (Swivels around vertical +Y)
            Entity steerJointEntity = scene.CreateEntity();
            JointComponent steerJoint;
            steerJoint.parentEntity = chassisEntity;
            steerJoint.childEntity = steerEntity;
            steerJoint.SetWorldSpace(worldSteerPivot, spawnRotation * glm::vec3(0.0f, 1.0f, 0.0f));
            steerJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(steerJointEntity, steerJoint);

            // 4. Horizontal Drive Axle Joint (Rolls around horizontal +X)
            Entity driveJointEntity = scene.CreateEntity();
            JointComponent driveJoint;
            driveJoint.parentEntity = steerEntity;
            driveJoint.childEntity = wheelEntity;
            driveJoint.SetWorldSpace(worldDrivePivot, spawnRotation * glm::vec3(1.0f, 0.0f, 0.0f));
            driveJoint.motorMode = JointMotorMode::Torque;
            scene.AddComponent(driveJointEntity, driveJoint);

            // 5. Swerve Module Component
            Entity moduleEntity = scene.CreateEntity();
            SwerveModuleComponent moduleComp;
            moduleComp.moduleIndex = i;
            moduleComp.moduleLocation = modLoc;
            moduleComp.steerEntity = steerEntity;
            moduleComp.wheelEntity = wheelEntity;
            moduleComp.steerJointEntity = steerJointEntity;
            moduleComp.driveJointEntity = driveJointEntity;
            moduleComp.steerMotor = CreateMotorModel(steerMotorType, steerGearRatio, steerEfficiency);
            moduleComp.driveMotor = CreateMotorModel(driveMotorType, driveGearRatio, driveEfficiency);
            moduleComp.wheelRadius = wheelRadius;
            moduleComp.steerCANId = steerCAN;
            moduleComp.driveCANId = driveCAN;
            moduleComp.canCoderId = canCoder;
            scene.AddComponent(moduleEntity, moduleComp);

            chassisComp.moduleEntities[i] = moduleEntity;

            result.moduleEntities[i] = moduleEntity;
            result.steerEntities[i] = steerEntity;
            result.wheelEntities[i] = wheelEntity;
            result.steerJointEntities[i] = steerJointEntity;
            result.driveJointEntities[i] = driveJointEntity;
        }

        scene.AddComponent(chassisEntity, chassisComp);
        result.chassisEntity = chassisEntity;
        result.isSuccess = true;

        return result;
    }
}