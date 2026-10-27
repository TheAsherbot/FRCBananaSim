/**
 * @file RobotSpawner.h
 * @brief Declarations for spawning complete physical robots into ECS with single-body dynamic physics and visual module hierarchies.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Core/Entity.h"
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <array>

namespace FRCBananaSim::Robot
{
    /**
     * @struct SpawnedRobot
     * @brief Collection of ECS entity handles composing the spawned swerve robot.
     */
    struct SpawnedRobot
    {
        Core::ECS::Core::Entity chassisEntity = Core::ECS::Core::invalidEntity;
        Core::ECS::Core::Entity batteryEntity = Core::ECS::Core::invalidEntity;
        std::array<Core::ECS::Core::Entity, 4> moduleEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };
        std::array<Core::ECS::Core::Entity, 4> steerEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };
        std::array<Core::ECS::Core::Entity, 4> wheelEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };

        // Deprecated Jolt joint handles preserved as invalidEntity until Phase 5 test rewrite
        std::array<Core::ECS::Core::Entity, 4> steerJointEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };
        std::array<Core::ECS::Core::Entity, 4> driveJointEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };

        bool isSuccess = false;
    };

    /**
     * @class RobotSpawner
     * @brief Factory for parsing JSON configuration and spawning physical robots into an ECS scene.
     */
    class RobotSpawner
    {
    public:
        /**
         * @brief Spawns a physical swerve robot into the scene from a JSON configuration file.
         * @param scene ECS scene to instantiate entities in.
         * @param jsonFilePath Relative or absolute filesystem path to the robot JSON file.
         * @param spawnPosition World position for the chassis center (+X Right, +Y Up, +Z Forward).
         * @param spawnRotation World orientation for the chassis.
         * @param robotGroupID Jolt collision group ID (default 1).
         * @return SpawnedRobot struct holding all spawned entity handles.
         */
        static SpawnedRobot SpawnFromJsonFile(Core::ECS::Core::Scene& scene,
            const std::string& jsonFilePath,
            const glm::vec3& spawnPosition = glm::vec3(0.0f, 0.5f, 0.0f),
            const glm::quat& spawnRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
            uint32_t robotGroupID = 1);

        /**
         * @brief Spawns a physical swerve robot into the scene from raw JSON configuration text.
         * @param scene ECS scene to instantiate entities in.
         * @param jsonString Serialized JSON configuration text.
         * @param spawnPosition World position for the chassis center (+X Right, +Y Up, +Z Forward).
         * @param spawnRotation World orientation for the chassis.
         * @param robotGroupID Jolt collision group ID (default 1).
         * @return SpawnedRobot struct holding all spawned entity handles.
         */
        static SpawnedRobot SpawnFromJsonString(Core::ECS::Core::Scene& scene,
            const std::string& jsonString,
            const glm::vec3& spawnPosition = glm::vec3(0.0f, 0.5f, 0.0f),
            const glm::quat& spawnRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
            uint32_t robotGroupID = 1);
    };
}