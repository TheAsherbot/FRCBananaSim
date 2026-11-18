/**
 * @file ContactZoneComponent.h
 * @brief Component marking an active engagement volume for intake capture or shooter wheel compression.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @struct ContactZoneComponent
     * @brief Metadata attached to sensor colliders designating specific mechanism roles.
     */
    struct ContactZoneComponent
    {
        std::string name;
        std::string role; ///< "intake", "shooter", "indexer"
        Core::ECS::Core::Entity parentLink = Core::ECS::Core::invalidEntity;

        // Full Geometric Retention in Engine Frame
        glm::vec3 localOrigin = { 0.0f, 0.0f, 0.0f };
        glm::quat localRotation = { 1.0f, 0.0f, 0.0f, 0.0f };
        glm::vec3 dimensions = { 0.0f, 0.0f, 0.0f };
        glm::vec3 tangentAxis = { 0.0f, 0.0f, 1.0f };
        glm::vec3 compressionAxis = { 0.0f, 1.0f, 0.0f };
        double nominalGapM = 0.0;

        bool isEngaged = false; ///< True if game pieces are currently in active contact within the zone.
    };
}