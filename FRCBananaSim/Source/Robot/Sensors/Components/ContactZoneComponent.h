/**
 * @file ContactZoneComponent.h
 * @brief Component marking an active engagement volume for intake capture or shooter wheel compression.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <glm/vec3.hpp>

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
        glm::vec3 dimensions = { 0.0f, 0.0f, 0.0f };
        bool isEngaged = false; ///< True if game pieces are currently in active contact within the zone.
    };
}