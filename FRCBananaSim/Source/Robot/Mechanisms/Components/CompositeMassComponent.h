/**
 * @file CompositeMassComponent.h
 * @brief Component holding dynamic composite mass, center of mass, and inertia tensor for a robot mechanism hierarchy.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <vector>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @struct CompositeMassComponent
     * @brief Aggregated mass properties tracking the chassis and articulating mechanism links.
     */
    struct CompositeMassComponent
    {
        Core::ECS::Core::Entity chassisEntity = Core::ECS::Core::invalidEntity; ///< Root chassis entity.
        std::vector<Core::ECS::Core::Entity> mechanismLinks;                   ///< Dynamic links articulating relative to chassis.

        // Runtime aggregated properties in WPILib NWU frame (+X Forward, +Y Left, +Z Up)
        double totalMassKg = 0.0;
        glm::vec3 compositeCenterOfMassNWU = { 0.0f, 0.0f, 0.0f };
        glm::mat3 compositeInertiaTensorNWU = glm::mat3(1.0f);

        // Runtime aggregated properties in Engine frame (+X Right/Left, +Y Up, +Z Forward)
        glm::vec3 compositeCenterOfMassEngine = { 0.0f, 0.0f, 0.0f };
        glm::mat3 compositeInertiaTensorEngine = glm::mat3(1.0f);
    };
}