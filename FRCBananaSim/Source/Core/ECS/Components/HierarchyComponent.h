/**
 * @file HierarchyComponent.h
 * @brief Scene graph relationships tracking parent and child entities.
 */

#pragma once

#include <vector>
#include "../Core/Entity.h"

namespace FRCBananaSim::Core::ECS::Components
{
    /**
     * @struct HierarchyComponent
     * @brief Establishes tree relationships for scene graph transform propagation.
     */
    struct HierarchyComponent
    {
        Entity parent{ invalidEntity };    ///< Parent entity handle (invalidEntity if root).
        std::vector<Entity> children;    ///< List of child entity handles.
    };
}