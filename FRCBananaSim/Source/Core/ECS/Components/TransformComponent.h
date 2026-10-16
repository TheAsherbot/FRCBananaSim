/**
 * @file TransformComponent.h
 * @brief Spatial transform representations for local and world coordinates.
 */

#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Core::ECS::Components
{
    using Vec3 = glm::vec3;
    using Quat = glm::quat;

    /**
     * @struct TransformComponent
     * @brief Local-space translation, rotation, and scaling relative to parent entity.
     */
    struct TransformComponent
    {
        Vec3 position{ 0.0f, 0.0f, 0.0f };         ///< Local position offset in meters.
        Quat rotation{ 1.0f, 0.0f, 0.0f, 0.0f };    ///< Local orientation quaternion (w, x, y, z).
        Vec3 scale{ 1.0f, 1.0f, 1.0f };            ///< Local scaling factors along local axes.
    };

    /**
     * @struct WorldTransformComponent
     * @brief Computed global-space transformation, updated by TransformPropagationSystem.
     */
    struct WorldTransformComponent
    {
        Vec3 worldPosition{ 0.0f, 0.0f, 0.0f };         ///< World-space origin position in meters.
        Quat worldRotation{ 1.0f, 0.0f, 0.0f, 0.0f };    ///< World-space absolute orientation quaternion.
        Vec3 worldScale{ 1.0f, 1.0f, 1.0f };            ///< World-space compound scale factors.
    };
}