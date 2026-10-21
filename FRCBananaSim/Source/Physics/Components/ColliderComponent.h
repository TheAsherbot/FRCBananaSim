/**
 * @file ColliderComponent.h
 * @brief Component specifying geometric collision bounds and local offsets for entities.
 */

#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>

namespace FRCBananaSim::Physics::Components
{
    /**
     * @enum ColliderShape
     * @brief Geometric primitive shapes supported for physics collision.
     */
    enum class ColliderShape : uint8_t
    {
        Box = 0,      ///< Rectangular cuboid defined by half-extents.
        Sphere = 1,   ///< Sphere defined by radius.
        Cylinder = 2, ///< Cylinder aligned along local Y-axis defined by half-height and radius.
        Capsule = 3   ///< Capsule aligned along local Y-axis defined by half-height and radius.
    };

    /**
     * @struct ColliderComponent
     * @brief Collision geometry definition attached to an ECS entity.
     */
    struct ColliderComponent
    {
        ColliderShape shape{ ColliderShape::Box };             ///< Primitive collision type.
        glm::vec3 boxHalfExtents{ 0.5f, 0.5f, 0.5f };          ///< Half-extents for Box shape in meters.
        float radius{ 0.5f };                                  ///< Radius for Sphere, Cylinder, and Capsule in meters.
        float halfHeight{ 0.5f };                              ///< Half-height for Cylinder and Capsule in meters.
        glm::vec3 localOffset{ 0.0f, 0.0f, 0.0f };             ///< Translation relative to entity TransformComponent.
        glm::quat localRotation{ 1.0f, 0.0f, 0.0f, 0.0f };     ///< Orientation relative to entity TransformComponent.
    };
}