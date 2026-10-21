/**
 * @file RigidBodyComponent.h
 * @brief Component representing a Jolt rigid body with physical material and kinematic properties.
 */

#pragma once

#include "Physics/PhysicsConfig.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <glm/vec3.hpp>

#include <cstdint>

namespace FRCBananaSim::Physics::Components
{
    /**
     * @enum BodyMotionType
     * @brief Simulation behavior classification for rigid bodies.
     */
    enum class BodyMotionType : uint8_t
    {
        Static = 0,    ///< Immovable geometry (e.g. floor, fixed obstacles).
        Kinematic = 1, ///< Controlled programmatically (e.g. driven mechanisms, animation).
        Dynamic = 2    ///< Fully simulated by Jolt forces, gravity, and contacts.
    };

    /**
     * @struct RigidBodyComponent
     * @brief Physical properties and runtime Jolt body handle bound to an ECS entity.
     */
    struct RigidBodyComponent
    {
        JPH::BodyID bodyID;                                                       ///< Jolt handle. Default constructed as invalid.
        BodyMotionType motionType{ BodyMotionType::Dynamic };                      ///< Motion behavior.
        float mass{ 1.0f };                                                       ///< Mass in kilograms (used for Dynamic bodies).
        float friction{ 0.7f };                                                   ///< Coulomb friction coefficient.
        float restitution{ 0.1f };                                                ///< Coefficient of restitution (bounciness).
        float linearDamping{ PhysicsConfig::defaultLinearDamping };               ///< Linear drag (strictly 0.0 per project rules).
        float angularDamping{ PhysicsConfig::defaultAngularDamping };             ///< Angular drag (strictly 0.0 per project rules).
        glm::vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };                             ///< Current linear velocity in m/s.
        glm::vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };                            ///< Current rotational velocity in rad/s.
        bool isSensor{ false };                                                   ///< If true, triggers contact events without impulse.
        bool isCreated{ false };                                                  ///< Tracks whether the Jolt body has been allocated.
    };
}