/**
 * @file FourBarComponent.h
 * @brief Component representing a closed-loop planar four-bar linkage mechanism.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @struct FourBarComponent
     * @brief ECS component holding joint and link entities for a closed kinematic four-bar loop.
     */
    struct FourBarComponent
    {
        std::string name;

        // Kinematic Link Entities
        Core::ECS::Core::Entity baseEntity = Core::ECS::Core::invalidEntity;     ///< Ground / chassis link L_0.
        Core::ECS::Core::Entity crankEntity = Core::ECS::Core::invalidEntity;    ///< Driver / crank link L_1.
        Core::ECS::Core::Entity couplerEntity = Core::ECS::Core::invalidEntity;  ///< Floating / coupler link L_2.
        Core::ECS::Core::Entity followerEntity = Core::ECS::Core::invalidEntity; ///< Rocker / follower link L_3.

        // Kinematic Hinge Joint Entities
        Core::ECS::Core::Entity crankJoint = Core::ECS::Core::invalidEntity;     ///< Hinge J_01: Base <-> Crank.
        Core::ECS::Core::Entity couplerJoint = Core::ECS::Core::invalidEntity;   ///< Hinge J_12: Crank <-> Coupler.
        Core::ECS::Core::Entity followerJoint = Core::ECS::Core::invalidEntity;  ///< Hinge J_03: Base <-> Follower.
        Core::ECS::Core::Entity closureJoint = Core::ECS::Core::invalidEntity;   ///< Hinge J_32: Follower <-> Coupler (loop closer).

        // Geometric Link Lengths (m)
        double groundLengthM = 0.0;    ///< Distance between base pivots A_0 and B_0.
        double crankLengthM = 0.0;     ///< Length of crank link A_0 to A_1.
        double couplerLengthM = 0.0;   ///< Length of coupler link A_1 to B_1.
        double followerLengthM = 0.0;  ///< Length of follower link B_0 to B_1.

        // Loop Closure Anchors (Engine Frame)
        glm::vec3 closureParentAnchor = { 0.0f, 0.0f, 0.0f }; ///< Anchor on follower link B_1.
        glm::vec3 closureChildAnchor = { 0.0f, 0.0f, 0.0f };  ///< Anchor on coupler link B_1.
        glm::vec3 hingeAxis = { 0.0f, 0.0f, 1.0f };           ///< Shared rotation axis perpendicular to plane.

        // Runtime Loop Diagnostics
        double loopClosureErrorM = 0.0; ///< Current distance between closure anchors (< 1.5 mm under load).
        bool isClosedLoop = true;       ///< Whether the fourth loop-closing hinge constraint is active.
    };
}