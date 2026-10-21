/**
 * @file PhysicsConfig.h
 * @brief Configuration constants, object layers, and numerical settings for Jolt Physics.
 */

#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

#include <cstdint>

namespace FRCBananaSim::Physics
{
    /**
     * @namespace ObjectLayers
     * @brief Collision layers assigned to individual rigid bodies and shapes.
     */
    namespace ObjectLayers
    {
        inline constexpr JPH::ObjectLayer NonMoving = 0; ///< Static field elements, floor, obstacles.
        inline constexpr JPH::ObjectLayer Moving = 1;    ///< Dynamic and kinematic robot bodies, game pieces.
        inline constexpr JPH::ObjectLayer Sensor = 2;    ///< Trigger zones, beam breaks, intake capture volumes.
        inline constexpr JPH::ObjectLayer NumLayers = 3; ///< Total number of object layers.
    }

    /**
     * @namespace BroadPhaseLayers
     * @brief Broadphase acceleration tree groupings for collision detection.
     */
    namespace BroadPhaseLayers
    {
        inline constexpr JPH::BroadPhaseLayer NonMoving(0); ///< Static tree.
        inline constexpr JPH::BroadPhaseLayer Moving(1);    ///< Dynamic tree.
        inline constexpr JPH::BroadPhaseLayer Sensor(2);    ///< Sensor/trigger tree.
        inline constexpr JPH::uint NumLayers = 3;           ///< Total number of broadphase layers.
    }

    /**
     * @struct PhysicsConfig
     * @brief Global numeric parameters governing the 200 Hz deterministic simulation loop.
     */
    struct PhysicsConfig
    {
        static constexpr double fixedDeltaTime = 1.0 / 200.0;           ///< 200 Hz (5 ms) simulation step.
        static constexpr float defaultLinearDamping = 0.0f;             ///< Zero artificial linear drag.
        static constexpr float defaultAngularDamping = 0.0f;            ///< Zero artificial rotational drag.
        static constexpr uint32_t defaultVelocitySteps = 10;            ///< Solver velocity iterations.
        static constexpr uint32_t defaultPositionSteps = 2;             ///< Solver position/relaxation iterations.
        static constexpr float defaultBaumgarte = 0.2f;                 ///< Constraint stabilization factor.
        static constexpr uint32_t maxBodies = 10240;                    ///< Maximum rigid bodies supported.
        static constexpr uint32_t numBodyMutexes = 0;                   ///< 0 lets Jolt determine mutex count.
        static constexpr uint32_t maxBodyPairs = 10240;                 ///< Broadphase overlapping pair capacity.
        static constexpr uint32_t maxContactConstraints = 10240;        ///< Narrowphase contact constraint capacity.
        static constexpr uint32_t tempAllocatorBytes = 16 * 1024 * 1024; ///< 16 MB per-thread temp allocation pool.
    };
}