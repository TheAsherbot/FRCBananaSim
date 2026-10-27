/**
 * @file JoltContext.h
 * @brief Subsystem wrapper managing Jolt Physics initialization, collision layers, and world stepping.
 */

#pragma once

#include "PhysicsConfig.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include <memory>

namespace FRCBananaSim::Physics
{
    class BPLayerInterfaceImpl;
    class ObjectVsBroadPhaseLayerFilterImpl;
    class ObjectLayerPairFilterImpl;

    /**
     * @class JoltContext
     * @brief Encapsulates a complete deterministic Jolt physics world instance.
     */
    class JoltContext
    {
    public:
        /**
         * @brief Default constructor. Prepares context in uninitialized state.
         */
        JoltContext();

        /**
         * @brief Destructor. Ensures clean teardown of Jolt structures and allocators.
         */
        ~JoltContext();

        // Non-copyable, non-movable context owning singleton factory references
        JoltContext(const JoltContext&) = delete;
        JoltContext& operator=(const JoltContext&) = delete;
        JoltContext(JoltContext&&) = delete;
        JoltContext& operator=(JoltContext&&) = delete;

        /**
         * @brief Initializes Jolt factories, job system, collision filtering, and physics system.
         * @return True if initialized successfully, false otherwise.
         */
        bool Initialize();

        /**
         * @brief Shuts down the physics world and deallocates Jolt resources.
         */
        void Shutdown();

        /**
         * @brief Advances the physics simulation by the given delta time.
         * @param deltaTime Step duration in seconds (typically PhysicsConfig::fixedDeltaTime).
         */
        void Step(float deltaTime);

        /**
         * @brief Re-indexes and optimizes broadphase acceleration structures.
         */
        void OptimizeBroadPhase();

        /**
         * @brief Accesses the internal Jolt physics system.
         * @return Reference to the JPH::PhysicsSystem.
         */
        JPH::PhysicsSystem& GetPhysicsSystem();
        const JPH::PhysicsSystem& GetPhysicsSystem() const;

        /**
         * @brief Accesses the body interface for creating and manipulating bodies.
         * @return Reference to JPH::BodyInterface.
         */
        JPH::BodyInterface& GetBodyInterface();
        const JPH::BodyInterface& GetBodyInterface() const;

        /**
         * @brief Checks whether the physics world has been successfully initialized.
         * @return True if active and ready to simulate.
         */
        bool IsInitialized() const;

    private:
        bool isInitialized;
        std::unique_ptr<JPH::TempAllocator> tempAllocator;
        std::unique_ptr<JPH::JobSystemSingleThreaded> jobSystem;
        std::unique_ptr<BPLayerInterfaceImpl> broadPhaseLayerInterface;
        std::unique_ptr<ObjectVsBroadPhaseLayerFilterImpl> objectVsBroadPhaseLayerFilter;
        std::unique_ptr<ObjectLayerPairFilterImpl> objectLayerPairFilter;
        std::unique_ptr<JPH::PhysicsSystem> physicsSystem;
    };
}