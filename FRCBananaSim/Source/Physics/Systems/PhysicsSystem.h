/**
 * @file PhysicsSystem.h
 * @brief ECS system managing rigid body lifecycles, transform synchronization, and 200 Hz Jolt stepping.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"

#include <vector>

namespace FRCBananaSim::Physics::Systems
{
    /**
     * @class PhysicsSystem
     * @brief Fixed-step physics execution system binding ECS entities to Jolt bodies.
     */
    class PhysicsSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the physics system.
         * @param scene Reference to the governing ECS scene.
         * @param context Reference to the initialized JoltContext.
         */
        PhysicsSystem(Core::ECS::Core::Scene& scene, JoltContext& context);

        /**
         * @brief Virtual destructor. Cleans up all spawned Jolt bodies.
         */
        virtual ~PhysicsSystem() override;

        /**
         * @brief Advances the physics simulation at the fixed timestep (200 Hz).
         * @param dt Fixed delta time in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Destroys and cleans up the Jolt rigid body attached to an entity.
         * @param entity The entity whose physical body should be deallocated.
         */
        void DestroyBody(Core::ECS::Core::Entity entity);

    private:
        /**
         * @brief Creates a Jolt body for any entity containing a RigidBodyComponent not yet initialized.
         */
        void CreatePendingBodies();

        /**
         * @brief Synchronizes kinematic entity transforms from ECS into Jolt bodies.
         */
        void PushKinematicTransforms();

        /**
         * @brief Synchronizes dynamic Jolt body poses and velocities back into ECS components.
         */
        void PullSimulatedTransforms();

        JoltContext& joltContext;
        std::vector<JPH::BodyID> createdBodies;
    };
}