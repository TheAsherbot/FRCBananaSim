/**
 * @file JointSystem.h
 * @brief ECS system managing physical Jolt constraints, motor updates, and joint deflection telemetry.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"

#include <vector>

namespace FRCBananaSim::Physics::Systems
{
    /**
     * @class JointSystem
     * @brief FixedUpdate system binding ECS JointComponents to Jolt physical constraints.
     */
    class JointSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the joint management system.
         * @param scene Reference to the governing ECS scene.
         * @param context Reference to the initialized JoltContext.
         */
        JointSystem(Core::ECS::Core::Scene& scene, JoltContext& context);

        /**
         * @brief Destructor. Removes all active constraints from Jolt.
         */
        virtual ~JointSystem() override;

        /**
         * @brief Fixed-step execution: instantiates pending constraints, drives motors, and reads deflection.
         * @param dt Fixed delta time in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Cleans up and destroys the constraint associated with an entity.
         * @param entity Entity holding the JointComponent to destroy.
         */
        void DestroyJoint(Core::ECS::Core::Entity entity);

    private:
        /**
         * @brief Creates Jolt constraints for entities whose rigid bodies have finished initializing.
         */
        void CreatePendingJoints();

        /**
         * @brief Synchronizes motor velocities, target angles, and torque limits into Jolt constraints.
         */
        void UpdateMotors();

        /**
         * @brief Reads back current angles, velocities, and anchor separation distance (deflection).
         */
        void ReadbackTelemetry();

        JoltContext& joltContext;
        std::vector<JPH::Ref<JPH::Constraint>> createdConstraints;
    };
}