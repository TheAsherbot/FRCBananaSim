/**
 * @file PhysicsDebugDrawSystem.h
 * @brief ECS presentation system that draws wireframe representations of colliders.
 */

#pragma once

#include "Core/ECS/Core/System.h"

namespace FRCBananaSim::Physics::Systems
{
    /**
     * @class PhysicsDebugDrawSystem
     * @brief Visualizes active physics shapes using the DebugDraw wireframe pipeline.
     */
    class PhysicsDebugDrawSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the physics debug draw system.
         * @param scene Reference to the governing ECS scene.
         */
        explicit PhysicsDebugDrawSystem(Core::ECS::Core::Scene& scene);

        /**
         * @brief Virtual destructor.
         */
        virtual ~PhysicsDebugDrawSystem() override;

        /**
         * @brief Renders wireframe bounds for all entities with ColliderComponent.
         */
        virtual void RenderUpdate() override;
    };
}