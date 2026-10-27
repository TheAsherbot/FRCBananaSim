/**
 * @file TransformPropagationSystem.cpp
 * @brief Implementation of recursive world transform hierarchy propagation.
 */

#include "TransformPropagationSystem.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

namespace FRCBananaSim::Core::ECS::Systems
{
    using Components::TransformComponent;
    using Components::WorldTransformComponent;
    using Components::HierarchyComponent;
    using Core::invalidEntity;

    TransformPropagationSystem::TransformPropagationSystem(Scene& scene)
        : ISystem(scene)
    {}

    void TransformPropagationSystem::Update()
    {
        SIM_PROFILE_SCOPE("TransformPropagationSystem::Update");
        auto entitiesWithTransform = scene.Query<TransformComponent>();
        for (Entity e : entitiesWithTransform)
        {
            auto* hierarchy = scene.GetComponent<HierarchyComponent>(e);
            if (!hierarchy || hierarchy->parent == invalidEntity)
            {
                Propagate(e,
                    glm::vec3(0.0f),
                    glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                    glm::vec3(1.0f));
            }
        }
    }

    void TransformPropagationSystem::Propagate(Entity entity,
        const glm::vec3& parentWorldPos,
        const glm::quat& parentWorldRot,
        const glm::vec3& parentWorldScale)
    {
        auto* local = scene.GetComponent<TransformComponent>(entity);
        if (!local)
        {
            return;
        }

        glm::vec3 worldPos = parentWorldPos + parentWorldRot * (parentWorldScale * local->position);
        glm::quat worldRot = parentWorldRot * local->rotation;
        glm::vec3 worldScale = parentWorldScale * local->scale;

        WorldTransformComponent world;
        world.worldPosition = worldPos;
        world.worldRotation = worldRot;
        world.worldScale = worldScale;
        scene.AddComponent<WorldTransformComponent>(entity, world);

        auto* hierarchy = scene.GetComponent<HierarchyComponent>(entity);
        if (!hierarchy)
        {
            return;
        }

        for (Entity child : hierarchy->children)
        {
            Propagate(child, worldPos, worldRot, worldScale);
        }
    }
}