/**
 * @file RenderSystem.cpp
 * @brief Implementation of the ECS render system pass.
 */

#include "Renderer/Systems/RenderSystem.h"
#include "Renderer/Renderer.h"
#include "Renderer/Components/MeshComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"

namespace FRCBananaSim::Renderer::Systems
{
    using Core::ECS::Scene;
    using Core::ECS::Entity;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Components::WorldTransformComponent;
    using Components::MeshComponent;

    RenderSystem::RenderSystem(Scene& scene, Renderer& renderer)
        : ISystem(scene),
        renderer(renderer)
    {}

    void RenderSystem::RenderUpdate()
    {
        auto entities = scene.Query<MeshComponent, WorldTransformComponent>();
        for (Entity e : entities)
        {
            auto* mesh = scene.GetComponent<MeshComponent>(e);
            auto* worldTransform = scene.GetComponent<WorldTransformComponent>(e);
            if (!mesh || !worldTransform)
            {
                continue;
            }

            renderer.DrawMesh(*mesh, *worldTransform);
        }

        renderer.FlushDebugDraw();
    }
}