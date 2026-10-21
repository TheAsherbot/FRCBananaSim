/**
 * @file PhysicsDebugDrawSystem.cpp
 * @brief Implementation of oriented wireframe visualization for collision shapes.
 */

#include "PhysicsDebugDrawSystem.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemetryMacros.h"
#include "Utils/DebugDraw.h"

namespace FRCBananaSim::Physics::Systems
{
    using Components::ColliderComponent;
    using Components::RigidBodyComponent;
    using Components::BodyMotionType;
    using Components::ColliderShape;
    using Core::ECS::Components::TransformComponent;
    using Utils::DebugDraw;

    PhysicsDebugDrawSystem::PhysicsDebugDrawSystem(Core::ECS::Core::Scene& scene)
        : ISystem(scene)
    {}

    PhysicsDebugDrawSystem::~PhysicsDebugDrawSystem()
    {}

    void PhysicsDebugDrawSystem::RenderUpdate()
    {
        SIM_PROFILE_SCOPE("PhysicsDebugDrawSystem::RenderUpdate");

        auto entities = scene.Query<ColliderComponent, TransformComponent>();

        for (auto entity : entities)
        {
            auto* collider = scene.GetComponent<ColliderComponent>(entity);
            auto* transform = scene.GetComponent<TransformComponent>(entity);
            if (!collider || !transform)
            {
                continue;
            }

            // Determine collider color based on motion type
            glm::vec3 color(0.2f, 0.9f, 0.3f); // Dynamic = green
            auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
            if (rigidBody)
            {
                if (rigidBody->isSensor)
                {
                    color = glm::vec3(0.9f, 0.9f, 0.1f); // Sensor = yellow
                }
                else if (rigidBody->motionType == BodyMotionType::Static)
                {
                    color = glm::vec3(0.5f, 0.5f, 0.5f); // Static = gray
                }
                else if (rigidBody->motionType == BodyMotionType::Kinematic)
                {
                    color = glm::vec3(0.2f, 0.6f, 0.95f); // Kinematic = blue
                }
            }

            // Calculate world-space center and orientation for the collider
            glm::vec3 center = transform->position + transform->rotation * collider->localOffset;
            glm::quat orientation = transform->rotation * collider->localRotation;

            switch (collider->shape)
            {
            case ColliderShape::Box:
            {
                // Draw oriented bounding box (OBB) 12 edges
                const glm::vec3& ext = collider->boxHalfExtents;
                glm::vec3 corners[8] = {
                    center + orientation * glm::vec3(-ext.x, -ext.y, -ext.z),
                    center + orientation * glm::vec3(ext.x, -ext.y, -ext.z),
                    center + orientation * glm::vec3(ext.x,  ext.y, -ext.z),
                    center + orientation * glm::vec3(-ext.x,  ext.y, -ext.z),
                    center + orientation * glm::vec3(-ext.x, -ext.y,  ext.z),
                    center + orientation * glm::vec3(ext.x, -ext.y,  ext.z),
                    center + orientation * glm::vec3(ext.x,  ext.y,  ext.z),
                    center + orientation * glm::vec3(-ext.x,  ext.y,  ext.z)
                };

                // Bottom loop
                DebugDraw::Line(corners[0], corners[1], color);
                DebugDraw::Line(corners[1], corners[2], color);
                DebugDraw::Line(corners[2], corners[3], color);
                DebugDraw::Line(corners[3], corners[0], color);

                // Top loop
                DebugDraw::Line(corners[4], corners[5], color);
                DebugDraw::Line(corners[5], corners[6], color);
                DebugDraw::Line(corners[6], corners[7], color);
                DebugDraw::Line(corners[7], corners[4], color);

                // Vertical pillars
                DebugDraw::Line(corners[0], corners[4], color);
                DebugDraw::Line(corners[1], corners[5], color);
                DebugDraw::Line(corners[2], corners[6], color);
                DebugDraw::Line(corners[3], corners[7], color);
                break;
            }

            case ColliderShape::Sphere:
            {
                DebugDraw::Sphere(center, collider->radius, color, 16, true);
                break;
            }

            case ColliderShape::Cylinder:
            case ColliderShape::Capsule:
            {
                DebugDraw::Cylinder(center, collider->radius, collider->halfHeight, color, 16, true);
                break;
            }
            }
        }
    }
}