/**
 * @file PhysicsSystem.cpp
 * @brief Implementation of the ECS physics synchronization and Jolt simulation step.
 */

#include "PhysicsSystem.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Telemetry/TelemetryMacros.h"
#include "Utils/Log.h"

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

namespace FRCBananaSim::Physics::Systems
{
    using Components::RigidBodyComponent;
    using Components::ColliderComponent;
    using Components::BodyMotionType;
    using Components::ColliderShape;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Components::WorldTransformComponent;
    using Core::ECS::Components::HierarchyComponent;
    using Core::ECS::Core::invalidEntity;

    PhysicsSystem::PhysicsSystem(Core::ECS::Core::Scene& scene, JoltContext& context)
        : ISystem(scene),
        joltContext(context)
    {}

    PhysicsSystem::~PhysicsSystem()
    {
        if (!joltContext.IsInitialized())
        {
            return;
        }

        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        for (const JPH::BodyID& bodyID : createdBodies)
        {
            if (!bodyID.IsInvalid())
            {
                bodyInterface.RemoveBody(bodyID);
                bodyInterface.DestroyBody(bodyID);
            }
        }
        createdBodies.clear();
    }

    void PhysicsSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("PhysicsSystem::FixedUpdate");

        if (!joltContext.IsInitialized())
        {
            return;
        }

        // 1. Allocate Jolt bodies for new entities
        CreatePendingBodies();

        // 2. Push kinematic movements into Jolt
        PushKinematicTransforms();

        // 3. Step physics simulation
        joltContext.Step(static_cast<float>(dt));

        // 4. Pull dynamic transforms and velocities back into ECS
        PullSimulatedTransforms();
    }

    void PhysicsSystem::DestroyBody(Core::ECS::Core::Entity entity)
    {
        auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
        if (!rigidBody || !rigidBody->isCreated || rigidBody->bodyID.IsInvalid())
        {
            return;
        }

        if (joltContext.IsInitialized())
        {
            JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
            bodyInterface.RemoveBody(rigidBody->bodyID);
            bodyInterface.DestroyBody(rigidBody->bodyID);
        }

        rigidBody->isCreated = false;
        rigidBody->bodyID = JPH::BodyID();
    }

    void PhysicsSystem::CreatePendingBodies()
    {
        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        auto entities = scene.Query<RigidBodyComponent, TransformComponent>();

        for (auto entity : entities)
        {
            auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
            if (!rigidBody || rigidBody->isCreated)
            {
                continue;
            }

            auto* transform = scene.GetComponent<TransformComponent>(entity);
            auto* collider = scene.GetComponent<ColliderComponent>(entity);

            // Create collision shape
            JPH::RefConst<JPH::Shape> shape;

            if (collider)
            {
                switch (collider->shape)
                {
                case ColliderShape::Box:
                {
                    JPH::BoxShapeSettings shapeSettings(JPH::Vec3(
                        collider->boxHalfExtents.x,
                        collider->boxHalfExtents.y,
                        collider->boxHalfExtents.z
                    ));
                    auto result = shapeSettings.Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                    break;
                }

                case ColliderShape::Sphere:
                {
                    JPH::SphereShapeSettings shapeSettings(collider->radius);
                    auto result = shapeSettings.Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                    break;
                }

                case ColliderShape::Cylinder:
                {
                    JPH::CylinderShapeSettings shapeSettings(collider->halfHeight, collider->radius);
                    auto result = shapeSettings.Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                    break;
                }

                case ColliderShape::Capsule:
                {
                    JPH::CapsuleShapeSettings shapeSettings(collider->halfHeight, collider->radius);
                    auto result = shapeSettings.Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                    break;
                }
                }
            }
            else
            {
                // Fallback default box scaled to transform extents
                JPH::BoxShapeSettings shapeSettings(JPH::Vec3(
                    transform->scale.x * 0.5f,
                    transform->scale.y * 0.5f,
                    transform->scale.z * 0.5f
                ));
                auto result = shapeSettings.Create();
                if (result.IsValid())
                {
                    shape = result.Get();
                }
            }

            if (!shape)
            {
                SIM_LOG_ERROR("Failed to construct Jolt collision shape for Entity " << entity);
                continue;
            }

            // Map motion type
            JPH::EMotionType motionType = JPH::EMotionType::Dynamic;
            switch (rigidBody->motionType)
            {
            case BodyMotionType::Static:    motionType = JPH::EMotionType::Static; break;
            case BodyMotionType::Kinematic: motionType = JPH::EMotionType::Kinematic; break;
            case BodyMotionType::Dynamic:   motionType = JPH::EMotionType::Dynamic; break;
            }

            // Assign collision layer
            JPH::ObjectLayer layer = ObjectLayers::Moving;
            if (rigidBody->isSensor)
            {
                layer = ObjectLayers::Sensor;
            }
            else if (motionType == JPH::EMotionType::Static)
            {
                layer = ObjectLayers::NonMoving;
            }

            // Determine spawn position including optional collider local offset
            glm::vec3 spawnPos = transform->position;
            glm::quat spawnRot = transform->rotation;
            if (collider)
            {
                spawnPos += spawnRot * collider->localOffset;
                spawnRot = spawnRot * collider->localRotation;
            }

            JPH::RVec3 joltPos(spawnPos.x, spawnPos.y, spawnPos.z);
            JPH::Quat joltRot(spawnRot.x, spawnRot.y, spawnRot.z, spawnRot.w);

            JPH::BodyCreationSettings bodySettings(shape, joltPos, joltRot, motionType, layer);
            bodySettings.mFriction = rigidBody->friction;
            bodySettings.mRestitution = rigidBody->restitution;
            bodySettings.mLinearDamping = rigidBody->linearDamping;
            bodySettings.mAngularDamping = rigidBody->angularDamping;
            bodySettings.mIsSensor = rigidBody->isSensor;

            if (motionType == JPH::EMotionType::Dynamic)
            {
                bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
                bodySettings.mMassPropertiesOverride.mMass = rigidBody->mass;
                bodySettings.mLinearVelocity = JPH::Vec3(
                    rigidBody->linearVelocity.x,
                    rigidBody->linearVelocity.y,
                    rigidBody->linearVelocity.z
                );
                bodySettings.mAngularVelocity = JPH::Vec3(
                    rigidBody->angularVelocity.x,
                    rigidBody->angularVelocity.y,
                    rigidBody->angularVelocity.z
                );
            }

            JPH::Body* body = bodyInterface.CreateBody(bodySettings);
            if (!body)
            {
                SIM_LOG_ERROR("Failed to allocate Jolt body for Entity " << entity);
                continue;
            }

            rigidBody->bodyID = body->GetID();
            bodyInterface.AddBody(rigidBody->bodyID, JPH::EActivation::Activate);
            rigidBody->isCreated = true;
            createdBodies.push_back(rigidBody->bodyID);
        }
    }

    void PhysicsSystem::PushKinematicTransforms()
    {
        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        auto entities = scene.Query<RigidBodyComponent, TransformComponent>();

        for (auto entity : entities)
        {
            auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
            if (!rigidBody || !rigidBody->isCreated || rigidBody->motionType != BodyMotionType::Kinematic)
            {
                continue;
            }

            auto* transform = scene.GetComponent<TransformComponent>(entity);
            auto* collider = scene.GetComponent<ColliderComponent>(entity);

            glm::vec3 pos = transform->position;
            glm::quat rot = transform->rotation;
            if (collider)
            {
                pos += rot * collider->localOffset;
                rot = rot * collider->localRotation;
            }

            JPH::RVec3 joltPos(pos.x, pos.y, pos.z);
            JPH::Quat joltRot(rot.x, rot.y, rot.z, rot.w);
            bodyInterface.SetPositionAndRotation(rigidBody->bodyID, joltPos, joltRot, JPH::EActivation::Activate);
        }
    }

    void PhysicsSystem::PullSimulatedTransforms()
    {
        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        auto entities = scene.Query<RigidBodyComponent, TransformComponent>();

        for (auto entity : entities)
        {
            auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
            if (!rigidBody || !rigidBody->isCreated || rigidBody->motionType != BodyMotionType::Dynamic)
            {
                continue;
            }

            auto* transform = scene.GetComponent<TransformComponent>(entity);
            auto* collider = scene.GetComponent<ColliderComponent>(entity);

            JPH::RVec3 joltPos = bodyInterface.GetPosition(rigidBody->bodyID);
            JPH::Quat joltRot = bodyInterface.GetRotation(rigidBody->bodyID);
            JPH::Vec3 joltLinVel = bodyInterface.GetLinearVelocity(rigidBody->bodyID);
            JPH::Vec3 joltAngVel = bodyInterface.GetAngularVelocity(rigidBody->bodyID);

            glm::vec3 worldPos(joltPos.GetX(), joltPos.GetY(), joltPos.GetZ());
            glm::quat worldRot(joltRot.GetW(), joltRot.GetX(), joltRot.GetY(), joltRot.GetZ());

            // Reverse collider local offset to recover entity origin
            if (collider)
            {
                worldRot = worldRot * glm::inverse(collider->localRotation);
                worldPos -= worldRot * collider->localOffset;
            }

            transform->position = worldPos;
            transform->rotation = worldRot;

            rigidBody->linearVelocity = glm::vec3(joltLinVel.GetX(), joltLinVel.GetY(), joltLinVel.GetZ());
            rigidBody->angularVelocity = glm::vec3(joltAngVel.GetX(), joltAngVel.GetY(), joltAngVel.GetZ());
        }
    }
}