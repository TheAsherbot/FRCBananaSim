/**
 * @file PhysicsSystem.cpp
 * @brief Implementation of the Jolt-backed physics simulation system with compound colliders, decoupled CoM, and inertia overrides.
 */

#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/Log.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

namespace FRCBananaSim::Physics::Systems
{
    using namespace Components;
    using Core::ECS::Components::TransformComponent;

    namespace
    {
        /**
         * @brief Helper to allocate a primitive Jolt shape settings object.
         */
        JPH::RefConst<JPH::ShapeSettings> CreatePrimitiveShapeSettings(
            ColliderShape shape,
            const glm::vec3& boxHalfExtents,
            float radius,
            float halfHeight)
        {
            switch (shape)
            {
            case ColliderShape::Box:
                return new JPH::BoxShapeSettings(JPH::Vec3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z));
            case ColliderShape::Sphere:
                return new JPH::SphereShapeSettings(radius);
            case ColliderShape::Cylinder:
                return new JPH::CylinderShapeSettings(halfHeight, radius);
            case ColliderShape::Capsule:
                return new JPH::CapsuleShapeSettings(halfHeight, radius);
            }
            return new JPH::BoxShapeSettings(JPH::Vec3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z));
        }
    }

    PhysicsSystem::PhysicsSystem(Core::ECS::Core::Scene& scene, JoltContext& context)
        : ISystem(scene),
        joltContext(context)
    {
        // Allocate collision group filter table with 32 sub-groups
        groupFilterTable = new JPH::GroupFilterTable(32);

        // Disable internal collisions between sub-groups of the same group
        for (uint32_t i = 0; i < 32; ++i)
        {
            for (uint32_t j = i + 1; j < 32; ++j)
            {
                groupFilterTable->DisableCollision(i, j);
            }
        }
    }

    PhysicsSystem::~PhysicsSystem()
    {
        if (joltContext.IsInitialized())
        {
            JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
            for (const auto& bodyID : createdBodies)
            {
                if (!bodyID.IsInvalid())
                {
                    if (bodyInterface.IsAdded(bodyID))
                    {
                        bodyInterface.RemoveBody(bodyID);
                    }
                    bodyInterface.DestroyBody(bodyID);
                }
            }
        }
        createdBodies.clear();
    }

    JPH::GroupFilterTable* PhysicsSystem::GetGroupFilterTable() const
    {

        return groupFilterTable.GetPtr();
    }

    void PhysicsSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("PhysicsSystem::FixedUpdate");

        if (!joltContext.IsInitialized())
        {
            return;
        }

        // 1. Instantiate physics bodies for any newly added RigidBody components
        CreatePendingBodies();

        // 2. Push transforms from ECS into Jolt kinematic bodies
        PushKinematicTransforms();

        // 3. Step the physical world
        joltContext.Step(static_cast<float>(dt));

        // 4. Synchronize simulated poses back to ECS transforms
        PullSimulatedTransforms();
    }

    void PhysicsSystem::DestroyBody(Core::ECS::Core::Entity entity)
    {
        auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
        if (!rigidBody || !rigidBody->isCreated || rigidBody->bodyID.IsInvalid())
        {
            return;
        }

        JPH::BodyID id = rigidBody->bodyID;
        if (joltContext.IsInitialized())
        {
            JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
            if (bodyInterface.IsAdded(id))
            {
                bodyInterface.RemoveBody(id);
            }
            bodyInterface.DestroyBody(id);
        }

        auto it = std::find(createdBodies.begin(), createdBodies.end(), id);
        if (it != createdBodies.end())
        {
            createdBodies.erase(it);
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
                if (collider->IsCompound())
                {
                    JPH::Ref<JPH::StaticCompoundShapeSettings> compoundSettings = new JPH::StaticCompoundShapeSettings();

                    auto primarySettings = CreatePrimitiveShapeSettings(
                        collider->shape, collider->boxHalfExtents, collider->radius, collider->halfHeight);
                    compoundSettings->AddShape(
                        JPH::Vec3(collider->localOffset.x, collider->localOffset.y, collider->localOffset.z),
                        JPH::Quat(collider->localRotation.x, collider->localRotation.y, collider->localRotation.z, collider->localRotation.w),
                        primarySettings);

                    for (const auto& sub : collider->subColliders)
                    {
                        auto subSettings = CreatePrimitiveShapeSettings(
                            sub.shape, sub.boxHalfExtents, sub.radius, sub.halfHeight);
                        compoundSettings->AddShape(
                            JPH::Vec3(sub.localOffset.x, sub.localOffset.y, sub.localOffset.z),
                            JPH::Quat(sub.localRotation.x, sub.localRotation.y, sub.localRotation.z, sub.localRotation.w),
                            subSettings);
                    }

                    auto result = compoundSettings->Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                }
                else if (rigidBody->hasCustomCenterOfMass)
                {
                    // Single shape with custom CoM: wrap with compound shape so localOffset/localRotation is anchored to link frame
                    JPH::Ref<JPH::StaticCompoundShapeSettings> compoundSettings = new JPH::StaticCompoundShapeSettings();
                    auto primarySettings = CreatePrimitiveShapeSettings(
                        collider->shape, collider->boxHalfExtents, collider->radius, collider->halfHeight);
                    compoundSettings->AddShape(
                        JPH::Vec3(collider->localOffset.x, collider->localOffset.y, collider->localOffset.z),
                        JPH::Quat(collider->localRotation.x, collider->localRotation.y, collider->localRotation.z, collider->localRotation.w),
                        primarySettings);

                    auto result = compoundSettings->Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
                    }
                }
                else
                {
                    // Legacy single shape without custom CoM
                    auto shapeSettings = CreatePrimitiveShapeSettings(
                        collider->shape, collider->boxHalfExtents, collider->radius, collider->halfHeight);
                    auto result = shapeSettings->Create();
                    if (result.IsValid())
                    {
                        shape = result.Get();
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

            // Apply Decoupled Center of Mass if specified
            if (rigidBody->hasCustomCenterOfMass)
            {
                JPH::Vec3 targetCOM(
                    rigidBody->centerOfMassOffset.x,
                    rigidBody->centerOfMassOffset.y,
                    rigidBody->centerOfMassOffset.z
                );
                JPH::Vec3 currentCOM = shape->GetCenterOfMass();
                JPH::Vec3 comShift = targetCOM - currentCOM;
                JPH::OffsetCenterOfMassShapeSettings comSettings(comShift, shape);
                auto comResult = comSettings.Create();
                if (comResult.IsValid())
                {
                    shape = comResult.Get();
                }
                else
                {
                    SIM_LOG_ERROR("Failed to offset center of mass for Entity " << entity);
                }
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

            // Determine spawn position:
            // For legacy non-compound bodies without custom CoM, apply local offset to spawn transform.
            // For compound or custom CoM bodies, the body origin aligns directly with entity transform.
            glm::vec3 spawnPos = transform->position;
            glm::quat spawnRot = transform->rotation;
            if (collider && !collider->IsCompound() && !rigidBody->hasCustomCenterOfMass)
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
            bodySettings.mMaxAngularVelocity = rigidBody->maxAngularVelocity;
            bodySettings.mGravityFactor = rigidBody->gravityFactor;
            bodySettings.mAllowSleeping = rigidBody->allowSleeping;
            bodySettings.mAllowDynamicOrKinematic = true;
            bodySettings.mIsSensor = rigidBody->isSensor;

            // Configure Jolt collision group filtering
            if (rigidBody->collisionGroupID != 0)
            {
                bodySettings.mCollisionGroup.SetGroupFilter(groupFilterTable);
                bodySettings.mCollisionGroup.SetGroupID(rigidBody->collisionGroupID);
                bodySettings.mCollisionGroup.SetSubGroupID(rigidBody->collisionSubGroupID);
            }

            if (motionType == JPH::EMotionType::Dynamic)
            {
                if (rigidBody->hasCustomInertia)
                {
                    bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
                    bodySettings.mMassPropertiesOverride.mMass = rigidBody->mass;

                    JPH::Mat44 joltInertia = JPH::Mat44::sZero();
                    for (int row = 0; row < 3; ++row)
                    {
                        for (int col = 0; col < 3; ++col)
                        {
                            joltInertia(row, col) = static_cast<float>(rigidBody->inertiaTensor[col][row]);
                        }
                    }
                    bodySettings.mMassPropertiesOverride.mInertia = joltInertia;
                }
                else
                {
                    bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
                    bodySettings.mMassPropertiesOverride.mMass = rigidBody->mass;
                }

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
            if (collider && !collider->IsCompound() && !rigidBody->hasCustomCenterOfMass)
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
        const JPH::BodyLockInterface& lockInterface = joltContext.GetPhysicsSystem().GetBodyLockInterfaceNoLock();
        auto entities = scene.Query<RigidBodyComponent, TransformComponent>();

        for (auto entity : entities)
        {
            auto* rigidBody = scene.GetComponent<RigidBodyComponent>(entity);
            if (!rigidBody || !rigidBody->isCreated || rigidBody->motionType != BodyMotionType::Dynamic)
            {
                continue;
            }

            const JPH::Body* body = lockInterface.TryGetBody(rigidBody->bodyID);
            if (!body)
            {
                continue;
            }

            auto* transform = scene.GetComponent<TransformComponent>(entity);
            auto* collider = scene.GetComponent<ColliderComponent>(entity);

            JPH::RVec3 joltPos = body->GetPosition();
            JPH::Quat joltRot = body->GetRotation();

            glm::vec3 worldPos(
                static_cast<float>(joltPos.GetX()),
                static_cast<float>(joltPos.GetY()),
                static_cast<float>(joltPos.GetZ())
            );
            glm::quat worldRot(
                joltRot.GetW(),
                joltRot.GetX(),
                joltRot.GetY(),
                joltRot.GetZ()
            );

            // Invert collider local offset if present on legacy non-compound body without custom CoM
            if (collider && !collider->IsCompound() && !rigidBody->hasCustomCenterOfMass)
            {
                worldRot = worldRot * glm::inverse(collider->localRotation);
                worldPos -= worldRot * collider->localOffset;
            }

            transform->position = worldPos;
            transform->rotation = worldRot;

            JPH::Vec3 linearVel = body->GetLinearVelocity();
            JPH::Vec3 angularVel = body->GetAngularVelocity();

            rigidBody->linearVelocity = glm::vec3(linearVel.GetX(), linearVel.GetY(), linearVel.GetZ());
            rigidBody->angularVelocity = glm::vec3(angularVel.GetX(), angularVel.GetY(), angularVel.GetZ());
        }
    }
}
