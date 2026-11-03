/**
 * @file JointSystem.cpp
 * @brief Implementation of physical joint creation, motor synchronization, and deflection measurement.
 */

#include "Physics/Systems/JointSystem.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Actuation/Components/MotorComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/Log.h"

#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <glm/geometric.hpp>

namespace FRCBananaSim::Physics::Components
{
    void JointComponent::SetWorldSpace(const glm::vec3& worldPivot, const glm::vec3& worldHingeAxis)
    {
        constraintSpace = JointConstraintSpace::WorldSpace;
        parentAnchor = worldPivot;
        childAnchor = worldPivot;
        parentAxis = glm::normalize(worldHingeAxis);
        childAxis = parentAxis;

        // Gram-Schmidt orthogonalization with right-handed basis alignment
        glm::vec3 refVector = (std::abs(parentAxis.y) < 0.9f)
            ? glm::vec3(0.0f, 1.0f, 0.0f)
            : glm::vec3(0.0f, 0.0f, 1.0f);

        parentNormal = glm::normalize(refVector - parentAxis * glm::dot(parentAxis, refVector));
        childNormal = parentNormal;
    }

    void JointComponent::SetWorldSpaceSlider(const glm::vec3& worldAnchor, const glm::vec3& worldSliderAxis,
        float minTravelM, float maxTravelM)
    {
        type = JointType::Slider;
        SetWorldSpace(worldAnchor, worldSliderAxis);
        hasLimits = true;
        minLimit = minTravelM;
        maxLimit = maxTravelM;
        hasLimitSpring = true;
        limitSpringFrequency = 40.0f;
        limitSpringDamping = 1.0f;
    }

    void JointComponent::SetLocalSpace(const glm::vec3& localPoint1, const glm::vec3& localAxis1,
        const glm::vec3& localPoint2, const glm::vec3& localAxis2)
    {
        constraintSpace = JointConstraintSpace::LocalToBodyCOM;
        parentAnchor = localPoint1;
        childAnchor = localPoint2;
        parentAxis = glm::normalize(localAxis1);
        childAxis = glm::normalize(localAxis2);

        glm::vec3 refVector1 = (std::abs(parentAxis.y) < 0.9f)
            ? glm::vec3(0.0f, 1.0f, 0.0f)
            : glm::vec3(0.0f, 0.0f, 1.0f);
        parentNormal = glm::normalize(refVector1 - parentAxis * glm::dot(parentAxis, refVector1));

        glm::vec3 refVector2 = (std::abs(childAxis.y) < 0.9f)
            ? glm::vec3(0.0f, 1.0f, 0.0f)
            : glm::vec3(0.0f, 0.0f, 1.0f);
        childNormal = glm::normalize(refVector2 - childAxis * glm::dot(childAxis, refVector2));
    }

    void JointComponent::SetLocalSpaceSlider(const glm::vec3& localPoint1, const glm::vec3& localAxis1,
        const glm::vec3& localPoint2, const glm::vec3& localAxis2,
        float minTravelM, float maxTravelM)
    {
        type = JointType::Slider;
        SetLocalSpace(localPoint1, localAxis1, localPoint2, localAxis2);
        hasLimits = true;
        minLimit = minTravelM;
        maxLimit = maxTravelM;
        hasLimitSpring = true;
        limitSpringFrequency = 40.0f;
        limitSpringDamping = 1.0f;
    }
}

namespace FRCBananaSim::Physics::Systems
{
    using Components::JointComponent;
    using Components::JointType;
    using Components::JointMotorMode;
    using Components::JointConstraintSpace;
    using Components::RigidBodyComponent;
    using Actuation::Components::MotorComponent;
    using Core::ECS::Core::invalidEntity;

    JointSystem::JointSystem(Core::ECS::Core::Scene& scene, JoltContext& context)
        : ISystem(scene),
        joltContext(context)
    {}

    JointSystem::~JointSystem()
    {
        if (joltContext.IsInitialized())
        {
            JPH::PhysicsSystem& physicsSystem = joltContext.GetPhysicsSystem();
            for (auto& constraint : createdConstraints)
            {
                if (constraint)
                {
                    physicsSystem.RemoveConstraint(constraint);
                }
            }
        }
        createdConstraints.clear();
    }

    void JointSystem::FixedUpdate(double /*dt*/)
    {
        SIM_PROFILE_SCOPE("JointSystem::FixedUpdate");

        // 1. Instantiate any pending joints whose rigid bodies are now ready in Jolt
        CreatePendingJoints();

        // 2. Synchronize motor targets and torque limits
        UpdateMotors();

        // 3. Read back live angle, angular velocity, and deflection telemetry
        ReadbackTelemetry();
    }

    void JointSystem::DestroyJoint(Core::ECS::Core::Entity entity)
    {
        auto* joint = scene.GetComponent<JointComponent>(entity);
        if (!joint || !joint->isCreated || !joint->constraint)
        {
            return;
        }

        if (joltContext.IsInitialized())
        {
            joltContext.GetPhysicsSystem().RemoveConstraint(joint->constraint);
        }

        joint->constraint = nullptr;
        joint->isCreated = false;
    }

    void JointSystem::CreatePendingJoints()
    {
        JPH::PhysicsSystem& physicsSystem = joltContext.GetPhysicsSystem();
        auto entities = scene.Query<JointComponent>();

        for (auto entity : entities)
        {
            auto* joint = scene.GetComponent<JointComponent>(entity);
            if (!joint || joint->isCreated)
            {
                continue;
            }

            // Check child entity: rigid body must exist and be instantiated in Jolt
            auto* childRigidBody = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
            if (!childRigidBody || !childRigidBody->isCreated || childRigidBody->bodyID.IsInvalid())
            {
                continue;
            }

            // Check parent entity: can be an existing rigid body, or invalidEntity for static world
            JPH::BodyID parentBodyID;
            if (joint->parentEntity != invalidEntity)
            {
                auto* parentRigidBody = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                if (!parentRigidBody || !parentRigidBody->isCreated || parentRigidBody->bodyID.IsInvalid())
                {
                    continue; // Wait until parent body is initialized
                }
                parentBodyID = parentRigidBody->bodyID;
            }

            JPH::TwoBodyConstraint* constraint = nullptr;

            if (joint->type == JointType::Slider)
            {
                // Setup Slider Constraint Settings
                JPH::SliderConstraintSettings settings;
                settings.mSpace = (joint->constraintSpace == JointConstraintSpace::WorldSpace)
                    ? JPH::EConstraintSpace::WorldSpace
                    : JPH::EConstraintSpace::LocalToBodyCOM;

                settings.mPoint1 = JPH::RVec3(joint->parentAnchor.x, joint->parentAnchor.y, joint->parentAnchor.z);
                settings.mPoint2 = JPH::RVec3(joint->childAnchor.x, joint->childAnchor.y, joint->childAnchor.z);
                settings.mSliderAxis1 = JPH::Vec3(joint->parentAxis.x, joint->parentAxis.y, joint->parentAxis.z);
                settings.mSliderAxis2 = JPH::Vec3(joint->childAxis.x, joint->childAxis.y, joint->childAxis.z);
                settings.mNormalAxis1 = JPH::Vec3(joint->parentNormal.x, joint->parentNormal.y, joint->parentNormal.z);
                settings.mNormalAxis2 = JPH::Vec3(joint->childNormal.x, joint->childNormal.y, joint->childNormal.z);

                if (joint->hasLimits)
                {
                    settings.mLimitsMin = joint->minLimit;
                    settings.mLimitsMax = joint->maxLimit;

                    if (joint->hasLimitSpring)
                    {
                        settings.mLimitsSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
                        settings.mLimitsSpringSettings.mFrequency = joint->limitSpringFrequency;
                        settings.mLimitsSpringSettings.mDamping = joint->limitSpringDamping;
                    }
                }
                else
                {
                    settings.mLimitsMin = -FLT_MAX;
                    settings.mLimitsMax = FLT_MAX;
                }

                settings.mMaxFrictionForce = joint->maxFrictionForce;
                settings.mMotorSettings.mSpringSettings.mFrequency = joint->springFrequency;
                settings.mMotorSettings.mSpringSettings.mDamping = joint->springDamping;
                settings.mMotorSettings.SetForceLimit(joint->maxMotorForce);

                // Create constraint under lock
                if (parentBodyID.IsInvalid())
                {
                    JPH::BodyLockWrite lock(physicsSystem.GetBodyLockInterface(), childRigidBody->bodyID);
                    if (lock.Succeeded())
                    {
                        constraint = settings.Create(JPH::Body::sFixedToWorld, lock.GetBody());
                    }
                }
                else
                {
                    const JPH::BodyID ids[2] = { parentBodyID, childRigidBody->bodyID };
                    JPH::BodyLockMultiWrite lock(physicsSystem.GetBodyLockInterface(), ids, 2);
                    JPH::Body* body1 = lock.GetBody(0);
                    JPH::Body* body2 = lock.GetBody(1);
                    if (body1 && body2)
                    {
                        constraint = settings.Create(*body1, *body2);
                    }
                }
            }
            else // JointType::Hinge (default)
            {
                // Setup Hinge Constraint Settings
                JPH::HingeConstraintSettings settings;
                settings.mSpace = (joint->constraintSpace == JointConstraintSpace::WorldSpace)
                    ? JPH::EConstraintSpace::WorldSpace
                    : JPH::EConstraintSpace::LocalToBodyCOM;

                settings.mPoint1 = JPH::RVec3(joint->parentAnchor.x, joint->parentAnchor.y, joint->parentAnchor.z);
                settings.mPoint2 = JPH::RVec3(joint->childAnchor.x, joint->childAnchor.y, joint->childAnchor.z);
                settings.mHingeAxis1 = JPH::Vec3(joint->parentAxis.x, joint->parentAxis.y, joint->parentAxis.z);
                settings.mHingeAxis2 = JPH::Vec3(joint->childAxis.x, joint->childAxis.y, joint->childAxis.z);
                settings.mNormalAxis1 = JPH::Vec3(joint->parentNormal.x, joint->parentNormal.y, joint->parentNormal.z);
                settings.mNormalAxis2 = JPH::Vec3(joint->childNormal.x, joint->childNormal.y, joint->childNormal.z);

                if (joint->hasLimits)
                {
                    settings.mLimitsMin = joint->minLimit;
                    settings.mLimitsMax = joint->maxLimit;

                    if (joint->hasLimitSpring)
                    {
                        settings.mLimitsSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
                        settings.mLimitsSpringSettings.mFrequency = joint->limitSpringFrequency;
                        settings.mLimitsSpringSettings.mDamping = joint->limitSpringDamping;
                    }
                }
                else
                {
                    settings.mLimitsMin = -JPH::JPH_PI;
                    settings.mLimitsMax = JPH::JPH_PI;
                }

                settings.mMaxFrictionTorque = joint->maxFrictionTorque;
                settings.mMotorSettings.mSpringSettings.mFrequency = joint->springFrequency;
                settings.mMotorSettings.mSpringSettings.mDamping = joint->springDamping;
                settings.mMotorSettings.SetTorqueLimit(joint->maxMotorTorque);

                // Create constraint under lock
                if (parentBodyID.IsInvalid())
                {
                    JPH::BodyLockWrite lock(physicsSystem.GetBodyLockInterface(), childRigidBody->bodyID);
                    if (lock.Succeeded())
                    {
                        constraint = settings.Create(JPH::Body::sFixedToWorld, lock.GetBody());
                    }
                }
                else
                {
                    const JPH::BodyID ids[2] = { parentBodyID, childRigidBody->bodyID };
                    JPH::BodyLockMultiWrite lock(physicsSystem.GetBodyLockInterface(), ids, 2);
                    JPH::Body* body1 = lock.GetBody(0);
                    JPH::Body* body2 = lock.GetBody(1);
                    if (body1 && body2)
                    {
                        constraint = settings.Create(*body1, *body2);
                    }
                }
            }

            if (!constraint)
            {
                SIM_LOG_ERROR("Failed to allocate Jolt constraint for Entity " << entity);
                continue;
            }

            physicsSystem.AddConstraint(constraint);
            joint->constraint = constraint;
            joint->isCreated = true;
            createdConstraints.push_back(constraint);
        }
    }

    void JointSystem::UpdateMotors()
    {
        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        auto entities = scene.Query<JointComponent>();

        for (auto entity : entities)
        {
            auto* joint = scene.GetComponent<JointComponent>(entity);
            if (!joint || !joint->isCreated || !joint->constraint)
            {
                continue;
            }

            // Wake up child body if motor is active so Jolt evaluates the drive
            if (joint->motorMode != JointMotorMode::Off)
            {
                auto* childRigidBody = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
                if (childRigidBody && childRigidBody->isCreated && !childRigidBody->bodyID.IsInvalid())
                {
                    bodyInterface.ActivateBody(childRigidBody->bodyID);
                }
            }

            if (joint->type == JointType::Slider)
            {
                auto* slider = static_cast<JPH::SliderConstraint*>(joint->constraint.GetPtr());

                switch (joint->motorMode)
                {
                case JointMotorMode::Off:
                    slider->SetMotorState(JPH::EMotorState::Off);
                    break;

                case JointMotorMode::Velocity:
                    slider->SetMotorState(JPH::EMotorState::Velocity);
                    slider->SetTargetVelocity(joint->targetVelocity);
                    slider->GetMotorSettings().SetForceLimit(joint->maxMotorForce);
                    break;

                case JointMotorMode::Position:
                    if (std::abs(joint->targetVelocity) > 1e-4f)
                    {
                        slider->SetMotorState(JPH::EMotorState::PositionAndVelocity);
                        slider->SetTargetVelocity(joint->targetVelocity);
                    }
                    else
                    {
                        slider->SetMotorState(JPH::EMotorState::Position);
                    }
                    slider->SetTargetPosition(joint->targetPosition);
                    slider->GetMotorSettings().SetForceLimit(joint->maxMotorForce);
                    slider->GetMotorSettings().mSpringSettings.mFrequency = joint->springFrequency;
                    slider->GetMotorSettings().mSpringSettings.mDamping = joint->springDamping;

                    if (std::abs(joint->appliedForce) > 1e-5f)
                    {
                        auto* childRb = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
                        if (childRb && childRb->isCreated && !childRb->bodyID.IsInvalid())
                        {
                            JPH::RMat44 com2 = bodyInterface.GetCenterOfMassTransform(childRb->bodyID);
                            JPH::Vec3 worldSliderAxis = com2.Multiply3x3(JPH::Vec3(joint->childAxis.x, joint->childAxis.y, joint->childAxis.z)).Normalized();
                            JPH::Vec3 forceVec = worldSliderAxis * joint->appliedForce;

                            bodyInterface.AddForce(childRb->bodyID, forceVec);
                            bodyInterface.ActivateBody(childRb->bodyID);

                            if (joint->parentEntity != invalidEntity)
                            {
                                auto* parentRb = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                                if (parentRb && parentRb->isCreated && !parentRb->bodyID.IsInvalid())
                                {
                                    bodyInterface.AddForce(parentRb->bodyID, -forceVec);
                                    bodyInterface.ActivateBody(parentRb->bodyID);
                                }
                            }
                        }
                    }
                    break;

                case JointMotorMode::Torque:
                    slider->SetMotorState(JPH::EMotorState::Off);
                    if (std::abs(joint->appliedForce) > 1e-5f)
                    {
                        auto* childRb = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
                        if (childRb && childRb->isCreated && !childRb->bodyID.IsInvalid())
                        {
                            JPH::RMat44 com2 = bodyInterface.GetCenterOfMassTransform(childRb->bodyID);
                            JPH::Vec3 worldSliderAxis = com2.Multiply3x3(JPH::Vec3(joint->childAxis.x, joint->childAxis.y, joint->childAxis.z)).Normalized();
                            JPH::Vec3 forceVec = worldSliderAxis * joint->appliedForce;

                            bodyInterface.AddForce(childRb->bodyID, forceVec);
                            bodyInterface.ActivateBody(childRb->bodyID);

                            if (joint->parentEntity != invalidEntity)
                            {
                                auto* parentRb = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                                if (parentRb && parentRb->isCreated && !parentRb->bodyID.IsInvalid())
                                {
                                    bodyInterface.AddForce(parentRb->bodyID, -forceVec);
                                    bodyInterface.ActivateBody(parentRb->bodyID);
                                }
                            }
                        }
                    }
                    break;
                }
            }
            else // JointType::Hinge
            {
                auto* hinge = static_cast<JPH::HingeConstraint*>(joint->constraint.GetPtr());

                // If an Actuation::MotorComponent exists on the same entity, sync torque limits
                auto* motorComp = scene.GetComponent<MotorComponent>(entity);
                if (motorComp)
                {
                    joint->maxMotorTorque = static_cast<float>(std::abs(motorComp->outputTorque));
                }

                switch (joint->motorMode)
                {
                case JointMotorMode::Off:
                    hinge->SetMotorState(JPH::EMotorState::Off);
                    break;

                case JointMotorMode::Velocity:
                    hinge->SetMotorState(JPH::EMotorState::Velocity);
                    hinge->SetTargetAngularVelocity(joint->targetVelocity);
                    hinge->GetMotorSettings().SetTorqueLimit(joint->maxMotorTorque);
                    break;

                case JointMotorMode::Position:
                    hinge->SetMotorState(JPH::EMotorState::Position);
                    hinge->SetTargetAngle(joint->targetAngle);
                    hinge->GetMotorSettings().SetTorqueLimit(joint->maxMotorTorque);
                    hinge->GetMotorSettings().mSpringSettings.mFrequency = joint->springFrequency;
                    hinge->GetMotorSettings().mSpringSettings.mDamping = joint->springDamping;
                    break;

                case JointMotorMode::Torque:
                    hinge->SetMotorState(JPH::EMotorState::Off);
                    if (std::abs(joint->appliedTorque) > 1e-5f)
                    {
                        auto* childRb = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
                        if (childRb && childRb->isCreated && !childRb->bodyID.IsInvalid())
                        {
                            JPH::RMat44 com2 = bodyInterface.GetCenterOfMassTransform(childRb->bodyID);
                            JPH::Vec3 worldHingeAxis = com2.Multiply3x3(hinge->GetLocalSpaceHingeAxis2()).Normalized();
                            JPH::Vec3 torqueVec = worldHingeAxis * joint->appliedTorque;

                            bodyInterface.AddTorque(childRb->bodyID, torqueVec);
                            bodyInterface.ActivateBody(childRb->bodyID);

                            if (joint->parentEntity != invalidEntity)
                            {
                                auto* parentRb = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                                if (parentRb && parentRb->isCreated && !parentRb->bodyID.IsInvalid())
                                {
                                    bodyInterface.AddTorque(parentRb->bodyID, -torqueVec);
                                    bodyInterface.ActivateBody(parentRb->bodyID);
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    void JointSystem::ReadbackTelemetry()
    {
        JPH::BodyInterface& bodyInterface = joltContext.GetBodyInterface();
        auto entities = scene.Query<JointComponent>();

        for (auto entity : entities)
        {
            auto* joint = scene.GetComponent<JointComponent>(entity);
            if (!joint || !joint->isCreated || !joint->constraint)
            {
                continue;
            }

            auto* childRigidBody = scene.GetComponent<RigidBodyComponent>(joint->childEntity);
            if (!childRigidBody || childRigidBody->bodyID.IsInvalid())
            {
                continue;
            }

            JPH::RMat44 com2 = bodyInterface.GetCenterOfMassTransform(childRigidBody->bodyID);

            if (joint->type == JointType::Slider)
            {
                auto* slider = static_cast<JPH::SliderConstraint*>(joint->constraint.GetPtr());

                // Read current linear slider travel
                joint->currentPosition = slider->GetCurrentPosition();
                joint->currentAngle = joint->currentPosition;

                // Read child and parent linear velocities
                JPH::Vec3 v2 = bodyInterface.GetLinearVelocity(childRigidBody->bodyID);
                JPH::Vec3 v1 = JPH::Vec3::sZero();
                if (joint->parentEntity != invalidEntity)
                {
                    auto* parentRigidBody = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                    if (parentRigidBody && !parentRigidBody->bodyID.IsInvalid())
                    {
                        v1 = bodyInterface.GetLinearVelocity(parentRigidBody->bodyID);
                    }
                }

                // Transform local slider axis to world space
                JPH::Vec3 worldSliderAxis = com2.Multiply3x3(JPH::Vec3(joint->childAxis.x, joint->childAxis.y, joint->childAxis.z)).Normalized();
                joint->currentVelocity = static_cast<float>((v2 - v1).Dot(worldSliderAxis));

                // Deflection calculation: distance between anchor 1 and anchor 2 perpendicular to slider axis
                JPH::RMat44 com1 = JPH::RMat44::sIdentity();
                if (joint->parentEntity != invalidEntity)
                {
                    auto* parentRigidBody = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                    if (parentRigidBody && !parentRigidBody->bodyID.IsInvalid())
                    {
                        com1 = bodyInterface.GetCenterOfMassTransform(parentRigidBody->bodyID);
                    }
                }

                JPH::RVec3 worldPoint1 = com1 * JPH::Vec3(joint->parentAnchor.x, joint->parentAnchor.y, joint->parentAnchor.z);
                JPH::RVec3 worldPoint2 = com2 * JPH::Vec3(joint->childAnchor.x, joint->childAnchor.y, joint->childAnchor.z);
                JPH::Vec3 diff = JPH::Vec3(worldPoint1 - worldPoint2);
                JPH::Vec3 parallel = worldSliderAxis * diff.Dot(worldSliderAxis);
                joint->currentDeflection = static_cast<float>((diff - parallel).Length());
            }
            else // JointType::Hinge
            {
                auto* hinge = static_cast<JPH::HingeConstraint*>(joint->constraint.GetPtr());

                // Relative rotation angle
                joint->currentAngle = hinge->GetCurrentAngle();
                joint->currentPosition = joint->currentAngle;

                // Transform local hinge axis into world coordinates
                JPH::Vec3 worldHingeAxis = com2.Multiply3x3(hinge->GetLocalSpaceHingeAxis2()).Normalized();

                // Read dynamic angular velocities
                JPH::Vec3 omega2 = bodyInterface.GetAngularVelocity(childRigidBody->bodyID);
                JPH::Vec3 omega1 = JPH::Vec3::sZero();
                if (joint->parentEntity != invalidEntity)
                {
                    auto* parentRigidBody = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                    if (parentRigidBody && !parentRigidBody->bodyID.IsInvalid())
                    {
                        omega1 = bodyInterface.GetAngularVelocity(parentRigidBody->bodyID);
                    }
                }

                // Project relative rotational velocity onto the hinge axis
                joint->currentVelocity = static_cast<float>((omega2 - omega1).Dot(worldHingeAxis));

                // Deflection calculation: distance between anchor 1 and anchor 2 in world coordinates
                JPH::RMat44 com1 = JPH::RMat44::sIdentity();
                if (joint->parentEntity != invalidEntity)
                {
                    auto* parentRigidBody = scene.GetComponent<RigidBodyComponent>(joint->parentEntity);
                    if (parentRigidBody && !parentRigidBody->bodyID.IsInvalid())
                    {
                        com1 = bodyInterface.GetCenterOfMassTransform(parentRigidBody->bodyID);
                    }
                }

                JPH::RVec3 worldPoint1 = com1 * hinge->GetLocalSpacePoint1();
                JPH::RVec3 worldPoint2 = com2 * hinge->GetLocalSpacePoint2();

                joint->currentDeflection = static_cast<float>((worldPoint1 - worldPoint2).Length());

                // If an Actuation::MotorComponent exists on the entity, forward kinematic readback
                auto* motorComp = scene.GetComponent<MotorComponent>(entity);
                if (motorComp)
                {
                    motorComp->angularPosition = joint->currentAngle;
                    motorComp->angularVelocity = joint->currentVelocity;
                }
            }
        }
    }
}