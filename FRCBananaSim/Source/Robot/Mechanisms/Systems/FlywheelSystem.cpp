/**
 * @file FlywheelSystem.cpp
 * @brief Implementation of 1D semi-implicit flywheel integration, Newton's third law recoil, and visual sync.
 */

#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Core/ECS/Core/Scene.h"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    using namespace Components;
    using Physics::Components::RigidBodyComponent;
    using Physics::Components::ColliderComponent;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Components::WorldTransformComponent;
    using Core::ECS::Components::HierarchyComponent;
    using Core::ECS::Core::invalidEntity;

    FlywheelSystem::FlywheelSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
        joltContext(jolt)
    {}

    FlywheelSystem::~FlywheelSystem()
    {}

    void FlywheelSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("FlywheelSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        auto entities = scene.Query<FlywheelComponent>();
        for (auto entity : entities)
        {
            auto* flywheel = scene.GetComponent<FlywheelComponent>(entity);
            if (flywheel)
            {
                UpdateFlywheel(*flywheel, dt);
            }
        }
    }

    void FlywheelSystem::RenderUpdate()
    {
        SIM_PROFILE_SCOPE("FlywheelSystem::RenderUpdate");

        auto entities = scene.Query<FlywheelComponent>();
        for (auto entity : entities)
        {
            auto* flywheel = scene.GetComponent<FlywheelComponent>(entity);
            if (!flywheel || flywheel->visualEntity == invalidEntity)
            {
                continue;
            }

            // Rotate visual mesh about spin axis by accumulated angular position
            glm::vec3 axis = glm::normalize(flywheel->spinAxis);
            glm::quat spinQuat = glm::angleAxis(static_cast<float>(flywheel->angularPositionRad), axis);

            if (flywheel->parentEntity != invalidEntity)
            {
                auto* parentT = scene.GetComponent<TransformComponent>(flywheel->parentEntity);
                if (parentT)
                {
                    glm::vec3 worldPos = parentT->position + (parentT->rotation * flywheel->visualLocalOffset);
                    glm::quat worldRot = glm::normalize(parentT->rotation * spinQuat);

                    auto* t = scene.GetComponent<TransformComponent>(flywheel->visualEntity);
                    if (t)
                    {
                        t->position = worldPos;
                        t->rotation = worldRot;
                    }

                    auto* wt = scene.GetComponent<WorldTransformComponent>(flywheel->visualEntity);
                    if (wt)
                    {
                        wt->worldPosition = worldPos;
                        wt->worldRotation = worldRot;
                    }

                    auto* h = scene.GetComponent<HierarchyComponent>(flywheel->visualEntity);
                    if (h)
                    {
                        for (auto child : h->children)
                        {
                            auto* ct = scene.GetComponent<TransformComponent>(child);
                            auto* cwt = scene.GetComponent<WorldTransformComponent>(child);
                            if (ct && cwt)
                            {
                                cwt->worldPosition = worldPos + (worldRot * ct->position);
                                cwt->worldRotation = glm::normalize(worldRot * ct->rotation);
                                cwt->worldScale = ct->scale;
                            }
                        }
                    }
                }
            }
            else
            {
                auto* t = scene.GetComponent<TransformComponent>(flywheel->visualEntity);
                if (t)
                {
                    t->rotation = spinQuat;
                }

                auto* wt = scene.GetComponent<WorldTransformComponent>(flywheel->visualEntity);
                if (wt)
                {
                    wt->worldRotation = spinQuat;
                }
            }
        }
    }

    void FlywheelSystem::UpdateFlywheel(FlywheelComponent& flywheel, double dt)
    {
        // 1. Motor Scaling & Reflected Rotor Inertia
        Actuation::DCMotorModel baseMotor = flywheel.motorModel;
        Actuation::DCMotorModel motor(
            baseMotor.GetNominalVoltage(),
            baseMotor.GetStallTorque(),
            baseMotor.GetStallCurrent(),
            baseMotor.GetFreeSpeed(),
            baseMotor.GetFreeCurrent(),
            baseMotor.GetRotorInertia(),
            flywheel.gearRatio,
            flywheel.efficiency
        );
        motor = motor.WithMotorCount(flywheel.motorCount);

        double gRatio = flywheel.gearRatio;
        flywheel.reflectedRotorInertiaKgm2 = motor.GetRotorInertia() * gRatio * gRatio;

        double rollerReflected = flywheel.rollerInertiaKgm2 * flywheel.rollerRatio * flywheel.rollerRatio;
        double totalInertia = flywheel.flywheelInertiaKgm2 + rollerReflected + flywheel.reflectedRotorInertiaKgm2;
        flywheel.totalRotationalInertiaKgm2 = std::max(totalInertia, 1e-6);

        // 2. Commanded Torque & Electrical Damping
        double appliedV = flywheel.commandedVoltage;
        auto affine = motor.CalculateAffineTorqueTerms(appliedV);
        double tauCmd = affine.stallTorqueCommanded;

        // In coast mode with zero voltage, back-EMF is open-circuited
        double dRot = (flywheel.brakeMode || std::abs(appliedV) > 1e-3) ? affine.electricalDamping : 0.0;
        double dTotal = dRot + flywheel.viscousDampingNms;

        // 3. Stator Current Limit Clamp on Commanded Torque
        double maxStatorTorque = motor.CalculateStatorTorqueLimit(flywheel.statorCurrentLimitA);
        tauCmd = std::clamp(tauCmd, -maxStatorTorque, maxStatorTorque);

        // 4. Sub-Stepped 1D Semi-Implicit Integration
        uint32_t nSub = std::clamp(flywheel.subSteps, 1u, 16u);
        double subDt = dt / static_cast<double>(nSub);

        double initialOmega = flywheel.angularVelocityRadPerSec;
        double currentOmega = initialOmega;
        double currentTheta = flywheel.angularPositionRad;
        double jTotal = flywheel.totalRotationalInertiaKgm2;

        for (uint32_t s = 0; s < nSub; ++s)
        {
            double denominator = 1.0 + ((dTotal * subDt) / jTotal);
            double netDrivingTorque = tauCmd - flywheel.externalLoadTorqueNm;

            // Semi-implicit velocity step:
            // omega^{t+subDt} = (omega^t + (netDrivingTorque * subDt) / J) / (1 + (dTotal * subDt) / J)
            currentOmega = (currentOmega + ((netDrivingTorque * subDt) / jTotal)) / denominator;

            // 1D Coulomb bearing friction step
            if (flywheel.coulombFrictionNm > 1e-6)
            {
                double frictionDeltaOmega = (flywheel.coulombFrictionNm * subDt) / jTotal;
                if (std::abs(currentOmega) <= frictionDeltaOmega)
                {
                    currentOmega = 0.0;
                }
                else
                {
                    currentOmega -= (currentOmega > 0.0 ? 1.0 : -1.0) * frictionDeltaOmega;
                }
            }

            // Angular position integration
            currentTheta += currentOmega * subDt;
        }

        // 5. State & Telemetry Updates
        flywheel.angularVelocityRadPerSec = currentOmega;
        constexpr double radPerSecToRpm = 60.0 / (2.0 * 3.14159265358979323846);
        flywheel.angularVelocityRpm = currentOmega * radPerSecToRpm;
        flywheel.angularPositionRad = currentTheta;
        flywheel.appliedVoltage = appliedV;
        flywheel.motorTorqueNm = tauCmd;
        flywheel.statorCurrentA = motor.CalculateCurrent(appliedV, currentOmega);

        // 6. Newton's Third Law Recoil Torque Applied to Parent Rigid Body
        double deltaOmega = currentOmega - initialOmega;
        double accelTorque = jTotal * (deltaOmega / dt);
        double recoilTorque = -accelTorque;
        flywheel.recoilTorqueNm = recoilTorque;

        if (flywheel.parentEntity != invalidEntity && joltContext && joltContext->IsInitialized())
        {
            auto* parentRb = scene.GetComponent<RigidBodyComponent>(flywheel.parentEntity);
            if (parentRb && !parentRb->bodyID.IsInvalid() && std::abs(recoilTorque) > 1e-4)
            {
                JPH::BodyInterface& bi = joltContext->GetBodyInterface();
                JPH::RMat44 com = bi.GetCenterOfMassTransform(parentRb->bodyID);

                // Transform spin axis from parent local space to world space
                glm::vec3 localAxis = glm::normalize(flywheel.spinAxis);
                JPH::Vec3 worldAxis = com.Multiply3x3(JPH::Vec3(localAxis.x, localAxis.y, localAxis.z)).Normalized();
                JPH::Vec3 recoilTorqueVec = worldAxis * static_cast<float>(recoilTorque);

                bi.AddTorque(parentRb->bodyID, recoilTorqueVec);
                bi.ActivateBody(parentRb->bodyID);
            }
        }

        // 7. Synchronize Visual Link Pose with Parent Body
        if (flywheel.visualEntity != invalidEntity && flywheel.parentEntity != invalidEntity)
        {
            glm::vec3 parentPos{ 0.0f };
            glm::quat parentRot{ 1.0f, 0.0f, 0.0f, 0.0f };
            bool hasParentPose = false;

            if (joltContext && joltContext->IsInitialized())
            {
                auto* parentRb = scene.GetComponent<RigidBodyComponent>(flywheel.parentEntity);
                if (parentRb && !parentRb->bodyID.IsInvalid())
                {
                    JPH::BodyInterface& bi = joltContext->GetBodyInterface();
                    JPH::RVec3 jPos = bi.GetPosition(parentRb->bodyID);
                    JPH::Quat jRot = bi.GetRotation(parentRb->bodyID);
                    parentPos = glm::vec3(static_cast<float>(jPos.GetX()), static_cast<float>(jPos.GetY()), static_cast<float>(jPos.GetZ()));
                    parentRot = glm::quat(jRot.GetW(), jRot.GetX(), jRot.GetY(), jRot.GetZ());

                    auto* parentCol = scene.GetComponent<ColliderComponent>(flywheel.parentEntity);
                    if (parentCol)
                    {
                        parentRot = glm::normalize(parentRot * glm::inverse(parentCol->localRotation));
                        parentPos -= parentRot * parentCol->localOffset;
                    }
                    hasParentPose = true;
                }
            }

            if (!hasParentPose)
            {
                auto* parentT = scene.GetComponent<TransformComponent>(flywheel.parentEntity);
                if (parentT)
                {
                    parentPos = parentT->position;
                    parentRot = parentT->rotation;
                    hasParentPose = true;
                }
            }

            if (hasParentPose)
            {
                glm::vec3 axis = glm::normalize(flywheel.spinAxis);
                glm::quat spinQuat = glm::angleAxis(static_cast<float>(flywheel.angularPositionRad), axis);
                glm::vec3 worldPos = parentPos + (parentRot * flywheel.visualLocalOffset);
                glm::quat worldRot = glm::normalize(parentRot * spinQuat);

                auto* t = scene.GetComponent<TransformComponent>(flywheel.visualEntity);
                if (t)
                {
                    t->position = worldPos;
                    t->rotation = worldRot;
                }

                auto* wt = scene.GetComponent<WorldTransformComponent>(flywheel.visualEntity);
                if (wt)
                {
                    wt->worldPosition = worldPos;
                    wt->worldRotation = worldRot;
                }

                if (joltContext && joltContext->IsInitialized())
                {
                    auto* visRb = scene.GetComponent<RigidBodyComponent>(flywheel.visualEntity);
                    if (visRb && !visRb->bodyID.IsInvalid())
                    {
                        JPH::BodyInterface& bi = joltContext->GetBodyInterface();
                        bi.SetPositionAndRotation(
                            visRb->bodyID,
                            JPH::RVec3(worldPos.x, worldPos.y, worldPos.z),
                            JPH::Quat(worldRot.x, worldRot.y, worldRot.z, worldRot.w),
                            JPH::EActivation::DontActivate
                        );
                        bi.SetLinearAndAngularVelocity(visRb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
                    }
                }
            }
        }
    }
}