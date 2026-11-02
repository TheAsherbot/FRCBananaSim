/**
 * @file CompositeMassSystem.cpp
 * @brief Implementation of composite mass aggregation and Parallel Axis Theorem inertia calculations.
 */

#include "Robot/Mechanisms/Systems/CompositeMassSystem.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    using namespace Components;
    using Physics::Components::RigidBodyComponent;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Components::WorldTransformComponent;
    using Core::ECS::Core::invalidEntity;
    using Robot::Schema::RobotSchemaLoader;

    CompositeMassSystem::CompositeMassSystem(Core::ECS::Core::Scene& scene,
        Robot::Drivetrain::Model::SwerveDrivetrainModel* swerveModel,
        Physics::JoltContext* jolt)
        : ISystem(scene),
        swerveModel(swerveModel),
        joltContext(jolt)
    {}

    CompositeMassSystem::~CompositeMassSystem()
    {}

    void CompositeMassSystem::SetSwerveModel(Robot::Drivetrain::Model::SwerveDrivetrainModel* model)
    {
        swerveModel = model;
    }

    void CompositeMassSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("CompositeMassSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        auto entities = scene.Query<CompositeMassComponent>();
        for (auto entity : entities)
        {
            auto* comp = scene.GetComponent<CompositeMassComponent>(entity);
            if (comp)
            {
                UpdateAssembly(*comp);
            }
        }
    }

    void CompositeMassSystem::UpdateAssembly(CompositeMassComponent& comp)
    {
        if (comp.chassisEntity == invalidEntity)
        {
            return;
        }

        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(comp.chassisEntity);
        if (!chassisRb)
        {
            return;
        }

        // Retrieve chassis world position and orientation
        glm::vec3 chassisPos{ 0.0f };
        glm::quat chassisRot{ 1.0f, 0.0f, 0.0f, 0.0f };

        if (joltContext && joltContext->IsInitialized() && !chassisRb->bodyID.IsInvalid())
        {
            JPH::BodyInterface& bi = joltContext->GetBodyInterface();
            JPH::RVec3 p = bi.GetPosition(chassisRb->bodyID);
            JPH::Quat q = bi.GetRotation(chassisRb->bodyID);
            chassisPos = glm::vec3(static_cast<float>(p.GetX()), static_cast<float>(p.GetY()), static_cast<float>(p.GetZ()));
            chassisRot = glm::quat(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
        }
        else
        {
            auto* chassisWt = scene.GetComponent<WorldTransformComponent>(comp.chassisEntity);
            if (chassisWt)
            {
                chassisPos = chassisWt->worldPosition;
                chassisRot = chassisWt->worldRotation;
            }
            else
            {
                auto* chassisT = scene.GetComponent<TransformComponent>(comp.chassisEntity);
                if (chassisT)
                {
                    chassisPos = chassisT->position;
                    chassisRot = chassisT->rotation;
                }
            }
        }

        glm::mat3 chassisRotInv = glm::transpose(glm::mat3_cast(chassisRot));

        // 1. Accumulate Total Mass and First Moment of Mass (Center of Mass in Chassis Frame)
        double totalMass = std::max(static_cast<double>(chassisRb->mass), 0.1);
        glm::dvec3 firstMoment(0.0); // Chassis base link COM is origin in its own frame

        struct LinkContribution
        {
            double mass = 0.0;
            glm::vec3 posRelChassis{ 0.0f };
            glm::mat3 rotRelChassis{ 1.0f };
            glm::mat3 localInertia{ 0.0f };
        };

        std::vector<LinkContribution> contributions;
        contributions.reserve(comp.mechanismLinks.size() + 1);

        // Add chassis link contribution (link 0)
        LinkContribution chassisContrib;
        chassisContrib.mass = chassisRb->mass;
        chassisContrib.posRelChassis = glm::vec3(0.0f);
        chassisContrib.rotRelChassis = glm::mat3(1.0f);
        chassisContrib.localInertia = chassisRb->hasCustomInertia ? chassisRb->inertiaTensor : glm::mat3(1.0f);
        contributions.push_back(chassisContrib);

        for (auto linkEntity : comp.mechanismLinks)
        {
            if (linkEntity == invalidEntity)
            {
                continue;
            }

            auto* linkRb = scene.GetComponent<RigidBodyComponent>(linkEntity);
            if (!linkRb)
            {
                continue;
            }

            glm::vec3 linkPos{ 0.0f };
            glm::quat linkRot{ 1.0f, 0.0f, 0.0f, 0.0f };

            if (joltContext && joltContext->IsInitialized() && !linkRb->bodyID.IsInvalid())
            {
                JPH::BodyInterface& bi = joltContext->GetBodyInterface();
                JPH::RVec3 p = bi.GetPosition(linkRb->bodyID);
                JPH::Quat q = bi.GetRotation(linkRb->bodyID);
                linkPos = glm::vec3(static_cast<float>(p.GetX()), static_cast<float>(p.GetY()), static_cast<float>(p.GetZ()));
                linkRot = glm::quat(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
            }
            else
            {
                auto* linkWt = scene.GetComponent<WorldTransformComponent>(linkEntity);
                if (linkWt)
                {
                    linkPos = linkWt->worldPosition;
                    linkRot = linkWt->worldRotation;
                }
                else
                {
                    auto* linkT = scene.GetComponent<TransformComponent>(linkEntity);
                    if (linkT)
                    {
                        linkPos = linkT->position;
                        linkRot = linkT->rotation;
                    }
                }
            }

            // Express link transform relative to chassis base frame
            glm::vec3 relPos = chassisRotInv * (linkPos - chassisPos);
            glm::mat3 relRot = chassisRotInv * glm::mat3_cast(linkRot);

            LinkContribution contrib;
            contrib.mass = static_cast<double>(linkRb->mass);
            contrib.posRelChassis = relPos;
            contrib.rotRelChassis = relRot;
            contrib.localInertia = linkRb->hasCustomInertia ? linkRb->inertiaTensor : glm::mat3(0.05f * static_cast<float>(contrib.mass));

            contributions.push_back(contrib);

            totalMass += contrib.mass;
            firstMoment += static_cast<double>(contrib.mass) * glm::dvec3(relPos);
        }

        glm::vec3 compositeCgEngine = glm::vec3(firstMoment / totalMass);

        // 2. Parallel Axis Theorem: Sum Rotated Link Inertias and Stepping Tensors
        glm::dmat3 compositeInertiaEngine(0.0);

        for (const auto& item : contributions)
        {
            // Rotated local inertia about link's own COM
            glm::dmat3 rMat = glm::dmat3(item.rotRelChassis);
            glm::dmat3 iRotated = rMat * glm::dmat3(item.localInertia) * glm::transpose(rMat);

            // Lever arm from composite CG to link COM
            glm::dvec3 d = glm::dvec3(item.posRelChassis - compositeCgEngine);

            // Parallel Axis offset tensor: Delta I = m * (||d||^2 * I_3 - d (x) d)
            double dSq = glm::dot(d, d);
            glm::dmat3 iParallel(
                item.mass * (dSq - d.x * d.x), -item.mass * d.x * d.y, -item.mass * d.x * d.z,
                -item.mass * d.x * d.y, item.mass * (dSq - d.y * d.y), -item.mass * d.y * d.z,
                -item.mass * d.x * d.z, -item.mass * d.y * d.z, item.mass * (dSq - d.z * d.z)
            );

            compositeInertiaEngine += iRotated + iParallel;
        }

        // Store Engine frame results
        comp.totalMassKg = totalMass;
        comp.compositeCenterOfMassEngine = compositeCgEngine;
        comp.compositeInertiaTensorEngine = glm::mat3(compositeInertiaEngine);

        // Convert Engine frame results to WPILib NWU frame for drivetrain solver handoff
        comp.compositeCenterOfMassNWU = RobotSchemaLoader::ConvertEngineVectorToNWU(compositeCgEngine);
        comp.compositeInertiaTensorNWU = glm::mat3(RobotSchemaLoader::ConvertEngineInertiaToNWU(compositeInertiaEngine));

        // 3. Update Swerve Model if registered
        if (swerveModel)
        {
            swerveModel->SetMassProperties(comp.totalMassKg, comp.compositeInertiaTensorNWU, comp.compositeCenterOfMassNWU);
        }
    }
}