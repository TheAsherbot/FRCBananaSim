/**
 * @file SwerveDriveSystem.cpp
 * @brief Implementation of the 6-DOF swerve drivetrain simulation system adapter.
 */

#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Telemetry/TelemetryMacros.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::System
{
    using namespace Control;
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Drivetrain::Model;
    using namespace Drivetrain::Components;
    using namespace Physics::Components;
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Core::Units::Literals;

    SwerveDriveSystem::SwerveDriveSystem(Scene& scene, Physics::JoltContext* joltContext)
        : ISystem(scene),
        joltContext(joltContext)
    {
        if (joltContext && joltContext->IsInitialized())
        {
            defaultProbe = std::make_unique<Systems::SwerveTerrainProbe>(*joltContext);
            activeTerrainProvider = std::shared_ptr<Model::ITerrainProvider>(defaultProbe.get(), [](Model::ITerrainProvider*) {});
        }
    }

    void SwerveDriveSystem::SetInputs(const SimInputs& inputs)
    {
        currentInputs = inputs;
    }

    const SimOutputs& SwerveDriveSystem::GetOutputs() const
    {
        return currentOutputs;
    }

    SimOutputs& SwerveDriveSystem::GetOutputs()
    {
        return currentOutputs;
    }

    void SwerveDriveSystem::SetCustomTerrainProvider(std::shared_ptr<Model::ITerrainProvider> provider)
    {
        activeTerrainProvider = provider;
    }

    SwerveDrivetrainModel* SwerveDriveSystem::GetModel()
    {
        auto chasses = scene.Query<SwerveChassisComponent>();
        if (chasses.empty())
        {
            return nullptr;
        }

        auto* chassis = scene.GetComponent<SwerveChassisComponent>(chasses[0]);
        return chassis ? chassis->drivetrainModel.get() : nullptr;
    }

    void SwerveDriveSystem::InitializeModel()
    {
        auto chasses = scene.Query<SwerveChassisComponent, RigidBodyComponent, TransformComponent>();
        if (chasses.empty())
        {
            return;
        }

        Entity chassisEntity = chasses[0];
        auto* chassisComp = scene.GetComponent<SwerveChassisComponent>(chassisEntity);
        auto* chassisTransform = scene.GetComponent<TransformComponent>(chassisEntity);
        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(chassisEntity);

        if (!chassisComp || chassisComp->drivetrainModel)
        {
            return;
        }

        // Create standard MK5n R3 configuration
        SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
        config.chassis.mass = chassisComp->chassisMass;

        // Ingest initial pose from ECS transform (+X Right, +Y Up, +Z Forward -> NWU: X Fwd, Y Left, Z Up)
        glm::vec3 posNWU(chassisTransform->position.z, -chassisTransform->position.x, chassisTransform->position.y);
        glm::quat q = chassisTransform->rotation;
        glm::quat rotNWU(q.w, q.z, -q.x, q.y);

        BatteryModel battery = BatteryModel::FRCStandard();
        if (chassisComp->batteryEntity != invalidEntity)
        {
            auto* batComp = scene.GetComponent<BatteryComponent>(chassisComp->batteryEntity);
            if (batComp)
            {
                battery = batComp->batteryModel;
            }
        }

        chassisComp->drivetrainModel = std::make_shared<SwerveDrivetrainModel>(config, battery);
        chassisComp->drivetrainModel->GetChassisState().position = posNWU;
        chassisComp->drivetrainModel->GetChassisState().orientation = rotNWU;

        if (defaultProbe && chassisRb)
        {
            defaultProbe->SetIgnoredBody(chassisRb->bodyID);
        }

        isInitialized = true;
    }

    void SwerveDriveSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("SwerveDriveSystem::FixedUpdate");

        if (!isInitialized)
        {
            InitializeModel();
        }

        auto chasses = scene.Query<SwerveChassisComponent, TransformComponent, RigidBodyComponent>();
        if (chasses.empty())
        {
            return;
        }

        Entity chassisEntity = chasses[0];
        auto* chassis = scene.GetComponent<SwerveChassisComponent>(chassisEntity);
        auto* chassisTransform = scene.GetComponent<TransformComponent>(chassisEntity);
        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(chassisEntity);

        if (!chassis || !chassis->drivetrainModel || !activeTerrainProvider)
        {
            return;
        }

        // 1. Ingest post-Jolt state and restore velocity offset: V = v_Jolt + Delta
        if (joltContext && joltContext->IsInitialized() && chassisRb && !chassisRb->bodyID.IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = joltContext->GetBodyInterface();
            JPH::RVec3 joltPos = bodyInterface.GetPosition(chassisRb->bodyID);
            JPH::Quat joltRot = bodyInterface.GetRotation(chassisRb->bodyID);
            JPH::Vec3 joltLinVel = bodyInterface.GetLinearVelocity(chassisRb->bodyID);
            JPH::Vec3 joltAngVel = bodyInterface.GetAngularVelocity(chassisRb->bodyID);

            // Convert Jolt Engine frame (+X Right, +Y Up, +Z Forward) to NWU (+X Fwd, +Y Left, +Z Up)
            glm::vec3 posNWU(static_cast<float>(joltPos.GetZ()), static_cast<float>(-joltPos.GetX()), static_cast<float>(joltPos.GetY()));
            glm::quat rotNWU(joltRot.GetW(), joltRot.GetZ(), -joltRot.GetX(), joltRot.GetY());

            glm::vec3 linVelNWU(joltLinVel.GetZ(), -joltLinVel.GetX(), joltLinVel.GetY());
            glm::vec3 angVelNWU(joltAngVel.GetZ(), -joltAngVel.GetX(), joltAngVel.GetY());

            // Fold in stored delta to maintain continuous momentum across external collisions
            linVelNWU += velocityDeltaLin;
            angVelNWU += velocityDeltaAng;

            chassis->drivetrainModel->ApplyExternalCorrections(posNWU, rotNWU, linVelNWU, angVelNWU);
        }

        // 2. Advance 6-DOF swerve plant across 4 sub-steps
        chassis->drivetrainModel->Step(dt, currentInputs, *activeTerrainProvider, 4);

        // 3. Compute midpoint velocity handoff for symplectic integration in Jolt: V_bar = (V + V') / 2
        glm::vec3 midLinVelNWU{ 0.0f };
        glm::vec3 midAngVelNWU{ 0.0f };
        chassis->drivetrainModel->GetMidpointVelocities(midLinVelNWU, midAngVelNWU);

        const auto& finalState = chassis->drivetrainModel->GetChassisState();
        velocityDeltaLin = finalState.linearVelocity - midLinVelNWU;
        velocityDeltaAng = finalState.angularVelocity - midAngVelNWU;

        // 4. Write midpoint velocities to Jolt rigid body
        if (joltContext && joltContext->IsInitialized() && chassisRb && !chassisRb->bodyID.IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = joltContext->GetBodyInterface();

            // Convert NWU -> Jolt Engine
            JPH::Vec3 midLinVelEng(-midLinVelNWU.y, midLinVelNWU.z, midLinVelNWU.x);
            JPH::Vec3 midAngVelEng(-midAngVelNWU.y, midAngVelNWU.z, midAngVelNWU.x);

            bodyInterface.SetLinearAndAngularVelocity(chassisRb->bodyID, midLinVelEng, midAngVelEng);
        }

        // 5. Populate feedback telemetry
        chassis->drivetrainModel->PopulateOutputs(currentOutputs);

        chassis->gyroYaw = currentOutputs.gyroYaw;
        chassis->gyroPitch = currentOutputs.gyroPitch;
        chassis->gyroRoll = currentOutputs.gyroRoll;
        chassis->gyroYawRate = currentOutputs.gyroYawRate;

        // Synchronize module telemetry
        for (uint32_t i = 0; i < 4; ++i)
        {
            if (chassis->moduleEntities[i] != invalidEntity)
            {
                auto* mod = scene.GetComponent<SwerveModuleComponent>(chassis->moduleEntities[i]);
                if (mod)
                {
                    const auto& modState = chassis->drivetrainModel->GetModule(i).GetState();
                    mod->currentSteerAngle = modState.steerAngle;
                    mod->currentSteerVelocity = modState.steerVelocity;
                    mod->unwrappedSteerAngle = modState.unwrappedSteerAngle;
                    mod->currentWheelPosition = modState.wheelPosition;
                    mod->currentWheelVelocity = modState.wheelVelocity;
                    mod->currentLinearVelocity = modState.wheelVelocity * mod->wheelRadius;
                    mod->distanceTraveled = modState.wheelPosition * mod->wheelRadius;
                }
            }
        }
    }

    void SwerveDriveSystem::RenderUpdate()
    {
        SIM_PROFILE_SCOPE("SwerveDriveSystem::RenderUpdate");

        auto chasses = scene.Query<SwerveChassisComponent>();
        if (chasses.empty())
        {
            return;
        }

        auto* chassis = scene.GetComponent<SwerveChassisComponent>(chasses[0]);
        if (!chassis || !chassis->drivetrainModel)
        {
            return;
        }

        for (uint32_t i = 0; i < 4; ++i)
        {
            if (chassis->moduleEntities[i] == invalidEntity)
            {
                continue;
            }

            auto* mod = scene.GetComponent<SwerveModuleComponent>(chassis->moduleEntities[i]);
            if (!mod)
            {
                continue;
            }

            const auto& modState = chassis->drivetrainModel->GetModule(i).GetState();

            // 1. Update steer housing visual rotation (swivels around vertical +Y)
            if (mod->steerEntity != invalidEntity)
            {
                auto* steerT = scene.GetComponent<TransformComponent>(mod->steerEntity);
                if (steerT)
                {
                    // In Jolt Engine frame: +Y Up is vertical rotation axis
                    steerT->rotation = glm::angleAxis(static_cast<float>(modState.steerAngle), glm::vec3(0.0f, 1.0f, 0.0f));
                }
            }

            // 2. Update wheel visual rotation (rolls around horizontal +X)
            if (mod->wheelEntity != invalidEntity)
            {
                auto* wheelT = scene.GetComponent<TransformComponent>(mod->wheelEntity);
                if (wheelT)
                {
                    // Rolls around +X horizontal axis
                    wheelT->rotation = glm::angleAxis(static_cast<float>(modState.wheelPosition), glm::vec3(1.0f, 0.0f, 0.0f));
                }
            }
        }
    }
}