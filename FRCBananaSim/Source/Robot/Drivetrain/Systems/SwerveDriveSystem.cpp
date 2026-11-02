/**
 * @file SwerveDriveSystem.cpp
 * @brief Implementation of the 6-DOF swerve drivetrain simulation system adapter.
 */

#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Core/Scene.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/Log.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Systems
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
            defaultProbe = std::make_unique<SwerveTerrainProbe>(*joltContext);
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
        if (provider)
        {
            activeTerrainProvider = provider;
        }
    }

    Model::SwerveDrivetrainModel* SwerveDriveSystem::GetModel()
    {
        if (!isInitialized)
        {
            InitializeModel();
        }

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

        // Ingest initial pose from ECS transform using canonical frame conversion
        glm::vec3 posNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(chassisTransform->position);
        glm::quat rotNWU = glm::quat(Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(glm::dquat(chassisTransform->rotation)));

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

        auto* compMass = scene.GetComponent<Mechanisms::Components::CompositeMassComponent>(chassisEntity);
        auto* collider = scene.GetComponent<ColliderComponent>(chassisEntity);

        // 1. Ingest post-Jolt state and restore velocity offset: V = v_Jolt + Delta
        if (joltContext && joltContext->IsInitialized() && chassisRb && !chassisRb->bodyID.IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = joltContext->GetBodyInterface();

            if (defaultProbe)
            {
                defaultProbe->SetIgnoredBody(chassisRb->bodyID);
            }

            if (compMass && bodyInterface.GetMotionType(chassisRb->bodyID) != JPH::EMotionType::Kinematic)
            {
                bodyInterface.SetMotionType(chassisRb->bodyID, JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
            }

            JPH::RVec3 joltPos = bodyInterface.GetPosition(chassisRb->bodyID);
            JPH::Quat joltRot = bodyInterface.GetRotation(chassisRb->bodyID);
            JPH::Vec3 joltLinVel = bodyInterface.GetLinearVelocity(chassisRb->bodyID);
            JPH::Vec3 joltAngVel = bodyInterface.GetAngularVelocity(chassisRb->bodyID);

            // Canonical conversion: Engine -> NWU
            glm::vec3 posEngine(static_cast<float>(joltPos.GetX()), static_cast<float>(joltPos.GetY()), static_cast<float>(joltPos.GetZ()));
            glm::dquat rotEngine(joltRot.GetW(), joltRot.GetX(), joltRot.GetY(), joltRot.GetZ());
            glm::vec3 linVelEngine(joltLinVel.GetX(), joltLinVel.GetY(), joltLinVel.GetZ());
            glm::vec3 angVelEngine(joltAngVel.GetX(), joltAngVel.GetY(), joltAngVel.GetZ());

            if (collider)
            {
                rotEngine = rotEngine * glm::inverse(glm::dquat(collider->localRotation));
                glm::vec3 worldOffset = glm::quat(rotEngine) * collider->localOffset;
                posEngine -= worldOffset;
                linVelEngine -= glm::cross(angVelEngine, worldOffset);
            }

            glm::vec3 posNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(posEngine);
            glm::quat rotNWU = glm::quat(Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(rotEngine));
            glm::vec3 linVelNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(linVelEngine);
            glm::vec3 angVelNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(angVelEngine);

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

        // 4. Write midpoint velocities to Jolt rigid body with strict finiteness validation
        if (joltContext && joltContext->IsInitialized() && chassisRb && !chassisRb->bodyID.IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = joltContext->GetBodyInterface();

            // Convert NWU -> Jolt Engine
            glm::vec3 midLinVelEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(midLinVelNWU);
            glm::vec3 midAngVelEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(midAngVelNWU);

            if (collider)   
            {
                JPH::Quat joltRot = bodyInterface.GetRotation(chassisRb->bodyID);
                glm::dquat rotEngine(joltRot.GetW(), joltRot.GetX(), joltRot.GetY(), joltRot.GetZ());
                rotEngine = rotEngine * glm::inverse(glm::dquat(collider->localRotation));
                glm::vec3 worldOffset = glm::quat(rotEngine) * collider->localOffset;
                midLinVelEng += glm::cross(midAngVelEng, worldOffset);
            }

            if (std::isfinite(midLinVelEng.x) && std::isfinite(midLinVelEng.y) && std::isfinite(midLinVelEng.z) &&
                std::isfinite(midAngVelEng.x) && std::isfinite(midAngVelEng.y) && std::isfinite(midAngVelEng.z))
            {
                JPH::Vec3 joltLin(midLinVelEng.x, midLinVelEng.y, midLinVelEng.z);
                JPH::Vec3 joltAng(midAngVelEng.x, midAngVelEng.y, midAngVelEng.z);
                bodyInterface.SetLinearAndAngularVelocity(chassisRb->bodyID, joltLin, joltAng);
            }
            else
            {
                SIM_LOG_ERROR("SwerveDriveSystem: Non-finite velocity encountered in midpoint integration!");
            }
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

        auto chasses = scene.Query<SwerveChassisComponent, TransformComponent>();
        if (chasses.empty())
        {
            return;
        }

        Entity chassisEntity = chasses[0];
        auto* chassis = scene.GetComponent<SwerveChassisComponent>(chassisEntity);
        auto* chassisT = scene.GetComponent<TransformComponent>(chassisEntity);
        if (!chassis || !chassisT || !chassis->drivetrainModel)
        {
            return;
        }

        glm::vec3 chassisPos = chassisT->position;
        glm::quat chassisRot = chassisT->rotation;

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

            // Module local offsets in Jolt frame (+X Right, +Y Up, +Z Forward)
            float halfChassisH = 0.075f;
            float halfSteerH = 0.030f;
            float steerH = 0.060f;
            float airGap = 0.010f;
            float wheelR = static_cast<float>(mod->wheelRadius);

            glm::vec3 localSteerOffset(mod->moduleLocation.x, -halfChassisH - halfSteerH, mod->moduleLocation.y);
            glm::vec3 localWheelOffset(mod->moduleLocation.x, -halfChassisH - steerH - airGap - wheelR, mod->moduleLocation.y);

            glm::vec3 worldSteerPos = chassisPos + chassisRot * localSteerOffset;
            glm::quat localSteerRot = glm::angleAxis(static_cast<float>(modState.steerAngle), glm::vec3(0.0f, 1.0f, 0.0f));
            glm::quat worldSteerRot = chassisRot * localSteerRot;

            glm::vec3 worldWheelPos = chassisPos + chassisRot * localWheelOffset;
            glm::quat localWheelRot = localSteerRot * glm::angleAxis(static_cast<float>(modState.wheelPosition), glm::vec3(1.0f, 0.0f, 0.0f));
            glm::quat worldWheelRot = chassisRot * localWheelRot;

            // 1. Update steer housing visual transform
            if (mod->steerEntity != invalidEntity)
            {
                auto* steerT = scene.GetComponent<TransformComponent>(mod->steerEntity);
                auto* steerWt = scene.GetComponent<WorldTransformComponent>(mod->steerEntity);
                if (steerT)
                {
                    steerT->position = worldSteerPos;
                    steerT->rotation = worldSteerRot;
                }
                if (steerWt)
                {
                    steerWt->worldPosition = worldSteerPos;
                    steerWt->worldRotation = worldSteerRot;
                }
            }

            // 2. Update wheel visual transform
            if (mod->wheelEntity != invalidEntity)
            {
                auto* wheelT = scene.GetComponent<TransformComponent>(mod->wheelEntity);
                auto* wheelWt = scene.GetComponent<WorldTransformComponent>(mod->wheelEntity);
                if (wheelT)
                {
                    wheelT->position = worldWheelPos;
                    wheelT->rotation = worldWheelRot;
                }
                if (wheelWt)
                {
                    wheelWt->worldPosition = worldWheelPos;
                    wheelWt->worldRotation = worldWheelRot;
                }
            }
        }
    }
}