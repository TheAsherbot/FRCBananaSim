/**
 * @file Main.cpp
 * @brief Dynamic actuator, battery, joint, and 6-DOF terrain-aware analytic swerve drivetrain sandbox with real-time telemetry.
 */

#include "Core/Engine.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Core/ECS/Systems/TransformPropagationSystem.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/JoltContext.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/PhysicsDebugDrawSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Actuation/BatteryModel.h"
#include "Actuation/DCMotorModel.h"
#include "Actuation/ServoModel.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Actuation/Components/MotorComponent.h"
#include "Actuation/Components/ServoComponent.h"
#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Control/SimControlBoundary.h"
#include "Core/Units/Units.h"
#include "Renderer/Renderer.h"
#include "Renderer/Components/MeshComponent.h"
#include "Renderer/Systems/RenderSystem.h"
#include "Telemetry/TelemetryMacros.h"
#include "Telemetry/TelemetryManager.h"
#include "Telemetry/TelemetryOverlay.h"
#include "Telemetry/ProfilerWindow.h"
#include "Telemetry/ProfileResult.h"
#include "Utils/DebugDraw.h"
#include "Utils/Log.h"

#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace FRCBananaSim::Core::Units::Literals;
using FRCBananaSim::Core::Engine;
using FRCBananaSim::Core::ECS::Core::Scene;
using FRCBananaSim::Core::ECS::Core::Entity;
using FRCBananaSim::Core::ECS::Core::invalidEntity;
using FRCBananaSim::Core::ECS::Components::TransformComponent;
using FRCBananaSim::Core::ECS::Components::WorldTransformComponent;
using FRCBananaSim::Core::ECS::Systems::TransformPropagationSystem;
using FRCBananaSim::Physics::PhysicsConfig;
using FRCBananaSim::Physics::JoltContext;
using FRCBananaSim::Physics::Components::RigidBodyComponent;
using FRCBananaSim::Physics::Components::ColliderComponent;
using FRCBananaSim::Physics::Components::JointComponent;
using FRCBananaSim::Physics::Components::JointType;
using FRCBananaSim::Physics::Components::JointMotorMode;
using FRCBananaSim::Physics::Components::BodyMotionType;
using FRCBananaSim::Physics::Components::ColliderShape;
using FRCBananaSim::Physics::Systems::PhysicsSystem;
using FRCBananaSim::Physics::Systems::PhysicsDebugDrawSystem;
using FRCBananaSim::Physics::Systems::JointSystem;
using FRCBananaSim::Actuation::BatteryModel;
using FRCBananaSim::Actuation::DCMotorModel;
using FRCBananaSim::Actuation::ServoModel;
using FRCBananaSim::Actuation::ServoMode;
using FRCBananaSim::Actuation::Components::BatteryComponent;
using FRCBananaSim::Actuation::Components::MotorComponent;
using FRCBananaSim::Actuation::Components::ServoComponent;
using FRCBananaSim::Robot::RobotSpawner;
using FRCBananaSim::Robot::SpawnedRobot;
using FRCBananaSim::Robot::Drivetrain::Components::SwerveChassisComponent;
using FRCBananaSim::Robot::Drivetrain::Components::SwerveModuleComponent;
using FRCBananaSim::Robot::Drivetrain::System::SwerveDriveSystem;
using FRCBananaSim::Robot::Drivetrain::Model::SwerveDrivetrainModel;
using FRCBananaSim::Robot::Drivetrain::Model::SwerveModuleState;
using FRCBananaSim::Robot::Drivetrain::Model::ChassisMotionState;
using namespace FRCBananaSim::Control;
using FRCBananaSim::Renderer::Renderer;
using FRCBananaSim::Renderer::Components::MeshComponent;
using FRCBananaSim::Renderer::Components::PrimitiveShape;
using FRCBananaSim::Renderer::Systems::RenderSystem;
using FRCBananaSim::Telemetry::TelemetryManager;
using FRCBananaSim::Telemetry::TelemetryMode;
using FRCBananaSim::Telemetry::TelemetryOverlay;
using FRCBananaSim::Telemetry::ProfilerWindow;
using FRCBananaSim::Telemetry::FrameSummary;
using FRCBananaSim::Utils::DebugDraw;

/**
 * @brief Constructs a DCMotorModel for any of the 9 standard motor presets with custom gear ratio and efficiency.
 * @param presetIndex Index in standard motors array [0..8].
 * @param gearRatio Mechanical reduction ratio.
 * @param efficiency Transmission efficiency (0.0 to 1.0).
 * @return Configured DCMotorModel.
 */
static DCMotorModel CreateMotorPreset(int presetIndex, double gearRatio = 1.0, double efficiency = 1.0)
{
    switch (presetIndex)
    {
    case 0: return DCMotorModel::KrakenX60(gearRatio, efficiency);
    case 1: return DCMotorModel::KrakenX44(gearRatio, efficiency);
    case 2: return DCMotorModel::Falcon500(gearRatio, efficiency);
    case 3: return DCMotorModel::NEOVortex(gearRatio, efficiency);
    case 4: return DCMotorModel::Minion(gearRatio, efficiency);
    case 5: return DCMotorModel::NEO1(gearRatio, efficiency);
    case 6: return DCMotorModel::NEO2(gearRatio, efficiency);
    case 7: return DCMotorModel::NEO550(gearRatio, efficiency);
    case 8: return DCMotorModel::GoBilda5202(gearRatio, efficiency);
    default: return DCMotorModel::Falcon500(gearRatio, efficiency);
    }
}

/**
 * @brief Returns datasheet rated transit speed in seconds per 60 degrees.
 * @param servoPresetIndex 0 = goBILDA Torque, 1 = goBILDA Speed, 2 = Standard FTC.
 * @return Rated speed (s / 60 deg).
 */
static double GetServoRatedSecPer60Deg(int servoPresetIndex)
{
    switch (servoPresetIndex)
    {
    case 0: return 0.20; // 0.20 s/60 deg (goBILDA 2000-0025-0002)
    case 1: return 0.12; // 0.12 s/60 deg (goBILDA 2000-0025-0003)
    case 2: return 0.18; // 0.18 s/60 deg (Standard FTC Hobby)
    default: return 0.20;
    }
}

int main(int argc, char* argv[])
{
    // 1. Initialize Telemetry System
    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal);

    // 2. Initialize Visual Renderer
    Renderer renderer(1600, 900, "FRC Banana Simulator - 6-DOF Terrain Swerve Sandbox");

    // Camera initial viewpoint (framed behind Station 5 looking towards ramp and bump along +Z)
    renderer.GetCamera().position = glm::vec3(3.2f, 3.2f, -3.8f);
    renderer.GetCamera().pitchDegrees = -16.0f;
    renderer.GetCamera().yawDegrees = 90.0f;
    renderer.GetCamera().moveSpeed = 8.0f;

    // 3. Initialize Jolt Physics Subsystem
    JoltContext joltContext;
    if (!joltContext.Initialize())
    {
        SIM_LOG_FATAL("Failed to initialize Jolt Physics subsystem");
        TelemetryManager::Get().Shutdown();
        return -1;
    }

    // 4. Telemetry Visualizers
    TelemetryOverlay telemetryOverlay;
    ProfilerWindow profilerWindow;

    // 5. ECS Scene & System Pipeline Registration
    Scene scene;
    scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

    scene.AddSystem(std::make_unique<JointSystem>(scene, joltContext));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene, joltContext));
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsDebugDrawSystem>(scene));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Helper lambda to assign both local and world transforms at creation
    auto assignTransforms = [&](Entity entity, const glm::vec3& pos, const glm::vec3& scale, const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        {
            TransformComponent transform;
            transform.position = pos;
            transform.scale = scale;
            transform.rotation = rot;
            scene.AddComponent(entity, transform);

            WorldTransformComponent worldTransform;
            worldTransform.worldPosition = pos;
            worldTransform.worldScale = scale;
            worldTransform.worldRotation = rot;
            scene.AddComponent(entity, worldTransform);
        };

    // -------------------------------------------------------------------------
    // 6. Environment Setup
    // -------------------------------------------------------------------------

    // A. Field Carpet
    Entity groundEntity = scene.CreateEntity();
    {
        assignTransforms(groundEntity, glm::vec3(0.0f, -0.05f, 0.0f), glm::vec3(28.0f, 0.1f, 18.0f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Plane;
        mesh.color = glm::vec3(0.22f, 0.52f, 0.28f);
        scene.AddComponent(groundEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        rigidBody.friction = 0.8f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(groundEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(14.0f, 0.05f, 9.0f);
        scene.AddComponent(groundEntity, collider);
    }

    // B. Central Robot Battery Entity (High-contrast casing with red/blue terminal posts)
    const glm::vec3 batteryPos = glm::vec3(0.0f, 0.35f, -2.8f);
    Entity batteryEntity = scene.CreateEntity();
    {
        assignTransforms(batteryEntity, batteryPos, glm::vec3(1.0f, 0.6f, 0.6f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.28f, 0.30f, 0.35f);
        scene.AddComponent(batteryEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(batteryEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.5f, 0.3f, 0.3f);
        scene.AddComponent(batteryEntity, collider);

        BatteryComponent battery;
        battery.batteryModel = BatteryModel::FRCStandard();
        battery.terminalVoltage = 12.60_V;
        battery.totalCurrent = 1.5_A;
        scene.AddComponent(batteryEntity, battery);
    }

    // Battery Positive Terminal
    Entity batPosTerminal = scene.CreateEntity();
    {
        assignTransforms(batPosTerminal, batteryPos + glm::vec3(0.25f, 0.40f, 0.0f), glm::vec3(0.15f, 0.20f, 0.15f));
        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.15f, 0.15f);
        scene.AddComponent(batPosTerminal, mesh);
    }

    // Battery Negative Terminal
    Entity batNegTerminal = scene.CreateEntity();
    {
        assignTransforms(batNegTerminal, batteryPos + glm::vec3(-0.25f, 0.40f, 0.0f), glm::vec3(0.15f, 0.20f, 0.15f));
        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.15f, 0.35f, 0.95f);
        scene.AddComponent(batNegTerminal, mesh);
    }

    // -------------------------------------------------------------------------
    // C. Station 1: High-Speed Flywheel Shooter Test Stand (FRC Brushless)
    // -------------------------------------------------------------------------
    const glm::vec3 st1Pos = glm::vec3(-4.8f, 0.0f, 0.0f);
    Entity st1Pedestal = scene.CreateEntity();
    {
        assignTransforms(st1Pedestal, st1Pos + glm::vec3(0.0f, 0.60f, 0.0f), glm::vec3(0.7f, 1.2f, 0.7f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.45f, 0.50f, 0.60f);
        scene.AddComponent(st1Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st1Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.35f, 0.60f, 0.35f);
        scene.AddComponent(st1Pedestal, collider);
    }

    Entity st1Flywheel = scene.CreateEntity();
    {
        assignTransforms(st1Flywheel, st1Pos + glm::vec3(0.0f, 1.50f, 0.0f), glm::vec3(1.1f, 0.16f, 1.1f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.98f, 0.80f, 0.15f);
        scene.AddComponent(st1Flywheel, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 1.5f;
        rigidBody.friction = 0.2f;
        rigidBody.restitution = 0.1f;
        scene.AddComponent(st1Flywheel, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.55f, 0.08f, 0.55f);
        scene.AddComponent(st1Flywheel, collider);
    }

    Entity st1JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = invalidEntity;
        joint.childEntity = st1Flywheel;
        joint.SetWorldSpace(st1Pos + glm::vec3(0.0f, 1.50f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        joint.motorMode = JointMotorMode::Velocity;
        joint.maxMotorTorque = 15.0f;
        joint.maxFrictionTorque = 0.005f;
        scene.AddComponent(st1JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = CreateMotorPreset(0, 1.0, 0.98); // Kraken X60 direct drive
        motor.commandedVoltage = 8.0_V;
        motor.statorCurrentLimit = 60.0_A;
        scene.AddComponent(st1JointEntity, motor);
    }

    // -------------------------------------------------------------------------
    // D. Station 2: Articulated Heavy Arm Test Stand (High-Reduction Gearboxes)
    // -------------------------------------------------------------------------
    const glm::vec3 st2Pos = glm::vec3(-1.6f, 0.0f, 0.0f);
    Entity st2Tower = scene.CreateEntity();
    {
        assignTransforms(st2Tower, st2Pos + glm::vec3(0.0f, 1.20f, 0.0f), glm::vec3(0.45f, 2.4f, 0.45f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.38f, 0.40f, 0.45f);
        scene.AddComponent(st2Tower, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st2Tower, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.225f, 1.20f, 0.225f);
        scene.AddComponent(st2Tower, collider);
    }

    Entity st2Arm = scene.CreateEntity();
    {
        assignTransforms(st2Arm, st2Pos + glm::vec3(0.0f, 1.75f, 0.45f), glm::vec3(0.18f, 1.2f, 0.18f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(1.0f, 0.50f, 0.08f);
        scene.AddComponent(st2Arm, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 3.5f;
        rigidBody.friction = 0.5f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(st2Arm, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.09f, 0.60f, 0.09f);
        scene.AddComponent(st2Arm, collider);
    }

    Entity st2JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = invalidEntity;
        joint.childEntity = st2Arm;
        joint.SetWorldSpace(st2Pos + glm::vec3(0.0f, 2.30f, 0.45f), glm::vec3(1.0f, 0.0f, 0.0f));
        joint.hasLimits = true;
        joint.minLimit = -glm::radians(75.0f);
        joint.maxLimit = glm::radians(75.0f);
        joint.motorMode = JointMotorMode::Position;
        joint.springFrequency = 14.0f;
        joint.springDamping = 1.0f;
        joint.maxMotorTorque = 65.0f;
        scene.AddComponent(st2JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = CreateMotorPreset(5, 25.0, 0.95); // NEO 1.1 at 25:1
        motor.commandedVoltage = 0.0_V;
        motor.statorCurrentLimit = 60.0_A;
        scene.AddComponent(st2JointEntity, motor);
    }

    // -------------------------------------------------------------------------
    // E. Station 5 Obstacles: 15° Inclined Ramp & 3 cm Cable-Protector Bump
    // -------------------------------------------------------------------------

    // 1. 15-Degree Inclined Ramp (Centered at X = 3.2m, Z = 3.5m)
    Entity rampEntity = scene.CreateEntity();
    {
        glm::vec3 rampPos(3.2f, 0.259f, 3.5f);
        glm::quat rampRot = glm::angleAxis(glm::radians(-15.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        assignTransforms(rampEntity, rampPos, glm::vec3(1.8f, 0.08f, 2.0f), rampRot);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.35f, 0.45f, 0.65f); // Slate Blue
        scene.AddComponent(rampEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        rigidBody.friction = 0.8f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(rampEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.9f, 0.04f, 1.0f);
        scene.AddComponent(rampEntity, collider);
    }

    // 2. 3 cm Cable-Protector Bump (Centered at X = 3.2m, Z = 7.0m)
    Entity bumpEntity = scene.CreateEntity();
    {
        glm::vec3 bumpPos(3.2f, 0.015f, 7.0f);
        assignTransforms(bumpEntity, bumpPos, glm::vec3(1.8f, 0.03f, 0.20f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.80f, 0.10f); // Hazard Yellow
        scene.AddComponent(bumpEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        rigidBody.friction = 0.8f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(bumpEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.9f, 0.015f, 0.10f);
        scene.AddComponent(bumpEntity, collider);
    }

    // -------------------------------------------------------------------------
    // F. Station 5: Swerve Robot Drivetrain on Field Carpet
    // -------------------------------------------------------------------------
    const glm::vec3 swerveSpawnPos = glm::vec3(3.2f, 0.248f, 0.0f);
    SpawnedRobot swerveRobot = RobotSpawner::SpawnFromJsonFile(scene,
        "FRCBananaSim/Resources/Robots/robot_swerve.json", swerveSpawnPos);

    if (swerveRobot.isSuccess)
    {
        // 1. Chassis Visual Mesh (Deep Metallic Blue)
        MeshComponent chassisMesh;
        chassisMesh.shape = PrimitiveShape::Cube;
        chassisMesh.color = glm::vec3(0.18f, 0.24f, 0.38f);
        scene.AddComponent(swerveRobot.chassisEntity, chassisMesh);

        WorldTransformComponent chassisWt;
        chassisWt.worldPosition = swerveSpawnPos;
        chassisWt.worldScale = glm::vec3(0.70f, 0.15f, 0.70f);
        scene.AddComponent(swerveRobot.chassisEntity, chassisWt);

        // 2. Steer Housings & Wheels Visual Meshes (Round Cylinders)
        for (uint32_t i = 0; i < 4; ++i)
        {
            MeshComponent steerMesh;
            steerMesh.shape = PrimitiveShape::Cylinder;
            steerMesh.color = glm::vec3(0.95f, 0.78f, 0.12f);
            scene.AddComponent(swerveRobot.steerEntities[i], steerMesh);

            WorldTransformComponent steerWt;
            steerWt.worldScale = glm::vec3(0.08f, 0.06f, 0.08f);
            scene.AddComponent(swerveRobot.steerEntities[i], steerWt);

            MeshComponent wheelMesh;
            wheelMesh.shape = PrimitiveShape::CylinderX;
            wheelMesh.color = glm::vec3(0.12f, 0.12f, 0.15f);
            scene.AddComponent(swerveRobot.wheelEntities[i], wheelMesh);

            WorldTransformComponent wheelWt;
            wheelWt.worldScale = glm::vec3(0.04f, 0.1016f, 0.1016f);
            scene.AddComponent(swerveRobot.wheelEntities[i], wheelWt);
        }
    }

    // Swerve drive system initialized with Jolt context for real-time terrain collision queries
    SwerveDriveSystem swerveSystem(scene, &joltContext);
    SimInputs swerveInputs;
    SimOutputs swerveOutputs;

    bool swerveEnabled = true;
    bool swervePinChassis = false;
    float swerveDriveVolts = 0.0f;
    float swerveTargetSteerAngle = 0.0f;
    int swerveNeutralModeIndex = 0; // Brake

    bool swerveRunStressTest = false;
    double swerveStressElapsed = 0.0;

    // Helper lambda to reset robot position and momentum
    auto resetRobotToPosition = [&](const glm::vec3& worldPos, const glm::quat& worldRot)
        {
            if (!swerveRobot.isSuccess)
            {
                return;
            }

            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(swerveRobot.chassisEntity);
            if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
            {
                JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                bi.SetPositionAndRotation(chassisRb->bodyID,
                    JPH::RVec3(worldPos.x, worldPos.y, worldPos.z),
                    JPH::Quat(worldRot.x, worldRot.y, worldRot.z, worldRot.w),
                    JPH::EActivation::Activate);
                bi.SetLinearAndAngularVelocity(chassisRb->bodyID,
                    JPH::Vec3::sZero(),
                    JPH::Vec3::sZero());
            }

            auto* chassisT = scene.GetComponent<TransformComponent>(swerveRobot.chassisEntity);
            if (chassisT)
            {
                chassisT->position = worldPos;
                chassisT->rotation = worldRot;
            }

            auto* chassisWt = scene.GetComponent<WorldTransformComponent>(swerveRobot.chassisEntity);
            if (chassisWt)
            {
                chassisWt->worldPosition = worldPos;
                chassisWt->worldRotation = worldRot;
            }

            auto* model = swerveSystem.GetModel();
            if (model)
            {
                // Convert Engine (+X Right, +Y Up, +Z Forward) to NWU (+X Fwd, +Y Left, +Z Up)
                glm::vec3 posNWU(worldPos.z, -worldPos.x, worldPos.y);
                glm::quat rotNWU(worldRot.w, worldRot.z, -worldRot.x, worldRot.y);
                model->ApplyExternalCorrections(posNWU, rotNWU, glm::vec3(0.0f), glm::vec3(0.0f));

                for (uint32_t i = 0; i < 4; ++i)
                {
                    model->GetModule(i).GetState() = SwerveModuleState{};
                }
            }

            swerveDriveVolts = 0.0f;
            swerveTargetSteerAngle = 0.0f;
        };

    // -------------------------------------------------------------------------
    // 7. Interactive State, Speed Control & Station Isolation
    // -------------------------------------------------------------------------
    double simSpeed = 1.0;
    double simAccumulator = 0.0;
    bool doSingleStep = false;
    uint64_t totalSimSteps = 0;

    bool triggerStallSpike = false;
    int activeStationIndex = 5; // Focus on Swerve Robot

    float st1CommandedVolts = 8.0f;

    float st2TargetAngleDegrees = 0.0f;
    bool st2AutoOscillate = true;
    double st2OscillationTime = 0.0;

    int spawnedCount = 0;
    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;

    // -------------------------------------------------------------------------
    // Simulation Step Function
    // -------------------------------------------------------------------------
    auto executeSimulationStep = [&](double fixedDeltaTime)
        {
            auto* battery = scene.GetComponent<BatteryComponent>(batteryEntity);
            if (!battery)
            {
                return;
            }

            double totalActuatorSupplyCurrent = 0.0;

            // --- A. Station 1: Flywheel ---
            auto* m1 = scene.GetComponent<MotorComponent>(st1JointEntity);
            auto* j1 = scene.GetComponent<JointComponent>(st1JointEntity);
            auto* rb1 = scene.GetComponent<RigidBodyComponent>(st1Flywheel);
            if (m1 && j1 && rb1)
            {
                bool st1Active = (activeStationIndex == 0 || activeStationIndex == 1);
                float speed = glm::dot(rb1->angularVelocity, j1->parentAxis);
                m1->angularVelocity = speed;

                if (!st1Active)
                {
                    m1->commandedVoltage = 0.0_V;
                    m1->appliedVoltage = 0.0_V;
                    m1->statorCurrent = 0.0_A;
                    m1->supplyCurrent = 0.0_A;
                    m1->outputTorque = 0.0;
                    j1->motorMode = JointMotorMode::Velocity;
                    j1->targetVelocity = 0.0f;
                    j1->maxMotorTorque = 0.1f;
                }
                else
                {
                    m1->commandedVoltage = static_cast<double>(st1CommandedVolts);
                    double rawStatorCurrent = m1->motorModel.CalculateCurrent(m1->commandedVoltage, speed);
                    m1->statorCurrent = std::clamp(rawStatorCurrent, -m1->statorCurrentLimit, m1->statorCurrentLimit);

                    if (battery->isBrownoutCritical)
                    {
                        m1->appliedVoltage = 0.0_V;
                    }
                    else
                    {
                        m1->appliedVoltage = battery->batteryModel.ApplySupplyCurrentLimit(
                            m1->commandedVoltage, m1->statorCurrent, battery->terminalVoltage);
                    }

                    double power = std::abs(m1->statorCurrent * m1->appliedVoltage);
                    m1->supplyCurrent = (battery->terminalVoltage > 0.01_V) ? (power / battery->terminalVoltage) : 0.0_A;
                    totalActuatorSupplyCurrent += m1->supplyCurrent;

                    constexpr double kFlywheelInertia = 0.08;
                    m1->outputTorque = m1->motorModel.CalculateTorque(
                        m1->appliedVoltage, speed, kFlywheelInertia, fixedDeltaTime);

                    j1->motorMode = JointMotorMode::Velocity;
                    j1->targetVelocity = (m1->appliedVoltage >= 0.0) ? 2000.0f : -2000.0f;
                    j1->maxMotorTorque = static_cast<float>(std::abs(m1->outputTorque));
                }
            }

            // --- B. Station 2: Arm ---
            auto* m2 = scene.GetComponent<MotorComponent>(st2JointEntity);
            auto* j2 = scene.GetComponent<JointComponent>(st2JointEntity);
            auto* rb2 = scene.GetComponent<RigidBodyComponent>(st2Arm);
            if (m2 && j2 && rb2)
            {
                bool st2Active = (activeStationIndex == 0 || activeStationIndex == 2);
                float currentArmAngle = j2->currentAngle;
                float speed = j2->currentVelocity;
                m2->angularPosition = currentArmAngle;
                m2->angularVelocity = speed;

                if (!st2Active)
                {
                    j2->motorMode = JointMotorMode::Position;
                    j2->targetAngle = 0.0f;
                    j2->maxMotorTorque = 0.1f;
                }
                else
                {
                    if (st2AutoOscillate)
                    {
                        st2OscillationTime += fixedDeltaTime;
                        st2TargetAngleDegrees = static_cast<float>(std::sin(st2OscillationTime * 1.8) * 60.0);
                    }

                    float desiredAngleRad = glm::radians(st2TargetAngleDegrees);
                    j2->motorMode = JointMotorMode::Position;
                    j2->targetAngle = desiredAngleRad;

                    float angleError = desiredAngleRad - currentArmAngle;
                    double demandedVolts = std::clamp(static_cast<double>(angleError * 14.0f), -12.0, 12.0);
                    m2->commandedVoltage = demandedVolts;

                    double rawStatorCurrent = m2->motorModel.CalculateCurrent(m2->commandedVoltage, speed);
                    m2->statorCurrent = std::clamp(rawStatorCurrent, -m2->statorCurrentLimit, m2->statorCurrentLimit);

                    if (battery->isBrownoutCritical)
                    {
                        m2->appliedVoltage = 0.0_V;
                    }
                    else
                    {
                        m2->appliedVoltage = battery->batteryModel.ApplySupplyCurrentLimit(
                            m2->commandedVoltage, m2->statorCurrent, battery->terminalVoltage);
                    }

                    double power = std::abs(m2->statorCurrent * m2->appliedVoltage);
                    m2->supplyCurrent = (battery->terminalVoltage > 0.01_V) ? (power / battery->terminalVoltage) : 0.0_A;
                    totalActuatorSupplyCurrent += m2->supplyCurrent;

                    constexpr double kArmInertia = 0.45;
                    m2->outputTorque = m2->motorModel.CalculateTorque(
                        m2->appliedVoltage, speed, kArmInertia, fixedDeltaTime);

                    j2->maxMotorTorque = static_cast<float>(std::abs(m2->outputTorque) + 5.0);
                }
            }

            // --- C. Station 5: Swerve Robot Simulation ---
            if (swerveRobot.isSuccess)
            {
                // Pin/Unpin chassis via motion type
                auto* chassisRb = scene.GetComponent<RigidBodyComponent>(swerveRobot.chassisEntity);
                if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
                {
                    JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                    JPH::EMotionType targetMotion = swervePinChassis ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
                    if (bi.GetMotionType(chassisRb->bodyID) != targetMotion)
                    {
                        bi.SetMotionType(chassisRb->bodyID, targetMotion, JPH::EActivation::Activate);
                    }
                }

                if (swerveRunStressTest)
                {
                    swerveStressElapsed += fixedDeltaTime;
                    swerveInputs.robotEnabled = true;

                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        // Full 12V drive and steer stall stress test
                        swerveInputs.motorCommands[i].mode = MotorControlMode::Voltage;
                        swerveInputs.motorCommands[i].demand = 12.0;
                        swerveInputs.motorCommands[i].statorCurrentLimit = 60.0;
                        swerveInputs.motorCommands[i].neutralMode = NeutralMode::Brake;

                        swerveInputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                        swerveInputs.motorCommands[4 + i].demand = 12.0;
                        swerveInputs.motorCommands[4 + i].statorCurrentLimit = 60.0;
                        swerveInputs.motorCommands[4 + i].neutralMode = NeutralMode::Brake;
                    }

                    if (swerveStressElapsed >= 60.0)
                    {
                        swerveRunStressTest = false;
                        SIM_LOG_INFO("60-Second Real-Time Swerve Stress Test Complete!");
                    }
                }
                else
                {
                    swerveInputs.robotEnabled = swerveEnabled;
                    NeutralMode nMode = (swerveNeutralModeIndex == 0) ? NeutralMode::Brake : NeutralMode::Coast;

                    // Azimuth Closed-Loop Heading Hold using feedback from swerveOutputs encoders
                    constexpr double kSteerP = 12.0;
                    constexpr double kSteerD = 0.6;

                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        double currentAngle = swerveOutputs.encoderPositions[i];
                        double currentVel = swerveOutputs.encoderVelocities[i];
                        double angleError = static_cast<double>(swerveTargetSteerAngle) - currentAngle;

                        while (angleError > 3.14159265358979323846)
                        {
                            angleError -= 2.0 * 3.14159265358979323846;
                        }
                        while (angleError < -3.14159265358979323846)
                        {
                            angleError += 2.0 * 3.14159265358979323846;
                        }

                        double steerDemandVolts = std::clamp((angleError * kSteerP) - (currentVel * kSteerD), -12.0, 12.0);

                        swerveInputs.motorCommands[i].mode = MotorControlMode::Voltage;
                        swerveInputs.motorCommands[i].demand = steerDemandVolts;
                        swerveInputs.motorCommands[i].statorCurrentLimit = 60.0;
                        swerveInputs.motorCommands[i].neutralMode = NeutralMode::Brake;

                        swerveInputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                        swerveInputs.motorCommands[4 + i].demand = swerveDriveVolts;
                        swerveInputs.motorCommands[4 + i].statorCurrentLimit = 60.0;
                        swerveInputs.motorCommands[4 + i].neutralMode = nMode;
                    }
                }

                swerveSystem.SetInputs(swerveInputs);
                swerveSystem.FixedUpdate(fixedDeltaTime);
                swerveOutputs = swerveSystem.GetOutputs();

                totalActuatorSupplyCurrent += swerveOutputs.totalBusCurrent;
            }

            // --- Central Battery Bus Update ---
            if (triggerStallSpike)
            {
                totalActuatorSupplyCurrent += 320.0_A;
            }

            battery->terminalVoltage = battery->batteryModel.CalculateTerminalVoltage(totalActuatorSupplyCurrent);
            battery->totalCurrent = totalActuatorSupplyCurrent + battery->batteryModel.GetQuiescentCurrent();
            battery->isBrownoutWarning = battery->batteryModel.IsBrownoutWarning(battery->terminalVoltage);
            battery->isBrownoutCritical = battery->batteryModel.IsBrownoutCritical(battery->terminalVoltage);

            // Step ECS physics
            scene.SetFixedDeltaTime(fixedDeltaTime);
            scene.FixedUpdateSystems();

            // Synchronize simulated transforms into WorldTransformComponent for immediate visual rendering
            auto syncWorldTransform = [&](Entity e)
                {
                    auto* t = scene.GetComponent<TransformComponent>(e);
                    auto* wt = scene.GetComponent<WorldTransformComponent>(e);
                    if (t && wt)
                    {
                        wt->worldPosition = t->position;
                        wt->worldRotation = t->rotation;
                        wt->worldScale = t->scale;
                    }
                };

            syncWorldTransform(st1Flywheel);
            syncWorldTransform(st2Arm);
            if (swerveRobot.isSuccess)
            {
                syncWorldTransform(swerveRobot.chassisEntity);
            }

            ++totalSimSteps;
        };

    // -------------------------------------------------------------------------
    // 8. Engine Loop Configuration
    // -------------------------------------------------------------------------
    Engine engine(200.0, 0.0);

    engine.SetFixedUpdateCallback([&](double fixedDeltaTime)
        {
            SIM_PROFILE_SCOPE("FixedUpdate");
            auto start = std::chrono::high_resolution_clock::now();

            if (simSpeed == 0.0)
            {
                if (doSingleStep)
                {
                    doSingleStep = false;
                    executeSimulationStep(fixedDeltaTime);
                }
            }
            else
            {
                simAccumulator += fixedDeltaTime * simSpeed;
                int stepsTaken = 0;
                while (simAccumulator >= fixedDeltaTime && stepsTaken < 8)
                {
                    simAccumulator -= fixedDeltaTime;
                    executeSimulationStep(fixedDeltaTime);
                    ++stepsTaken;
                }
            }

            auto end = std::chrono::high_resolution_clock::now();
            accumulatedPhysicsMs += std::chrono::duration<double, std::milli>(end - start).count();
        });

    // -------------------------------------------------------------------------
    // 9. Per-Frame Update Callback & ImGui HUD
    // -------------------------------------------------------------------------
    engine.SetUpdateCallback([&](double deltaTime)
        {
            SIM_PROFILE_SCOPE("Update");
            auto start = std::chrono::high_resolution_clock::now();

            if (renderer.ShouldClose())
            {
                engine.RequestExit();
                return;
            }

            auto& input = renderer.GetInput();

            // Keyboard Driving Controls (Arrow Keys)
            if (!swerveRunStressTest && swerveEnabled && !ImGui::GetIO().WantTextInput)
            {
                // Drive Forward / Reverse (Up / Down)
                if (input.IsKeyDown(GLFW_KEY_UP) || input.IsKeyDown(GLFW_KEY_KP_8))
                {
                    swerveDriveVolts = 10.0f;
                }
                else if (input.IsKeyDown(GLFW_KEY_DOWN) || input.IsKeyDown(GLFW_KEY_KP_2))
                {
                    swerveDriveVolts = -10.0f;
                }
                else
                {
                    swerveDriveVolts = 0.0f;
                }

                // Steer Azimuth Angle (Left / Right)
                if (input.IsKeyDown(GLFW_KEY_LEFT) || input.IsKeyDown(GLFW_KEY_KP_4))
                {
                    swerveTargetSteerAngle += static_cast<float>(deltaTime * 2.5); // Slew Left
                }
                else if (input.IsKeyDown(GLFW_KEY_RIGHT) || input.IsKeyDown(GLFW_KEY_KP_6))
                {
                    swerveTargetSteerAngle -= static_cast<float>(deltaTime * 2.5); // Slew Right
                }
                else
                {
                    // Smoothly return steer heading to center (forward) when keys are released
                    if (std::abs(swerveTargetSteerAngle) > 0.01f)
                    {
                        float returnStep = static_cast<float>(deltaTime * 4.0);
                        if (swerveTargetSteerAngle > 0.0f)
                        {
                            swerveTargetSteerAngle = std::max(0.0f, swerveTargetSteerAngle - returnStep);
                        }
                        else
                        {
                            swerveTargetSteerAngle = std::min(0.0f, swerveTargetSteerAngle + returnStep);
                        }
                    }
                    else
                    {
                        swerveTargetSteerAngle = 0.0f;
                    }
                }
            }
            else if (!swerveRunStressTest)
            {
                swerveDriveVolts = 0.0f;
            }

            // Simulation Speed Hotkeys (Keys 0-9)
            if (input.WasKeyPressed(GLFW_KEY_0)) simSpeed = 0.0;
            else if (input.WasKeyPressed(GLFW_KEY_1)) simSpeed = 0.1;
            else if (input.WasKeyPressed(GLFW_KEY_2)) simSpeed = 0.25;
            else if (input.WasKeyPressed(GLFW_KEY_3)) simSpeed = 0.50;
            else if (input.WasKeyPressed(GLFW_KEY_7)) simSpeed = 1.00;
            else if (input.WasKeyPressed(GLFW_KEY_8)) simSpeed = 2.00;
            else if (input.WasKeyPressed(GLFW_KEY_9)) simSpeed = 4.00;

            if (input.WasKeyPressed(GLFW_KEY_PERIOD))
            {
                if (simSpeed == 0.0) doSingleStep = true;
            }

            // F3: Toggle Profiler
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                TelemetryMode current = TelemetryManager::Get().GetMode();
                TelemetryMode next = (current == TelemetryMode::Off) ? TelemetryMode::Minimal :
                    ((current == TelemetryMode::Minimal) ? TelemetryMode::Full : TelemetryMode::Off);

                if (next == TelemetryMode::Full) profilerWindow.Open(renderer.GetWindow());
                else if (profilerWindow.IsOpen()) profilerWindow.Close(renderer.GetWindow());

                TelemetryManager::Get().SetMode(next);
            }

            // Render Frame
            renderer.BeginFrame();
            scene.RenderUpdateSystems();
            swerveSystem.RenderUpdate();

            // Render Telemetry HUD
            telemetryOverlay.Render();

            auto* battery = scene.GetComponent<BatteryComponent>(batteryEntity);

            // --- ImGui Interactive Control Center ---
            ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(500.0f, 740.0f), ImGuiCond_FirstUseEver);

            if (ImGui::Begin("FRCBananaSim — 6-DOF Multi-Actuator Control"))
            {
                // Simulation Speed Controls
                ImGui::Text("Simulation Time: %.2f s | Speed: %.2fx",
                    static_cast<double>(totalSimSteps) * PhysicsConfig::fixedDeltaTime, simSpeed);

                if (ImGui::Button("Pause (0x)")) simSpeed = 0.0; ImGui::SameLine();
                if (ImGui::Button("0.25x")) simSpeed = 0.25; ImGui::SameLine();
                if (ImGui::Button("1.0x (Realtime)")) simSpeed = 1.0; ImGui::SameLine();
                if (ImGui::Button("2.0x (Fast)")) simSpeed = 2.0;

                ImGui::Separator();

                // Main Tab Bar
                if (ImGui::BeginTabBar("MainTabs"))
                {
                    // =============================================================
                    // TAB 1: SWERVE ROBOT (DRIVE & 6-DOF TERRAIN)
                    // =============================================================
                    if (ImGui::BeginTabItem("Swerve Robot (Gate 1)"))
                    {
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "STATION 5: 6-DOF TERRAIN-AWARE SWERVE CHASSIS (50 KG)");
                        ImGui::Spacing();

                        // Reset Shortcuts
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "OBSTACLE COURSE & BENCH POSITIONS");
                        if (ImGui::Button("Reset to Start Line", ImVec2(180.0f, 26.0f)))
                        {
                            resetRobotToPosition(glm::vec3(3.2f, 0.248f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Place on 15 deg Ramp", ImVec2(180.0f, 26.0f)))
                        {
                            resetRobotToPosition(glm::vec3(3.2f, 0.508f, 3.5f),
                                glm::angleAxis(glm::radians(-15.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
                        }

                        ImGui::Spacing();

                        // Mode: Test Stand vs Field Drive
                        ImGui::Checkbox("Pin Chassis to Stand (Kinematic)", &swervePinChassis);
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("When checked, the chassis body is held Kinematic so stall torque and peak RPM can be tested safely.\nUncheck to let the robot drive across carpet, ramps, and bumps.");
                        }

                        ImGui::Checkbox("Robot Enabled", &swerveEnabled);

                        const char* neutralModes[] = { "Brake (Dynamic Damping)", "Coast (Free Spin)" };
                        ImGui::Combo("Neutral Mode", &swerveNeutralModeIndex, neutralModes, IM_ARRAYSIZE(neutralModes));

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Real-Time 60s Stall Stress Test Section
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "GATE 1: 60-SECOND MASS-RATIO STRESS TEST");

                        if (!swerveRunStressTest)
                        {
                            if (ImGui::Button("Run 60s Max Stall Stress Test Live", ImVec2(-1.0f, 28.0f)))
                            {
                                swerveRunStressTest = true;
                                swerveStressElapsed = 0.0;
                                swervePinChassis = true;
                                SIM_LOG_INFO("Started 60-Second Real-Time Swerve Stress Test");
                            }
                        }
                        else
                        {
                            float progress = static_cast<float>(swerveStressElapsed / 60.0);
                            char progBuf[64];
                            std::snprintf(progBuf, sizeof(progBuf), "Running: %.1f / 60.0 s (%.0f%%)", swerveStressElapsed, progress * 100.0f);
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
                            ImGui::ProgressBar(progress, ImVec2(-1.0f, 24.0f), progBuf);
                            ImGui::PopStyleColor();

                            if (ImGui::Button("Abort Stress Test"))
                            {
                                swerveRunStressTest = false;
                            }
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Manual Controls
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "MANUAL DRIVE CONTROLS (OR USE ARROW KEYS)");
                        ImGui::SliderFloat("Drive Voltage (V)", &swerveDriveVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Forward (10V)")) swerveDriveVolts = 10.0f; ImGui::SameLine();
                        if (ImGui::Button("Stop (0V)")) { swerveDriveVolts = 0.0f; swerveTargetSteerAngle = 0.0f; } ImGui::SameLine();
                        if (ImGui::Button("Reverse (-10V)")) swerveDriveVolts = -10.0f;

                        float steerAngleDeg = glm::degrees(swerveTargetSteerAngle);
                        if (ImGui::SliderFloat("Steer Target (deg)", &steerAngleDeg, -90.0f, 90.0f, "%.1f deg"))
                        {
                            swerveTargetSteerAngle = glm::radians(steerAngleDeg);
                        }
                        if (ImGui::Button("Turn Right (-45 deg)")) swerveTargetSteerAngle = glm::radians(-45.0f); ImGui::SameLine();
                        if (ImGui::Button("Center (0 deg)")) swerveTargetSteerAngle = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Turn Left (+45 deg)")) swerveTargetSteerAngle = glm::radians(45.0f);

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Live Telemetry Readout
                        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "LIVE TELEMETRY & 6-DOF KINEMATICS");

                        // Battery Terminal Voltage Gauge
                        float vFraction = static_cast<float>(swerveOutputs.batteryTerminalVoltage / 13.0);
                        ImVec4 vColor = (swerveOutputs.batteryTerminalVoltage > 11.0) ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) :
                            ((swerveOutputs.batteryTerminalVoltage > 8.5) ? ImVec4(0.9f, 0.8f, 0.1f, 1.0f) : ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, vColor);
                        char vBuf[64];
                        std::snprintf(vBuf, sizeof(vBuf), "Battery: %.2f V | Bus: %.1f A",
                            swerveOutputs.batteryTerminalVoltage, swerveOutputs.totalBusCurrent);
                        ImGui::ProgressBar(vFraction, ImVec2(-1.0f, 20.0f), vBuf);
                        ImGui::PopStyleColor();

                        // 6-DOF IMU Heading & Attitude
                        ImGui::Text("IMU Yaw:   %.1f deg | Pitch: %.1f deg | Roll: %.1f deg",
                            glm::degrees(swerveOutputs.gyroYaw), glm::degrees(swerveOutputs.gyroPitch), glm::degrees(swerveOutputs.gyroRoll));
                        ImGui::Text("Yaw Rate:  %.1f deg/s", glm::degrees(swerveOutputs.gyroYawRate));

                        // Chassis linear velocity & altitude from Jolt
                        auto* chassisT = scene.GetComponent<TransformComponent>(swerveRobot.chassisEntity);
                        auto* chassisRb = scene.GetComponent<RigidBodyComponent>(swerveRobot.chassisEntity);
                        glm::vec3 chassisLinVel(0.0f);
                        if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
                        {
                            JPH::Vec3 vel = joltContext.GetBodyInterface().GetLinearVelocity(chassisRb->bodyID);
                            chassisLinVel = glm::vec3(vel.GetX(), vel.GetY(), vel.GetZ());
                        }
                        float groundSpeed = glm::length(glm::vec2(chassisLinVel.x, chassisLinVel.z));
                        float elevationY = chassisT ? chassisT->position.y : 0.0f;
                        ImGui::Text("Speed:     %.2f m/s (%.1f ft/s) | Elev (Y): %.3f m", groundSpeed, groundSpeed * 3.28084f, elevationY);

                        // Energy Ledger
                        auto* model = swerveSystem.GetModel();
                        if (model)
                        {
                            const auto& ledger = model->GetEnergyLedger();
                            ImGui::Text("Energy In: %.1f J | KE: %.2f J | Residual: %.3f J",
                                ledger.batteryEnergyDrawnJoules, ledger.kineticEnergyJoules, ledger.energyResidualJoules);
                        }

                        ImGui::Spacing();

                        // 4 Swerve Modules Telemetry Table
                        if (ImGui::BeginTable("ModuleTable", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                        {
                            ImGui::TableSetupColumn("Module");
                            ImGui::TableSetupColumn("Steer");
                            ImGui::TableSetupColumn("Drive m/s");
                            ImGui::TableSetupColumn("RPM");
                            ImGui::TableSetupColumn("Current");
                            ImGui::TableSetupColumn("Norm Force");
                            ImGui::TableSetupColumn("Slip (n)");
                            ImGui::TableHeadersRow();

                            const char* modNames[] = { "FL [0]", "FR [1]", "BL [2]", "BR [3]" };
                            for (int i = 0; i < 4; ++i)
                            {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::Text("%s", modNames[i]);

                                ImGui::TableSetColumnIndex(1);
                                ImGui::Text("%.1f deg", glm::degrees(swerveOutputs.encoderPositions[i]));

                                double linearV = 0.0;
                                double driveRpm = (swerveOutputs.encoderVelocities[4 + i] * 60.0) / (2.0 * 3.141592653589793);
                                double normalF = 0.0;
                                double slipRatio = 0.0;

                                if (model)
                                {
                                    const auto& modState = model->GetModule(i).GetState();
                                    linearV = modState.linearVelocity;
                                    normalF = modState.contactState.normalForce;
                                    slipRatio = modState.contactState.saturationRatio;
                                }

                                ImGui::TableSetColumnIndex(2);
                                ImGui::Text("%.2f", linearV);

                                ImGui::TableSetColumnIndex(3);
                                ImGui::Text("%.0f", driveRpm);

                                ImGui::TableSetColumnIndex(4);
                                ImGui::Text("%.1f A", swerveOutputs.motorCurrents[4 + i]);

                                ImGui::TableSetColumnIndex(5);
                                ImGui::Text("%.1f N", normalF);

                                ImGui::TableSetColumnIndex(6);
                                if (slipRatio > 1.0)
                                {
                                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%.2f (SLIP)", slipRatio);
                                }
                                else
                                {
                                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "%.2f (TRACTION)", slipRatio);
                                }
                            }
                            ImGui::EndTable();
                        }

                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 2: FLYWHEEL TEST STAND
                    // =============================================================
                    if (ImGui::BeginTabItem("Flywheel"))
                    {
                        ImGui::Text("Station 1: High-Speed Flywheel Test Stand (Uncapped RPM)");
                        ImGui::SliderFloat("Commanded Voltage##1", &st1CommandedVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Stop (0V)##1")) st1CommandedVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("12.0 V##1")) st1CommandedVolts = 12.0f;

                        auto* m1 = scene.GetComponent<MotorComponent>(st1JointEntity);
                        if (m1)
                        {
                            double rpm = (m1->angularVelocity * 60.0) / (2.0 * 3.1415926535);
                            ImGui::Text("Output Speed:    %.0f RPM", rpm);
                            ImGui::Text("Applied Voltage: %.2f V", m1->appliedVoltage);
                            ImGui::Text("Current Draw:    %.1f A", m1->statorCurrent);
                        }
                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 3: ARTICULATED ARM
                    // =============================================================
                    if (ImGui::BeginTabItem("Geared Arm"))
                    {
                        ImGui::Text("Station 2: Heavy Articulated Arm (High Reduction)");
                        ImGui::Checkbox("Auto Oscillate Arm", &st2AutoOscillate);
                        if (!st2AutoOscillate)
                        {
                            ImGui::SliderFloat("Target Angle##2", &st2TargetAngleDegrees, -70.0f, 70.0f, "%.1f deg");
                        }

                        auto* m2 = scene.GetComponent<MotorComponent>(st2JointEntity);
                        auto* j2 = scene.GetComponent<JointComponent>(st2JointEntity);
                        if (m2 && j2)
                        {
                            ImGui::Text("Arm Angle:       %.1f deg", glm::degrees(j2->currentAngle));
                            ImGui::Text("Holding Torque:  %.2f N*m", m2->outputTorque);
                        }
                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 4: CONTROLS & CAMERA
                    // =============================================================
                    if (ImGui::BeginTabItem("Controls & Sandbox"))
                    {
                        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.2f, 1.0f), "Camera Flight Controls:");
                        ImGui::BulletText("Hold Right Mouse Button + WASD to fly camera freely.");
                        ImGui::BulletText("Hold Space to ascend, Left Shift to descend.");
                        ImGui::BulletText("Press F3 to toggle Profiler & Flame Graph.");
                        ImGui::BulletText("Press 0..9 on keyboard to change simulation speed.");
                        ImGui::Spacing();
                        ImGui::Separator();

                        if (ImGui::Button("Spawn Dynamic Test Box (Falls from ceiling)"))
                        {
                            ++spawnedCount;
                            Entity dropEntity = scene.CreateEntity();
                            float offsetX = static_cast<float>((spawnedCount % 5) - 2) * 1.5f;
                            assignTransforms(dropEntity, glm::vec3(offsetX, 5.0f, 0.2f), glm::vec3(0.5f, 0.5f, 0.5f));

                            MeshComponent mesh;
                            mesh.shape = (spawnedCount % 2 == 0) ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
                            mesh.color = glm::vec3(0.95f, 0.85f, 0.15f);
                            scene.AddComponent(dropEntity, mesh);

                            RigidBodyComponent rigidBody;
                            rigidBody.motionType = BodyMotionType::Dynamic;
                            rigidBody.mass = 2.0f;
                            scene.AddComponent(dropEntity, rigidBody);

                            ColliderComponent collider;
                            collider.shape = (mesh.shape == PrimitiveShape::Cube) ? ColliderShape::Box : ColliderShape::Sphere;
                            collider.boxHalfExtents = glm::vec3(0.25f, 0.25f, 0.25f);
                            collider.radius = 0.25f;
                            scene.AddComponent(dropEntity, collider);
                        }
                        ImGui::Text("Total Dynamic Boxes Spawned: %d", spawnedCount);
                        ImGui::EndTabItem();
                    }

                    ImGui::EndTabBar();
                }
            }
            ImGui::End();

            renderer.EndFrame();

            if (TelemetryManager::Get().GetMode() == TelemetryMode::Full && profilerWindow.IsOpen())
            {
                profilerWindow.Render(renderer.GetWindow());
            }

            auto end = std::chrono::high_resolution_clock::now();
            double renderMs = std::chrono::duration<double, std::milli>(end - start).count();

            FrameSummary summary;
            summary.physicsMs = accumulatedPhysicsMs;
            summary.updateMs = currentUpdateMs;
            summary.renderMs = renderMs;
            summary.totalFrameMs = accumulatedPhysicsMs + currentUpdateMs + renderMs;
            TelemetryManager::Get().PushFrameSummary(summary);

            accumulatedPhysicsMs = 0.0;
        });

    // 10. Run Engine Loop
    engine.Run();

    // 11. Clean Teardown
    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    joltContext.Shutdown();
    TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator shut down cleanly.");
    return 0;
}