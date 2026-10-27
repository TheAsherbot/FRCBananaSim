/**
 * @file Main.cpp
 * @brief Dedicated 6-DOF terrain-aware analytic swerve drivetrain simulation sandbox with real-time telemetry.
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
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/PhysicsDebugDrawSystem.h"
#include "Actuation/BatteryModel.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Robot/RobotSpawner.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Robot/Drivetrain/Model/TerrainProvider.h"
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
#include "Bridge/HALSimWebSocketServer.h"

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
using FRCBananaSim::Physics::Components::BodyMotionType;
using FRCBananaSim::Physics::Components::ColliderShape;
using FRCBananaSim::Physics::Systems::PhysicsSystem;
using FRCBananaSim::Physics::Systems::PhysicsDebugDrawSystem;
using FRCBananaSim::Actuation::BatteryModel;
using FRCBananaSim::Actuation::Components::BatteryComponent;
using FRCBananaSim::Robot::RobotSpawner;
using FRCBananaSim::Robot::SpawnedRobot;
using FRCBananaSim::Robot::Drivetrain::Components::SwerveChassisComponent;
using FRCBananaSim::Robot::Drivetrain::Components::SwerveModuleComponent;
using FRCBananaSim::Robot::Drivetrain::System::SwerveDriveSystem;
using FRCBananaSim::Robot::Drivetrain::Model::SwerveDrivetrainModel;
using FRCBananaSim::Robot::Drivetrain::Model::SwerveModuleState;
using FRCBananaSim::Robot::Drivetrain::Model::ChassisMotionState;
using FRCBananaSim::Robot::Drivetrain::Model::ITerrainProvider;
using FRCBananaSim::Robot::Drivetrain::Model::WheelContactProbe;
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
 * @class AnalyticFieldTerrain
 * @brief Standalone analytic ground terrain provider modeling flat carpet, the 15° ramp, and 3 cm cable bump without Jolt.
 */
class AnalyticFieldTerrain : public ITerrainProvider
{
public:
    virtual WheelContactProbe QueryWheelContact(const glm::vec3& hubPosition,
        double wheelRadius, double /*wheelWidth*/) const override
    {
        WheelContactProbe probe;

        double hubX = hubPosition.x; // NWU +X Forward
        double hubY = hubPosition.y; // NWU +Y Left
        double hubZ = hubPosition.z; // NWU +Z Up

        double groundZ = 0.0;
        glm::vec3 normal(0.0f, 0.0f, 1.0f);

        // Station 5 track lateral bounds in NWU: Y in [2.2, 4.2] (centered at Y = 3.2m)
        bool inTrack = (hubY >= 2.2 && hubY <= 4.2);

        if (inTrack && hubX >= 2.5 && hubX <= 4.5)
        {
            // 15° Inclined ramp section: rises from X = 2.5 to 4.5 along NWU +X
            constexpr double rampAngle = 0.2617993877991494; // 15 degrees in radians
            double distAlongRamp = hubX - 2.5;
            groundZ = distAlongRamp * std::tan(rampAngle);
            normal = glm::vec3(-std::sin(rampAngle), 0.0f, std::cos(rampAngle));
        }
        else if (inTrack && hubX >= 6.9 && hubX <= 7.1)
        {
            // 3 cm Cable protector bump
            groundZ = 0.03;
            normal = glm::vec3(0.0f, 0.0f, 1.0f);
        }

        double distAboveGround = (hubZ - groundZ);
        double depth = wheelRadius - distAboveGround;

        if (depth > 0.0)
        {
            probe.hasContact = true;
            probe.penetrationDepth = depth;
            probe.surfaceNormal = normal;
            probe.contactPoint = hubPosition - normal * static_cast<float>(wheelRadius);
        }
        else
        {
            probe.hasContact = false;
            probe.penetrationDepth = 0.0;
            probe.surfaceNormal = normal;
            probe.contactPoint = glm::vec3(hubX, hubY, static_cast<float>(groundZ));
        }

        return probe;
    }
};

/**
 * @brief Draws oriented bounding box (OBB) wireframes using DebugDraw lines.
 */
static void DrawOrientedBox(const glm::vec3& center, const glm::quat& orientation,
    const glm::vec3& halfExtents, const glm::vec3& color, bool alwaysOnTop)
{
    glm::vec3 c[8] = {
        center + orientation * glm::vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
        center + orientation * glm::vec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
        center + orientation * glm::vec3(halfExtents.x,  halfExtents.y, -halfExtents.z),
        center + orientation * glm::vec3(-halfExtents.x,  halfExtents.y, -halfExtents.z),
        center + orientation * glm::vec3(-halfExtents.x, -halfExtents.y,  halfExtents.z),
        center + orientation * glm::vec3(halfExtents.x, -halfExtents.y,  halfExtents.z),
        center + orientation * glm::vec3(halfExtents.x,  halfExtents.y,  halfExtents.z),
        center + orientation * glm::vec3(-halfExtents.x,  halfExtents.y,  halfExtents.z)
    };

    bool depthTest = !alwaysOnTop;

    // Bottom loop
    DebugDraw::Line(c[0], c[1], color, depthTest);
    DebugDraw::Line(c[1], c[2], color, depthTest);
    DebugDraw::Line(c[2], c[3], color, depthTest);
    DebugDraw::Line(c[3], c[0], color, depthTest);

    // Top loop
    DebugDraw::Line(c[4], c[5], color, depthTest);
    DebugDraw::Line(c[5], c[6], color, depthTest);
    DebugDraw::Line(c[6], c[7], color, depthTest);
    DebugDraw::Line(c[7], c[4], color, depthTest);

    // Vertical pillars
    DebugDraw::Line(c[0], c[4], color, depthTest);
    DebugDraw::Line(c[1], c[5], color, depthTest);
    DebugDraw::Line(c[2], c[6], color, depthTest);
    DebugDraw::Line(c[3], c[7], color, depthTest);

    // Top face cross (X) for heading identification
    DebugDraw::Line(c[4], c[6], color * 0.75f, depthTest);
    DebugDraw::Line(c[5], c[7], color * 0.75f, depthTest);
}

/**
 * @brief Renders wireframe debug visuals for all scene colliders and wheel contact patches.
 */
static void RenderSceneColliderDebugDraw(Scene& scene, bool alwaysOnTop,
    const SpawnedRobot& swerveRobot, SwerveDriveSystem& swerveSystem,
    Entity groundEntity, Entity rampEntity, Entity bumpEntity, bool showWheels)
{
    auto entities = scene.Query<ColliderComponent, TransformComponent>();

    for (Entity entity : entities)
    {
        auto* collider = scene.GetComponent<ColliderComponent>(entity);
        auto* transform = scene.GetComponent<TransformComponent>(entity);
        if (!collider || !transform)
        {
            continue;
        }

        glm::vec3 center = transform->position + transform->rotation * collider->localOffset;
        glm::quat orientation = transform->rotation * collider->localRotation;

        glm::vec3 color(0.2f, 0.9f, 0.3f);
        if (entity == groundEntity)
        {
            color = glm::vec3(0.15f, 0.95f, 0.35f); // Vibrant Carpet Green
        }
        else if (entity == rampEntity)
        {
            color = glm::vec3(0.20f, 0.85f, 1.0f);  // Cyan Ramp
        }
        else if (entity == bumpEntity)
        {
            color = glm::vec3(1.0f, 0.85f, 0.15f);  // Amber Cable Bump
        }
        else if (entity == swerveRobot.chassisEntity)
        {
            color = glm::vec3(1.0f, 0.35f, 0.25f);  // Coral Chassis Bumper
        }

        switch (collider->shape)
        {
        case ColliderShape::Box:
            DrawOrientedBox(center, orientation, collider->boxHalfExtents, color, alwaysOnTop);
            break;
        case ColliderShape::Sphere:
            DebugDraw::Sphere(center, collider->radius, color, 16, !alwaysOnTop);
            break;
        case ColliderShape::Cylinder:
        case ColliderShape::Capsule:
            DebugDraw::Cylinder(center, collider->radius, collider->halfHeight, color, 16, !alwaysOnTop);
            break;
        }
    }

    // Wheel visualizer
    if (showWheels && swerveRobot.isSuccess)
    {
        auto* model = swerveSystem.GetModel();
        constexpr int kSegments = 16;
        constexpr float kRadius = 0.0508f; // 2 inches radius = 4" wheel

        for (uint32_t i = 0; i < 4; ++i)
        {
            if (swerveRobot.wheelEntities[i] == invalidEntity)
            {
                continue;
            }

            auto* wheelT = scene.GetComponent<TransformComponent>(swerveRobot.wheelEntities[i]);
            if (!wheelT)
            {
                continue;
            }

            glm::vec3 wheelPos = wheelT->position;
            glm::quat wheelRot = wheelT->rotation;

            // Rolling circumference circle in wheel local YZ plane
            glm::vec3 prevPoint = wheelPos + wheelRot * glm::vec3(0.0f, kRadius, 0.0f);
            for (int s = 1; s <= kSegments; ++s)
            {
                float angle = (static_cast<float>(s) / static_cast<float>(kSegments)) * 6.2831853f;
                glm::vec3 localPt(0.0f, std::cos(angle) * kRadius, std::sin(angle) * kRadius);
                glm::vec3 worldPt = wheelPos + wheelRot * localPt;
                DebugDraw::Line(prevPoint, worldPt, glm::vec3(1.0f, 0.85f, 0.2f), !alwaysOnTop);
                prevPoint = worldPt;
            }

            // Normal contact load indicator
            double normalF = 0.0;
            if (model)
            {
                normalF = model->GetModule(i).GetState().contactState.normalForce;
            }

            glm::vec3 bottomPt = wheelPos - glm::vec3(0.0f, kRadius, 0.0f);
            if (normalF > 0.0)
            {
                float spikeH = static_cast<float>(std::min(0.20, normalF / 800.0));
                DebugDraw::Line(bottomPt, bottomPt + glm::vec3(0.0f, spikeH, 0.0f), glm::vec3(0.1f, 1.0f, 0.2f), !alwaysOnTop);
            }
            else
            {
                DebugDraw::Line(bottomPt, bottomPt + glm::vec3(0.0f, 0.04f, 0.0f), glm::vec3(1.0f, 0.2f, 0.2f), !alwaysOnTop);
            }
        }
    }
}

int main(int argc, char* argv[])
{
    FRCBananaSim::Bridge::HALSimWebSocketServer wsServer;
    wsServer.Start();

    // 1. Initialize Telemetry System
    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal);

    // 2. Initialize Visual Renderer
    Renderer renderer(1600, 900, "FRC Banana Simulator - 6-DOF Terrain Swerve Sandbox");

    // Camera initial viewpoint (framed behind robot looking along the track towards ramp and bump)
    renderer.GetCamera().position = glm::vec3(3.2f, 2.6f, -3.2f);
    renderer.GetCamera().pitchDegrees = -14.0f;
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

    scene.AddSystem(std::make_unique<PhysicsSystem>(scene, joltContext));
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Helper lambda to assign both local and world transforms at creation
    auto assignTransforms = [&](Entity entity, const glm::vec3& pos, const glm::vec3& scale,
        const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
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

    // A. Field Carpet (28m x 18m)
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

    // B. Obstacle 1: 15° Inclined Ramp (Centered at X = 3.2m, Z = 3.5m)
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

    // C. Obstacle 2: 3 cm Cable-Protector Bump (Centered at X = 3.2m, Z = 7.0m)
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

    // D. Central Robot Battery Entity
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

    // -------------------------------------------------------------------------
    // 7. Swerve Robot Spawning & Visual Attachment
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

    // Drivetrain Adapter initialized with Jolt context for terrain probes
    SwerveDriveSystem swerveSystem(scene, &joltContext);
    auto analyticTerrain = std::make_shared<AnalyticFieldTerrain>();

    SimInputs swerveInputs;
    SimOutputs swerveOutputs;

    // Simulation Backend Selection: true = Jolt Middleware, false = No Jolt (Pure Analytic Math)
    bool useJoltPhysics = true;

    // Debug Draw Options
    bool showColliderDebugDraw = true;
    bool debugDrawAlwaysOnTop = false;
    bool showWheelContactDebugDraw = true;

    bool swerveEnabled = true;
    bool swervePinChassis = false;
    float swerveDriveVolts = 0.0f;
    float swerveTargetSteerAngle = 0.0f;
    int swerveNeutralModeIndex = 0; // 0 = Brake, 1 = Coast

    bool swerveRunStressTest = false;
    double swerveStressElapsed = 0.0;

    // Helper lambda to reposition the robot cleanly
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
                // Convert Engine (+X Left, +Y Up, +Z Forward) to NWU (+X Fwd, +Y Left, +Z Up)
                glm::vec3 posNWU(worldPos.z, worldPos.x, worldPos.y);
                glm::quat rotNWU(worldRot.w, worldRot.z, worldRot.x, worldRot.y);
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
    // 8. Interactive State & Simulation Stepper
    // -------------------------------------------------------------------------
    double simSpeed = 1.0;
    double simAccumulator = 0.0;
    bool doSingleStep = false;
    uint64_t totalSimSteps = 0;

    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;

    auto executeSimulationStep = [&](double fixedDeltaTime)
        {
            if (!swerveRobot.isSuccess)
            {
                return;
            }


            auto* battery = scene.GetComponent<BatteryComponent>(batteryEntity);

            // 1. Pin/Unpin chassis via motion type (in Jolt mode)
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(swerveRobot.chassisEntity);
            if (useJoltPhysics && chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
            {
                JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                JPH::EMotionType targetMotion = swervePinChassis ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
                if (bi.GetMotionType(chassisRb->bodyID) != targetMotion)
                {
                    bi.SetMotionType(chassisRb->bodyID, targetMotion, JPH::EActivation::Activate);
                }
            }

            // Ingest incoming motor demands from WPILib robot code if connected
            if (wsServer.IsClientConnected())
            {
                SimInputs netInputs = swerveInputs;
                if (wsServer.UpdateInputs(netInputs))
                {
                    swerveInputs = netInputs;
                }
            }

            // 2. Control demands
            if (swerveRunStressTest)
            {
                swerveStressElapsed += fixedDeltaTime;
                swerveInputs.robotEnabled = true;

                for (uint32_t i = 0; i < 4; ++i)
                {
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

                // Closed-loop azimuth heading hold via simulated encoder feedback
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

            // 3. Advance Swerve Drivetrain Simulation
            swerveSystem.SetInputs(swerveInputs);
            swerveSystem.FixedUpdate(fixedDeltaTime);
            swerveOutputs = swerveSystem.GetOutputs();

            // Broadcast sensory outputs to connected WPILib robot code
            if (wsServer.IsClientConnected())
            {
                wsServer.SendOutputs(swerveOutputs);
            }

            // 4. Central Battery Bus Update
            if (battery)
            {
                double totalActuatorSupplyCurrent = swerveOutputs.totalBusCurrent;
                battery->terminalVoltage = battery->batteryModel.CalculateTerminalVoltage(totalActuatorSupplyCurrent);
                battery->totalCurrent = totalActuatorSupplyCurrent + battery->batteryModel.GetQuiescentCurrent();
                battery->isBrownoutWarning = battery->batteryModel.IsBrownoutWarning(battery->terminalVoltage);
                battery->isBrownoutCritical = battery->batteryModel.IsBrownoutCritical(battery->terminalVoltage);
            }

            // 5. Physics Integration Mode
            if (useJoltPhysics)
            {
                // Jolt physics integration pass
                scene.SetFixedDeltaTime(fixedDeltaTime);
                scene.FixedUpdateSystems();

                // Sync chassis transform into WorldTransformComponent
                auto* t = scene.GetComponent<TransformComponent>(swerveRobot.chassisEntity);
                auto* wt = scene.GetComponent<WorldTransformComponent>(swerveRobot.chassisEntity);
                if (t && wt)
                {
                    wt->worldPosition = t->position;
                    wt->worldRotation = t->rotation;
                    wt->worldScale = t->scale;
                }
            }
            else
            {
                // No-Jolt Mode: SwerveDrivetrainModel already integrated its 6-DOF pose in Step()
                auto* model = swerveSystem.GetModel();
                if (model)
                {
                    const auto& motion = model->GetChassisState();
                    // NWU (+X Fwd, +Y Left, +Z Up) -> Engine (+X Left, +Y Up, +Z Forward)
                    glm::vec3 posEng(motion.position.y, motion.position.z, motion.position.x);
                    glm::quat rotEng(motion.orientation.w, motion.orientation.y, motion.orientation.z, motion.orientation.x);

                    auto* chassisT = scene.GetComponent<TransformComponent>(swerveRobot.chassisEntity);
                    if (chassisT)
                    {
                        chassisT->position = posEng;
                        chassisT->rotation = rotEng;
                    }

                    auto* chassisWt = scene.GetComponent<WorldTransformComponent>(swerveRobot.chassisEntity);
                    if (chassisWt)
                    {
                        chassisWt->worldPosition = posEng;
                        chassisWt->worldRotation = rotEng;
                    }

                    // Keep Jolt body position synchronized in background for seamless transitions
                    if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
                    {
                        JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                        bi.SetPositionAndRotation(chassisRb->bodyID,
                            JPH::RVec3(posEng.x, posEng.y, posEng.z),
                            JPH::Quat(rotEng.x, rotEng.y, rotEng.z, rotEng.w),
                            JPH::EActivation::DontActivate);
                    }
                }
            }

            ++totalSimSteps;
        };

    // -------------------------------------------------------------------------
    // 9. Engine Loop Configuration
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
    // 10. Per-Frame Update Callback & ImGui HUD
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
                    swerveTargetSteerAngle += static_cast<float>(deltaTime * 2.5); // Slew Left (+Y NWU)
                }
                else if (input.IsKeyDown(GLFW_KEY_RIGHT) || input.IsKeyDown(GLFW_KEY_KP_6))
                {
                    swerveTargetSteerAngle -= static_cast<float>(deltaTime * 2.5); // Slew Right (-Y NWU)
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

            // 1. Queue all collider wireframes (Green carpet, Cyan ramp, Amber bump, Coral chassis)
            if (showColliderDebugDraw)
            {
                RenderSceneColliderDebugDraw(scene, debugDrawAlwaysOnTop, swerveRobot, swerveSystem,
                    groundEntity, rampEntity, bumpEntity, showWheelContactDebugDraw);
            }

            // 2. Render solid meshes
            scene.RenderUpdateSystems();
            swerveSystem.RenderUpdate();

            // 3. Flush all queued debug wireframes to screen
            renderer.FlushDebugDraw();

            // 4. Render Telemetry HUD
            telemetryOverlay.Render();

            // --- ImGui Interactive Control Center ---
            ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(520.0f, 760.0f), ImGuiCond_FirstUseEver);

            if (ImGui::Begin("FRCBananaSim — 6-DOF Terrain Swerve Sandbox"))
            {
                // Simulation Speed Controls
                ImGui::Text("Simulation Time: %.2f s | Speed: %.2fx",
                    static_cast<double>(totalSimSteps) * PhysicsConfig::fixedDeltaTime, simSpeed);

                if (ImGui::Button("Pause (0x)")) simSpeed = 0.0; ImGui::SameLine();
                if (ImGui::Button("0.25x")) simSpeed = 0.25; ImGui::SameLine();
                if (ImGui::Button("1.0x (Realtime)")) simSpeed = 1.0; ImGui::SameLine();
                if (ImGui::Button("2.0x (Fast)")) simSpeed = 2.0;

                ImGui::Separator();

                // -------------------------------------------------------------
                // SIMULATION BACKEND SELECTION: JOLT VS. NO-JOLT
                // -------------------------------------------------------------
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "SIMULATION BACKEND MODE:");
                if (ImGui::RadioButton("Jolt Physics (Middleware)", useJoltPhysics))
                {
                    if (!useJoltPhysics)
                    {
                        useJoltPhysics = true;
                        swerveSystem.SetCustomTerrainProvider(nullptr); // Restores Jolt CollideShape probe
                    }
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("No Jolt (Pure Analytic Math)", !useJoltPhysics))
                {
                    if (useJoltPhysics)
                    {
                        useJoltPhysics = false;
                        swerveSystem.SetCustomTerrainProvider(analyticTerrain);
                    }
                }

                if (useJoltPhysics)
                {
                    ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "Active: Jolt 6-DOF Body + Jolt Terrain CollideShape Queries");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "Active: Pure Analytic Math + AnalyticFieldTerrain (Jolt Bypassed)");
                }

                ImGui::Separator();

                // -------------------------------------------------------------
                // DEBUG DRAW OPTIONS
                // -------------------------------------------------------------
                ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.85f, 1.0f), "COLLIDER DEBUG DRAW OPTIONS:");
                ImGui::Checkbox("Draw All Colliders (Wireframe OBBs)", &showColliderDebugDraw);
                ImGui::SameLine();
                ImGui::Checkbox("Always On Top (No Depth Occlusion)", &debugDrawAlwaysOnTop);
                ImGui::Checkbox("Draw Wheel Contact Patches & Load Spikes", &showWheelContactDebugDraw);

                ImGui::Separator();

                // -------------------------------------------------------------
                // WPILIB HALSIM WEBSOCKET BRIDGE
                // -------------------------------------------------------------
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "WPILIB HALSIM WEBSOCKET BRIDGE (:3300/wpilibws):");
                if (wsServer.IsClientConnected())
                {
                    ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.3f, 1.0f), "● CONNECTED to Live Robot Code");
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "○ Waiting for ./gradlew simulateNative...");
                }

                bool isDsEnabled = wsServer.IsDriverStationEnabled();
                if (ImGui::Checkbox("DS Enable Robot", &isDsEnabled))
                {
                    wsServer.SetDriverStation(isDsEnabled, wsServer.IsAutonomous());
                }
                ImGui::SameLine();
                bool isAuto = wsServer.IsAutonomous();
                if (ImGui::Checkbox("Autonomous Mode", &isAuto))
                {
                    wsServer.SetDriverStation(wsServer.IsDriverStationEnabled(), isAuto);
                }

                if (ImGui::Button("Dump Discovered SimDevices to JSON"))
                {
                    wsServer.DumpSimDevices("Resources/Telemetry/sim_device_dump.json");
                }

                ImGui::Separator();

                // Main Tab Bar
                if (ImGui::BeginTabBar("MainTabs"))
                {
                    // =============================================================
                    // TAB 1: SWERVE ROBOT (DRIVE & 6-DOF TERRAIN)
                    // =============================================================
                    if (ImGui::BeginTabItem("Swerve Robot"))
                    {
                        // Reset Shortcuts
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "OBSTACLE COURSE BENCH POSITIONS");
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
                        ImGui::Checkbox("Pin Chassis to Stand", &swervePinChassis);
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("When checked, the chassis body is held stationary so stall torque and peak RPM can be tested safely.\nUncheck to let the robot drive across carpet, ramps, and bumps.");
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

                        // Chassis linear velocity & altitude
                        auto* chassisT = scene.GetComponent<TransformComponent>(swerveRobot.chassisEntity);
                        glm::vec3 chassisLinVel(0.0f);
                        auto* model = swerveSystem.GetModel();
                        if (model)
                        {
                            // Model linear velocity in NWU -> Engine (+X Left, +Y Up, +Z Forward)
                            const auto& motion = model->GetChassisState();
                            chassisLinVel = glm::vec3(motion.linearVelocity.y, motion.linearVelocity.z, motion.linearVelocity.x);
                        }
                        float groundSpeed = glm::length(glm::vec2(chassisLinVel.x, chassisLinVel.z));
                        float elevationY = chassisT ? chassisT->position.y : 0.0f;
                        ImGui::Text("Speed:     %.2f m/s (%.1f ft/s) | Elev (Y): %.3f m", groundSpeed, groundSpeed * 3.28084f, elevationY);

                        // Energy Ledger
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
                    // TAB 2: CONTROLS & CAMERA
                    // =============================================================
                    if (ImGui::BeginTabItem("Controls & Camera"))
                    {
                        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.2f, 1.0f), "Camera Flight Controls:");
                        ImGui::BulletText("Hold Right Mouse Button + WASD to fly camera freely.");
                        ImGui::BulletText("Hold Space to ascend, Left Shift to descend.");
                        ImGui::BulletText("Press F3 to toggle Profiler & Flame Graph.");
                        ImGui::BulletText("Press 0..9 on keyboard to change simulation speed.");
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

    // 11. Run Engine Loop
    engine.Run();

    // 12. Clean Teardown
    wsServer.Stop();

    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    joltContext.Shutdown();
    TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator shut down cleanly.");
    return 0;
}