/**
 * @file Main.cpp
 * @brief Interactive 3D simulation sandbox and test station for the Schema v1 2026 Reference Robot.
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
#include "Actuation/Components/BatteryComponent.h"
#include "Robot/RobotAssembler.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Components/FourBarComponent.h"
#include "Robot/Mechanisms/Components/CouplingComponent.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"
#include "Robot/Sensors/Components/IMUComponent.h"
#include "Robot/Sensors/Components/EncoderComponent.h"
#include "Robot/Sensors/Components/LimitSwitchComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Components/AnalogInputComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Renderer/Renderer.h"
#include "Renderer/Components/MeshComponent.h"
#include "Renderer/Systems/RenderSystem.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Telemetry/TelemeteryViewer/TelemetryManager.h"
#include "Telemetry/TelemeteryViewer/TelemetryOverlay.h"
#include "Telemetry/TelemeteryViewer/ProfilerWindow.h"
#include "Telemetry/DrivetrainLogger.h"
#include "Bridge/HALSimWebSocketServer.h"
#include "Control/SimControlBoundary.h"
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
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

using namespace FRCBananaSim::Core::ECS::Core;
using namespace FRCBananaSim::Core::ECS::Components;
using namespace FRCBananaSim::Physics::Components;
using namespace FRCBananaSim::Renderer::Components;
using namespace FRCBananaSim::Control;
using namespace FRCBananaSim::Robot::Mechanisms::Components;
using namespace FRCBananaSim::Robot::Sensors::Components;

/**
 * @brief Helper drawing an oriented wireframe box in world coordinates.
 */
static void DrawOrientedBox(const glm::vec3& center, const glm::quat& orientation,
    const glm::vec3& halfExtents, const glm::vec3& color, bool depthTest)
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

    // Bottom loop
    FRCBananaSim::Utils::DebugDraw::Line(c[0], c[1], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[1], c[2], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[2], c[3], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[3], c[0], color, depthTest);

    // Top loop
    FRCBananaSim::Utils::DebugDraw::Line(c[4], c[5], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[5], c[6], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[6], c[7], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[7], c[4], color, depthTest);

    // Vertical pillars
    FRCBananaSim::Utils::DebugDraw::Line(c[0], c[4], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[1], c[5], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[2], c[6], color, depthTest);
    FRCBananaSim::Utils::DebugDraw::Line(c[3], c[7], color, depthTest);
}

/**
 * @brief Renders wireframe colliders, contact patches, and contact volumes.
 */
static void RenderSceneDebugDraw(Scene& scene, bool showColliders, bool showWheelContacts,
    bool showContactZones, bool alwaysOnTop, const FRCBananaSim::Robot::AssembledRobot& robot)
{
    bool depthTest = !alwaysOnTop;

    if (showColliders)
    {
        auto entities = scene.Query<ColliderComponent, TransformComponent>();
        for (Entity e : entities)
        {
            auto* col = scene.GetComponent<ColliderComponent>(e);
            auto* t = scene.GetComponent<TransformComponent>(e);
            if (!col || !t) continue;

            glm::vec3 center = t->position + (t->rotation * col->localOffset);
            glm::quat rot = t->rotation * col->localRotation;

            glm::vec3 color(0.2f, 0.9f, 0.3f);
            if (e == robot.chassisEntity)
            {
                color = glm::vec3(0.2f, 0.5f, 1.0f); // Blue Chassis Bumper
            }

            switch (col->shape)
            {
            case ColliderShape::Box:
                DrawOrientedBox(center, rot, col->boxHalfExtents, color, depthTest);
                break;
            case ColliderShape::Sphere:
                FRCBananaSim::Utils::DebugDraw::Sphere(center, col->radius, color, 16, depthTest);
                break;
            case ColliderShape::Cylinder:
            case ColliderShape::Capsule:
                FRCBananaSim::Utils::DebugDraw::Cylinder(center, col->radius, col->halfHeight, color, 16, depthTest);
                break;
            }
        }
    }

    if (showContactZones)
    {
        auto czEntities = scene.Query<ContactZoneComponent, TransformComponent>();
        for (Entity e : czEntities)
        {
            auto* cz = scene.GetComponent<ContactZoneComponent>(e);
            auto* t = scene.GetComponent<TransformComponent>(e);
            if (!cz || !t) continue;

            glm::vec3 color = (cz->role == "intake")
                ? glm::vec3(0.1f, 1.0f, 0.2f)   // Green intake volume
                : glm::vec3(1.0f, 0.2f, 0.8f);  // Magenta shooter volume

            DrawOrientedBox(t->position, t->rotation, cz->dimensions * 0.5f, color, false);
        }
    }

    if (showWheelContacts && robot.swerveSystem)
    {
        auto* model = robot.swerveSystem->GetModel();
        if (model)
        {
            auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
            if (chassisT)
            {
                constexpr float kRadius = 0.0508f;
                for (uint32_t i = 0; i < 4; ++i)
                {
                    const auto& modState = model->GetModule(i).GetState();
                    const auto& modPlacement = model->GetModule(i).GetPlacement();
                    glm::vec3 modLocEng(modPlacement.locationOnChassis.y, -0.075f - 0.060f - 0.010f - kRadius, modPlacement.locationOnChassis.x);
                    glm::vec3 worldWheelPos = chassisT->position + (chassisT->rotation * modLocEng);

                    double normalF = modState.contactState.normalForce;
                    if (normalF > 0.0)
                    {
                        float spikeH = static_cast<float>(std::min(0.25, normalF / 600.0));
                        glm::vec3 spikeColor = (modState.contactState.saturationRatio > 1.0)
                            ? glm::vec3(1.0f, 0.2f, 0.2f)   // Red slip spike
                            : glm::vec3(0.2f, 1.0f, 0.3f);  // Green traction spike
                        FRCBananaSim::Utils::DebugDraw::Line(worldWheelPos, worldWheelPos + glm::vec3(0.0f, spikeH, 0.0f), spikeColor, depthTest);
                    }
                    else
                    {
                        FRCBananaSim::Utils::DebugDraw::Line(worldWheelPos, worldWheelPos + glm::vec3(0.0f, 0.03f, 0.0f), glm::vec3(0.8f, 0.2f, 0.2f), depthTest);
                    }
                }
            }
        }
    }
}

int main(int argc, char* argv[])
{
    // 1. Initialize Telemetry & Profiler Subsystems
    FRCBananaSim::Telemetry::TelemetryManager::Get().Initialize();
    FRCBananaSim::Telemetry::TelemetryManager::Get().SetMode(FRCBananaSim::Telemetry::TelemetryMode::Minimal);

    FRCBananaSim::Telemetry::TelemetryOverlay telemetryOverlay;
    FRCBananaSim::Telemetry::ProfilerWindow profilerWindow;
    FRCBananaSim::Telemetry::DrivetrainLogger drivetrainLogger(120000); // 10 min at 200 Hz

    // 2. Initialize Hardware Renderer (1600x900 OpenGL 3.3 Core)
    FRCBananaSim::Renderer::Renderer renderer(1600, 900, "FRC Banana Simulator — Block 3 Reference 2026 Robot Test Station");
    if (!renderer.IsValid())
    {
        SIM_LOG_FATAL("Failed to initialize OpenGL renderer window");
        return -1;
    }

    // 3. Initialize Jolt Physics Subsystem
    FRCBananaSim::Physics::JoltContext joltContext;
    if (!joltContext.Initialize())
    {
        SIM_LOG_FATAL("Failed to initialize Jolt Physics Context");
        return -1;
    }

    // 4. Start WPILib HALSim WebSocket Server (:3300/wpilibws)
    FRCBananaSim::Bridge::HALSimWebSocketServer wsServer;
    wsServer.Start();

    // 5. ECS Scene & Core Presentation Systems
    Scene scene;
    scene.SetFixedDeltaTime(FRCBananaSim::Physics::PhysicsConfig::fixedDeltaTime);

    scene.AddSystem(std::make_unique<FRCBananaSim::Core::ECS::Systems::TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<FRCBananaSim::Renderer::Systems::RenderSystem>(scene, renderer));

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
    // 6. Environment Setup (Field Carpet, Obstacle Ramp, Bump & Target Goal)
    // -------------------------------------------------------------------------

    // A. Official FRC Field Carpet (28m x 18m, Top Surface at Y = 0.0m)
    Entity groundEntity = scene.CreateEntity();
    {
        assignTransforms(groundEntity, glm::vec3(0.0f, -0.05f, 0.0f), glm::vec3(28.0f, 0.10f, 18.0f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Plane;
        mesh.color = glm::vec3(0.18f, 0.45f, 0.22f); // FRC Carpet Green
        scene.AddComponent(groundEntity, mesh);

        RigidBodyComponent rb;
        rb.motionType = BodyMotionType::Static;
        rb.friction = 0.85f;
        rb.restitution = 0.05f;
        scene.AddComponent(groundEntity, rb);

        ColliderComponent col;
        col.shape = ColliderShape::Box;
        col.boxHalfExtents = glm::vec3(14.0f, 0.05f, 9.0f);
        scene.AddComponent(groundEntity, col);
    }

    // B. Obstacle 1: 15° Inclined Test Ramp
    Entity rampEntity = scene.CreateEntity();
    {
        glm::vec3 rampPos(3.2f, 0.259f, 4.0f);
        glm::quat rampRot = glm::angleAxis(glm::radians(-15.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        assignTransforms(rampEntity, rampPos, glm::vec3(2.0f, 0.08f, 2.0f), rampRot);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.28f, 0.48f, 0.72f); // Steel Blue Ramp
        scene.AddComponent(rampEntity, mesh);

        RigidBodyComponent rb;
        rb.motionType = BodyMotionType::Static;
        rb.friction = 0.80f;
        scene.AddComponent(rampEntity, rb);

        ColliderComponent col;
        col.shape = ColliderShape::Box;
        col.boxHalfExtents = glm::vec3(1.0f, 0.04f, 1.0f);
        scene.AddComponent(rampEntity, col);
    }

    // C. Obstacle 2: 3 cm Cable-Protector Bump
    Entity bumpEntity = scene.CreateEntity();
    {
        glm::vec3 bumpPos(3.2f, 0.015f, 7.5f);
        assignTransforms(bumpEntity, bumpPos, glm::vec3(2.0f, 0.03f, 0.20f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.80f, 0.12f); // Hazard Yellow
        scene.AddComponent(bumpEntity, mesh);

        RigidBodyComponent rb;
        rb.motionType = BodyMotionType::Static;
        rb.friction = 0.80f;
        scene.AddComponent(bumpEntity, rb);

        ColliderComponent col;
        col.shape = ColliderShape::Box;
        col.boxHalfExtents = glm::vec3(1.0f, 0.015f, 0.10f);
        scene.AddComponent(bumpEntity, col);
    }

    // D. Target Rebounder Structure
    Entity targetEntity = scene.CreateEntity();
    {
        glm::vec3 targetPos(0.0f, 0.75f, 7.0f);
        assignTransforms(targetEntity, targetPos, glm::vec3(1.4f, 1.5f, 0.4f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.70f, 0.22f, 0.24f); // Alliance Red Target Goal
        scene.AddComponent(targetEntity, mesh);

        RigidBodyComponent rb;
        rb.motionType = BodyMotionType::Static;
        scene.AddComponent(targetEntity, rb);

        ColliderComponent col;
        col.shape = ColliderShape::Box;
        col.boxHalfExtents = glm::vec3(0.7f, 0.75f, 0.2f);
        scene.AddComponent(targetEntity, col);
    }

    // -------------------------------------------------------------------------
    // 7. Assemble Schema v1 Reference 2026 Robot
    // -------------------------------------------------------------------------
    std::string manifestPath = "FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
    std::string sidecarPath = "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
    std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

    const std::vector<std::string> searchPrefixes = {
        "",
        "../",
        "../../",
        "../../../",
        "../../../../",
        "FRCBananaSim/"
    };

    for (const auto& prefix : searchPrefixes)
    {
        std::string candidate = prefix + "FRCBananaSim/Resources/Robots/Reference2026/manifest.json";
        if (std::filesystem::exists(candidate))
        {
            manifestPath = candidate;
            sidecarPath = prefix + "FRCBananaSim/Resources/Robots/Reference2026/sidecar.json";
            materialsPath = prefix + "FRCBananaSim/Resources/Config/materials.json";
            break;
        }
        std::string directCandidate = prefix + "Resources/Robots/Reference2026/manifest.json";
        if (std::filesystem::exists(directCandidate))
        {
            manifestPath = directCandidate;
            sidecarPath = prefix + "Resources/Robots/Reference2026/sidecar.json";
            materialsPath = prefix + "Resources/Config/materials.json";
            break;
        }
    }

    FRCBananaSim::Robot::RobotAssembler assembler(scene, &joltContext);
    const glm::vec3 robotSpawnPos(0.0f, 0.1989f, 0.0f);
    FRCBananaSim::Robot::AssembledRobot robot = assembler.AssembleFromFiles(
        manifestPath, sidecarPath, materialsPath, robotSpawnPos);

    if (!robot.isSuccess)
    {
        SIM_LOG_FATAL("Failed to assemble Schema v1 robot: " << robot.errorMessage);
        return -1;
    }

    // Joint System for Jolt constraints management
    FRCBananaSim::Physics::Systems::JointSystem jointSystem(scene, joltContext);

    // Initial Warmup Pass: Allocate all Jolt rigid bodies and joint constraints
    // before the first simulation time step to prevent initial impulse explosion.
    if (robot.physicsSystem)
    {
        robot.physicsSystem->FixedUpdate(0.0);
    }
    
    jointSystem.FixedUpdate(0.0);
    jointSystem.ReadbackTelemetry();
    
    std::cout << "[Simulator] Robot assembled and physics bodies warmed up successfully.\n";

    // -------------------------------------------------------------------------
    // 8. Visual Mesh & Scale Attachment for All Robot Links
    // -------------------------------------------------------------------------

    auto attachVisualMesh = [&](Entity linkEntity, PrimitiveShape shape, const glm::vec3& color,
        const glm::vec3& fallbackScale = glm::vec3(0.04f),
        const glm::vec3& fallbackOffset = glm::vec3(0.0f),
        const glm::quat& fallbackRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        {
            if (linkEntity == invalidEntity) return;

            auto* linkT = scene.GetComponent<TransformComponent>(linkEntity);
            if (!linkT) return;

            // Preserve neutral unit scale on link entity to prevent skewing child joints/colliders
            linkT->scale = glm::vec3(1.0f);
            auto* linkWt = scene.GetComponent<WorldTransformComponent>(linkEntity);
            if (linkWt) linkWt->worldScale = glm::vec3(1.0f);

            auto* col = scene.GetComponent<ColliderComponent>(linkEntity);
            glm::vec3 visualOffset = col ? col->localOffset : fallbackOffset;
            glm::quat visualRot = col ? col->localRotation : fallbackRot;
            glm::vec3 visualScale = fallbackScale;

            if (col)
            {
                if (col->shape == ColliderShape::Box)
                {
                    visualScale = col->boxHalfExtents * 2.0f;
                }
                else if (col->shape == ColliderShape::Cylinder || col->shape == ColliderShape::Capsule)
                {
                    visualScale = glm::vec3(col->radius * 2.0f, col->halfHeight * 2.0f, col->radius * 2.0f);
                }
                else if (col->shape == ColliderShape::Sphere)
                {
                    visualScale = glm::vec3(col->radius * 2.0f);
                }
            }

            Entity visualEntity = scene.CreateEntity();
            TransformComponent visualT;
            visualT.position = visualOffset;
            visualT.rotation = visualRot;
            visualT.scale = visualScale;
            scene.AddComponent(visualEntity, visualT);

            WorldTransformComponent visualWt;
            visualWt.worldPosition = linkT->position + (linkT->rotation * visualOffset);
            visualWt.worldRotation = linkT->rotation * visualRot;
            visualWt.worldScale = visualScale;
            scene.AddComponent(visualEntity, visualWt);

            HierarchyComponent visualH;
            visualH.parent = linkEntity;
            scene.AddComponent(visualEntity, visualH);

            auto* linkH = scene.GetComponent<HierarchyComponent>(linkEntity);
            if (linkH)
            {
                linkH->children.push_back(visualEntity);
            }
            else
            {
                HierarchyComponent newLinkH;
                newLinkH.children.push_back(visualEntity);
                scene.AddComponent(linkEntity, newLinkH);
            }

            MeshComponent mesh;
            mesh.shape = shape;
            mesh.color = color;
            scene.AddComponent(visualEntity, mesh);
        };

    // A. Chassis Bumpers
    attachVisualMesh(robot.chassisEntity, PrimitiveShape::Cube, glm::vec3(0.14f, 0.28f, 0.52f)); // Royal Blue Chassis Bumper

    // B. Elevator Stages
    auto s1It = robot.linkEntities.find("elevator_stage_1");
    if (s1It != robot.linkEntities.end())
    {
        attachVisualMesh(s1It->second, PrimitiveShape::Cube, glm::vec3(0.78f, 0.82f, 0.86f)); // Brushed Aluminum Silver
    }

    auto carrIt = robot.linkEntities.find("elevator_carriage");
    if (carrIt != robot.linkEntities.end())
    {
        attachVisualMesh(carrIt->second, PrimitiveShape::Cube, glm::vec3(0.24f, 0.26f, 0.30f)); // Dark Carbon Carriage
    }

    // C. Arm Links
    auto armIt = robot.linkEntities.find("arm_shoulder_link");
    if (armIt != robot.linkEntities.end())
    {
        attachVisualMesh(armIt->second, PrimitiveShape::Cube, glm::vec3(0.88f, 0.44f, 0.12f)); // Anodized Orange Arm Tube
    }

    auto wristIt = robot.linkEntities.find("arm_wrist_link");
    if (wristIt != robot.linkEntities.end())
    {
        attachVisualMesh(wristIt->second, PrimitiveShape::Cube, glm::vec3(0.18f, 0.68f, 0.88f)); // Cyan Wrist Bracket
    }

    // D. Intake Links
    auto icIt = robot.linkEntities.find("intake_coupler_link");
    if (icIt != robot.linkEntities.end())
    {
        attachVisualMesh(icIt->second, PrimitiveShape::Cube, glm::vec3(0.28f, 0.85f, 0.32f)); // Neon Green Intake Roller
    }

    auto fcrankIt = robot.linkEntities.find("intake_crank_link");
    if (fcrankIt != robot.linkEntities.end())
    {
        attachVisualMesh(fcrankIt->second, PrimitiveShape::Cube, glm::vec3(0.62f, 0.68f, 0.76f),
            glm::vec3(0.06f, 0.04f, 0.24f), glm::vec3(0.0f, 0.0f, 0.12f));
    }

    auto ffolIt = robot.linkEntities.find("intake_follower_link");
    if (ffolIt != robot.linkEntities.end())
    {
        attachVisualMesh(ffolIt->second, PrimitiveShape::Cube, glm::vec3(0.76f, 0.60f, 0.36f),
            glm::vec3(0.06f, 0.04f, 0.24f), glm::vec3(0.0f, 0.0f, 0.12f));
    }

    // E. Shooter Flywheel Wheel
    auto fwIt = robot.linkEntities.find("shooter_flywheel_visual");
    if (fwIt != robot.linkEntities.end())
    {
        attachVisualMesh(fwIt->second, PrimitiveShape::CylinderX, glm::vec3(0.95f, 0.88f, 0.18f),
            glm::vec3(0.10f, 0.12f, 0.12f)); // High-RPM Yellow Flywheel
    }

    // -------------------------------------------------------------------------
    // 9. Interactive State, Control Demands & Camera Configuration
    // -------------------------------------------------------------------------
    auto& camera = renderer.GetCamera();
    camera.position = glm::vec3(0.0f, 2.5f, -3.8f);
    camera.pitchDegrees = -16.0f;
    camera.yawDegrees = 90.0f;
    camera.moveSpeed = 6.0f;
    camera.SetMode(FRCBananaSim::Renderer::Camera::Mode::Orbit);
    camera.orbitTarget = robotSpawnPos + glm::vec3(0.0f, 0.35f, 0.0f);
    camera.orbitDistance = 3.6f;
    glm::vec3 lastChassisPos = robotSpawnPos;

    SimInputs robotInputs;
    SimOutputs robotOutputs;
    robot.electricalSystem->SetOutputs(&robotOutputs);
    robot.sensorSystem->SetOutputs(&robotOutputs);

    double simSpeed = 1.0;
    double simAccumulator = 0.0;
    bool doSingleStep = false;
    uint64_t totalSimSteps = 0;

    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;

    // Interactive Manual Control Overrides
    bool robotMasterEnabled = true;
    bool pinChassisStand = false;
    int cameraModeIndex = 1; // 0 = FreeFly, 1 = Orbit Robot

    // Drivetrain Manual Demands (UI Sliders + Keyboard Offsets)
    float manualDriveVolts = 0.0f;
    float manualStrafeVolts = 0.0f;
    float manualSpinVolts = 0.0f;
    float manualSteerAngleDeg = 0.0f;
    float keyDriveVolts = 0.0f;
    float keyStrafeVolts = 0.0f;
    float keySpinVolts = 0.0f;
    bool runSwerveStressTest = false;
    double swerveStressElapsed = 0.0;

    // Mechanism Manual Demands
    float manualElevatorVolts = 0.0f;
    float manualArmVolts = 0.0f;
    float manualWristVolts = 0.0f;
    float manualFlywheelVolts = 0.0f;
    bool intakeDeployed = false;

    // Visual Debug Options
    bool showColliders = true;
    bool showWheelContacts = true;
    bool showContactZones = true;
    bool debugDrawAlwaysOnTop = false;

    // Reposition Helper
    auto resetRobot = [&](const glm::vec3& worldPos, const glm::quat& worldRot)
        {
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized())
            {
                glm::vec3 joltTargetPos = worldPos;
                glm::quat joltTargetRot = worldRot;
                auto* chassisCol = scene.GetComponent<ColliderComponent>(robot.chassisEntity);
                if (chassisCol)
                {
                    joltTargetPos += worldRot * chassisCol->localOffset;
                    joltTargetRot = glm::normalize(worldRot * chassisCol->localRotation);
                }

                JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                bi.SetPositionAndRotation(chassisRb->bodyID,
                    JPH::RVec3(joltTargetPos.x, joltTargetPos.y, joltTargetPos.z),
                    JPH::Quat(joltTargetRot.x, joltTargetRot.y, joltTargetRot.z, joltTargetRot.w),
                    JPH::EActivation::Activate);
                bi.SetLinearAndAngularVelocity(chassisRb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            }

            auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
            if (chassisT)
            {
                chassisT->position = worldPos;
                chassisT->rotation = worldRot;
            }

            auto* chassisWt = scene.GetComponent<WorldTransformComponent>(robot.chassisEntity);
            if (chassisWt)
            {
                chassisWt->worldPosition = worldPos;
                chassisWt->worldRotation = worldRot;
            }

            if (robot.swerveSystem)
            {
                auto* model = robot.swerveSystem->GetModel();
                if (model)
                {
                    glm::vec3 posNWU = FRCBananaSim::Robot::Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(worldPos);
                    glm::quat rotNWU = glm::quat(FRCBananaSim::Robot::Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(glm::dquat(worldRot)));
                    model->ApplyExternalCorrections(posNWU, rotNWU, glm::vec3(0.0f), glm::vec3(0.0f));
                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        model->GetModule(i).GetState() = FRCBananaSim::Robot::Drivetrain::Model::SwerveModuleState{};
                    }
                }
            }

            manualDriveVolts = 0.0f;
            manualStrafeVolts = 0.0f;
            manualSpinVolts = 0.0f;
            manualSteerAngleDeg = 0.0f;
            lastChassisPos = worldPos;
            if (camera.GetMode() == FRCBananaSim::Renderer::Camera::Mode::Orbit)
            {
                camera.orbitTarget = worldPos + glm::vec3(0.0f, 0.35f, 0.0f);
            }
        };

    // -------------------------------------------------------------------------
    // 10. Single Fixed-Timestep Simulation Step (200 Hz = 5 ms)
    // -------------------------------------------------------------------------
    auto executeSimulationStep = [&](double dt)
        {
            SIM_PROFILE_SCOPE("SimulationStep");

            // 1. Chassis Pinning on Test Stand
            auto* chassisRb = scene.GetComponent<RigidBodyComponent>(robot.chassisEntity);
            if (chassisRb && !chassisRb->bodyID.IsInvalid() && joltContext.IsInitialized() && pinChassisStand)
            {
                JPH::BodyInterface& bi = joltContext.GetBodyInterface();
                bi.SetLinearAndAngularVelocity(chassisRb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            }

            // 2. Command Arbitration (WPILib Client vs Standalone Manual)
            if (wsServer.IsClientConnected())
            {
                wsServer.UpdateInputs(robotInputs);
                robotInputs.robotEnabled = wsServer.IsDriverStationEnabled();
            }
            else
            {
                robotInputs.robotEnabled = robotMasterEnabled;

                if (runSwerveStressTest)
                {
                    swerveStressElapsed += dt;
                    pinChassisStand = true;

                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        robotInputs.motorCommands[i].mode = MotorControlMode::Voltage;
                        robotInputs.motorCommands[i].demand = 12.0;
                        robotInputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                        robotInputs.motorCommands[4 + i].demand = 12.0;
                    }

                    if (swerveStressElapsed >= 60.0)
                    {
                        runSwerveStressTest = false;
                        SIM_LOG_INFO("60-Second Real-Time Swerve Stress Test Complete!");
                    }
                }
                else
                {
                    // Drivetrain Swerve Commands: [0..3] Steer, [4..7] Drive
                    // Combine UI slider demands with keyboard driving demands
                    double cmdVx = std::clamp(static_cast<double>(manualDriveVolts + keyDriveVolts), -12.0, 12.0);
                    double cmdVy = std::clamp(static_cast<double>(manualStrafeVolts + keyStrafeVolts), -12.0, 12.0);
                    double cmdOmega = std::clamp(static_cast<double>(manualSpinVolts + keySpinVolts), -12.0, 12.0);

                    constexpr double kP = 18.0;
                    constexpr double kD = 0.6;
                    constexpr double kInvSqrt2 = 0.7071067811865475;
                    // Module unit offset directions (rx, ry) in NWU (+X Forward, +Y Left): FL, FR, BL, BR
                    constexpr double modRx[4] = { +kInvSqrt2, +kInvSqrt2, -kInvSqrt2, -kInvSqrt2 };
                    constexpr double modRy[4] = { +kInvSqrt2, -kInvSqrt2, +kInvSqrt2, -kInvSqrt2 };

                    double modVx[4] = { 0.0, 0.0, 0.0, 0.0 };
                    double modVy[4] = { 0.0, 0.0, 0.0, 0.0 };
                    double modMag[4] = { 0.0, 0.0, 0.0, 0.0 };
                    double maxModMag = 0.0;

                    bool useDirectHeading = (std::abs(cmdVy) < 1e-3 && std::abs(cmdOmega) < 1e-3 && std::abs(manualSteerAngleDeg) > 1e-3f);
                    if (!useDirectHeading)
                    {
                        for (uint32_t i = 0; i < 4; ++i)
                        {
                            modVx[i] = cmdVx - (cmdOmega * modRy[i]);
                            modVy[i] = cmdVy + (cmdOmega * modRx[i]);
                            modMag[i] = std::hypot(modVx[i], modVy[i]);
                            maxModMag = std::max(maxModMag, modMag[i]);
                        }
                    }

                    double desaturateScale = (maxModMag > 12.0) ? (12.0 / maxModMag) : 1.0;

                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        double targetAngle = static_cast<double>(glm::radians(manualSteerAngleDeg));
                        double driveVolts = cmdVx;

                        if (!useDirectHeading)
                        {
                            if (modMag[i] > 1e-3)
                            {
                                targetAngle = std::atan2(modVy[i], modVx[i]);
                                driveVolts = modMag[i] * desaturateScale;
                            }
                            else
                            {
                                targetAngle = static_cast<double>(glm::radians(manualSteerAngleDeg));
                                driveVolts = 0.0;
                            }
                        }

                        double curAngle = robotOutputs.encoderPositions[i];
                        double curVel = robotOutputs.encoderVelocities[i];
                        double err = targetAngle - curAngle;

                        while (err > 3.141592653589793) err -= 6.283185307179586;
                        while (err < -3.141592653589793) err += 6.283185307179586;

                        // Swerve module 180-degree azimuth optimization + cosine velocity scaling
                        if (std::abs(driveVolts) > 1e-3)
                        {
                            if (err > 1.5707963267948966)
                            {
                                err -= 3.141592653589793;
                                driveVolts = -driveVolts;
                            }
                            else if (err < -1.5707963267948966)
                            {
                                err += 3.141592653589793;
                                driveVolts = -driveVolts;
                            }
                            driveVolts *= std::cos(err);
                        }

                        double steerV = std::clamp((err * kP) - (curVel * kD), -12.0, 12.0);

                        robotInputs.motorCommands[i].mode = MotorControlMode::Voltage;
                        robotInputs.motorCommands[i].demand = steerV;
                        robotInputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                        robotInputs.motorCommands[4 + i].demand = driveVolts;
                    }

                    // Mechanisms Motor Slots:
                    // [8] Elevator Drive
                    robotInputs.motorCommands[8].mode = MotorControlMode::Voltage;
                    robotInputs.motorCommands[8].demand = manualElevatorVolts;

                    // [9] Arm Shoulder
                    robotInputs.motorCommands[9].mode = MotorControlMode::Voltage;
                    robotInputs.motorCommands[9].demand = manualArmVolts;

                    // [10] Arm Wrist
                    robotInputs.motorCommands[10].mode = MotorControlMode::Voltage;
                    robotInputs.motorCommands[10].demand = manualWristVolts;

                    // [11] Intake 4-Bar Deploy
                    robotInputs.motorCommands[11].mode = MotorControlMode::Voltage;
                    double intakeAngle = 0.0;
                    double intakeVel = 0.0;
                    auto intakeCheckIt = robot.mechanismEntities.find("intake_fourbar_deploy");
                    if (intakeCheckIt != robot.mechanismEntities.end())
                    {
                        auto* intakeArm = scene.GetComponent<ArmComponent>(intakeCheckIt->second);
                        if (intakeArm)
                        {
                            intakeAngle = intakeArm->currentAngleRad;
                            intakeVel = intakeArm->currentVelocityRadPerSec;
                        }
                    }
                    double targetIntakeAngle = intakeDeployed ? 1.20 : 0.0;
                    double intakeErr = targetIntakeAngle - intakeAngle;
                    if (std::abs(intakeErr) > 0.008 || std::abs(intakeVel) > 0.05)
                    {
                        double pdVolts = (intakeErr * 18.0) - (intakeVel * 1.2);
                        robotInputs.motorCommands[11].demand = std::clamp(pdVolts, -6.0, 6.0);
                    }
                    else
                    {
                        robotInputs.motorCommands[11].demand = 0.0;
                    }

                    // [12] Shooter Flywheel
                    robotInputs.motorCommands[12].mode = MotorControlMode::Voltage;
                    robotInputs.motorCommands[12].demand = manualFlywheelVolts;
                }
            }

            // 3. Dispatch Demands to Subsystems
            if (robot.swerveSystem)
            {
                robot.swerveSystem->SetInputs(robotInputs);
            }

            auto elevIt = robot.mechanismEntities.find("elevator_drive");
            if (elevIt != robot.mechanismEntities.end())
            {
                auto* elev = scene.GetComponent<ElevatorComponent>(elevIt->second);
                if (elev) elev->commandedVoltage = robotInputs.motorCommands[8].demand;
            }

            auto armMIt = robot.mechanismEntities.find("arm_shoulder");
            if (armMIt != robot.mechanismEntities.end())
            {
                auto* arm = scene.GetComponent<ArmComponent>(armMIt->second);
                if (arm) arm->commandedVoltage = robotInputs.motorCommands[9].demand;
            }

            auto wristMIt = robot.mechanismEntities.find("arm_wrist");
            if (wristMIt != robot.mechanismEntities.end())
            {
                auto* wrist = scene.GetComponent<ArmComponent>(wristMIt->second);
                if (wrist) wrist->commandedVoltage = robotInputs.motorCommands[10].demand;
            }

            auto intakeMIt = robot.mechanismEntities.find("intake_fourbar_deploy");
            if (intakeMIt != robot.mechanismEntities.end())
            {
                auto* intakeArm = scene.GetComponent<ArmComponent>(intakeMIt->second);
                if (intakeArm) intakeArm->commandedVoltage = robotInputs.motorCommands[11].demand;
            }

            auto fwMIt = robot.mechanismEntities.find("shooter_flywheel");
            if (fwMIt != robot.mechanismEntities.end())
            {
                auto* fw = scene.GetComponent<FlywheelComponent>(fwMIt->second);
                if (fw) fw->commandedVoltage = robotInputs.motorCommands[12].demand;
            }

            // 4. Step Jolt Constraints & 8-Stage Pipeline
            jointSystem.FixedUpdate(dt);
            FRCBananaSim::Robot::RobotAssembler::StepPipeline(robot, dt);
            jointSystem.ReadbackTelemetry();

            if (totalSimSteps == 0)
            {
                std::cout << "[Simulator] Simulation Step 0 executed smoothly.\n";
            }

            // 5. Pull Telemetry & Broadcast Outgoing Frames
            robotOutputs = robot.sensorSystem->GetOutputs();
            if (robot.swerveSystem)
            {
                const auto& swerveOut = robot.swerveSystem->GetOutputs();
                for (uint32_t i = 0; i < 8; ++i)
                {
                    robotOutputs.encoderPositions[i] = swerveOut.encoderPositions[i];
                    robotOutputs.encoderVelocities[i] = swerveOut.encoderVelocities[i];
                    robotOutputs.rotorPositionsRot[i] = swerveOut.rotorPositionsRot[i];
                    robotOutputs.rotorVelocitiesRps[i] = swerveOut.rotorVelocitiesRps[i];
                    robotOutputs.motorCurrents[i] = swerveOut.motorCurrents[i];
                    robotOutputs.motorSupplyCurrents[i] = swerveOut.motorSupplyCurrents[i];
                }
                robotOutputs.gyroYaw = swerveOut.gyroYaw;
                robotOutputs.gyroPitch = swerveOut.gyroPitch;
                robotOutputs.gyroRoll = swerveOut.gyroRoll;
                robotOutputs.gyroYawRate = swerveOut.gyroYawRate;
            }
            robotOutputs.batteryTerminalVoltage = robot.electricalSystem->GetTerminalVoltage();
            robotOutputs.totalBusCurrent = robot.electricalSystem->GetTotalCurrent();
            robotOutputs.isBrownedOut = robot.electricalSystem->IsBrownedOut();

            if (wsServer.IsClientConnected())
            {
                wsServer.SendOutputs(robotOutputs);
            }

            // 6. Record Calibration Telemetry
            if (drivetrainLogger.IsRecording())
            {
                FRCBananaSim::Telemetry::DrivetrainLogSample sample;
                sample.timestamp = static_cast<double>(totalSimSteps) * dt;
                sample.commandedVoltage = manualDriveVolts;
                sample.batteryVoltage = robotOutputs.batteryTerminalVoltage;
                sample.totalBusCurrent = robotOutputs.totalBusCurrent;

                auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
                sample.chassisElevation = chassisT ? chassisT->position.y : 0.0;

                if (robot.swerveSystem && robot.swerveSystem->GetModel())
                {
                    const auto& motion = robot.swerveSystem->GetModel()->GetChassisState();
                    glm::vec3 linVel(motion.linearVelocity.y, motion.linearVelocity.z, motion.linearVelocity.x);
                    sample.chassisSpeed = glm::length(glm::vec2(linVel.x, linVel.z));

                    for (uint32_t i = 0; i < 4; ++i)
                    {
                        const auto& modState = robot.swerveSystem->GetModel()->GetModule(i).GetState();
                        sample.steerAnglesDeg[i] = glm::degrees(modState.steerAngle);
                        sample.wheelVelocitiesRadS[i] = modState.wheelVelocity;
                        sample.driveCurrentsA[i] = modState.driveStatorCurrent;
                        sample.normalForcesN[i] = modState.contactState.normalForce;
                        sample.slipRatios[i] = modState.contactState.saturationRatio;
                    }
                }

                sample.gyroYawDeg = glm::degrees(robotOutputs.gyroYaw);
                sample.gyroPitchDeg = glm::degrees(robotOutputs.gyroPitch);
                sample.gyroRollDeg = glm::degrees(robotOutputs.gyroRoll);
                sample.gyroYawRateDegS = glm::degrees(robotOutputs.gyroYawRate);

                drivetrainLogger.RecordSample(sample);
            }

            if (totalSimSteps <= 400 && (totalSimSteps % 50 == 0))
            {
                auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
                auto* couplerT = scene.GetComponent<TransformComponent>(robot.linkEntities["intake_coupler_link"]);
                auto* armT = scene.GetComponent<TransformComponent>(robot.linkEntities["arm_shoulder_link"]);
                auto* wristT = scene.GetComponent<TransformComponent>(robot.linkEntities["arm_wrist_link"]);
                auto* fwT = scene.GetComponent<TransformComponent>(robot.linkEntities["shooter_flywheel_visual"]);
                auto* elevT = scene.GetComponent<TransformComponent>(robot.linkEntities["elevator_stage_1"]);

                double wVel0 = 0.0, wVel1 = 0.0, wVel2 = 0.0, wVel3 = 0.0;
                double wN0 = 0.0, wN1 = 0.0, wN2 = 0.0, wN3 = 0.0;
                if (robot.swerveSystem && robot.swerveSystem->GetModel())
                {
                    auto* m = robot.swerveSystem->GetModel();
                    wVel0 = m->GetModule(0).GetState().wheelVelocity;
                    wVel1 = m->GetModule(1).GetState().wheelVelocity;
                    wVel2 = m->GetModule(2).GetState().wheelVelocity;
                    wVel3 = m->GetModule(3).GetState().wheelVelocity;
                    wN0 = m->GetModule(0).GetState().contactState.normalForce;
                    wN1 = m->GetModule(1).GetState().contactState.normalForce;
                    wN2 = m->GetModule(2).GetState().contactState.normalForce;
                    wN3 = m->GetModule(3).GetState().contactState.normalForce;
                }

                double crankAngleDeg = 0.0;
                double folAngleDeg = 0.0;
                auto cjIt = robot.jointEntities.find("fourbar_crank_hinge");
                auto fjIt = robot.jointEntities.find("fourbar_follower_hinge");
                if (cjIt != robot.jointEntities.end())
                {
                    if (auto* cjc = scene.GetComponent<JointComponent>(cjIt->second))
                    {
                        crankAngleDeg = glm::degrees(cjc->currentAngle);
                    }
                }
                if (fjIt != robot.jointEntities.end())
                {
                    if (auto* fjc = scene.GetComponent<JointComponent>(fjIt->second))
                    {
                        folAngleDeg = glm::degrees(fjc->currentAngle);
                    }
                }

                double lastTotalMs = 0.0, lastPhysMs = 0.0, lastUpdMs = 0.0, lastRenMs = 0.0;
                const auto& hist = FRCBananaSim::Telemetry::TelemetryManager::Get().GetFrameHistory();
                if (!hist.empty())
                {
                    lastTotalMs = hist.back().totalFrameMs;
                    lastPhysMs = hist.back().physicsMs;
                    lastUpdMs = hist.back().updateMs;
                    lastRenMs = hist.back().renderMs;
                }

                std::cout << "[Step " << totalSimSteps << "] "
                    << "ChassisPos=(" << (chassisT ? chassisT->position.x : 0.0f) << ", "
                    << (chassisT ? chassisT->position.y : 0.0f) << ", "
                    << (chassisT ? chassisT->position.z : 0.0f) << ") "
                    << "PitchDeg=" << glm::degrees(robotOutputs.gyroPitch) << " "
                    << "RollDeg=" << glm::degrees(robotOutputs.gyroRoll) << " "
                    << "WheelVel=[" << wVel0 << ", " << wVel1 << ", " << wVel2 << ", " << wVel3 << "] "
                    << "4BarDeg=[Crank=" << crankAngleDeg << ", Fol=" << folAngleDeg << "] "
                    << "CouplerPos=(" << (couplerT ? couplerT->position.x : 0.0f) << ", "
                    << (couplerT ? couplerT->position.y : 0.0f) << ", "
                    << (couplerT ? couplerT->position.z : 0.0f) << ") "
                    << "FrameMs=" << lastTotalMs << " (Phys=" << lastPhysMs
                    << ", Upd=" << lastUpdMs << ", Ren=" << lastRenMs << ") "
                    << "Vbat=" << robotOutputs.batteryTerminalVoltage << std::endl;
            }

            ++totalSimSteps;
        };

    // -------------------------------------------------------------------------
    // 11. Engine Loop Configuration
    // -------------------------------------------------------------------------
    FRCBananaSim::Core::Engine engine(200.0, 0.0);

    engine.SetFixedUpdateCallback([&](double dt)
        {
            if (simSpeed == 0.0)
            {
                if (doSingleStep)
                {
                    doSingleStep = false;
                    executeSimulationStep(dt);
                }
            }
            else
            {
                simAccumulator += dt * simSpeed;
                int steps = 0;
                while (simAccumulator >= dt && steps < 4)
                {
                    simAccumulator -= dt;
                    executeSimulationStep(dt);
                    ++steps;
                }
            }
        });

    // -------------------------------------------------------------------------
    // 12. Frame Presentation & Interactive ImGui HUD
    // -------------------------------------------------------------------------
    engine.SetUpdateCallback([&](double dt)
        {
            if (renderer.ShouldClose())
            {
                engine.RequestExit();
                return;
            }

            renderer.BeginFrame();
            cameraModeIndex = (camera.GetMode() == FRCBananaSim::Renderer::Camera::Mode::FreeFly) ? 0 : 1;

            auto& input = renderer.GetInput();

            // 1. Keyboard Driving Controls (Arrow Keys or WASD + Q/E when RMB is not held)
            keyDriveVolts = 0.0f;
            keyStrafeVolts = 0.0f;
            keySpinVolts = 0.0f;
            bool rmbHeld = input.IsMouseButtonDown(GLFW_MOUSE_BUTTON_RIGHT);

            if (!runSwerveStressTest && robotMasterEnabled && !ImGui::GetIO().WantTextInput && !rmbHeld)
            {
                if (input.IsKeyDown(GLFW_KEY_UP) || input.IsKeyDown(GLFW_KEY_W))
                {
                    keyDriveVolts = 10.0f;
                }
                else if (input.IsKeyDown(GLFW_KEY_DOWN) || input.IsKeyDown(GLFW_KEY_S))
                {
                    keyDriveVolts = -10.0f;
                }

                if (input.IsKeyDown(GLFW_KEY_LEFT) || input.IsKeyDown(GLFW_KEY_A))
                {
                    keyStrafeVolts = 10.0f;
                }
                else if (input.IsKeyDown(GLFW_KEY_RIGHT) || input.IsKeyDown(GLFW_KEY_D))
                {
                    keyStrafeVolts = -10.0f;
                }

                if (input.IsKeyDown(GLFW_KEY_Q))
                {
                    keySpinVolts = 10.0f;
                }
                else if (input.IsKeyDown(GLFW_KEY_E))
                {
                    keySpinVolts = -10.0f;
                }
            }

            // 2. Hotkeys: Speed, Step, Reset & Camera
            if (input.WasKeyPressed(GLFW_KEY_0)) simSpeed = 0.0;
            else if (input.WasKeyPressed(GLFW_KEY_1)) simSpeed = 0.25;
            else if (input.WasKeyPressed(GLFW_KEY_7)) simSpeed = 1.0;
            else if (input.WasKeyPressed(GLFW_KEY_8)) simSpeed = 2.0;

            if (input.WasKeyPressed(GLFW_KEY_PERIOD))
            {
                if (simSpeed == 0.0) doSingleStep = true;
            }

            if (input.WasKeyPressed(GLFW_KEY_R))
            {
                resetRobot(robotSpawnPos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
            }

            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                auto mode = FRCBananaSim::Telemetry::TelemetryManager::Get().GetMode();
                auto next = (mode == FRCBananaSim::Telemetry::TelemetryMode::Off)
                    ? FRCBananaSim::Telemetry::TelemetryMode::Minimal
                    : ((mode == FRCBananaSim::Telemetry::TelemetryMode::Minimal)
                        ? FRCBananaSim::Telemetry::TelemetryMode::Full
                        : FRCBananaSim::Telemetry::TelemetryMode::Off);

                if (next == FRCBananaSim::Telemetry::TelemetryMode::Full) profilerWindow.Open(renderer.GetWindow());
                else if (profilerWindow.IsOpen()) profilerWindow.Close(renderer.GetWindow());

                FRCBananaSim::Telemetry::TelemetryManager::Get().SetMode(next);
            }

            // Update Camera Orbit Target incrementally with chassis displacement so user panning is preserved
            auto* chassisT = scene.GetComponent<TransformComponent>(robot.chassisEntity);
            if (chassisT)
            {
                glm::vec3 chassisDelta = chassisT->position - lastChassisPos;
                if (camera.GetMode() == FRCBananaSim::Renderer::Camera::Mode::Orbit)
                {
                    camera.orbitTarget += chassisDelta;
                }
                lastChassisPos = chassisT->position;
            }
        });

    engine.SetRenderCallback([&](double alpha)
        {
            if (renderer.ShouldClose())
            {
                return;
            }

            // A. Immediate-mode Debug Wireframes
            RenderSceneDebugDraw(scene, showColliders, showWheelContacts, showContactZones,
                debugDrawAlwaysOnTop, robot);

            // B. Render Solid 3D Meshes
            scene.RenderUpdateSystems();
            if (robot.flywheelSystem) robot.flywheelSystem->RenderUpdate();
            if (robot.swerveSystem) robot.swerveSystem->RenderUpdate();

            // C. Flush Line Buffer to GPU
            renderer.FlushDebugDraw();

            // D. Telemetry Overlay
            telemetryOverlay.Render();

            // -------------------------------------------------------------
            // 4. ImGui Master Control Center
            // -------------------------------------------------------------
            ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(560.0f, 820.0f), ImGuiCond_FirstUseEver);

            if (ImGui::Begin("FRCBananaSim — Block 3 Reference 2026 Robot Test Station"))
            {
                // Time & Time-Dilation Bar
                ImGui::Text("Sim Time: %.2f s | Timestep: 200 Hz (5 ms) | Speed: %.2fx",
                    static_cast<double>(totalSimSteps) * 0.005, simSpeed);

                if (ImGui::Button("Pause (0x)")) simSpeed = 0.0; ImGui::SameLine();
                if (ImGui::Button("0.25x")) simSpeed = 0.25; ImGui::SameLine();
                if (ImGui::Button("1.0x (Realtime)")) simSpeed = 1.0; ImGui::SameLine();
                if (ImGui::Button("2.0x (Fast)")) simSpeed = 2.0; ImGui::SameLine();
                if (ImGui::Button("Step Single Tick (.)")) { if (simSpeed == 0.0) doSingleStep = true; }

                ImGui::Separator();

                // Master Enable & Brownout Alarm Bar
                ImGui::Checkbox("Master Enable Robot", &robotMasterEnabled);
                ImGui::SameLine();
                ImGui::Checkbox("Pin Chassis to Stand", &pinChassisStand);

                // Battery Bus Status Bar
                double vBat = robotOutputs.batteryTerminalVoltage;
                double iBus = robotOutputs.totalBusCurrent;
                float vFraction = static_cast<float>(vBat / 13.0);
                ImVec4 vColor = (vBat > 11.0) ? ImVec4(0.2f, 0.85f, 0.25f, 1.0f) :
                    ((vBat > 6.8) ? ImVec4(0.95f, 0.80f, 0.15f, 1.0f) : ImVec4(0.95f, 0.20f, 0.20f, 1.0f));

                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, vColor);
                char vBuf[64];
                std::snprintf(vBuf, sizeof(vBuf), "Battery: %.2f V | Bus: %.1f A %s",
                    vBat, iBus, robotOutputs.isBrownedOut ? "[CRITICAL BROWNOUT]" : "");
                ImGui::ProgressBar(vFraction, ImVec2(-1.0f, 22.0f), vBuf);
                ImGui::PopStyleColor();

                ImGui::Separator();

                // Main Subsystem Tabs
                if (ImGui::BeginTabBar("RobotTabs"))
                {
                    // =============================================================
                    // TAB 1: 6-DOF SWERVE DRIVETRAIN
                    // =============================================================
                    if (ImGui::BeginTabItem("Swerve Drivetrain"))
                    {
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "SPAWN & COURSE PRESETS");
                        if (ImGui::Button("Start Line (Carpet)", ImVec2(165.0f, 24.0f)))
                        {
                            resetRobot(robotSpawnPos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("15 deg Ramp", ImVec2(165.0f, 24.0f)))
                        {
                            resetRobot(glm::vec3(3.2f, 0.52f, 4.0f),
                                glm::angleAxis(glm::radians(-15.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cable Bump", ImVec2(165.0f, 24.0f)))
                        {
                            resetRobot(glm::vec3(3.2f, 0.20f, 7.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // 60-Second Real-Time Stall Stress Test
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "GATE 1: 60-SECOND MASS-RATIO STRESS TEST");
                        if (!runSwerveStressTest)
                        {
                            if (ImGui::Button("Run 60s Max Stall Stress Test Live", ImVec2(-1.0f, 26.0f)))
                            {
                                runSwerveStressTest = true;
                                swerveStressElapsed = 0.0;
                                pinChassisStand = true;
                                SIM_LOG_INFO("Started 60-Second Real-Time Swerve Stress Test");
                            }
                        }
                        else
                        {
                            float prog = static_cast<float>(swerveStressElapsed / 60.0);
                            char progBuf[64];
                            std::snprintf(progBuf, sizeof(progBuf), "Running: %.1f / 60.0 s (%.0f%%)", swerveStressElapsed, prog * 100.0f);
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
                            ImGui::ProgressBar(prog, ImVec2(-1.0f, 22.0f), progBuf);
                            ImGui::PopStyleColor();

                            if (ImGui::Button("Abort Stress Test")) runSwerveStressTest = false;
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Manual Driving Controls
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "MANUAL SWERVE OVERRIDES (WASD / ARROWS = DRIVE & STRAFE, Q/E = SPIN)");
                        ImGui::SliderFloat("Forward/Back (Vx)", &manualDriveVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Forward (+10V)")) manualDriveVolts = 10.0f; ImGui::SameLine();
                        if (ImGui::Button("Reverse (-10V)")) manualDriveVolts = -10.0f; ImGui::SameLine();
                        if (ImGui::Button("Stop All (0V)"))
                        {
                            manualDriveVolts = 0.0f;
                            manualStrafeVolts = 0.0f;
                            manualSpinVolts = 0.0f;
                            manualSteerAngleDeg = 0.0f;
                        }

                        ImGui::SliderFloat("Strafe Left/Right (Vy)", &manualStrafeVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Strafe Left (+10V)")) manualStrafeVolts = 10.0f; ImGui::SameLine();
                        if (ImGui::Button("Center Strafe (0V)")) manualStrafeVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Strafe Right (-10V)")) manualStrafeVolts = -10.0f;

                        ImGui::SliderFloat("Spin / Yaw (Omega)", &manualSpinVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Spin CCW (+10V)")) manualSpinVolts = 10.0f; ImGui::SameLine();
                        if (ImGui::Button("Stop Spin (0V)")) manualSpinVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Spin CW (-10V)")) manualSpinVolts = -10.0f;

                        ImGui::SliderFloat("Steer Heading (deg)", &manualSteerAngleDeg, -90.0f, 90.0f, "%.1f deg");
                        if (ImGui::Button("Right (-45 deg)")) manualSteerAngleDeg = -45.0f; ImGui::SameLine();
                        if (ImGui::Button("Center (0 deg)")) manualSteerAngleDeg = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Left (+45 deg)")) manualSteerAngleDeg = 45.0f;

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Telemetry & 4-Module Status Table
                        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "LIVE SWERVE MODULE TELEMETRY");
                        auto* model = robot.swerveSystem ? robot.swerveSystem->GetModel() : nullptr;

                        if (ImGui::BeginTable("SwerveTable", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                        {
                            ImGui::TableSetupColumn("Module");
                            ImGui::TableSetupColumn("Steer");
                            ImGui::TableSetupColumn("m/s");
                            ImGui::TableSetupColumn("RPM");
                            ImGui::TableSetupColumn("Current");
                            ImGui::TableSetupColumn("Normal (N)");
                            ImGui::TableSetupColumn("Traction");
                            ImGui::TableHeadersRow();

                            const char* modLabels[] = { "FL [0]", "FR [1]", "BL [2]", "BR [3]" };
                            for (int i = 0; i < 4; ++i)
                            {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::Text("%s", modLabels[i]);

                                ImGui::TableSetColumnIndex(1);
                                ImGui::Text("%.1f deg", glm::degrees(robotOutputs.encoderPositions[i]));

                                double vLinear = 0.0;
                                double driveRpm = (robotOutputs.encoderVelocities[4 + i] * 60.0) / (2.0 * 3.141592653589793);
                                double normalF = 0.0;
                                double slipRatio = 0.0;

                                if (model)
                                {
                                    const auto& st = model->GetModule(i).GetState();
                                    vLinear = st.linearVelocity;
                                    normalF = st.contactState.normalForce;
                                    slipRatio = st.contactState.saturationRatio;
                                }

                                ImGui::TableSetColumnIndex(2);
                                ImGui::Text("%.2f", vLinear);

                                ImGui::TableSetColumnIndex(3);
                                ImGui::Text("%.0f", driveRpm);

                                ImGui::TableSetColumnIndex(4);
                                ImGui::Text("%.1f A", robotOutputs.motorCurrents[4 + i]);

                                ImGui::TableSetColumnIndex(5);
                                ImGui::Text("%.1f N", normalF);

                                ImGui::TableSetColumnIndex(6);
                                if (slipRatio > 1.0)
                                {
                                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%.2f (SLIP)", slipRatio);
                                }
                                else
                                {
                                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "%.2f (OK)", slipRatio);
                                }
                            }
                            ImGui::EndTable();
                        }

                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 2: MECHANISMS (ELEVATOR, ARM, WRIST, 4-BAR, FLYWHEEL)
                    // =============================================================
                    if (ImGui::BeginTabItem("Mechanisms"))
                    {
                        // A. Dual-Stage Cascade Elevator
                        ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "1. CASCADE ELEVATOR (Slot 8, Dual Kraken X60, G = 9.0)");
                        auto elevIt2 = robot.mechanismEntities.find("elevator_drive");
                        ElevatorComponent* elevComp = (elevIt2 != robot.mechanismEntities.end())
                            ? scene.GetComponent<ElevatorComponent>(elevIt2->second) : nullptr;

                        ImGui::SliderFloat("Elevator Voltage (V)", &manualElevatorVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Up (+8V)")) manualElevatorVolts = 8.0f; ImGui::SameLine();
                        if (ImGui::Button("Hold (0V)")) manualElevatorVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Down (-8V)")) manualElevatorVolts = -8.0f;

                        if (elevComp)
                        {
                            double s1Pos = elevComp->stages.empty() ? 0.0 : elevComp->stages[0].currentPositionM;
                            double s1Vel = elevComp->stages.empty() ? 0.0 : elevComp->stages[0].currentVelocityMps;
                            ImGui::Text("Stage 1 Height: %.3f m | Carriage Height: %.3f m (2x)", s1Pos, s1Pos * 2.0);
                            ImGui::Text("Stage 1 Speed:  %.3f m/s | Applied Force: %.1f N", s1Vel, elevComp->appliedLinearForceN);
                            ImGui::Text("Coulomb Stick:  %s | Spring Assist: %.1f N",
                                elevComp->isStickLocked ? "LATCHED (Zero Creep)" : "SLIDING", elevComp->springPreloadForceN);
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // B. Articulated Shoulder Arm & Wrist
                        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), "2. ARTICULATED ARM & WRIST (Slots 9 & 10)");
                        auto armIt2 = robot.mechanismEntities.find("arm_shoulder");
                        ArmComponent* armComp = (armIt2 != robot.mechanismEntities.end())
                            ? scene.GetComponent<ArmComponent>(armIt2->second) : nullptr;

                        auto wristIt2 = robot.mechanismEntities.find("arm_wrist");
                        ArmComponent* wristComp = (wristIt2 != robot.mechanismEntities.end())
                            ? scene.GetComponent<ArmComponent>(wristIt2->second) : nullptr;

                        ImGui::SliderFloat("Shoulder Volts (V)", &manualArmVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Raise Shoulder (+6V)")) manualArmVolts = 6.0f; ImGui::SameLine();
                        if (ImGui::Button("Hold Shoulder (0V)")) manualArmVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Lower Shoulder (-6V)")) manualArmVolts = -6.0f;

                        ImGui::SliderFloat("Wrist Volts (V)", &manualWristVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Pitch Wrist Up (+6V)")) manualWristVolts = 6.0f; ImGui::SameLine();
                        if (ImGui::Button("Hold Wrist (0V)")) manualWristVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Pitch Wrist Down (-6V)")) manualWristVolts = -6.0f;

                        if (armComp)
                        {
                            ImGui::Text("Shoulder Angle: %.1f deg (%.2f rad) | Rate: %.1f deg/s",
                                glm::degrees(armComp->currentAngleRad), armComp->currentAngleRad, glm::degrees(armComp->currentVelocityRadPerSec));
                            ImGui::Text("Shoulder Torque: %.1f Nm | Stick: %s",
                                armComp->appliedTorqueNm, armComp->isStickLocked ? "LATCHED" : "FREE");
                        }
                        if (wristComp)
                        {
                            ImGui::Text("Wrist Angle:    %.1f deg (%.2f rad) | Stick: %s",
                                glm::degrees(wristComp->currentAngleRad), wristComp->currentAngleRad,
                                wristComp->isStickLocked ? "LATCHED" : "FREE");
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // C. Closed-Loop Intake Four-Bar
                        ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.4f, 1.0f), "3. INTAKE FOUR-BAR LINKAGE (Slot 11, Kraken X44)");
                        if (ImGui::Button(intakeDeployed ? "Stow Intake (-6V)" : "Deploy Intake (+6V)", ImVec2(-1.0f, 26.0f)))
                        {
                            intakeDeployed = !intakeDeployed;
                        }

                        auto fbIt = scene.Query<FourBarComponent>();
                        if (!fbIt.empty())
                        {
                            auto* fb = scene.GetComponent<FourBarComponent>(fbIt[0]);
                            if (fb)
                            {
                                auto* cjc = (fb->crankJoint != invalidEntity)
                                    ? scene.GetComponent<JointComponent>(fb->crankJoint) : nullptr;
                                auto* fjc = (fb->followerJoint != invalidEntity)
                                    ? scene.GetComponent<JointComponent>(fb->followerJoint) : nullptr;
                                if (cjc && fjc)
                                {
                                    ImGui::Text("Crank Angle: %.1f deg | Follower Angle: %.1f deg",
                                        glm::degrees(cjc->currentAngle), glm::degrees(fjc->currentAngle));
                                }
                                ImGui::Text("Loop Closure Drift: %.3f mm %s",
                                    fb->loopClosureErrorM * 1000.0,
                                    (fb->loopClosureErrorM < 0.0015) ? "(OK < 1.5mm)" : "(DRIFT WARNING)");
                                ImGui::Text("Kinematic Loop:     %s", fb->isClosedLoop ? "ENFORCED" : "OPEN");
                            }
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // D. High-RPM Shooter Flywheel
                        ImGui::TextColored(ImVec4(0.95f, 0.88f, 0.2f, 1.0f), "4. SHOOTER FLYWHEEL (Slot 12, Dual Kraken X60, J = 0.0032)");
                        ImGui::SliderFloat("Flywheel Volts (V)", &manualFlywheelVolts, 0.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Spin 6000 RPM (12V)")) manualFlywheelVolts = 12.0f; ImGui::SameLine();
                        if (ImGui::Button("Coast (0V)")) manualFlywheelVolts = 0.0f;

                        auto fwIt2 = robot.mechanismEntities.find("shooter_flywheel");
                        FlywheelComponent* fwComp = (fwIt2 != robot.mechanismEntities.end())
                            ? scene.GetComponent<FlywheelComponent>(fwIt2->second) : nullptr;

                        if (fwComp)
                        {
                            ImGui::Text("Flywheel Velocity: %.0f RPM (%.1f rad/s)", fwComp->angularVelocityRpm, fwComp->angularVelocityRadPerSec);
                            ImGui::Text("Motor Current:     %.1f A | Stator Clamp: 80.0 A", fwComp->statorCurrentA);
                            ImGui::Text("Chassis Recoil:    %.2f Nm (Newton's 3rd Law)", fwComp->recoilTorqueNm);
                        }

                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 3: ELECTRICAL & SENSORS
                    // =============================================================
                    if (ImGui::BeginTabItem("Sensors & Electrical"))
                    {
                        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "6-AXIS PIGEON 2 IMU (WPILib NWU Frame)");
                        ImGui::Text("Continuous Yaw:   %.2f deg", glm::degrees(robotOutputs.gyroYaw));
                        ImGui::Text("Pitch / Roll:     %.2f deg / %.2f deg", glm::degrees(robotOutputs.gyroPitch), glm::degrees(robotOutputs.gyroRoll));
                        ImGui::Text("Angular Velocity: (%.2f, %.2f, %.2f) rad/s",
                            robotOutputs.gyroAngularVelocityRadPerSec.x, robotOutputs.gyroAngularVelocityRadPerSec.y, robotOutputs.gyroAngularVelocityRadPerSec.z);
                        ImGui::Text("Specific Force:   (%.2f, %.2f, %.2f) m/s^2",
                            robotOutputs.accelSpecificForceMetersPerSec2.x, robotOutputs.accelSpecificForceMetersPerSec2.y, robotOutputs.accelSpecificForceMetersPerSec2.z);

                        ImGui::Spacing();
                        ImGui::Separator();

                        ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "DIGITAL I/O & SENSORS (SimOutputs::digitalInputs)");
                        bool dio0 = (robotOutputs.digitalInputs & (1 << 0)) != 0; // Elevator Lower Limit
                        bool dio1 = (robotOutputs.digitalInputs & (1 << 1)) != 0; // Elevator Upper Limit
                        bool dio2 = (robotOutputs.digitalInputs & (1 << 2)) != 0; // Optical Beam Break

                        ImGui::Text("DIO 0 [Elevator Lower Limit]: %s", dio0 ? "TRIPPED" : "CLEAR");
                        ImGui::Text("DIO 1 [Elevator Upper Limit]: %s", dio1 ? "TRIPPED" : "CLEAR");
                        ImGui::Text("DIO 2 [Intake Beam Break]:   %s (10 ms Debounce)", dio2 ? "OCCLUDED" : "CLEAR");

                        ImGui::Spacing();
                        ImGui::Separator();

                        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "ANALOG INPUTS & CONTACT ZONES");
                        ImGui::Text("Analog 0 [Shoulder Pot]: %.3f V", robotOutputs.batteryTerminalVoltage > 0.0 ? 2.500 : 0.0);
                        ImGui::Text("Intake Capture Zone:     180 x 300 x 100 mm (Role: Intake)");
                        ImGui::Text("Shooter Compression:     220 x 450 x 140 mm (Gap: 90 mm)");

                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 4: WPILIB BRIDGE & LOGGING
                    // =============================================================
                    if (ImGui::BeginTabItem("Bridge & Logs"))
                    {
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "WPILIB HALSIM WEBSOCKET SERVER (:3300/wpilibws)");
                        auto stats = wsServer.GetStatistics();
                        if (wsServer.IsClientConnected())
                        {
                            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.3f, 1.0f),
                                "● CONNECTED (Packets In: %llu | Out: %llu)", stats.packetsReceived, stats.packetsSent);
                        }
                        else
                        {
                            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
                                "○ Waiting for ./gradlew simulateNative... (Packets: %llu)", stats.packetsReceived);
                        }

                        bool dsEn = wsServer.IsDriverStationEnabled();
                        if (ImGui::Checkbox("DS Enabled", &dsEn)) wsServer.SetDriverStation(dsEn, wsServer.IsAutonomous());
                        ImGui::SameLine();
                        bool dsAuto = wsServer.IsAutonomous();
                        if (ImGui::Checkbox("Auto Mode", &dsAuto)) wsServer.SetDriverStation(wsServer.IsDriverStationEnabled(), dsAuto);

                        if (ImGui::Button("Dump Discovered SimDevices to JSON"))
                        {
                            wsServer.DumpSimDevices("sim_device_dump.json");
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), "HIGH-FREQUENCY TELEMETRY LOGGER (200 Hz)");
                        if (drivetrainLogger.IsRecording())
                        {
                            if (ImGui::Button("Stop Recording", ImVec2(140.0f, 24.0f))) drivetrainLogger.StopRecording();
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.25f, 1.0f), "● REC (%zu samples)", drivetrainLogger.GetSampleCount());
                        }
                        else
                        {
                            if (ImGui::Button("Start Recording", ImVec2(140.0f, 24.0f))) drivetrainLogger.StartRecording();
                            ImGui::SameLine();
                            ImGui::Text("Stored: %zu / %zu", drivetrainLogger.GetSampleCount(), drivetrainLogger.GetCapacity());
                        }

                        if (ImGui::Button("Export to CSV")) drivetrainLogger.ExportToCsv("drivetrain_telemetry.csv");
                        ImGui::SameLine();
                        if (ImGui::Button("Export FTC Schema CSV")) drivetrainLogger.ExportToFtcSchemaCsv("drivetrain_ftc_schema.csv");
                        ImGui::SameLine();
                        if (ImGui::Button("Clear")) drivetrainLogger.Clear();

                        ImGui::EndTabItem();
                    }

                    // =============================================================
                    // TAB 5: VISUALS & CAMERA
                    // =============================================================
                    if (ImGui::BeginTabItem("Visuals & Camera"))
                    {
                        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.85f, 1.0f), "COLLIDER & CONTACT DEBUG DRAW");
                        ImGui::Checkbox("Draw All Colliders (Wireframe OBBs)", &showColliders);
                        ImGui::Checkbox("Draw Wheel Contact Patches & Load Spikes", &showWheelContacts);
                        ImGui::Checkbox("Draw Mechanism Contact Zones (Intake/Shooter)", &showContactZones);
                        ImGui::Checkbox("Always On Top (No Depth Occlusion)", &debugDrawAlwaysOnTop);

                        ImGui::Spacing();
                        ImGui::Separator();

                        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.2f, 1.0f), "CAMERA CONTROLS");
                        const char* camModes[] = { "Free-Fly (WASD + Mouse)", "Orbit Robot (RMB Drag + Zoom)" };
                        if (ImGui::Combo("Camera Mode (Tab)", &cameraModeIndex, camModes, IM_ARRAYSIZE(camModes)))
                        {
                            camera.SetMode((cameraModeIndex == 0)
                                ? FRCBananaSim::Renderer::Camera::Mode::FreeFly
                                : FRCBananaSim::Renderer::Camera::Mode::Orbit);
                        }

                        ImGui::BulletText("Orbit Mode: Hold RMB and drag mouse to swing camera around robot.");
                        ImGui::BulletText("Scroll Wheel: Zoom in and out.");
                        ImGui::BulletText("Free-Fly: Hold RMB and use WASD to fly camera freely.");
                        ImGui::BulletText("F3: Toggle Flame Graph Profiler.");
                        ImGui::BulletText("0..9: Change simulation speed multiplier.");

                        ImGui::EndTabItem();
                    }

                    ImGui::EndTabBar();
                }
            }
            ImGui::End();

            // 5. Presentation
            renderer.EndFrame();
        });

    engine.SetPostRenderCallback([&]()
        {
            if (FRCBananaSim::Telemetry::TelemetryManager::Get().GetMode() == FRCBananaSim::Telemetry::TelemetryMode::Full
                && profilerWindow.IsOpen())
            {
                profilerWindow.Render(renderer.GetWindow());
            }
        });

    // 13. Run Main Engine Loop
    engine.Run();

    // 14. Clean Teardown
    wsServer.Stop();
    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    joltContext.Shutdown();
    FRCBananaSim::Telemetry::TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator 2026 Reference Robot Station shut down cleanly.");
    return 0;
}