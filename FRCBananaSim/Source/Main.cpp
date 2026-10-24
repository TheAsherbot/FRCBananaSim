/**
 * @file Main.cpp
 * @brief Dynamic actuator, battery, and joint test showcase with uncapped motor speeds, unified presets, and real-time telemetry.
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
    default: return DCMotorModel::KrakenX60(gearRatio, efficiency);
    }
}

/**
 * @brief Returns retailer rated seconds-per-60-degree specification.
 * @param presetIndex Index of selected servo preset.
 * @return Seconds per 60 degrees.
 */
static double GetServoRatedSecPer60Deg(int presetIndex)
{
    switch (presetIndex)
    {
    case 0: return 0.200;
    case 1: return 0.090;
    case 2: return 0.080;
    case 3: return 0.112;
    case 4: return 0.062;
    case 5: return 0.092;
    case 6: return 0.110;
    default: return 0.200;
    }
}

int main()
{
    SIM_LOG_INFO("Starting FRC Banana Simulator - Actuator & Joint Demonstration...");

    // 1. Initialize Telemetry Logging
    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal);

    // 2. Initialize Renderer
    Renderer renderer(1280, 720, "FRC Banana Simulator - Actuator & Battery Showcase");
    if (!renderer.IsValid())
    {
        SIM_LOG_FATAL("Failed to initialize Renderer subsystem");
        TelemetryManager::Get().Shutdown();
        return -1;
    }

    // Configure Camera for close-up, clear framing of all 4 test benches
    renderer.GetCamera().position = glm::vec3(-0.6f, 3.2f, 8.8f);
    renderer.GetCamera().pitchDegrees = -14.0f;
    renderer.GetCamera().yawDegrees = -90.0f;
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
    scene.AddSystem(std::make_unique<JointSystem>(scene, joltContext));
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
        mesh.color = glm::vec3(0.98f, 0.80f, 0.15f); // Vivid Golden Yellow
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
        // Pivot is at Y = 2.30m, Z = 0.45m. Center of arm hangs at Y = 1.75m.
        assignTransforms(st2Arm, st2Pos + glm::vec3(0.0f, 1.75f, 0.45f), glm::vec3(0.18f, 1.2f, 0.18f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(1.0f, 0.50f, 0.08f); // High-Vis Orange
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
    // E. Station 3: Intake Roller Stand (NEO 550 vs goBILDA 5202 FTC Motor)
    // -------------------------------------------------------------------------
    const glm::vec3 st3Pos = glm::vec3(1.6f, 0.0f, 0.0f);
    Entity st3Pedestal = scene.CreateEntity();
    {
        assignTransforms(st3Pedestal, st3Pos + glm::vec3(0.0f, 0.50f, 0.0f), glm::vec3(0.55f, 1.0f, 0.55f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.42f, 0.45f, 0.50f);
        scene.AddComponent(st3Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st3Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.275f, 0.50f, 0.275f);
        scene.AddComponent(st3Pedestal, collider);
    }

    Entity st3Roller = scene.CreateEntity();
    {
        assignTransforms(st3Roller, st3Pos + glm::vec3(0.0f, 1.40f, 0.0f), glm::vec3(0.45f, 0.45f, 0.85f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.10f, 0.90f, 0.95f); // Vibrant Cyan
        scene.AddComponent(st3Roller, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 0.5f;
        rigidBody.friction = 0.5f;
        scene.AddComponent(st3Roller, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.225f, 0.225f, 0.425f);
        scene.AddComponent(st3Roller, collider);
    }

    Entity st3JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = invalidEntity;
        joint.childEntity = st3Roller;
        joint.SetWorldSpace(st3Pos + glm::vec3(0.0f, 1.40f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        joint.motorMode = JointMotorMode::Velocity;
        joint.maxMotorTorque = 12.0f;
        scene.AddComponent(st3JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = CreateMotorPreset(7, 4.0, 0.96); // NEO 550 at 4:1
        motor.commandedVoltage = 6.0_V;
        motor.statorCurrentLimit = 40.0_A;
        scene.AddComponent(st3JointEntity, motor);
    }

    // -------------------------------------------------------------------------
    // F. Station 4: Precision Robotic Servo Stand (Dual-Mode Servo Presets)
    // -------------------------------------------------------------------------
    const glm::vec3 st4Pos = glm::vec3(4.8f, 0.0f, 0.0f);
    Entity st4Pedestal = scene.CreateEntity();
    {
        assignTransforms(st4Pedestal, st4Pos + glm::vec3(0.0f, 0.50f, 0.0f), glm::vec3(0.55f, 1.0f, 0.55f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.45f, 0.25f, 0.65f); // Deep Indigo/Purple
        scene.AddComponent(st4Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st4Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.275f, 0.50f, 0.275f);
        scene.AddComponent(st4Pedestal, collider);
    }

    Entity st4Horn = scene.CreateEntity();
    {
        // Pivot at Y = 1.20m, Z = 0.35m. Horn center is at Y = 1.55m.
        assignTransforms(st4Horn, st4Pos + glm::vec3(0.0f, 1.55f, 0.35f), glm::vec3(0.12f, 0.80f, 0.12f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.25f, 0.98f, 0.35f); // Neon Lime Green
        scene.AddComponent(st4Horn, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 0.15f;
        rigidBody.friction = 0.3f;
        scene.AddComponent(st4Horn, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.06f, 0.40f, 0.06f);
        scene.AddComponent(st4Horn, collider);
    }

    Entity st4JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = invalidEntity;
        joint.childEntity = st4Horn;
        joint.SetWorldSpace(st4Pos + glm::vec3(0.0f, 1.20f, 0.35f), glm::vec3(0.0f, 0.0f, 1.0f));
        joint.hasLimits = true;
        joint.minLimit = -glm::radians(150.0f);
        joint.maxLimit = glm::radians(150.0f);
        joint.motorMode = JointMotorMode::Position;
        joint.springFrequency = 14.0f;
        joint.springDamping = 2.0f; // Over-damped to prevent oscillation
        joint.maxFrictionTorque = 0.2f; // Internal gearbox friction prevents ringing
        joint.maxMotorTorque = 4.0f;
        scene.AddComponent(st4JointEntity, joint);

        ServoComponent servo;
        servo.servoModel = ServoModel::GoBildaTorque(ServoMode::Positional);
        servo.targetFraction = 0.5;
        scene.AddComponent(st4JointEntity, servo);
    }

    // -------------------------------------------------------------------------
    // 7. Interactive State, Speed Control & Station Isolation
    // -------------------------------------------------------------------------
    double simSpeed = 1.0;
    double simAccumulator = 0.0;
    bool doSingleStep = false;
    uint64_t totalSimSteps = 0;

    int batteryPresetIndex = 0;
    bool triggerStallSpike = false;

    // Station Isolation: 0 = All Stations Active, 1 = Station 1 Only, 2 = Station 2 Only, 3 = Station 3 Only, 4 = Station 4 Only
    int activeStationIndex = 0;

    // Station 1 controls
    int st1MotorPresetIndex = 0; // Default Kraken X60
    float st1CommandedVolts = 8.0f;

    // Station 2 controls
    int st2MotorPresetIndex = 5; // Default NEO 1.1
    int st2GearboxPresetIndex = 0; // 0 = 25:1, 1 = 50:1, 2 = 75:1, 3 = 100:1
    double st2GearRatio = 25.0;
    float st2TargetAngleDegrees = 0.0f;
    bool st2AutoOscillate = true;
    double st2OscillationTime = 0.0;

    // Station 3 controls
    int st3MotorPresetIndex = 7; // Default NEO 550
    int st3GearboxPresetIndex = 0; // 0 = 4:1, 1 = 1:1 Direct, 2 = 19.2:1 Planetary
    double st3GearRatio = 4.0;
    float st3CommandedVolts = 6.0f;

    // Station 4 controls
    int st4ServoPresetIndex = 0;
    int st4ServoModeIndex = 0;
    float st4TargetAngleDegrees = 0.0f;
    float st4ContinuousSpeed = 0.0f;
    bool st4AutoSweep = true;
    int st4SweepPattern = 0; // 0 = Step Jumps (Full Max Speed Test), 1 = Smooth Sine Wave
    float st4StepIntervalSec = 1.3f;
    double st4StepTimer = 0.0;
    bool st4StepState = false;
    double st4SweepTime = 0.0;
    double st4ProfileAngle = 0.0; // Slew-rate-limited trajectory angle

    // Real-Time Speed Telemetry (sampled over real-world wall clock time)
    float speedWindowRealSeconds = 2.0f;
    struct SpeedSample
    {
        std::chrono::steady_clock::time_point timestamp;
        double speedRadPerSec;
    };
    std::vector<SpeedSample> servoSpeedHistory;
    double servoLiveSpeedDegPerSec = 0.0;
    double servoAvgSpeedDegPerSec = 0.0;
    double servoMaxSpeedDegPerSec = 0.0;
    double servoLiveSpeedRadPerSec = 0.0;
    double servoAvgSpeedRadPerSec = 0.0;
    double servoMaxSpeedRadPerSec = 0.0;

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

            // --- A. Station 1: High-Speed Flywheel ---
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
                    double freeSpeedShaft = m1->motorModel.GetFreeSpeed() / m1->motorModel.GetGearRatio();
                    double targetVel = (m1->appliedVoltage / m1->motorModel.GetNominalVoltage()) * freeSpeedShaft;
                    j1->targetVelocity = static_cast<float>(targetVel);

                    double stallTorquePeak = m1->motorModel.GetStallTorque() * m1->motorModel.GetGearRatio() * m1->motorModel.GetGearboxEfficiency();
                    j1->maxMotorTorque = static_cast<float>(stallTorquePeak);
                }
            }

            // --- B. Station 2: Heavy Geared Arm ---
            auto* m2 = scene.GetComponent<MotorComponent>(st2JointEntity);
            auto* j2 = scene.GetComponent<JointComponent>(st2JointEntity);
            auto* rb2 = scene.GetComponent<RigidBodyComponent>(st2Arm);
            if (m2 && j2 && rb2)
            {
                bool st2Active = (activeStationIndex == 0 || activeStationIndex == 2);
                float speed = glm::dot(rb2->angularVelocity, j2->parentAxis);
                m2->angularVelocity = speed;

                if (!st2Active)
                {
                    m2->commandedVoltage = 0.0_V;
                    m2->appliedVoltage = 0.0_V;
                    m2->statorCurrent = 0.0_A;
                    m2->supplyCurrent = 0.0_A;
                    m2->outputTorque = 0.0;
                    j2->motorMode = JointMotorMode::Position;
                    j2->maxMotorTorque = 0.0f;
                }
                else
                {
                    if (st2AutoOscillate)
                    {
                        st2OscillationTime += fixedDeltaTime;
                        st2TargetAngleDegrees = static_cast<float>(std::sin(st2OscillationTime * 2.5) * 55.0);
                    }

                    float desiredAngleRad = glm::radians(st2TargetAngleDegrees);
                    double angleError = desiredAngleRad - j2->currentAngle;
                    m2->commandedVoltage = std::clamp(angleError * 14.0, -12.0_V, 12.0_V);

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

                    constexpr double kArmLoadInertia = 0.45;
                    m2->outputTorque = m2->motorModel.CalculateTorque(
                        m2->appliedVoltage, speed, kArmLoadInertia, fixedDeltaTime);

                    j2->motorMode = JointMotorMode::Position;
                    j2->targetAngle = desiredAngleRad;

                    double armStallTorque = m2->motorModel.GetStallTorque() * m2->motorModel.GetGearRatio() * m2->motorModel.GetGearboxEfficiency();
                    j2->maxMotorTorque = static_cast<float>(armStallTorque);
                }
            }

            // --- C. Station 3: Intake Roller ---
            auto* m3 = scene.GetComponent<MotorComponent>(st3JointEntity);
            auto* j3 = scene.GetComponent<JointComponent>(st3JointEntity);
            auto* rb3 = scene.GetComponent<RigidBodyComponent>(st3Roller);
            if (m3 && j3 && rb3)
            {
                bool st3Active = (activeStationIndex == 0 || activeStationIndex == 3);
                float speed = glm::dot(rb3->angularVelocity, j3->parentAxis);
                m3->angularVelocity = speed;

                if (!st3Active)
                {
                    m3->commandedVoltage = 0.0_V;
                    m3->appliedVoltage = 0.0_V;
                    m3->statorCurrent = 0.0_A;
                    m3->supplyCurrent = 0.0_A;
                    m3->outputTorque = 0.0;
                    j3->motorMode = JointMotorMode::Velocity;
                    j3->targetVelocity = 0.0f;
                    j3->maxMotorTorque = 0.1f;
                }
                else
                {
                    m3->commandedVoltage = static_cast<double>(st3CommandedVolts);

                    double rawStatorCurrent = m3->motorModel.CalculateCurrent(m3->commandedVoltage, speed);
                    m3->statorCurrent = std::clamp(rawStatorCurrent, -m3->statorCurrentLimit, m3->statorCurrentLimit);

                    if (battery->isBrownoutCritical)
                    {
                        m3->appliedVoltage = 0.0_V;
                    }
                    else
                    {
                        m3->appliedVoltage = battery->batteryModel.ApplySupplyCurrentLimit(
                            m3->commandedVoltage, m3->statorCurrent, battery->terminalVoltage);
                    }

                    double power = std::abs(m3->statorCurrent * m3->appliedVoltage);
                    m3->supplyCurrent = (battery->terminalVoltage > 0.01_V) ? (power / battery->terminalVoltage) : 0.0_A;
                    totalActuatorSupplyCurrent += m3->supplyCurrent;

                    constexpr double kRollerInertia = 0.015;
                    m3->outputTorque = m3->motorModel.CalculateTorque(
                        m3->appliedVoltage, speed, kRollerInertia, fixedDeltaTime);

                    j3->motorMode = JointMotorMode::Velocity;
                    double freeSpeedShaft = m3->motorModel.GetFreeSpeed() / m3->motorModel.GetGearRatio();
                    double targetVel = (m3->appliedVoltage / m3->motorModel.GetNominalVoltage()) * freeSpeedShaft;
                    j3->targetVelocity = static_cast<float>(targetVel);

                    double rollerStallTorque = m3->motorModel.GetStallTorque() * m3->motorModel.GetGearRatio() * m3->motorModel.GetGearboxEfficiency();
                    j3->maxMotorTorque = static_cast<float>(rollerStallTorque);
                }
            }

            // --- D. Station 4: Precision Robotic Servo ---
            auto* s4 = scene.GetComponent<ServoComponent>(st4JointEntity);
            auto* j4 = scene.GetComponent<JointComponent>(st4JointEntity);
            auto* rb4 = scene.GetComponent<RigidBodyComponent>(st4Horn);
            if (s4 && j4 && rb4)
            {
                bool st4Active = (activeStationIndex == 0 || activeStationIndex == 4);
                float speed = glm::dot(rb4->angularVelocity, j4->parentAxis);
                s4->currentVelocity = speed;
                s4->currentAngle = j4->currentAngle;

                if (!st4Active)
                {
                    s4->outputTorque = 0.0;
                    s4->currentDraw = 0.0_A;
                    j4->motorMode = JointMotorMode::Position;
                    j4->maxMotorTorque = 0.0f;
                }
                else
                {
                    double vrmRailVoltage = std::min(6.0_V, battery->terminalVoltage);
                    s4->servoModel.SetSupplyVoltage(vrmRailVoltage);

                    double ratedStallTorque = s4->servoModel.GetStallTorque();
                    double voltageScaling = (s4->servoModel.GetNominalVoltage() > 0.01_V) ? (vrmRailVoltage / s4->servoModel.GetNominalVoltage()) : 1.0;
                    double availableServoTorque = ratedStallTorque * voltageScaling;

                    double maxDatasheetSpeed = s4->servoModel.GetFreeSpeed() * voltageScaling;

                    if (s4->servoModel.GetMode() == ServoMode::Continuous)
                    {
                        s4->servoModel.SetSpeed(static_cast<double>(st4ContinuousSpeed));
                        j4->motorMode = JointMotorMode::Velocity;

                        j4->targetVelocity = static_cast<float>(st4ContinuousSpeed * maxDatasheetSpeed);
                        j4->maxMotorTorque = static_cast<float>(availableServoTorque);
                    }
                    else
                    {
                        if (st4AutoSweep)
                        {
                            if (st4SweepPattern == 0)
                            {
                                // Step Jumps (Full Max Speed Traverse Test)
                                st4StepTimer += fixedDeltaTime;
                                if (st4StepTimer >= static_cast<double>(st4StepIntervalSec))
                                {
                                    st4StepTimer = 0.0;
                                    st4StepState = !st4StepState;
                                }
                                st4TargetAngleDegrees = st4StepState ? 100.0f : -100.0f;
                            }
                            else
                            {
                                // Smooth Sinusoidal Sweep
                                st4SweepTime += fixedDeltaTime;
                                st4TargetAngleDegrees = static_cast<float>(std::sin(st4SweepTime * 2.5) * 110.0);
                            }
                        }

                        double desiredAngleRad = glm::radians(st4TargetAngleDegrees);
                        s4->servoModel.SetTargetAngle(desiredAngleRad);

                        // Align Jolt's position constraint target with ServoModel's internal trajectory rate
                        double distanceToTarget = std::abs(desiredAngleRad - st4ProfileAngle);
                        double speedScale = 1.0;
                        if (distanceToTarget < glm::radians(8.0f))
                        {
                            speedScale = std::clamp(distanceToTarget / glm::radians(8.0f), 0.15, 1.0);
                        }

                        double maxStep = maxDatasheetSpeed * speedScale * fixedDeltaTime;
                        double angleDifference = desiredAngleRad - st4ProfileAngle;
                        if (std::abs(angleDifference) <= maxStep)
                        {
                            st4ProfileAngle = desiredAngleRad;
                        }
                        else
                        {
                            st4ProfileAngle += std::copysign(maxStep, angleDifference);
                        }

                        j4->motorMode = JointMotorMode::Position;
                        j4->targetAngle = static_cast<float>(st4ProfileAngle);
                        j4->springFrequency = 14.0f;
                        j4->springDamping = 2.0f;
                        j4->maxFrictionTorque = 0.2f;
                        j4->maxMotorTorque = static_cast<float>(availableServoTorque);
                    }

                    constexpr double kHornInertia = 0.005;
                    // ServoModel now internally clamps torque and current
                    s4->outputTorque = s4->servoModel.CalculateTorque(
                        s4->currentAngle, s4->currentVelocity, kHornInertia, fixedDeltaTime);

                    s4->currentDraw = s4->servoModel.CalculateCurrent(s4->currentVelocity);
                    totalActuatorSupplyCurrent += s4->currentDraw;
                }
            }

            // --- E. Central Battery State & Voltage Sag Update ---
            if (triggerStallSpike)
            {
                totalActuatorSupplyCurrent += 320.0_A;
            }

            battery->terminalVoltage = battery->batteryModel.CalculateTerminalVoltage(totalActuatorSupplyCurrent);
            battery->totalCurrent = totalActuatorSupplyCurrent + battery->batteryModel.GetQuiescentCurrent();
            battery->isBrownoutWarning = battery->batteryModel.IsBrownoutWarning(battery->terminalVoltage);
            battery->isBrownoutCritical = battery->batteryModel.IsBrownoutCritical(battery->terminalVoltage);

            // Advance ECS physics & joint systems
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
            syncWorldTransform(st3Roller);
            syncWorldTransform(st4Horn);

            ++totalSimSteps;
        };

    // -------------------------------------------------------------------------
    // 8. Engine Loop Configuration with Speed Modulation
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
    // 9. Per-Frame Update Callback & Input Speed Controls
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

            // Keyboard Simulation Speed Hotkeys (Keys 0-9 & Numpad)
            if (input.WasKeyPressed(GLFW_KEY_0) || input.WasKeyPressed(GLFW_KEY_KP_0))
            {
                simSpeed = 0.0;
                SIM_LOG_INFO("Simulation PAUSED");
            }
            else if (input.WasKeyPressed(GLFW_KEY_1) || input.WasKeyPressed(GLFW_KEY_KP_1))
            {
                simSpeed = 0.05;
                SIM_LOG_INFO("Simulation speed: 0.05x (Slow-Mo)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_2) || input.WasKeyPressed(GLFW_KEY_KP_2))
            {
                simSpeed = 0.10;
                SIM_LOG_INFO("Simulation speed: 0.10x (Slow-Mo)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_3) || input.WasKeyPressed(GLFW_KEY_KP_3))
            {
                simSpeed = 0.25;
                SIM_LOG_INFO("Simulation speed: 0.25x (Quarter Speed)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_4) || input.WasKeyPressed(GLFW_KEY_KP_4))
            {
                simSpeed = 0.50;
                SIM_LOG_INFO("Simulation speed: 0.50x (Half Speed)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_5) || input.WasKeyPressed(GLFW_KEY_KP_5))
            {
                simSpeed = 0.75;
                SIM_LOG_INFO("Simulation speed: 0.75x");
            }
            else if (input.WasKeyPressed(GLFW_KEY_6) || input.WasKeyPressed(GLFW_KEY_KP_6))
            {
                simSpeed = 0.90;
                SIM_LOG_INFO("Simulation speed: 0.90x");
            }
            else if (input.WasKeyPressed(GLFW_KEY_7) || input.WasKeyPressed(GLFW_KEY_KP_7))
            {
                simSpeed = 1.00;
                SIM_LOG_INFO("Simulation speed: 1.00x (Normal)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_8) || input.WasKeyPressed(GLFW_KEY_KP_8))
            {
                simSpeed = 2.00;
                SIM_LOG_INFO("Simulation speed: 2.00x (Fast Forward)");
            }
            else if (input.WasKeyPressed(GLFW_KEY_9) || input.WasKeyPressed(GLFW_KEY_KP_9))
            {
                simSpeed = 4.00;
                SIM_LOG_INFO("Simulation speed: 4.00x (Quad Speed)");
            }

            // Single Step forward when paused (. key or numpad decimal)
            if (input.WasKeyPressed(GLFW_KEY_PERIOD) || input.WasKeyPressed(GLFW_KEY_KP_DECIMAL))
            {
                if (simSpeed == 0.0)
                {
                    doSingleStep = true;
                    SIM_LOG_INFO("Single step forward executed (step #" << totalSimSteps + 1 << ")");
                }
            }

            // F3: Cycle telemetry visualizer
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                TelemetryMode current = TelemetryManager::Get().GetMode();
                TelemetryMode next = (current == TelemetryMode::Off) ? TelemetryMode::Minimal :
                    ((current == TelemetryMode::Minimal) ? TelemetryMode::Full : TelemetryMode::Off);

                if (next == TelemetryMode::Full)
                {
                    profilerWindow.Open(renderer.GetWindow());
                }
                else if (profilerWindow.IsOpen())
                {
                    profilerWindow.Close(renderer.GetWindow());
                }

                TelemetryManager::Get().SetMode(next);
            }

            // Sample physical velocity for real-time servo speed telemetry
            auto* rb4 = scene.GetComponent<RigidBodyComponent>(st4Horn);
            auto* j4 = scene.GetComponent<JointComponent>(st4JointEntity);
            double currentSpeedRadPerSec = 0.0;
            if (rb4 && j4)
            {
                currentSpeedRadPerSec = std::abs(glm::dot(rb4->angularVelocity, j4->parentAxis));
            }

            auto now = std::chrono::steady_clock::now();
            servoSpeedHistory.push_back({ now, currentSpeedRadPerSec });

            // Purge samples older than speedWindowRealSeconds in real wall-clock time
            auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(speedWindowRealSeconds));

            size_t validStart = 0;
            while (validStart < servoSpeedHistory.size() && servoSpeedHistory[validStart].timestamp < cutoff)
            {
                ++validStart;
            }
            if (validStart > 0)
            {
                servoSpeedHistory.erase(servoSpeedHistory.begin(), servoSpeedHistory.begin() + validStart);
            }

            servoLiveSpeedRadPerSec = currentSpeedRadPerSec;
            servoLiveSpeedDegPerSec = glm::degrees(currentSpeedRadPerSec);

            double maxSpd = 0.0;
            double sumSpd = 0.0;
            for (const auto& sample : servoSpeedHistory)
            {
                maxSpd = std::max(maxSpd, sample.speedRadPerSec);
                sumSpd += sample.speedRadPerSec;
            }
            servoMaxSpeedRadPerSec = maxSpd;
            servoMaxSpeedDegPerSec = glm::degrees(maxSpd);
            servoAvgSpeedRadPerSec = servoSpeedHistory.empty() ? 0.0 : (sumSpd / static_cast<double>(servoSpeedHistory.size()));
            servoAvgSpeedDegPerSec = glm::degrees(servoAvgSpeedRadPerSec);

            // 3D Visual Lines & Field Markings
            const glm::vec3 batTermPos = batteryPos + glm::vec3(0.0f, 0.35f, 0.0f);
            const glm::vec3 redWireColor = glm::vec3(1.0f, 0.20f, 0.20f);
            const glm::vec3 blackWireColor = glm::vec3(0.25f, 0.25f, 0.30f);

            // Power Distribution Wires
            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st1Pos + glm::vec3(0.0f, 0.60f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st1Pos + glm::vec3(0.0f, 0.60f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st2Pos + glm::vec3(0.0f, 1.20f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st2Pos + glm::vec3(0.0f, 1.20f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st3Pos + glm::vec3(0.0f, 0.50f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st3Pos + glm::vec3(0.0f, 0.50f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st4Pos + glm::vec3(0.0f, 0.50f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st4Pos + glm::vec3(0.0f, 0.50f, 0.0f), blackWireColor);

            // Field Perimeter Lines
            const glm::vec3 whiteLine(0.95f, 0.95f, 0.95f);
            DebugDraw::Line(glm::vec3(-13.0f, 0.02f, -8.0f), glm::vec3(13.0f, 0.02f, -8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(13.0f, 0.02f, -8.0f), glm::vec3(13.0f, 0.02f, 8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(13.0f, 0.02f, 8.0f), glm::vec3(-13.0f, 0.02f, 8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(-13.0f, 0.02f, 8.0f), glm::vec3(-13.0f, 0.02f, -8.0f), whiteLine);

            // Flywheel Rotation Marker
            auto* t1 = scene.GetComponent<TransformComponent>(st1Flywheel);
            if (t1)
            {
                glm::vec3 c = t1->position;
                glm::quat q = t1->rotation;
                glm::vec3 armX = q * glm::vec3(0.55f, 0.0f, 0.0f);
                glm::vec3 armZ = q * glm::vec3(0.0f, 0.0f, 0.55f);
                DebugDraw::Line(c - armX + glm::vec3(0, 0.10f, 0), c + armX + glm::vec3(0, 0.10f, 0), glm::vec3(0.05f, 0.05f, 0.05f));
                DebugDraw::Line(c - armZ + glm::vec3(0, 0.10f, 0), c + armZ + glm::vec3(0, 0.10f, 0), glm::vec3(0.95f, 0.15f, 0.15f));
            }

            // Roller Stripes
            auto* t3 = scene.GetComponent<TransformComponent>(st3Roller);
            if (t3)
            {
                glm::vec3 c = t3->position;
                glm::quat q = t3->rotation;
                glm::vec3 stripe1 = q * glm::vec3(0.24f, 0.0f, 0.0f);
                glm::vec3 stripe2 = q * glm::vec3(0.0f, 0.24f, 0.0f);
                DebugDraw::Line(c + stripe1 - glm::vec3(0, 0, 0.42f), c + stripe1 + glm::vec3(0, 0, 0.42f), glm::vec3(1.0f, 1.0f, 0.1f));
                DebugDraw::Line(c - stripe1 - glm::vec3(0, 0, 0.42f), c - stripe1 + glm::vec3(0, 0, 0.42f), glm::vec3(1.0f, 1.0f, 0.1f));
                DebugDraw::Line(c + stripe2 - glm::vec3(0, 0, 0.42f), c + stripe2 + glm::vec3(0, 0, 0.42f), glm::vec3(0.1f, 0.1f, 0.1f));
            }

            scene.UpdateSystems();

            auto end = std::chrono::high_resolution_clock::now();
            currentUpdateMs = std::chrono::duration<double, std::milli>(end - start).count();
        });

    // -------------------------------------------------------------------------
    // 10. Render Callback & Interactive ImGui Control Panel
    // -------------------------------------------------------------------------
    engine.SetRenderCallback([&](double alpha)
        {
            SIM_PROFILE_SCOPE("Render");
            auto start = std::chrono::high_resolution_clock::now();

            renderer.BeginFrame();
            scene.RenderUpdateSystems();
            telemetryOverlay.Render();

            // Right-Side Docked Control Dashboard
            ImGui::SetNextWindowPos(ImVec2(750.0f, 16.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(514.0f, 740.0f), ImGuiCond_FirstUseEver);

            if (ImGui::Begin("FRC Banana Simulator - Showcase Dashboard"))
            {
                auto* battery = scene.GetComponent<BatteryComponent>(batteryEntity);
                auto* m1 = scene.GetComponent<MotorComponent>(st1JointEntity);
                auto* j1 = scene.GetComponent<JointComponent>(st1JointEntity);
                auto* m2 = scene.GetComponent<MotorComponent>(st2JointEntity);
                auto* j2 = scene.GetComponent<JointComponent>(st2JointEntity);
                auto* m3 = scene.GetComponent<MotorComponent>(st3JointEntity);
                auto* j3 = scene.GetComponent<JointComponent>(st3JointEntity);
                auto* s4 = scene.GetComponent<ServoComponent>(st4JointEntity);
                auto* j4 = scene.GetComponent<JointComponent>(st4JointEntity);

                // Unified motor choices across all 3 non-servo stations
                const char* allMotors[] = {
                    "Kraken X60 (12V, 7.09 Nm, 366A Stall, 6000 RPM)",
                    "Kraken X44 (12V, 4.11 Nm, 279A Stall, 7758 RPM)",
                    "Falcon 500 (12V, 4.69 Nm, 257A Stall, 6380 RPM)",
                    "NEO Vortex (12V, 3.60 Nm, 211A Stall, 6784 RPM)",
                    "Minion (12V, 3.17 Nm, 211A Stall, 7700 RPM)",
                    "NEO 1.1 (12V, 2.60 Nm, 105A Stall, 5676 RPM)",
                    "NEO 2.0 (12V, 2.60 Nm, 105A Stall, 5676 RPM)",
                    "NEO 550 (12V, 0.97 Nm, 100A Stall, 11000 RPM)",
                    "goBILDA 5202 FTC (12V, 0.25 Nm, 9.2A Stall, 6000 RPM)"
                };

                // --- 1. Simulation Speed Control Banner ---
                ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "SIMULATION TIME CONTROL");
                ImGui::Separator();

                if (simSpeed == 0.0)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
                        "[ STATUS: PAUSED ] Press '.' to Step 1 Tick (5ms)");
                }
                else if (simSpeed < 1.0)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.25f, 1.0f),
                        "[ STATUS: SLOW MOTION (%.2fx) ]", simSpeed);
                }
                else if (simSpeed > 1.0)
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f),
                        "[ STATUS: FAST FORWARD (%.1fx) ]", simSpeed);
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f),
                        "[ STATUS: NORMAL REAL-TIME (1.0x) ]");
                }

                // Speed buttons
                if (ImGui::Button("Pause (0)")) simSpeed = 0.0; ImGui::SameLine();
                if (ImGui::Button("Step (.)")) { if (simSpeed == 0.0) doSingleStep = true; } ImGui::SameLine();
                if (ImGui::Button("0.1x (2)")) simSpeed = 0.10; ImGui::SameLine();
                if (ImGui::Button("0.5x (4)")) simSpeed = 0.50; ImGui::SameLine();
                if (ImGui::Button("1.0x (7)")) simSpeed = 1.00; ImGui::SameLine();
                if (ImGui::Button("2.0x (8)")) simSpeed = 2.00; ImGui::SameLine();
                if (ImGui::Button("4.0x (9)")) simSpeed = 4.00;

                ImGui::TextDisabled("Hotkeys: 1-6=Slow (<=1x), 7=1x, 8=2x, 9=4x, 0=Pause, .=Step");
                ImGui::Spacing();
                ImGui::Separator();

                // --- 2. Station Isolation / Single-Station Testing Selector ---
                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.20f, 1.0f), "ACTIVE TESTING STATION ISOLATION");
                ImGui::RadioButton("All Stations", &activeStationIndex, 0); ImGui::SameLine();
                ImGui::RadioButton("St 1 (Flywheel)", &activeStationIndex, 1); ImGui::SameLine();
                ImGui::RadioButton("St 2 (Arm)", &activeStationIndex, 2);
                ImGui::RadioButton("St 3 (Roller)", &activeStationIndex, 3); ImGui::SameLine();
                ImGui::RadioButton("St 4 (Servo)", &activeStationIndex, 4);

                if (activeStationIndex != 0)
                {
                    const char* stationNames[] = {
                        "",
                        "Station 1: High-Speed Flywheel",
                        "Station 2: Articulated Geared Arm",
                        "Station 3: Intake Roller Stand",
                        "Station 4: Precision Robotic Servo"
                    };
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f),
                        "[ ISOLATED ]: %s (Other stations draw 0.0A)", stationNames[activeStationIndex]);
                }
                else
                {
                    ImGui::TextDisabled("All stations active concurrently. Battery bus shared across all 4 stations.");
                }

                ImGui::Spacing();
                ImGui::Separator();

                // --- 3. Battery Electrical Header ---
                if (battery)
                {
                    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.2f, 1.0f), "BATTERY ELECTRICAL BUS");

                    if (battery->isBrownoutCritical)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f),
                            "[ CRITICAL BROWNOUT (< 6.3V) - MOTOR OUTPUTS CUT OFF ]");
                    }
                    else if (battery->isBrownoutWarning)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.1f, 1.0f),
                            "[ BROWNOUT WARNING (< 6.8V) - BUS SEVERELY SAGGED ]");
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.3f, 1.0f),
                            "[ NORMAL OPERATION - BUS HEALTHY ]");
                    }

                    float fraction = static_cast<float>(battery->terminalVoltage / 13.0);
                    ImVec4 barColor = (battery->terminalVoltage > 11.5) ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) :
                        ((battery->terminalVoltage > 8.0) ? ImVec4(0.9f, 0.8f, 0.1f, 1.0f) : ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
                    char overlayBuf[64];
                    std::snprintf(overlayBuf, sizeof(overlayBuf), "Terminal: %.2f V / %.2f V",
                        battery->terminalVoltage, battery->batteryModel.GetNominalVoltage());
                    ImGui::ProgressBar(fraction, ImVec2(-1.0f, 20.0f), overlayBuf);
                    ImGui::PopStyleColor();

                    ImGui::Text("Total Current: %.1f A  (Quiescent: %.1f A)",
                        battery->totalCurrent, battery->batteryModel.GetQuiescentCurrent());

                    const char* batteryPresets[] = {
                        "FRC Standard 12.6V SLA (23 mOhm)",
                        "Worn / High Resistance FRC (48 mOhm)",
                        "FTC 12V Slim Pack (150 mOhm)"
                    };
                    if (ImGui::Combo("Battery Type", &batteryPresetIndex, batteryPresets, IM_ARRAYSIZE(batteryPresets)))
                    {
                        if (batteryPresetIndex == 0)
                        {
                            battery->batteryModel = BatteryModel::FRCStandard();
                        }
                        else if (batteryPresetIndex == 1)
                        {
                            battery->batteryModel = BatteryModel(11.80_V, 0.048_ohm, 1.5_A, 60.0_A, 40.0_A, 6.8_V, 6.3_V);
                        }
                        else
                        {
                            battery->batteryModel = BatteryModel::FTCStandard();
                        }
                    }

                    ImGui::Button("Hold for 320A Stall Spike (Test Brownout)");
                    triggerStallSpike = ImGui::IsItemActive();
                }

                ImGui::Spacing();
                ImGui::Separator();

                // --- 4. Actuator Showcase Tabs ---
                if (ImGui::BeginTabBar("ShowcaseTabs"))
                {
                    // TAB 1: Station 1 Flywheel
                    if (ImGui::BeginTabItem("Flywheel (Shooter)"))
                    {
                        ImGui::Text("Station 1: High-Speed Flywheel Test Stand (Uncapped RPM)");
                        if (activeStationIndex != 1)
                        {
                            if (ImGui::Button("Isolate Station 1 (Test Exclusively)")) activeStationIndex = 1;
                        }
                        else
                        {
                            if (ImGui::Button("Activate All Stations")) activeStationIndex = 0;
                        }
                        ImGui::Spacing();

                        if (ImGui::Combo("Motor Model##1", &st1MotorPresetIndex, allMotors, IM_ARRAYSIZE(allMotors)))
                        {
                            if (m1)
                            {
                                m1->motorModel = CreateMotorPreset(st1MotorPresetIndex, 1.0, 0.98);
                            }
                        }

                        ImGui::SliderFloat("Commanded Voltage##1", &st1CommandedVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Stop (0V)##1")) st1CommandedVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("6.0 V##1")) st1CommandedVolts = 6.0f; ImGui::SameLine();
                        if (ImGui::Button("Full 12.0 V##1")) st1CommandedVolts = 12.0f; ImGui::SameLine();
                        if (ImGui::Button("Reverse (-12V)##1")) st1CommandedVolts = -12.0f;

                        if (m1 && j1)
                        {
                            double rpm = (m1->angularVelocity * 60.0) / (2.0 * 3.1415926535);
                            ImGui::Text("Output Speed:    %.0f RPM  (%.1f rad/s)", rpm, m1->angularVelocity);
                            ImGui::Text("Applied Voltage: %.2f V (Commanded: %.1f V)", m1->appliedVoltage, st1CommandedVolts);
                            ImGui::Text("Stator Current:  %.1f A", m1->statorCurrent);
                            ImGui::Text("Supply Current:  %.1f A", m1->supplyCurrent);
                            ImGui::Text("Output Torque:   %.3f N*m", m1->outputTorque);
                            ImGui::Text("Joint Deflection: %.4f mm", j1->currentDeflection * 1000.0f);
                        }
                        ImGui::EndTabItem();
                    }

                    // TAB 2: Station 2 Arm
                    if (ImGui::BeginTabItem("Geared Arm"))
                    {
                        ImGui::Text("Station 2: Heavy Articulated Arm (High Reduction)");
                        if (activeStationIndex != 2)
                        {
                            if (ImGui::Button("Isolate Station 2 (Test Exclusively)")) activeStationIndex = 2;
                        }
                        else
                        {
                            if (ImGui::Button("Activate All Stations")) activeStationIndex = 0;
                        }
                        ImGui::Spacing();

                        const char* armGearboxes[] = { "25:1 Reduction", "50:1 Reduction", "75:1 High-Reduction", "100:1 Extreme Lift" };
                        bool mChanged = ImGui::Combo("Motor Model##2", &st2MotorPresetIndex, allMotors, IM_ARRAYSIZE(allMotors));
                        bool gChanged = ImGui::Combo("Gearbox Ratio##2", &st2GearboxPresetIndex, armGearboxes, IM_ARRAYSIZE(armGearboxes));

                        if ((mChanged || gChanged) && m2)
                        {
                            double ratios[] = { 25.0, 50.0, 75.0, 100.0 };
                            st2GearRatio = ratios[st2GearboxPresetIndex];
                            m2->motorModel = CreateMotorPreset(st2MotorPresetIndex, st2GearRatio, 0.95);
                        }

                        ImGui::Checkbox("Auto Oscillate Arm (Sine Wave)", &st2AutoOscillate);
                        if (!st2AutoOscillate)
                        {
                            ImGui::SliderFloat("Target Angle##2", &st2TargetAngleDegrees, -70.0f, 70.0f, "%.1f deg");
                            if (ImGui::Button("Level 0 deg##2")) st2TargetAngleDegrees = 0.0f; ImGui::SameLine();
                            if (ImGui::Button("+45 deg##2")) st2TargetAngleDegrees = 45.0f; ImGui::SameLine();
                            if (ImGui::Button("-45 deg##2")) st2TargetAngleDegrees = -45.0f;
                        }

                        if (m2 && j2)
                        {
                            float currentDeg = glm::degrees(j2->currentAngle);
                            ImGui::Text("Arm Angle:       %.1f deg (Target: %.1f deg)", currentDeg, st2TargetAngleDegrees);
                            ImGui::Text("Applied Voltage: %.2f V", m2->appliedVoltage);
                            ImGui::Text("Stator Current:  %.1f A (Holding Torque)", m2->statorCurrent);
                            ImGui::Text("Supply Current:  %.1f A", m2->supplyCurrent);
                            ImGui::Text("Holding Torque:  %.2f N*m", m2->outputTorque);
                            ImGui::Text("Joint Deflection: %.4f mm", j2->currentDeflection * 1000.0f);
                        }
                        ImGui::EndTabItem();
                    }

                    // TAB 3: Station 3 Intake Roller
                    if (ImGui::BeginTabItem("Roller (Intake)"))
                    {
                        ImGui::Text("Station 3: Intake Roller Stand (Uncapped RPM)");
                        if (activeStationIndex != 3)
                        {
                            if (ImGui::Button("Isolate Station 3 (Test Exclusively)")) activeStationIndex = 3;
                        }
                        else
                        {
                            if (ImGui::Button("Activate All Stations")) activeStationIndex = 0;
                        }
                        ImGui::Spacing();

                        const char* rollerGearboxes[] = { "4:1 Reduction (NEO 550 Standard)", "1:1 Direct Drive", "19.2:1 FTC Planetary" };
                        bool mChanged = ImGui::Combo("Motor Model##3", &st3MotorPresetIndex, allMotors, IM_ARRAYSIZE(allMotors));
                        bool gChanged = ImGui::Combo("Roller Gearing##3", &st3GearboxPresetIndex, rollerGearboxes, IM_ARRAYSIZE(rollerGearboxes));

                        if ((mChanged || gChanged) && m3)
                        {
                            double ratios[] = { 4.0, 1.0, 19.2 };
                            st3GearRatio = ratios[st3GearboxPresetIndex];
                            m3->motorModel = CreateMotorPreset(st3MotorPresetIndex, st3GearRatio, 0.95);
                        }

                        ImGui::SliderFloat("Commanded Voltage##3", &st3CommandedVolts, -12.0f, 12.0f, "%.1f V");
                        if (ImGui::Button("Stop (0V)##3")) st3CommandedVolts = 0.0f; ImGui::SameLine();
                        if (ImGui::Button("Full 12.0 V##3")) st3CommandedVolts = 12.0f;

                        if (m3 && j3)
                        {
                            double rpm = (m3->angularVelocity * 60.0) / (2.0 * 3.1415926535);
                            ImGui::Text("Roller Speed:    %.0f RPM", rpm);
                            ImGui::Text("Applied Voltage: %.2f V", m3->appliedVoltage);
                            ImGui::Text("Stator Current:  %.1f A", m3->statorCurrent);
                            ImGui::Text("Supply Current:  %.1f A", m3->supplyCurrent);
                            ImGui::Text("Torque:          %.3f N*m", m3->outputTorque);
                            ImGui::Text("Joint Deflection: %.4f mm", j3->currentDeflection * 1000.0f);
                        }
                        ImGui::EndTabItem();
                    }

                    // TAB 4: Station 4 Robotic Servos
                    if (ImGui::BeginTabItem("Servos"))
                    {
                        ImGui::Text("Station 4: Precision Robotic Servo Mechanism");
                        if (activeStationIndex != 4)
                        {
                            if (ImGui::Button("Isolate Station 4 (Test Exclusively)")) activeStationIndex = 4;
                        }
                        else
                        {
                            if (ImGui::Button("Activate All Stations")) activeStationIndex = 0;
                        }
                        ImGui::Spacing();

                        const char* servoPresets[] = {
                            "goBILDA Dual-Mode Torque (21.6 kg*cm, 0.20 s/60 deg)",
                            "goBILDA Dual-Mode Speed (9.3 kg*cm, 0.09 s/60 deg)",
                            "goBILDA Super Speed (4.7 kg*cm, 0.08 s/60 deg)",
                            "Swyft Robotics Torque (33.5 kg*cm, 0.112 s/60 deg)",
                            "Swyft Robotics Speed (19.0 kg*cm, 0.062 s/60 deg)",
                            "Swyft Robotics Balanced (27.3 kg*cm, 0.092 s/60 deg)",
                            "Swyft Robotics Nano (10.2 kg*cm, 0.110 s/60 deg)"
                        };
                        const char* servoModes[] = { "Positional Tracking", "Continuous Rotation" };

                        bool presetChanged = ImGui::Combo("Servo Model", &st4ServoPresetIndex, servoPresets, IM_ARRAYSIZE(servoPresets));
                        bool modeChanged = ImGui::Combo("Operating Mode", &st4ServoModeIndex, servoModes, IM_ARRAYSIZE(servoModes));

                        if ((presetChanged || modeChanged) && s4)
                        {
                            ServoMode sm = (st4ServoModeIndex == 0) ? ServoMode::Positional : ServoMode::Continuous;
                            if (st4ServoPresetIndex == 0) s4->servoModel = ServoModel::GoBildaTorque(sm);
                            else if (st4ServoPresetIndex == 1) s4->servoModel = ServoModel::GoBildaSpeed(sm);
                            else if (st4ServoPresetIndex == 2) s4->servoModel = ServoModel::GoBildaSuperSpeed(sm);
                            else if (st4ServoPresetIndex == 3) s4->servoModel = ServoModel::SwyftTorque(sm);
                            else if (st4ServoPresetIndex == 4) s4->servoModel = ServoModel::SwyftSpeed(sm);
                            else if (st4ServoPresetIndex == 5) s4->servoModel = ServoModel::SwyftBalanced(sm);
                            else if (st4ServoPresetIndex == 6) s4->servoModel = ServoModel::SwyftNano(sm);

                            if (j4)
                            {
                                st4ProfileAngle = j4->currentAngle;
                            }
                            servoSpeedHistory.clear();
                        }

                        if (st4ServoModeIndex == 0)
                        {
                            ImGui::Checkbox("Auto Sweep Back & Forth", &st4AutoSweep);
                            if (st4AutoSweep)
                            {
                                const char* sweepPatterns[] = {
                                    "Step Jumps (Full Max Speed Test)",
                                    "Smooth Sine Wave"
                                };
                                ImGui::Combo("Sweep Pattern", &st4SweepPattern, sweepPatterns, IM_ARRAYSIZE(sweepPatterns));
                                if (st4SweepPattern == 0)
                                {
                                    ImGui::SliderFloat("Step Interval", &st4StepIntervalSec, 0.5f, 3.0f, "%.1f s");
                                }
                            }
                            else
                            {
                                ImGui::SliderFloat("Target Angle##4", &st4TargetAngleDegrees, -150.0f, 150.0f, "%.1f deg");
                                if (ImGui::Button("Center 0 deg##4")) st4TargetAngleDegrees = 0.0f; ImGui::SameLine();
                                if (ImGui::Button("-90 deg##4")) st4TargetAngleDegrees = -90.0f; ImGui::SameLine();
                                if (ImGui::Button("+90 deg##4")) st4TargetAngleDegrees = 90.0f;
                            }
                        }
                        else
                        {
                            ImGui::SliderFloat("Continuous Speed", &st4ContinuousSpeed, -1.0f, 1.0f, "%.2f");
                            if (ImGui::Button("Stop (0.0)##4")) st4ContinuousSpeed = 0.0f;
                        }

                        if (s4 && j4)
                        {
                            float currentDeg = glm::degrees(j4->currentAngle);
                            double kgCm = s4->outputTorque / 0.0980665;
                            ImGui::Text("Servo Angle:     %.1f deg (Target: %.1f deg)", currentDeg, st4TargetAngleDegrees);
                            ImGui::Text("Rail Voltage:    %.2f V (Scaled from Bus)", s4->servoModel.GetSupplyVoltage());
                            ImGui::Text("Current Draw:    %.2f A (%.0f mA)", s4->currentDraw, s4->currentDraw * 1000.0);
                            ImGui::Text("Output Torque:   %.2f N*m (%.1f kg*cm)", s4->outputTorque, kgCm);
                            ImGui::Text("Joint Deflection: %.4f mm", j4->currentDeflection * 1000.0f);

                            // --- Real-Time Speed Telemetry Dashboard ---
                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::TextColored(ImVec4(0.3f, 0.9f, 1.0f, 1.0f), "REAL-TIME SPEED TELEMETRY");
                            ImGui::SliderFloat("Window (Real Seconds)", &speedWindowRealSeconds, 0.5f, 5.0f, "%.1f s");

                            double liveSecPer60 = (servoLiveSpeedDegPerSec > 1.0) ? (60.0 / servoLiveSpeedDegPerSec) : 0.0;
                            double avgSecPer60 = (servoAvgSpeedDegPerSec > 1.0) ? (60.0 / servoAvgSpeedDegPerSec) : 0.0;
                            double maxSecPer60 = (servoMaxSpeedDegPerSec > 1.0) ? (60.0 / servoMaxSpeedDegPerSec) : 0.0;

                            ImGui::Text("Current Speed:   %6.1f deg/s  (%5.2f rad/s | %.3f s/60deg)",
                                servoLiveSpeedDegPerSec, servoLiveSpeedRadPerSec, liveSecPer60);
                            ImGui::Text("Average Speed:   %6.1f deg/s  (%5.2f rad/s | %.3f s/60deg)",
                                servoAvgSpeedDegPerSec, servoAvgSpeedRadPerSec, avgSecPer60);
                            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f),
                                "Maximum Speed:   %6.1f deg/s  (%5.2f rad/s | %.3f s/60deg)",
                                servoMaxSpeedDegPerSec, servoMaxSpeedRadPerSec, maxSecPer60);

                            double ratedSec60 = GetServoRatedSecPer60Deg(st4ServoPresetIndex);
                            ImGui::TextDisabled("Datasheet Rating: %.3f s/60deg (%.1f deg/s at 6.0V nominal)",
                                ratedSec60, 60.0 / ratedSec60);
                        }
                        ImGui::EndTabItem();
                    }

                    // TAB 5: Controls & Sandbox Tools
                    if (ImGui::BeginTabItem("Controls & Sandbox"))
                    {
                        ImGui::Text("Camera & Flight Controls:");
                        ImGui::BulletText("Hold Right Mouse Button + WASD to fly camera freely.");
                        ImGui::BulletText("Hold Space to fly upward.");
                        ImGui::BulletText("Press Tab to toggle Free-Fly vs Orbit camera.");
                        ImGui::BulletText("Press F3 to toggle Profiler overlay & Flame graph.");
                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Text("Dynamic Physics Testing:");
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
                            rigidBody.friction = 0.6f;
                            rigidBody.restitution = 0.35f;
                            scene.AddComponent(dropEntity, rigidBody);

                            ColliderComponent collider;
                            if (mesh.shape == PrimitiveShape::Cube)
                            {
                                collider.shape = ColliderShape::Box;
                                collider.boxHalfExtents = glm::vec3(0.25f, 0.25f, 0.25f);
                            }
                            else
                            {
                                collider.shape = ColliderShape::Sphere;
                                collider.radius = 0.25f;
                            }
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

    // 11. Run Engine Loop
    engine.Run();

    // 12. Clean Teardown
    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    joltContext.Shutdown();
    TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator shut down cleanly.");
    return 0;
}