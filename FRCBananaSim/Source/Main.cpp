/**
 * @file Main.cpp
 * @brief Actuator, battery, and joint test showcase with simulation speed control and single-stepping.
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

    // Configure Camera for optimal, close-up framing of all 4 test benches
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

    // A. Field Carpet (Bright, high-contrast grass/carpet green)
    Entity groundEntity = scene.CreateEntity();
    {
        assignTransforms(groundEntity, glm::vec3(0.0f, -0.05f, 0.0f), glm::vec3(28.0f, 0.1f, 18.0f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Plane;
        mesh.color = glm::vec3(0.22f, 0.52f, 0.28f); // High-contrast green
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
        mesh.color = glm::vec3(0.28f, 0.30f, 0.35f); // Metallic Slate
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

    // Battery Positive Terminal (+12V Red Post)
    Entity batPosTerminal = scene.CreateEntity();
    {
        assignTransforms(batPosTerminal, batteryPos + glm::vec3(0.25f, 0.40f, 0.0f), glm::vec3(0.15f, 0.20f, 0.15f));
        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.15f, 0.15f); // Red
        scene.AddComponent(batPosTerminal, mesh);
    }

    // Battery Negative Terminal (GND Blue/Black Post)
    Entity batNegTerminal = scene.CreateEntity();
    {
        assignTransforms(batNegTerminal, batteryPos + glm::vec3(-0.25f, 0.40f, 0.0f), glm::vec3(0.15f, 0.20f, 0.15f));
        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.15f, 0.35f, 0.95f); // Blue
        scene.AddComponent(batNegTerminal, mesh);
    }

    // -------------------------------------------------------------------------
    // C. Station 1: High-Speed Flywheel Shooter Test Stand (FRC Brushless)
    // -------------------------------------------------------------------------
    const glm::vec3 st1Pos = glm::vec3(-4.8f, 0.0f, 0.0f);
    Entity st1Pedestal = scene.CreateEntity();
    {
        assignTransforms(st1Pedestal, st1Pos + glm::vec3(0.0f, 0.75f, 0.0f), glm::vec3(0.7f, 1.5f, 0.7f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.45f, 0.50f, 0.60f); // Slate Blue
        scene.AddComponent(st1Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st1Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.35f, 0.75f, 0.35f);
        scene.AddComponent(st1Pedestal, collider);
    }

    Entity st1Flywheel = scene.CreateEntity();
    {
        assignTransforms(st1Flywheel, st1Pos + glm::vec3(0.0f, 1.70f, 0.0f), glm::vec3(1.1f, 0.20f, 1.1f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.98f, 0.80f, 0.15f); // Vivid Golden Yellow
        scene.AddComponent(st1Flywheel, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 1.6f;
        rigidBody.friction = 0.4f;
        rigidBody.restitution = 0.1f;
        scene.AddComponent(st1Flywheel, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Cylinder;
        collider.radius = 0.55f;
        collider.halfHeight = 0.10f;
        scene.AddComponent(st1Flywheel, collider);
    }

    Entity st1JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = st1Pedestal;
        joint.childEntity = st1Flywheel;
        joint.SetWorldSpace(st1Pos + glm::vec3(0.0f, 1.60f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        joint.motorMode = JointMotorMode::Velocity;
        joint.maxMotorTorque = 10.0f;
        joint.maxFrictionTorque = 0.02f;
        scene.AddComponent(st1JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = DCMotorModel::KrakenX60();
        motor.commandedVoltage = 0.0_V;
        motor.statorCurrentLimit = 60.0_A;
        scene.AddComponent(st1JointEntity, motor);
    }

    // -------------------------------------------------------------------------
    // D. Station 2: Articulated Heavy Arm Test Stand (High-Reduction Gearboxes)
    // -------------------------------------------------------------------------
    const glm::vec3 st2Pos = glm::vec3(-1.6f, 0.0f, 0.0f);
    Entity st2Tower = scene.CreateEntity();
    {
        assignTransforms(st2Tower, st2Pos + glm::vec3(0.0f, 1.35f, 0.0f), glm::vec3(0.5f, 2.7f, 0.5f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.38f, 0.40f, 0.45f);
        scene.AddComponent(st2Tower, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st2Tower, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.25f, 1.35f, 0.25f);
        scene.AddComponent(st2Tower, collider);
    }

    Entity st2Arm = scene.CreateEntity();
    {
        assignTransforms(st2Arm, st2Pos + glm::vec3(0.0f, 2.0f, 0.40f), glm::vec3(0.22f, 1.4f, 0.22f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(1.0f, 0.50f, 0.08f); // High-Vis Orange
        scene.AddComponent(st2Arm, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 4.0f; // 4 kg arm fighting gravity
        rigidBody.friction = 0.6f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(st2Arm, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.11f, 0.70f, 0.11f);
        scene.AddComponent(st2Arm, collider);
    }

    Entity st2JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = st2Tower;
        joint.childEntity = st2Arm;
        joint.SetWorldSpace(st2Pos + glm::vec3(0.0f, 2.65f, 0.40f), glm::vec3(1.0f, 0.0f, 0.0f));
        joint.hasLimits = true;
        joint.minLimit = -glm::radians(75.0f);
        joint.maxLimit = glm::radians(75.0f);
        joint.motorMode = JointMotorMode::Position;
        joint.springFrequency = 14.0f;
        joint.springDamping = 1.0f;
        joint.maxMotorTorque = 65.0f;
        scene.AddComponent(st2JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = DCMotorModel::NEO1(25.0, 0.95);
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
        assignTransforms(st3Pedestal, st3Pos + glm::vec3(0.0f, 0.6f, 0.0f), glm::vec3(0.55f, 1.2f, 0.55f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.42f, 0.45f, 0.50f);
        scene.AddComponent(st3Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st3Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.275f, 0.6f, 0.275f);
        scene.AddComponent(st3Pedestal, collider);
    }

    Entity st3Roller = scene.CreateEntity();
    {
        assignTransforms(st3Roller, st3Pos + glm::vec3(0.0f, 1.4f, 0.0f), glm::vec3(0.5f, 0.5f, 0.9f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.10f, 0.90f, 0.95f); // Vibrant Cyan
        scene.AddComponent(st3Roller, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 0.5f;
        rigidBody.friction = 0.9f;
        scene.AddComponent(st3Roller, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Cylinder;
        collider.radius = 0.25f;
        collider.halfHeight = 0.45f;
        scene.AddComponent(st3Roller, collider);
    }

    Entity st3JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = st3Pedestal;
        joint.childEntity = st3Roller;
        joint.SetWorldSpace(st3Pos + glm::vec3(0.0f, 1.4f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        joint.motorMode = JointMotorMode::Velocity;
        joint.maxMotorTorque = 12.0f;
        scene.AddComponent(st3JointEntity, joint);

        MotorComponent motor;
        motor.motorModel = DCMotorModel::NEO550(4.0, 0.96);
        motor.commandedVoltage = 0.0_V;
        motor.statorCurrentLimit = 40.0_A;
        scene.AddComponent(st3JointEntity, motor);
    }

    // -------------------------------------------------------------------------
    // F. Station 4: Precision Robotic Servo Stand (Dual-Mode Servo Presets)
    // -------------------------------------------------------------------------
    const glm::vec3 st4Pos = glm::vec3(4.8f, 0.0f, 0.0f);
    Entity st4Pedestal = scene.CreateEntity();
    {
        assignTransforms(st4Pedestal, st4Pos + glm::vec3(0.0f, 0.6f, 0.0f), glm::vec3(0.55f, 1.2f, 0.55f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.45f, 0.25f, 0.65f); // Deep Indigo/Purple
        scene.AddComponent(st4Pedestal, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        scene.AddComponent(st4Pedestal, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.275f, 0.6f, 0.275f);
        scene.AddComponent(st4Pedestal, collider);
    }

    Entity st4Horn = scene.CreateEntity();
    {
        assignTransforms(st4Horn, st4Pos + glm::vec3(0.0f, 1.55f, 0.30f), glm::vec3(0.14f, 0.85f, 0.14f));

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.25f, 0.98f, 0.35f); // Neon Lime Green
        scene.AddComponent(st4Horn, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 0.25f;
        rigidBody.friction = 0.5f;
        scene.AddComponent(st4Horn, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.07f, 0.425f, 0.07f);
        scene.AddComponent(st4Horn, collider);
    }

    Entity st4JointEntity = scene.CreateEntity();
    {
        JointComponent joint;
        joint.type = JointType::Hinge;
        joint.parentEntity = st4Pedestal;
        joint.childEntity = st4Horn;
        joint.SetWorldSpace(st4Pos + glm::vec3(0.0f, 1.25f, 0.30f), glm::vec3(0.0f, 0.0f, 1.0f));
        joint.hasLimits = true;
        joint.minLimit = -glm::radians(150.0f);
        joint.maxLimit = glm::radians(150.0f);
        joint.motorMode = JointMotorMode::Position;
        joint.springFrequency = 18.0f;
        joint.springDamping = 1.0f;
        joint.maxMotorTorque = 4.0f;
        scene.AddComponent(st4JointEntity, joint);

        ServoComponent servo;
        servo.servoModel = ServoModel::GoBildaTorque(ServoMode::Positional);
        servo.targetFraction = 0.5;
        scene.AddComponent(st4JointEntity, servo);
    }

    // -------------------------------------------------------------------------
    // 7. Interactive State, Speed Control & Presets
    // -------------------------------------------------------------------------
    double simSpeed = 1.0;
    double simAccumulator = 0.0;
    bool doSingleStep = false;
    uint64_t totalSimSteps = 0;

    int batteryPresetIndex = 0;
    bool triggerStallSpike = false;

    // Station 1 controls
    int st1MotorPresetIndex = 0;
    float st1CommandedVolts = 0.0f;

    // Station 2 controls
    int st2MotorPresetIndex = 0;
    float st2TargetAngleDegrees = 0.0f;
    bool st2AutoOscillate = false;
    double st2OscillationTime = 0.0;

    // Station 3 controls
    int st3MotorPresetIndex = 0;
    float st3CommandedVolts = 0.0f;

    // Station 4 controls
    int st4ServoPresetIndex = 0;
    int st4ServoModeIndex = 0;
    float st4TargetAngleDegrees = 0.0f;
    float st4ContinuousSpeed = 0.0f;
    bool st4AutoSweep = false;
    double st4SweepTime = 0.0;

    int spawnedCount = 0;
    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;

    // -------------------------------------------------------------------------
    // Simulation Step Function (Executed deterministically at 200 Hz)
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
                float speed = glm::dot(rb1->angularVelocity, j1->parentAxis);
                m1->angularVelocity = speed;
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
                j1->maxMotorTorque = static_cast<float>(std::abs(m1->outputTorque));
            }

            // --- B. Station 2: Heavy Geared Arm ---
            auto* m2 = scene.GetComponent<MotorComponent>(st2JointEntity);
            auto* j2 = scene.GetComponent<JointComponent>(st2JointEntity);
            auto* rb2 = scene.GetComponent<RigidBodyComponent>(st2Arm);
            if (m2 && j2 && rb2)
            {
                float speed = glm::dot(rb2->angularVelocity, j2->parentAxis);
                m2->angularVelocity = speed;

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
                j2->maxMotorTorque = std::max(0.5f, static_cast<float>(std::abs(m2->outputTorque)));
            }

            // --- C. Station 3: Intake Roller ---
            auto* m3 = scene.GetComponent<MotorComponent>(st3JointEntity);
            auto* j3 = scene.GetComponent<JointComponent>(st3JointEntity);
            auto* rb3 = scene.GetComponent<RigidBodyComponent>(st3Roller);
            if (m3 && j3 && rb3)
            {
                float speed = glm::dot(rb3->angularVelocity, j3->parentAxis);
                m3->angularVelocity = speed;
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
                j3->maxMotorTorque = static_cast<float>(std::abs(m3->outputTorque));
            }

            // --- D. Station 4: Precision Robotic Servo ---
            auto* s4 = scene.GetComponent<ServoComponent>(st4JointEntity);
            auto* j4 = scene.GetComponent<JointComponent>(st4JointEntity);
            auto* rb4 = scene.GetComponent<RigidBodyComponent>(st4Horn);
            if (s4 && j4 && rb4)
            {
                float speed = glm::dot(rb4->angularVelocity, j4->parentAxis);
                s4->currentVelocity = speed;
                s4->currentAngle = j4->currentAngle;

                double vrmRailVoltage = std::min(6.0_V, battery->terminalVoltage);
                s4->servoModel.SetSupplyVoltage(vrmRailVoltage);

                if (s4->servoModel.GetMode() == ServoMode::Continuous)
                {
                    s4->servoModel.SetSpeed(static_cast<double>(st4ContinuousSpeed));
                    j4->motorMode = JointMotorMode::Velocity;

                    double maxContinuousRadPerSec = s4->servoModel.GetFreeSpeed();
                    j4->targetVelocity = static_cast<float>(st4ContinuousSpeed * maxContinuousRadPerSec);
                }
                else
                {
                    if (st4AutoSweep)
                    {
                        st4SweepTime += fixedDeltaTime;
                        st4TargetAngleDegrees = static_cast<float>(std::sin(st4SweepTime * 2.0) * 120.0);
                    }
                    s4->servoModel.SetTargetAngle(glm::radians(st4TargetAngleDegrees));
                    j4->motorMode = JointMotorMode::Position;
                    j4->targetAngle = static_cast<float>(s4->servoModel.GetTargetAngle());
                }

                constexpr double kHornInertia = 0.005;
                s4->outputTorque = s4->servoModel.CalculateTorque(
                    s4->currentAngle, s4->currentVelocity, kHornInertia, fixedDeltaTime);
                s4->currentDraw = s4->servoModel.CalculateCurrent(s4->currentVelocity);
                totalActuatorSupplyCurrent += s4->currentDraw;

                j4->maxMotorTorque = std::max(0.05f, static_cast<float>(std::abs(s4->outputTorque)));
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

            // Visual Field Markings & 3D Wirelines
            const glm::vec3 batTermPos = batteryPos + glm::vec3(0.0f, 0.35f, 0.0f);
            const glm::vec3 redWireColor = glm::vec3(1.0f, 0.20f, 0.20f);
            const glm::vec3 blackWireColor = glm::vec3(0.25f, 0.25f, 0.30f);

            // Power Distribution Wires
            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st1Pos + glm::vec3(0.0f, 0.75f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st1Pos + glm::vec3(0.0f, 0.75f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st2Pos + glm::vec3(0.0f, 1.35f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st2Pos + glm::vec3(0.0f, 1.35f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st3Pos + glm::vec3(0.0f, 0.6f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st3Pos + glm::vec3(0.0f, 0.6f, 0.0f), blackWireColor);

            DebugDraw::Line(batTermPos + glm::vec3(0.25f, 0.0f, 0.0f), st4Pos + glm::vec3(0.0f, 0.6f, 0.0f), redWireColor);
            DebugDraw::Line(batTermPos - glm::vec3(0.25f, 0.0f, 0.0f), st4Pos + glm::vec3(0.0f, 0.6f, 0.0f), blackWireColor);

            // Field Perimeter Lines (White boundary markings on carpet)
            const glm::vec3 whiteLine(0.95f, 0.95f, 0.95f);
            DebugDraw::Line(glm::vec3(-13.0f, 0.02f, -8.0f), glm::vec3(13.0f, 0.02f, -8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(13.0f, 0.02f, -8.0f), glm::vec3(13.0f, 0.02f, 8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(13.0f, 0.02f, 8.0f), glm::vec3(-13.0f, 0.02f, 8.0f), whiteLine);
            DebugDraw::Line(glm::vec3(-13.0f, 0.02f, 8.0f), glm::vec3(-13.0f, 0.02f, -8.0f), whiteLine);

            // Flywheel Rotation Marker (draws cross on rotor to clearly see high-speed spin)
            auto* j1 = scene.GetComponent<JointComponent>(st1JointEntity);
            if (j1)
            {
                float angle = j1->currentAngle;
                glm::vec3 dirA = glm::vec3(std::cos(angle), 0.0f, std::sin(angle)) * 0.5f;
                glm::vec3 dirB = glm::vec3(-std::sin(angle), 0.0f, std::cos(angle)) * 0.5f;
                glm::vec3 center = st1Pos + glm::vec3(0.0f, 1.82f, 0.0f);
                DebugDraw::Line(center - dirA, center + dirA, glm::vec3(0.1f, 0.1f, 0.1f));
                DebugDraw::Line(center - dirB, center + dirB, glm::vec3(0.1f, 0.1f, 0.1f));
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

            // Right-Side Docked Control Dashboard (Leaves 3D View unobstructed on the left)
            ImGui::SetNextWindowPos(ImVec2(750.0f, 16.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(514.0f, 688.0f), ImGuiCond_FirstUseEver);

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

                // --- 2. Battery Electrical Header ---
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

                // --- 3. Actuator Showcase Tabs ---
                if (ImGui::BeginTabBar("ShowcaseTabs"))
                {
                    // TAB 1: Station 1 Flywheel
                    if (ImGui::BeginTabItem("Flywheel (Shooter)"))
                    {
                        ImGui::Text("Station 1: High-Speed Flywheel Test Stand");
                        const char* st1Motors[] = {
                            "Kraken X60 (12V, 7.09 Nm, 366A Stall, 6000 RPM)",
                            "Falcon 500 (12V, 4.69 Nm, 257A Stall, 6380 RPM)",
                            "NEO Vortex (12V, 3.60 Nm, 211A Stall, 6784 RPM)",
                            "Minion (12V, 3.17 Nm, 211A Stall, 7700 RPM)",
                            "NEO 1.1 (12V, 2.60 Nm, 105A Stall, 5676 RPM)"
                        };
                        if (ImGui::Combo("Motor Preset##1", &st1MotorPresetIndex, st1Motors, IM_ARRAYSIZE(st1Motors)))
                        {
                            if (m1)
                            {
                                if (st1MotorPresetIndex == 0) m1->motorModel = DCMotorModel::KrakenX60();
                                else if (st1MotorPresetIndex == 1) m1->motorModel = DCMotorModel::Falcon500();
                                else if (st1MotorPresetIndex == 2) m1->motorModel = DCMotorModel::NEOVortex();
                                else if (st1MotorPresetIndex == 3) m1->motorModel = DCMotorModel::Minion();
                                else if (st1MotorPresetIndex == 4) m1->motorModel = DCMotorModel::NEO1();
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
                        const char* st2Motors[] = {
                            "NEO 1.1 (25:1 Gearbox, 65 Nm Output Stall)",
                            "NEO Vortex (25:1 Gearbox, 90 Nm Output Stall)",
                            "Kraken X60 (50:1 Gearbox, 354 Nm Output Stall)",
                            "Falcon 500 (50:1 Gearbox, 234 Nm Output Stall)"
                        };
                        if (ImGui::Combo("Motor & Gearbox##2", &st2MotorPresetIndex, st2Motors, IM_ARRAYSIZE(st2Motors)))
                        {
                            if (m2)
                            {
                                if (st2MotorPresetIndex == 0) m2->motorModel = DCMotorModel::NEO1(25.0, 0.95);
                                else if (st2MotorPresetIndex == 1) m2->motorModel = DCMotorModel::NEOVortex(25.0, 0.95);
                                else if (st2MotorPresetIndex == 2) m2->motorModel = DCMotorModel::KrakenX60(50.0, 0.95);
                                else if (st2MotorPresetIndex == 3) m2->motorModel = DCMotorModel::Falcon500(50.0, 0.95);
                            }
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
                    if (ImGui::BeginTabItem("Roller (NEO 550 / FTC)"))
                    {
                        ImGui::Text("Station 3: Intake Roller Stand");
                        const char* st3Motors[] = {
                            "NEO 550 (4:1 Reduction, 11000 RPM Free Rotor)",
                            "goBILDA 5202 FTC Planetary (19.2:1 Reduction, 312 RPM)"
                        };
                        if (ImGui::Combo("Motor Preset##3", &st3MotorPresetIndex, st3Motors, IM_ARRAYSIZE(st3Motors)))
                        {
                            if (m3)
                            {
                                if (st3MotorPresetIndex == 0) m3->motorModel = DCMotorModel::NEO550(4.0, 0.96);
                                else if (st3MotorPresetIndex == 1) m3->motorModel = DCMotorModel::GoBilda5202(19.2, 0.90);
                            }
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
                        }

                        if (st4ServoModeIndex == 0)
                        {
                            ImGui::Checkbox("Auto Sweep Back & Forth", &st4AutoSweep);
                            if (!st4AutoSweep)
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
                        }
                        ImGui::EndTabItem();
                    }

                    // TAB 5: Controls & Sandbox Tools
                    if (ImGui::BeginTabItem("Controls & Sandbox"))
                    {
                        ImGui::Text("Camera & Flight Controls:");
                        ImGui::BulletText("Hold Right Mouse Button + WASD to fly camera freely.");
                        ImGui::BulletText("Hold Space to fly upward (no accidental drops!).");
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