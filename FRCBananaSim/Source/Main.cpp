/**
 * @file Main.cpp
 * @brief Entry point and 200 Hz ECS physics integration demonstration with Jolt.
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
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <chrono>
#include <memory>
#include <vector>

using FRCBananaSim::Core::Engine;
using FRCBananaSim::Core::ECS::Scene;
using FRCBananaSim::Core::ECS::Entity;
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
    SIM_LOG_INFO("Starting FRC Banana Simulator - Jolt Physics & ECS Integration...");

    // 1. Initialize Telemetry Logging
    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal); // Start with HUD overlay active

    // 2. Initialize Renderer (OpenGL 3.3 Core Window)
    Renderer renderer(1280, 720, "FRC Banana Simulator - 200 Hz Physics Demo");
    if (!renderer.IsValid())
    {
        SIM_LOG_FATAL("Failed to initialize Renderer subsystem");
        TelemetryManager::Get().Shutdown();
        return -1;
    }

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

    // 5. Instantiate ECS Scene and register systems
    Scene scene;
    scene.SetFixedDeltaTime(PhysicsConfig::fixedDeltaTime);

    // Physics fixed-step execution
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene, joltContext));
    // Transform propagation
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    // Visual debug drawing of colliders
    scene.AddSystem(std::make_unique<PhysicsDebugDrawSystem>(scene));
    // Render presentation
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // 6. Build Demonstration Scene

    // A. Static Ground Field Carpet
    Entity groundEntity = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, -0.05f, 0.0f);
        transform.scale = glm::vec3(30.0f, 0.1f, 30.0f);
        scene.AddComponent(groundEntity, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Plane;
        mesh.color = glm::vec3(0.25f, 0.35f, 0.25f); // Carpet green
        scene.AddComponent(groundEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Static;
        rigidBody.friction = 0.8f;
        rigidBody.restitution = 0.05f;
        scene.AddComponent(groundEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(15.0f, 0.05f, 15.0f);
        scene.AddComponent(groundEntity, collider);
    }

    // B. Dynamic Stacked Boxes (Simulating robot chassis / battery / weight)
    for (int i = 0; i < 3; ++i)
    {
        Entity boxEntity = scene.CreateEntity();
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, 1.0f + static_cast<float>(i) * 1.2f, 0.0f);
        transform.scale = glm::vec3(1.0f, 1.0f, 1.0f);
        scene.AddComponent(boxEntity, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.85f, 0.15f); // Banana yellow
        scene.AddComponent(boxEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 10.0f + static_cast<float>(i) * 5.0f; // 10 kg, 15 kg, 20 kg
        rigidBody.friction = 0.7f;
        rigidBody.restitution = 0.1f;
        scene.AddComponent(boxEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(0.5f, 0.5f, 0.5f);
        scene.AddComponent(boxEntity, collider);
    }

    // C. Bouncing Dynamic Game Piece (High restitution sphere)
    Entity gamePiece = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(2.5f, 4.0f, 1.0f);
        transform.scale = glm::vec3(0.8f, 0.8f, 0.8f);
        scene.AddComponent(gamePiece, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Sphere;
        mesh.color = glm::vec3(0.95f, 0.45f, 0.1f); // Note orange
        scene.AddComponent(gamePiece, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 0.25f; // 250 grams
        rigidBody.friction = 0.6f;
        rigidBody.restitution = 0.65f; // Bouncy
        scene.AddComponent(gamePiece, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Sphere;
        collider.radius = 0.4f;
        scene.AddComponent(gamePiece, collider);
    }

    // D. Rolling Dynamic Wheel/Cylinder
    Entity wheelEntity = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(-2.5f, 3.0f, 0.5f);
        transform.scale = glm::vec3(0.8f, 0.8f, 0.8f);
        scene.AddComponent(wheelEntity, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.2f, 0.6f, 0.95f); // Accent blue
        scene.AddComponent(wheelEntity, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Dynamic;
        rigidBody.mass = 1.5f;
        rigidBody.friction = 0.85f;
        rigidBody.restitution = 0.1f;
        scene.AddComponent(wheelEntity, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Cylinder;
        collider.radius = 0.4f;
        collider.halfHeight = 0.2f;
        scene.AddComponent(wheelEntity, collider);
    }

    // E. Kinematic Rotating Sweeper (Moves unstoppable by code control)
    Entity kinematicSweeper = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, 0.5f, -3.0f);
        transform.scale = glm::vec3(4.0f, 0.4f, 0.4f);
        scene.AddComponent(kinematicSweeper, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.8f, 0.2f, 0.2f); // Red sweeper bar
        scene.AddComponent(kinematicSweeper, mesh);

        RigidBodyComponent rigidBody;
        rigidBody.motionType = BodyMotionType::Kinematic; // Kinematic!
        scene.AddComponent(kinematicSweeper, rigidBody);

        ColliderComponent collider;
        collider.shape = ColliderShape::Box;
        collider.boxHalfExtents = glm::vec3(2.0f, 0.2f, 0.2f);
        scene.AddComponent(kinematicSweeper, collider);
    }

    // 7. Configure Engine Loop (200 Hz fixed physics, uncapped FPS)
    Engine engine(200.0, 0.0);
    float sweeperAngleDegrees = 0.0f;
    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;
    int spawnedCount = 0;

    // Fixed-step simulation hook (200 Hz)
    engine.SetFixedUpdateCallback([&](double fixedDeltaTime)
        {
            SIM_PROFILE_SCOPE("FixedUpdate");
            auto start = std::chrono::high_resolution_clock::now();

            scene.SetFixedDeltaTime(fixedDeltaTime);
            scene.FixedUpdateSystems();

            auto end = std::chrono::high_resolution_clock::now();
            accumulatedPhysicsMs += std::chrono::duration<double, std::milli>(end - start).count();
        });

    // Variable per-frame logic update hook
    engine.SetUpdateCallback([&](double deltaTime)
        {
            SIM_PROFILE_SCOPE("Update");
            auto start = std::chrono::high_resolution_clock::now();

            // Window close request
            if (renderer.ShouldClose())
            {
                engine.RequestExit();
                return;
            }

            // F3: Cycle telemetry visualization mode (Off -> Minimal -> Full -> Off)
            if (renderer.GetInput().WasKeyPressed(GLFW_KEY_F3))
            {
                TelemetryMode current = TelemetryManager::Get().GetMode();
                TelemetryMode next = TelemetryMode::Off;

                if (current == TelemetryMode::Off)
                {
                    next = TelemetryMode::Minimal;
                    SIM_LOG_INFO("Telemetry mode: Minimal (HUD Overlay)");
                }
                else if (current == TelemetryMode::Minimal)
                {
                    next = TelemetryMode::Full;
                    profilerWindow.Open(renderer.GetWindow());
                    SIM_LOG_INFO("Telemetry mode: Full (Flame Graph Window)");
                }
                else
                {
                    next = TelemetryMode::Off;
                    if (profilerWindow.IsOpen())
                    {
                        profilerWindow.Close(renderer.GetWindow());
                    }
                    SIM_LOG_INFO("Telemetry mode: Off");
                }

                TelemetryManager::Get().SetMode(next);
            }

            // SPACE: Interactively spawn a new dynamic falling object from above!
            if (renderer.GetInput().WasKeyPressed(GLFW_KEY_SPACE))
            {
                ++spawnedCount;
                Entity dropEntity = scene.CreateEntity();

                TransformComponent transform;
                // Randomize horizontal offset slightly
                float offsetX = static_cast<float>((spawnedCount % 5) - 2) * 0.6f;
                float offsetZ = static_cast<float>((spawnedCount % 3) - 1) * 0.6f;
                transform.position = glm::vec3(offsetX, 6.0f, offsetZ);
                transform.scale = glm::vec3(0.6f, 0.6f, 0.6f);
                scene.AddComponent(dropEntity, transform);

                MeshComponent mesh;
                mesh.shape = (spawnedCount % 2 == 0) ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
                mesh.color = glm::vec3(0.3f, 0.8f, 0.9f);
                scene.AddComponent(dropEntity, mesh);

                RigidBodyComponent rigidBody;
                rigidBody.motionType = BodyMotionType::Dynamic;
                rigidBody.mass = 2.0f;
                rigidBody.friction = 0.7f;
                rigidBody.restitution = 0.3f;
                scene.AddComponent(dropEntity, rigidBody);

                ColliderComponent collider;
                if (mesh.shape == PrimitiveShape::Cube)
                {
                    collider.shape = ColliderShape::Box;
                    collider.boxHalfExtents = glm::vec3(0.3f, 0.3f, 0.3f);
                }
                else
                {
                    collider.shape = ColliderShape::Sphere;
                    collider.radius = 0.3f;
                }
                scene.AddComponent(dropEntity, collider);

                SIM_LOG_INFO("Spawned dynamic physics body #" << spawnedCount << " at height Y = 6.0m" );
            }

            // Animate Kinematic Sweeper Bar (moves unstoppable via direct rotation)
            sweeperAngleDegrees += static_cast<float>(deltaTime) * 60.0f;
            auto* sweeperTransform = scene.GetComponent<TransformComponent>(kinematicSweeper);
            if (sweeperTransform)
            {
                float radians = glm::radians(sweeperAngleDegrees);
                sweeperTransform->rotation = glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
            }

            // Diagnostic origin crosshairs
            DebugDraw::Line(glm::vec3(-10.0f, 0.0f, 0.0f), glm::vec3(10.0f, 0.0f, 0.0f), glm::vec3(0.8f, 0.2f, 0.2f));
            DebugDraw::Line(glm::vec3(0.0f, 0.0f, -10.0f), glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.2f, 0.2f, 0.8f));

            // Advance all registered ECS logic systems
            scene.UpdateSystems();

            auto end = std::chrono::high_resolution_clock::now();
            currentUpdateMs = std::chrono::duration<double, std::milli>(end - start).count();
        });

    // Presentation render callback
    engine.SetRenderCallback([&](double alpha)
        {
            SIM_PROFILE_SCOPE("Render");
            auto start = std::chrono::high_resolution_clock::now();

            renderer.BeginFrame();

            // RenderSystem draws meshes and flushes wireframe debug lines
            scene.RenderUpdateSystems();

            // Render on-screen HUD (Mode 1 / Minimal)
            telemetryOverlay.Render();

            renderer.EndFrame();

            // Render secondary flame-graph window (Mode 2 / Full)
            if (TelemetryManager::Get().GetMode() == TelemetryMode::Full && profilerWindow.IsOpen())
            {
                profilerWindow.Render(renderer.GetWindow());
            }

            auto end = std::chrono::high_resolution_clock::now();
            double renderMs = std::chrono::duration<double, std::milli>(end - start).count();

            // Feed live subsystem metrics into telemetry rolling buffer
            FrameSummary summary;
            summary.physicsMs = accumulatedPhysicsMs;
            summary.updateMs = currentUpdateMs;
            summary.renderMs = renderMs;
            summary.totalFrameMs = accumulatedPhysicsMs + currentUpdateMs + renderMs;
            TelemetryManager::Get().PushFrameSummary(summary);

            accumulatedPhysicsMs = 0.0;
        });

    // 8. Run the engine loop
    engine.Run();

    // 9. Clean teardown in reverse order
    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    joltContext.Shutdown();
    TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator shut down cleanly.");
    return 0;
}