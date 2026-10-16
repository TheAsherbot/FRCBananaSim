/**
 * @file Main.cpp
 * @brief Entry point and initial ECS, rendering, and telemetry integration demonstration.
 */

#include "Core/Engine.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Core/ECS/Systems/TransformPropagationSystem.h"
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

using FRCBananaSim::Core::Engine;
using FRCBananaSim::Core::ECS::Scene;
using FRCBananaSim::Core::ECS::Entity;
using FRCBananaSim::Core::ECS::Components::TransformComponent;
using FRCBananaSim::Core::ECS::Components::WorldTransformComponent;
using FRCBananaSim::Core::ECS::Systems::TransformPropagationSystem;
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
    SIM_LOG_INFO("Starting FRC Banana Simulator - ECS & Renderer Test...");

    // 1. Initialize Telemetry Logging
    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal); // Start with HUD active

    // 2. Initialize Renderer (OpenGL 3.3 Core Window)
    Renderer renderer(1280, 720, "FRC Banana Simulator - ECS & Renderer Test");
    if (!renderer.IsValid())
    {
        SIM_LOG_FATAL("Failed to initialize Renderer subsystem");
        TelemetryManager::Get().Shutdown();
        return -1;
    }

    // 3. Telemetry Visualizers
    TelemetryOverlay telemetryOverlay;
    ProfilerWindow profilerWindow;

    // 4. Instantiate ECS Scene and register core systems
    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // 5. Create Scene Entities

    // Ground Plane Entity
    Entity groundEntity = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, -0.05f, 0.0f);
        transform.scale = glm::vec3(30.0f, 1.0f, 30.0f);
        scene.AddComponent(groundEntity, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Plane;
        mesh.color = glm::vec3(0.25f, 0.35f, 0.25f); // Field carpet green
        scene.AddComponent(groundEntity, mesh);
    }

    // Parent Entity (Robot Chassis)
    Entity robotChassis = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, 0.5f, 0.0f);
        transform.scale = glm::vec3(1.6f, 0.6f, 1.6f);
        scene.AddComponent(robotChassis, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Cube;
        mesh.color = glm::vec3(0.95f, 0.85f, 0.15f); // Banana yellow
        scene.AddComponent(robotChassis, mesh);
    }

    // Child Entity (Turret / Intake mounted on Robot Chassis)
    Entity turretEntity = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(0.0f, 0.7f, 0.4f); // Local offset relative to chassis
        transform.scale = glm::vec3(0.5f, 0.5f, 0.5f);
        scene.AddComponent(turretEntity, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Sphere;
        mesh.color = glm::vec3(0.2f, 0.6f, 0.95f); // Accent blue
        scene.AddComponent(turretEntity, mesh);

        // Bind parent-child relationship in ECS scene graph
        scene.SetParent(turretEntity, robotChassis);
    }

    // Static Field Obstacle / Game Piece
    Entity gamePiece = scene.CreateEntity();
    {
        TransformComponent transform;
        transform.position = glm::vec3(3.5f, 0.4f, 2.0f);
        transform.scale = glm::vec3(0.8f, 0.8f, 0.8f);
        scene.AddComponent(gamePiece, transform);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Sphere;
        mesh.color = glm::vec3(0.95f, 0.45f, 0.1f); // Note orange
        scene.AddComponent(gamePiece, mesh);
    }

    // 6. Configure Engine Loop (60 Hz physics update, 144 max FPS)
    Engine engine(60.0, 74);

    float rotationAngleDegrees = 0.0f;
    double accumulatedPhysicsMs = 0.0;
    double currentUpdateMs = 0.0;

    // Fixed-step simulation hook (physics, kinematics)
    engine.SetFixedUpdateCallback([&](double fixedDeltaTime)
        {
            SIM_PROFILE_SCOPE("FixedUpdate");
            auto start = std::chrono::high_resolution_clock::now();

            scene.FixedUpdateSystems();

            auto end = std::chrono::high_resolution_clock::now();
            accumulatedPhysicsMs += std::chrono::duration<double, std::milli>(end - start).count();
        });

    // Variable per-frame logic update hook
    engine.SetUpdateCallback([&](double deltaTime)
        {
            SIM_PROFILE_SCOPE("Update");
            auto start = std::chrono::high_resolution_clock::now();

            // Window exit request check
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

            // Animate robot rotation to test hierarchy transform propagation
            rotationAngleDegrees += static_cast<float>(deltaTime) * 45.0f;
            auto* chassisTransform = scene.GetComponent<TransformComponent>(robotChassis);
            if (chassisTransform)
            {
                float radians = glm::radians(rotationAngleDegrees);
                chassisTransform->rotation = glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
            }

            // Diagnostic visual overlays via DebugDraw
            // Origin coordinate crosshairs
            DebugDraw::Line(glm::vec3(-10.0f, 0.0f, 0.0f), glm::vec3(10.0f, 0.0f, 0.0f), glm::vec3(0.8f, 0.2f, 0.2f));
            DebugDraw::Line(glm::vec3(0.0f, 0.0f, -10.0f), glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.2f, 0.2f, 0.8f));

            // Wireframe indicator over target piece
            DebugDraw::Sphere(glm::vec3(3.5f, 0.4f, 2.0f), 0.5f, glm::vec3(1.0f, 1.0f, 0.0f), 16, true);

            // Advance all registered ECS logic systems
            scene.UpdateSystems();

            auto end = std::chrono::high_resolution_clock::now();
            currentUpdateMs = std::chrono::duration<double, std::milli>(end - start).count();
        });

    // Presentation render callback
    engine.SetRenderCallback([&](double /*alpha*/)
        {
            SIM_PROFILE_SCOPE("Render");
            auto start = std::chrono::high_resolution_clock::now();

            renderer.BeginFrame();

            // RenderSystem draws active meshes and flushes DebugDraw geometry
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

            // Feed live subsystem metrics into the rolling frame history buffer
            FrameSummary summary;
            summary.physicsMs = accumulatedPhysicsMs;
            summary.updateMs = currentUpdateMs;
            summary.renderMs = renderMs;
            summary.totalFrameMs = accumulatedPhysicsMs + currentUpdateMs + renderMs;
            TelemetryManager::Get().PushFrameSummary(summary);

            accumulatedPhysicsMs = 0.0;
        });

    // 7. Run the engine loop
    engine.Run();

    // 8. Clean shutdown
    if (profilerWindow.IsOpen())
    {
        profilerWindow.Close(renderer.GetWindow());
    }

    TelemetryManager::Get().Shutdown();

    SIM_LOG_INFO("FRC Banana Simulator shut down cleanly.");
    return 0;
}