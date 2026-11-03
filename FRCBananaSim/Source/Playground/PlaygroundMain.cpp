/**
 * @file PlaygroundMain.cpp
 * @brief Standalone executable runner for visual differential mechanism prototyping.
 */

#include "Core/Engine.h"
#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Systems/TransformPropagationSystem.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/JoltContext.h"
#include "Renderer/Renderer.h"
#include "Renderer/Systems/RenderSystem.h"
#include "Playground/PlaygroundScene.h"
#include "Telemetry/TelemeteryViewer/TelemetryManager.h"
#include "Utils/Log.h"

#include <iostream>

using namespace FRCBananaSim::Core::ECS::Core;
using namespace FRCBananaSim::Playground;

int main(int argc, char* argv[])
{
    // 1. Initialize Telemetry Infrastructure
    FRCBananaSim::Telemetry::TelemetryManager::Get().Initialize();
    FRCBananaSim::Telemetry::TelemetryManager::Get().SetMode(FRCBananaSim::Telemetry::TelemetryMode::Minimal);

    // 2. Initialize 3D Hardware Renderer (1600x900 OpenGL 3.3 Core)
    FRCBananaSim::Renderer::Renderer renderer(1600, 900, "FRC Banana Simulator — Differential Mechanism Visual Playground");
    if (!renderer.IsValid())
    {
        SIM_LOG_FATAL("Failed to initialize OpenGL renderer window");
        return -1;
    }

    // Configure 3D Camera Orbiting Centered on the Wrist Pivot (0.0, 1.0, 0.2)
    auto& camera = renderer.GetCamera();
    camera.SetMode(FRCBananaSim::Renderer::Camera::Mode::Orbit);
    camera.orbitTarget = glm::vec3(0.0f, 1.0f, 0.15f);
    camera.orbitDistance = 1.8f;
    camera.pitchDegrees = 18.0f;
    camera.yawDegrees = -45.0f;

    // 3. Initialize Jolt Physics
    FRCBananaSim::Physics::JoltContext joltContext;
    if (!joltContext.Initialize())
    {
        SIM_LOG_FATAL("Failed to initialize Jolt Physics Context");
        return -1;
    }

    // 4. ECS Scene & Presentation Systems
    Scene scene;
    scene.SetFixedDeltaTime(FRCBananaSim::Physics::PhysicsConfig::fixedDeltaTime);
    scene.AddSystem(std::make_unique<FRCBananaSim::Core::ECS::Systems::TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<FRCBananaSim::Renderer::Systems::RenderSystem>(scene, renderer));

    // 5. Initialize the Playground Scene
    PlaygroundScene playground;
    if (!playground.Initialize(scene, joltContext))
    {
        SIM_LOG_FATAL("Failed to initialize Differential Mechanism Playground");
        return -1;
    }

    std::cout << "===================================================================\n";
    std::cout << "  FRC Banana Simulator: 2-DOF Differential Wrist Playground Ready\n";
    std::cout << "  Orbit Camera: Hold Right Mouse Button + Drag to rotate\n";
    std::cout << "  Zoom: Mouse Scroll Wheel\n";
    std::cout << "===================================================================\n";

    // 6. Fixed 200 Hz Physics Step Loop
    FRCBananaSim::Core::Engine engine(200.0, 0.0);

    engine.SetFixedUpdateCallback([&](double dt)
        {
            playground.FixedUpdate(dt);
        });

    // 7. Frame Update & Presentation Loop
    engine.SetUpdateCallback([&](double dt)
        {
            if (renderer.ShouldClose())
            {
                engine.RequestExit();
                return;
            }

            renderer.BeginFrame();

            // Step camera navigation
            camera.ProcessInput(renderer.GetInput(), static_cast<float>(dt));


            // Render 3D Scene Mesh Hierarchy
            scene.RenderUpdateSystems();

            // Render Playground UI Overlay
            playground.RenderUI();

            renderer.EndFrame();
        });

    engine.Run();

    FRCBananaSim::Telemetry::TelemetryManager::Get().Shutdown();
    return 0;
}