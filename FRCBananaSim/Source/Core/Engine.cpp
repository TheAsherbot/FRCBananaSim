/**
 * @file Engine.cpp
 * @brief Implementation of the main Engine coordinator and execution loop.
 */

#include "Core/Engine.h"
#include "Telemetry/TelemetryMacros.h"
#include "Telemetry/TelemetryManager.h"
#include "Utils/FixedTimestep.h"
#include "Utils/Log.h"

#include <chrono>
#include <thread>

#define NOMINMAX
#include <windows.h>
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")

namespace FRCBananaSim::Core
{
    Engine::Engine(double physicsHz, double maxFrameRate)
        : isRunning(true),
        maxFrameRate(maxFrameRate),
        fixedTimestep(new FixedTimestep(physicsHz))
    {
        if (maxFrameRate > 0.0)
        {
            SIM_LOG_INFO("Engine constructed at " << physicsHz << " Hz (max frame rate: " << maxFrameRate << " FPS)");
        }
        else
        {
            SIM_LOG_INFO("Engine constructed at " << physicsHz << " Hz (max frame rate: unlimited)");
        }
    }

    Engine::~Engine()
    {
        delete fixedTimestep;
    }

    void Engine::SetMaxFrameRate(double maxFrameRate)
    {
        this->maxFrameRate = maxFrameRate;
        if (maxFrameRate > 0.0)
        {
            SIM_LOG_INFO("Max frame rate set to " << maxFrameRate << " FPS");
        }
        else
        {
            SIM_LOG_INFO("Max frame rate set to unlimited");
        }
    }

    double Engine::GetMaxFrameRate() const
    {
        return maxFrameRate;
    }

    void Engine::SetFixedUpdateCallback(FixedUpdateCallback fixedUpdateCallBack)
    {
        this->fixedUpdateCallback = std::move(fixedUpdateCallBack);
    }

    void Engine::SetUpdateCallback(UpdateCallback updateCallBack)
    {
        this->updateCallback = std::move(updateCallBack);
    }

    void Engine::SetRenderCallback(RenderCallback renderCallBack)
    {
        this->renderCallback = std::move(renderCallBack);
    }

    void Engine::SetPostRenderCallback(PostRenderCallback postRenderCallBack)
    {
        this->postRenderCallback = std::move(postRenderCallBack);
    }

    void Engine::RequestExit()
    {
        SIM_LOG_INFO("Exit requested");
        isRunning = false;
    }

    void Engine::Run()
    {
        SIM_LOG_INFO("Engine loop starting");

#if defined(_WIN32)
        // Request 1 ms OS timer granularity on Windows so thread sleeping is accurate
        timeBeginPeriod(1);
#endif

        while (isRunning)
        {
            auto frameStart = std::chrono::high_resolution_clock::now();

            std::uint32_t steps = fixedTimestep->Step();
            double fixedDeltaTime = fixedTimestep->FixedDeltaSeconds();
            double deltaTime = fixedTimestep->DeltaSeconds();

            if (steps > 0)
            {
                // Physics step executed: profile this frame
                Telemetry::TelemetryManager::Get().SetRecordingEnabled(true);
                SIM_PROFILE_SCOPE("Frame");

                double physicsMs = 0.0;
                {
                    SIM_PROFILE_SCOPE("FixedUpdate");
                    auto physicsStart = std::chrono::high_resolution_clock::now();

                    for (std::uint32_t i = 0; i < steps; i++)
                    {
                        if (fixedUpdateCallback)
                        {
                            fixedUpdateCallback(fixedDeltaTime);
                        }
                    }

                    auto physicsEnd = std::chrono::high_resolution_clock::now();
                    physicsMs = std::chrono::duration<double, std::milli>(physicsEnd - physicsStart).count();
                }

                double updateMs = 0.0;
                {
                    SIM_PROFILE_SCOPE("Update");
                    auto updateStart = std::chrono::high_resolution_clock::now();

                    if (updateCallback)
                    {
                        updateCallback(deltaTime);
                    }

                    auto updateEnd = std::chrono::high_resolution_clock::now();
                    updateMs = std::chrono::duration<double, std::milli>(updateEnd - updateStart).count();
                }

                double alpha = fixedTimestep->Alpha();
                double renderMs = 0.0;
                {
                    SIM_PROFILE_SCOPE("Render");
                    auto renderStart = std::chrono::high_resolution_clock::now();

                    if (renderCallback)
                    {
                        renderCallback(alpha);
                    }

                    auto renderEnd = std::chrono::high_resolution_clock::now();
                    renderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
                }

                // Render tool windows outside the timed "Render" scope
                if (postRenderCallback)
                {
                    postRenderCallback();
                }

                if (maxFrameRate > 0.0)
                {
                    SIM_PROFILE_SCOPE("FrameLimiter");

                    const std::chrono::duration<double> targetFrameDuration(1.0 / maxFrameRate);
                    const auto targetEndTime = frameStart + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(targetFrameDuration);

                    auto now = std::chrono::high_resolution_clock::now();
                    while (now < targetEndTime)
                    {
                        auto remaining = targetEndTime - now;

                        if (remaining > std::chrono::milliseconds(2))
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        else if (remaining > std::chrono::microseconds(200))
                        {
                            std::this_thread::yield();
                        }
                        else
                        {
                        }

                        now = std::chrono::high_resolution_clock::now();
                    }
                }

                auto frameEnd = std::chrono::high_resolution_clock::now();
                double totalFrameMs = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();

                Telemetry::TelemetryManager::Get().PushFrameSummary({ totalFrameMs, physicsMs, updateMs, renderMs });
            }
            else
            {
                // No physics step this tick: disable telemetry recording so empty render frames don't bloat the history
                Telemetry::TelemetryManager::Get().SetRecordingEnabled(false);

                if (updateCallback)
                {
                    updateCallback(deltaTime);
                }

                double alpha = fixedTimestep->Alpha();
                if (renderCallback)
                {
                    renderCallback(alpha);
                }

                if (postRenderCallback)
                {
                    postRenderCallback();
                }

                if (maxFrameRate > 0.0)
                {
                    const std::chrono::duration<double> targetFrameDuration(1.0 / maxFrameRate);
                    const auto targetEndTime = frameStart + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(targetFrameDuration);

                    auto now = std::chrono::high_resolution_clock::now();
                    while (now < targetEndTime)
                    {
                        auto remaining = targetEndTime - now;

                        if (remaining > std::chrono::milliseconds(2))
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        else if (remaining > std::chrono::microseconds(200))
                        {
                            std::this_thread::yield();
                        }
                        else
                        {
                        }

                        now = std::chrono::high_resolution_clock::now();
                    }
                }
            }
        }

#if defined(_WIN32)
        timeEndPeriod(1);
#endif

        SIM_LOG_INFO("Engine loop stopped");
    }
}