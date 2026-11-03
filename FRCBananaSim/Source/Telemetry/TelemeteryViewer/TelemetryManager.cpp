/**
 * @file TelemetryManager.cpp
 * @brief Implementation of the central telemetry coordinator and trace serializer.
 */

#include "Telemetry/TelemeteryViewer/TelemetryManager.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace FRCBananaSim::Telemetry
{
    TelemetryManager& TelemetryManager::Get()
    {
        static TelemetryManager instance;
        return instance;
    }

    TelemetryManager::TelemetryManager()
        : profileCount(0),
        isInitialized(false),
        isRecordingEnabled(true),
        currentMode(TelemetryMode::Off)
    {}

    TelemetryManager::~TelemetryManager()
    {
        if (isInitialized)
        {
            Shutdown();
        }
    }

    void TelemetryManager::SetMode(TelemetryMode mode)
    {
        currentMode = mode;
    }

    TelemetryMode TelemetryManager::GetMode() const
    {
        return currentMode;
    }

    void TelemetryManager::SetRecordingEnabled(bool enabled)
    {
        isRecordingEnabled = enabled;
    }

    bool TelemetryManager::IsRecordingEnabled() const
    {
        return isRecordingEnabled;
    }

    void TelemetryManager::Initialize()
    {
        if (isInitialized)
        {
            return;
        }

        namespace fs = std::filesystem;
        fs::path logDir = "telemetry_logs";
        if (!fs::exists(logDir))
        {
            fs::create_directories(logDir);
        }

        auto now = std::chrono::system_clock::now();
        std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
        std::tm timeInfo;

#if defined(_WIN32)
        localtime_s(&timeInfo, &nowTime);
#else
        localtime_r(&nowTime, &timeInfo);
#endif

        std::stringstream ss;
        ss << std::put_time(&timeInfo, "%Y-%m-%d_%H-%M-%S");

        std::string filepath = (logDir / ("run_" + ss.str() + ".json")).string();

        outputStream.open(filepath);
        WriteHeader();
        liveResults.reserve(32768);
        frameHistory.reserve(128);
        isInitialized = true;
    }

    void TelemetryManager::Shutdown()
    {
        if (!isInitialized)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex);

        if (outputStream.is_open())
        {
            for (const auto& result : liveResults)
            {
                if (profileCount++ > 0)
                {
                    outputStream << ",";
                }

                const char* safeName = result.name ? result.name : "Unknown";
                outputStream << "{\"cat\":\"function\",\"dur\":" << (result.endTimestamp - result.startTimestamp)
                    << ",\"name\":\"" << safeName << "\",\"ph\":\"X\",\"pid\":0,\"tid\":" << result.threadId
                    << ",\"ts\":" << result.startTimestamp;

                if (result.memoryCurrentBytes > 0 || result.memoryPeakBytes > 0)
                {
                    outputStream << ",\"memory_current\":" << result.memoryCurrentBytes
                        << ",\"memory_peak\":" << result.memoryPeakBytes
                        << ",\"memory_allocs\":" << result.memoryAllocations;
                }

                outputStream << "}";
            }

            WriteFooter();
            outputStream.close();
        }

        isInitialized = false;
        profileCount = 0;
    }

    void TelemetryManager::WriteProfile(const ProfileResult& result)
    {
        if (!isInitialized || !isRecordingEnabled)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex);

        if (liveResults.size() >= 32768)
        {
            liveResults.erase(liveResults.begin(), liveResults.begin() + 8192);
        }
        liveResults.push_back(result);
    }

    void TelemetryManager::WriteHeader()
    {
        outputStream << "{\"traceEvents\":[";
    }

    void TelemetryManager::WriteFooter()
    {
        outputStream << "]}";
        outputStream.flush();
    }

    void TelemetryManager::PushFrameSummary(const FrameSummary& summary)
    {
        std::lock_guard<std::mutex> lock(mutex);

        frameHistory.push_back(summary);
        if (frameHistory.size() > 120)
        {
            frameHistory.erase(frameHistory.begin());
        }
    }

    const std::vector<FrameSummary>& TelemetryManager::GetFrameHistory() const
    {
        return frameHistory;
    }

    const std::vector<ProfileResult>& TelemetryManager::GetLiveResults() const
    {
        return liveResults;
    }

    void TelemetryManager::ClearLiveResults()
    {
        std::lock_guard<std::mutex> lock(mutex);
        liveResults.clear();
    }
}
