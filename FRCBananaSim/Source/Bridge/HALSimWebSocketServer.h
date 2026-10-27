/**
 * @file HALSimWebSocketServer.h
 * @brief Embedded RFC 6455 WebSocket server implementing the WPILib HALSim protocol for live robot code integration.
 */

#pragma once

#include "Control/SimControlBoundary.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace FRCBananaSim::Bridge
{
    /**
     * @struct BridgeConfig
     * @brief Configuration settings for the WPILib HALSim WebSocket server.
     */
    struct BridgeConfig
    {
        uint16_t port = 3300;                ///< TCP listening port (standard: 3300).
        std::string endpoint = "/wpilibws";  ///< Target WebSocket HTTP URI path.
        double publishRateHz = 100.0;        ///< Sensory update rate broadcast to robot code (Hz).
        double heartbeatRateHz = 50.0;       ///< DriverStation heartbeat rate (Hz).
    };

    /**
     * @struct BridgeStatistics
     * @brief Live network throughput and packet counters for telemetry display.
     */
    struct BridgeStatistics
    {
        uint64_t packetsReceived = 0;   ///< Total JSON message frames parsed.
        uint64_t packetsSent = 0;       ///< Total JSON message frames broadcast.
        uint64_t bytesReceived = 0;     ///< Total raw network bytes received.
        uint64_t bytesSent = 0;         ///< Total raw network bytes sent.
        double lastPacketTime = 0.0;    ///< Timestamp of last packet receipt.
    };

    /**
     * @class HALSimWebSocketServer
     * @brief Listens for incoming WPILib halsim_ws client connections, managing motor demands and sensor returns.
     */
    class HALSimWebSocketServer
    {
    public:
        /**
         * @brief Constructs HALSim WebSocket bridge server.
         * @param config Network port and rate settings.
         */
        explicit HALSimWebSocketServer(const BridgeConfig& config = BridgeConfig{});

        /**
         * @brief Destructor; terminates threads and cleans up network sockets.
         */
        ~HALSimWebSocketServer();

        // Non-copyable
        HALSimWebSocketServer(const HALSimWebSocketServer&) = delete;
        HALSimWebSocketServer& operator=(const HALSimWebSocketServer&) = delete;

        /**
         * @brief Starts the background TCP listener and client dispatch threads.
         * @return True if listening socket was successfully opened.
         */
        bool Start();

        /**
         * @brief Terminates all client sessions and shuts down the listening socket.
         */
        void Stop();

        /**
         * @brief Checks if server is actively listening.
         * @return True if running.
         */
        bool IsRunning() const;

        /**
         * @brief Checks if an active WPILib robot code client is currently connected.
         * @return True if client completed the WebSocket handshake.
         */
        bool IsClientConnected() const;

        /**
         * @brief Retrieves latest motor control inputs received from robot code.
         * @param outInputs Mutable reference to SimInputs populated with incoming commands.
         * @return True if new inputs were available and written.
         */
        bool UpdateInputs(Control::SimInputs& outInputs);

        /**
         * @brief Broadcasts current simulation sensory outputs to connected robot code.
         * @param outputs Live physical state (encoders, IMU yaw/pitch/roll, battery voltage).
         */
        void SendOutputs(const Control::SimOutputs& outputs);

        /**
         * @brief Commands DriverStation enable and mode flags.
         * @param enabled Enable robot operation if true.
         * @param autonomous Autonomous mode if true, Teleop if false.
         */
        void SetDriverStation(bool enabled, bool autonomous = false);

        /**
         * @brief Gets current DriverStation enabled state.
         * @return True if enabled.
         */
        bool IsDriverStationEnabled() const;

        /**
         * @brief Gets current DriverStation mode.
         * @return True if autonomous, false if teleop.
         */
        bool IsAutonomous() const;

        /**
         * @brief Retrieves live network statistics.
         * @return BridgeStatistics struct.
         */
        BridgeStatistics GetStatistics() const;

        /**
         * @brief Exports all discovered SimDevice JSON tree descriptors to file.
         * @param filepath Destination file path.
         * @return True on success.
         */
        bool DumpSimDevices(const std::string& filepath) const;

        /**
         * @brief Dispatches raw incoming JSON payload (public for unit testing without live network sockets).
         * @param jsonText Valid JSON message string.
         * @return True if message was recognized and handled.
         */
        bool DispatchIncomingJson(std::string_view jsonText);

        /**
         * @brief Encodes payload into RFC 6455 unmasked server text frame (public for unit testing).
         * @param text Text payload.
         * @return Byte buffer of the encoded WebSocket frame.
         */
        static std::vector<uint8_t> EncodeTextFrame(std::string_view text);

        /**
         * @brief Decodes masked or unmasked RFC 6455 frame from raw buffer (public for unit testing).
         * @param inOutBuffer Raw byte buffer; consumed bytes are removed from front.
         * @param outPayload Extracted text payload.
         * @param outOpcode Extracted opcode.
         * @return True if a complete frame was parsed.
         */
        static bool DecodeFrame(std::vector<uint8_t>& inOutBuffer, std::string& outPayload, uint8_t& outOpcode);

    private:
        BridgeConfig config;
        std::atomic<bool> isRunning = false;
        std::atomic<bool> isConnected = false;

        std::atomic<bool> dsEnabled = false;
        std::atomic<bool> dsAutonomous = false;

        mutable std::mutex inputsMutex;
        Control::SimInputs currentInputs;
        bool hasNewInputs = false;

        mutable std::mutex outputsMutex;
        Control::SimOutputs latestOutputs;

        mutable std::mutex statsMutex;
        BridgeStatistics stats;

        mutable std::mutex devicesMutex;
        std::unordered_set<std::string> discoveredDevices;
        std::unordered_map<std::string, std::string> rawDeviceJson;

        std::thread workerThread;
        uintptr_t listenSocket = ~0ULL; // INVALID_SOCKET

        void RunServerLoop();
    };
}