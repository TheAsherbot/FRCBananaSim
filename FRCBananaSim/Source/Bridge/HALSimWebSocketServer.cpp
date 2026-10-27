/**
 * @file HALSimWebSocketServer.cpp
 * @brief Implementation of the WinSock2 RFC 6455 WPILib HALSim bridge server.
 */

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "Bridge/HALSimWebSocketServer.h"
#include "Bridge/Sha1.h"
#include "Utils/Log.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

#pragma comment(lib, "ws2_32.lib")

namespace FRCBananaSim::Bridge
{
    HALSimWebSocketServer::HALSimWebSocketServer(const BridgeConfig& config)
        : config(config),
        isRunning(false),
        isConnected(false),
        dsEnabled(false),
        dsAutonomous(false)
    {}

    HALSimWebSocketServer::~HALSimWebSocketServer()
    {
        Stop();
    }

    bool HALSimWebSocketServer::Start()
    {
        if (isRunning)
        {
            return true;
        }

        WSADATA wsaData;
        int wsaErr = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (wsaErr != 0)
        {
            SIM_LOG_ERROR("WSAStartup failed with error: " << wsaErr);
            return false;
        }

        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET)
        {
            SIM_LOG_ERROR("Failed to create bridge listening socket: " << WSAGetLastError());
            WSACleanup();
            return false;
        }

        // Reuse address to avoid bind conflicts during rapid relaunch
        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));

        sockaddr_in serverAddr = {};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        serverAddr.sin_port = htons(config.port);

        if (bind(sock, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR)
        {
            SIM_LOG_ERROR("Failed to bind HALSim socket to port " << config.port << ": " << WSAGetLastError());
            closesocket(sock);
            WSACleanup();
            return false;
        }

        if (listen(sock, 1) == SOCKET_ERROR)
        {
            SIM_LOG_ERROR("Failed to listen on HALSim socket: " << WSAGetLastError());
            closesocket(sock);
            WSACleanup();
            return false;
        }

        listenSocket = static_cast<uintptr_t>(sock);
        isRunning = true;
        workerThread = std::thread(&HALSimWebSocketServer::RunServerLoop, this);

        SIM_LOG_INFO("WPILib HALSim WebSocket Server listening on ws://127.0.0.1:"
            << config.port << config.endpoint);
        return true;
    }

    void HALSimWebSocketServer::Stop()
    {
        if (!isRunning)
        {
            return;
        }

        isRunning = false;
        isConnected = false;

        if (listenSocket != ~0ULL)
        {
            closesocket(static_cast<SOCKET>(listenSocket));
            listenSocket = ~0ULL;
        }

        if (workerThread.joinable())
        {
            workerThread.join();
        }

        WSACleanup();
        SIM_LOG_INFO("WPILib HALSim WebSocket Server shut down cleanly.");
    }

    bool HALSimWebSocketServer::IsRunning() const
    {
        return isRunning;
    }

    bool HALSimWebSocketServer::IsClientConnected() const
    {
        return isConnected;
    }

    bool HALSimWebSocketServer::UpdateInputs(Control::SimInputs& outInputs)
    {
        std::lock_guard<std::mutex> lock(inputsMutex);
        if (!hasNewInputs)
        {
            return false;
        }

        outInputs = currentInputs;
        hasNewInputs = false;
        return true;
    }

    void HALSimWebSocketServer::SetDriverStation(bool enabled, bool autonomous)
    {
        dsEnabled = enabled;
        dsAutonomous = autonomous;
    }

    bool HALSimWebSocketServer::IsDriverStationEnabled() const
    {
        return dsEnabled;
    }

    bool HALSimWebSocketServer::IsAutonomous() const
    {
        return dsAutonomous;
    }

    BridgeStatistics HALSimWebSocketServer::GetStatistics() const
    {
        std::lock_guard<std::mutex> lock(statsMutex);
        return stats;
    }

    std::vector<uint8_t> HALSimWebSocketServer::EncodeTextFrame(std::string_view text)
    {
        std::vector<uint8_t> frame;
        frame.reserve(text.size() + 10);

        frame.push_back(0x81); // FIN (0x80) | Text (0x01)

        size_t len = text.size();
        if (len <= 125)
        {
            frame.push_back(static_cast<uint8_t>(len));
        }
        else if (len <= 65535)
        {
            frame.push_back(126);
            frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
            frame.push_back(static_cast<uint8_t>(len & 0xFF));
        }
        else
        {
            frame.push_back(127);
            for (int shift = 56; shift >= 0; shift -= 8)
            {
                frame.push_back(static_cast<uint8_t>((len >> shift) & 0xFF));
            }
        }

        frame.insert(frame.end(), text.begin(), text.end());
        return frame;
    }

    bool HALSimWebSocketServer::DecodeFrame(std::vector<uint8_t>& inOutBuffer,
        std::string& outPayload, uint8_t& outOpcode)
    {
        if (inOutBuffer.size() < 2)
        {
            return false;
        }

        uint8_t b0 = inOutBuffer[0];
        uint8_t b1 = inOutBuffer[1];

        outOpcode = b0 & 0x0F;
        bool masked = (b1 & 0x80) != 0;
        uint64_t payloadLen = b1 & 0x7F;

        size_t headerSize = 2;

        if (payloadLen == 126)
        {
            if (inOutBuffer.size() < 4)
            {
                return false;
            }
            payloadLen = (static_cast<uint64_t>(inOutBuffer[2]) << 8) | inOutBuffer[3];
            headerSize = 4;
        }
        else if (payloadLen == 127)
        {
            if (inOutBuffer.size() < 10)
            {
                return false;
            }
            payloadLen = 0;
            for (size_t i = 0; i < 8; ++i)
            {
                payloadLen = (payloadLen << 8) | inOutBuffer[2 + i];
            }
            headerSize = 10;
        }

        uint8_t mask[4] = { 0 };
        if (masked)
        {
            if (inOutBuffer.size() < headerSize + 4)
            {
                return false;
            }
            mask[0] = inOutBuffer[headerSize];
            mask[1] = inOutBuffer[headerSize + 1];
            mask[2] = inOutBuffer[headerSize + 2];
            mask[3] = inOutBuffer[headerSize + 3];
            headerSize += 4;
        }

        if (inOutBuffer.size() < headerSize + payloadLen)
        {
            return false; // Wait for full payload
        }

        outPayload.resize(payloadLen);
        for (size_t i = 0; i < payloadLen; ++i)
        {
            uint8_t byteVal = inOutBuffer[headerSize + i];
            if (masked)
            {
                byteVal ^= mask[i % 4];
            }
            outPayload[i] = static_cast<char>(byteVal);
        }

        inOutBuffer.erase(inOutBuffer.begin(), inOutBuffer.begin() + headerSize + payloadLen);
        return true;
    }

    bool HALSimWebSocketServer::DispatchIncomingJson(std::string_view jsonText)
    {
        nlohmann::json j;
        try
        {
            j = nlohmann::json::parse(jsonText);
        }
        catch (...)
        {
            return false;
        }

        if (!j.is_object() || !j.contains("type") || !j.contains("data"))
        {
            return false;
        }

        std::string type = j["type"].get<std::string>();
        std::string device = j.value("device", "");
        const auto& data = j["data"];

        if (!device.empty())
        {
            std::lock_guard<std::mutex> lock(devicesMutex);
            discoveredDevices.insert(device);
            rawDeviceJson[device] = j.dump();
        }

        std::lock_guard<std::mutex> lock(inputsMutex);

        if (type == "PWM")
        {
            int channel = -1;
            try
            {
                channel = std::stoi(device);
            }
            catch (...) {}

            if (channel >= 0 && channel < 8)
            {
                double demand = 0.0;
                if (data.contains(">speed"))
                {
                    demand = data[">speed"].get<double>() * 12.0;
                }
                else if (data.contains(">voltage"))
                {
                    demand = data[">voltage"].get<double>();
                }

                currentInputs.motorCommands[channel].mode = Control::MotorControlMode::Voltage;
                currentInputs.motorCommands[channel].demand = std::clamp(demand, -12.0, 12.0);
                hasNewInputs = true;
                return true;
            }
        }
        else if (type == "SimDevice")
        {
            int canId = -1;
            size_t openBracket = device.find('[');
            size_t closeBracket = device.find(']');
            if (openBracket != std::string::npos && closeBracket != std::string::npos &&
                closeBracket > openBracket)
            {
                try
                {
                    canId = std::stoi(device.substr(openBracket + 1, closeBracket - openBracket - 1));
                }
                catch (...) {}
            }

            if (canId > 0)
            {
                // Odd CAN IDs (1, 3, 5, 7) = Steer [0..3]; Even CAN IDs (2, 4, 6, 8) = Drive [4..7]
                int targetIdx = -1;
                if (canId % 2 == 1 && canId <= 7)
                {
                    targetIdx = (canId - 1) / 2;
                }
                else if (canId % 2 == 0 && canId <= 8)
                {
                    targetIdx = 4 + (canId / 2 - 1);
                }

                if (targetIdx >= 0 && targetIdx < 8)
                {
                    double demand = 0.0;
                    Control::MotorControlMode mode = Control::MotorControlMode::Voltage;

                    if (data.contains(">motorOutputLead"))
                    {
                        demand = data[">motorOutputLead"].get<double>() * 12.0;
                    }
                    else if (data.contains(">motorVoltage"))
                    {
                        demand = data[">motorVoltage"].get<double>();
                    }
                    else if (data.contains(">Applied Output"))
                    {
                        demand = data[">Applied Output"].get<double>() * 12.0;
                    }
                    else if (data.contains(">torqueCurrent"))
                    {
                        demand = data[">torqueCurrent"].get<double>();
                        mode = Control::MotorControlMode::TorqueCurrent;
                    }

                    currentInputs.motorCommands[targetIdx].mode = mode;
                    currentInputs.motorCommands[targetIdx].demand = (mode == Control::MotorControlMode::Voltage) ?
                        std::clamp(demand, -12.0, 12.0) : demand;
                    hasNewInputs = true;
                    return true;
                }
            }
        }

        return false;
    }

    void HALSimWebSocketServer::SendOutputs(const Control::SimOutputs& outputs)
    {
        // Broadcasts are dispatched inside the client event loop
    }

    bool HALSimWebSocketServer::DumpSimDevices(const std::string& filepath) const
    {
        std::lock_guard<std::mutex> lock(devicesMutex);
        nlohmann::json dump;
        dump["deviceCount"] = discoveredDevices.size();
        dump["devices"] = nlohmann::json::array();

        for (const auto& dev : discoveredDevices)
        {
            nlohmann::json item;
            item["name"] = dev;
            auto it = rawDeviceJson.find(dev);
            if (it != rawDeviceJson.end())
            {
                item["lastPayload"] = it->second;
            }
            dump["devices"].push_back(item);
        }

        std::ofstream out(filepath);
        if (!out.is_open())
        {
            return false;
        }
        out << dump.dump(4);
        return true;
    }

    void HALSimWebSocketServer::RunServerLoop()
    {
        SOCKET listenSock = static_cast<SOCKET>(listenSocket);

        while (isRunning)
        {
            fd_set readFds;
            FD_ZERO(&readFds);
            FD_SET(listenSock, &readFds);

            timeval tv = { 0, 50000 }; // 50 ms select timeout
            int activity = select(0, &readFds, nullptr, nullptr, &tv);

            if (activity <= 0 || !FD_ISSET(listenSock, &readFds))
            {
                continue;
            }

            sockaddr_in clientAddr = {};
            int addrLen = sizeof(clientAddr);
            SOCKET clientSock = accept(listenSock, reinterpret_cast<sockaddr*>(&clientAddr), &addrLen);
            if (clientSock == INVALID_SOCKET)
            {
                continue;
            }

            // Set client socket to non-blocking
            u_long nonBlocking = 1;
            ioctlsocket(clientSock, FIONBIO, &nonBlocking);

            std::vector<uint8_t> readBuffer;
            bool handshaked = false;

            auto lastDsHeartbeat = std::chrono::steady_clock::now();
            auto lastPublish = std::chrono::steady_clock::now();

            while (isRunning)
            {
                fd_set cFds;
                FD_ZERO(&cFds);
                FD_SET(clientSock, &cFds);

                timeval cTv = { 0, 10000 }; // 10 ms
                int cAct = select(0, &cFds, nullptr, nullptr, &cTv);

                if (cAct > 0 && FD_ISSET(clientSock, &cFds))
                {
                    char tempBuf[4096];
                    int bytes = recv(clientSock, tempBuf, sizeof(tempBuf), 0);

                    if (bytes > 0)
                    {
                        readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + bytes);
                        std::lock_guard<std::mutex> sLock(statsMutex);
                        stats.bytesReceived += bytes;
                    }
                    else if (bytes == 0)
                    {
                        break; // Client disconnected
                    }
                    else if (WSAGetLastError() != WSAEWOULDBLOCK)
                    {
                        break; // Socket error
                    }
                }

                // 1. Process HTTP WebSocket Handshake
                if (!handshaked)
                {
                    std::string req(readBuffer.begin(), readBuffer.end());
                    size_t headerEnd = req.find("\r\n\r\n");
                    if (headerEnd != std::string::npos)
                    {
                        std::string keyHeader = "Sec-WebSocket-Key: ";
                        size_t keyPos = req.find(keyHeader);
                        if (keyPos == std::string::npos)
                        {
                            keyHeader = "sec-websocket-key: ";
                            keyPos = req.find(keyHeader);
                        }

                        if (keyPos != std::string::npos)
                        {
                            size_t keyStart = keyPos + keyHeader.size();
                            size_t keyEnd = req.find("\r\n", keyStart);
                            std::string clientKey = req.substr(keyStart, keyEnd - keyStart);
                            while (!clientKey.empty() && (clientKey.back() == ' ' || clientKey.back() == '\r'))
                            {
                                clientKey.pop_back();
                            }

                            std::string acceptKey = ComputeWebSocketAcceptKey(clientKey);
                            std::string resp =
                                "HTTP/1.1 101 Switching Protocols\r\n"
                                "Upgrade: websocket\r\n"
                                "Connection: Upgrade\r\n"
                                "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n";

                            send(clientSock, resp.data(), static_cast<int>(resp.size()), 0);
                            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + headerEnd + 4);
                            handshaked = true;
                            isConnected = true;
                            SIM_LOG_INFO("WPILib robot code client connected successfully.");
                        }
                    }
                }
                else
                {
                    // 2. Process incoming WebSocket frames
                    std::string payload;
                    uint8_t opcode = 0;
                    while (DecodeFrame(readBuffer, payload, opcode))
                    {
                        if (opcode == 0x01) // Text JSON frame
                        {
                            DispatchIncomingJson(payload);
                            std::lock_guard<std::mutex> sLock(statsMutex);
                            stats.packetsReceived++;
                        }
                        else if (opcode == 0x08) // Close
                        {
                            break;
                        }
                        else if (opcode == 0x09) // Ping -> reply Pong (0x0A)
                        {
                            std::vector<uint8_t> pong = EncodeTextFrame(payload);
                            pong[0] = 0x8A;
                            send(clientSock, reinterpret_cast<const char*>(pong.data()),
                                static_cast<int>(pong.size()), 0);
                        }
                    }

                    // 3. Send 50 Hz DriverStation heartbeat
                    auto now = std::chrono::steady_clock::now();
                    double dsElapsed = std::chrono::duration<double>(now - lastDsHeartbeat).count();
                    if (dsElapsed >= (1.0 / config.heartbeatRateHz))
                    {
                        lastDsHeartbeat = now;
                        nlohmann::json dsMsg = {
                            { "type", "DriverStation" },
                            { "device", "" },
                            { "data", {
                                { ">enabled", dsEnabled.load() },
                                { ">autonomous", dsAutonomous.load() },
                                { ">test", false },
                                { ">estop", false },
                                { ">ds_attached", true }
                            } }
                        };
                        std::string jsonStr = dsMsg.dump();
                        std::vector<uint8_t> frame = EncodeTextFrame(jsonStr);
                        send(clientSock, reinterpret_cast<const char*>(frame.data()),
                            static_cast<int>(frame.size()), 0);

                        std::lock_guard<std::mutex> sLock(statsMutex);
                        stats.packetsSent++;
                        stats.bytesSent += frame.size();
                    }
                }
            }

            closesocket(clientSock);
            isConnected = false;
            SIM_LOG_INFO("WPILib robot code client disconnected.");
        }
    }
}