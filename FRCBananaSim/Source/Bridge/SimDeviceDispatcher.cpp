/**
 * @file SimDeviceDispatcher.cpp
 * @brief Implementation of SimDevice frame mapping, JSON serialization, and follower propagation.
 */

#include "Bridge/SimDeviceDispatcher.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Bridge
{
    SimDeviceDispatcher::SimDeviceDispatcher()
        : motorsByCanId(),
        cancodersByCanId(),
        imuBindings(),
        pidControllers()
    {}

    SimDeviceDispatcher::~SimDeviceDispatcher()
    {}

    void SimDeviceDispatcher::ConfigureFromSidecar(const FRCBananaSim::Robot::Schema::RobotSidecar& sidecar)
    {
        motorsByCanId.clear();
        cancodersByCanId.clear();
        imuBindings.clear();

        // 1. Swerve Modules Mapping
        for (size_t i = 0; i < sidecar.swerve.modules.size() && i < 4; ++i)
        {
            const auto& mod = sidecar.swerve.modules[i];

            // Steer Motor -> Motor Slot i
            RegisterMotor(mod.steerCanId, static_cast<uint32_t>(i), "talonfx", mod.canBus, false);

            // Drive Motor -> Motor Slot 4 + i
            RegisterMotor(mod.driveCanId, static_cast<uint32_t>(4 + i), "talonfx", mod.canBus, false);

            // CANcoder -> CANcoder Slot i
            RegisterCANcoder(mod.cancoderId, static_cast<uint32_t>(i), mod.canBus);
        }

        // 2. Mechanisms Controllers Mapping
        uint32_t nextMechanismSlot = 8;
        for (const auto& mech : sidecar.mechanisms)
        {
            uint32_t mechanismPrimarySlot = nextMechanismSlot;
            uint32_t leaderCanId = 0;

            for (const auto& ctrl : mech.controllers)
            {
                if (ctrl.role == "leader" || leaderCanId == 0)
                {
                    leaderCanId = ctrl.canId;
                    RegisterMotor(ctrl.canId, mechanismPrimarySlot, ctrl.vendor, ctrl.canBus, ctrl.inverted);
                }
                else
                {
                    RegisterFollower(ctrl.canId, leaderCanId, ctrl.inverted);
                }
            }

            if (nextMechanismSlot < Control::MAX_MOTOR_SLOTS)
            {
                ++nextMechanismSlot;
            }
        }

        // 3. Sensors Mapping
        for (const auto& sensor : sidecar.sensors)
        {
            if (sensor.type == "cancoder")
            {
                uint32_t slot = static_cast<uint32_t>(cancodersByCanId.size());
                if (slot < Control::MAX_CANCODER_SLOTS)
                {
                    RegisterCANcoder(sensor.canId, slot, sensor.bus);
                }
            }
            else if (sensor.type == "imu_pigeon2")
            {
                RegisterIMU(sensor.canId, "pigeon2", sensor.bus);
            }
            else if (sensor.type == "imu_navx")
            {
                RegisterIMU(0, "navx", sensor.bus);
            }
        }
    }

    bool SimDeviceDispatcher::LoadKeymap(const std::string& keymapPath)
    {
        std::ifstream file(keymapPath);
        if (!file.is_open())
        {
            return false;
        }

        try
        {
            nlohmann::json root = nlohmann::json::parse(file);
            return root.contains("devices");
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    void SimDeviceDispatcher::RegisterMotor(uint32_t canId, uint32_t motorSlot,
        const std::string& type, const std::string& bus, bool inverted)
    {
        MotorBinding binding;
        binding.canId = canId;
        binding.motorSlot = motorSlot;
        binding.canBus = bus;
        binding.inverted = inverted;
        binding.isFollower = false;
        binding.leaderCanId = 0;

        std::string lowerType = type;
        std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(), ::tolower);
        if (lowerType == "sparkmax" || lowerType == "spark_max")
        {
            binding.type = DeviceType::SparkMax;
        }
        else
        {
            binding.type = DeviceType::TalonFX;
        }

        motorsByCanId[canId] = binding;
    }

    void SimDeviceDispatcher::RegisterFollower(uint32_t followerCanId, uint32_t leaderCanId, bool inverted)
    {
        auto it = motorsByCanId.find(leaderCanId);
        uint32_t leaderSlot = (it != motorsByCanId.end()) ? it->second.motorSlot : 0;
        DeviceType leaderType = (it != motorsByCanId.end()) ? it->second.type : DeviceType::TalonFX;

        MotorBinding binding;
        binding.canId = followerCanId;
        binding.motorSlot = leaderSlot;
        binding.type = leaderType;
        binding.canBus = (it != motorsByCanId.end()) ? it->second.canBus : "rio";
        binding.inverted = inverted;
        binding.isFollower = true;
        binding.leaderCanId = leaderCanId;

        motorsByCanId[followerCanId] = binding;
    }

    void SimDeviceDispatcher::RegisterCANcoder(uint32_t canId, uint32_t cancoderSlot, const std::string& bus)
    {
        CANcoderBinding binding;
        binding.canId = canId;
        binding.cancoderSlot = cancoderSlot;
        binding.canBus = bus;
        cancodersByCanId[canId] = binding;
    }

    void SimDeviceDispatcher::RegisterIMU(uint32_t canId, const std::string& type, const std::string& bus)
    {
        IMUBinding binding;
        binding.canId = canId;
        binding.canBus = bus;

        std::string lowerType = type;
        std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(), ::tolower);
        if (lowerType.find("navx") != std::string::npos)
        {
            binding.type = DeviceType::NavX;
        }
        else
        {
            binding.type = DeviceType::Pigeon2;
        }

        imuBindings.push_back(binding);
    }

    bool SimDeviceDispatcher::ParseDeviceString(std::string_view deviceName, DeviceType& outType,
        uint32_t& outId, std::string& outBus)
    {
        outType = DeviceType::Unknown;
        outId = 0;
        outBus = "rio";

        size_t openBracket = deviceName.find('[');
        size_t closeBracket = deviceName.find(']', openBracket);

        if (openBracket != std::string_view::npos && closeBracket != std::string_view::npos)
        {
            std::string_view idStr = deviceName.substr(openBracket + 1, closeBracket - openBracket - 1);
            try
            {
                outId = static_cast<uint32_t>(std::stoul(std::string(idStr)));
            }
            catch (...)
            {
                outId = 0;
            }

            std::string_view prefix = deviceName.substr(0, openBracket);

            // Check for bus separator ':' e.g. "TalonFX:canivore"
            size_t colon = prefix.find(':');
            if (colon != std::string_view::npos)
            {
                outBus = std::string(prefix.substr(colon + 1));
                prefix = prefix.substr(0, colon);
            }

            std::string lowerPrefix(prefix);
            std::transform(lowerPrefix.begin(), lowerPrefix.end(), lowerPrefix.begin(), ::tolower);

            if (lowerPrefix.find("talon") != std::string::npos)
            {
                outType = DeviceType::TalonFX;
                return true;
            }
            if (lowerPrefix.find("cancoder") != std::string::npos)
            {
                outType = DeviceType::CANcoder;
                return true;
            }
            if (lowerPrefix.find("pigeon") != std::string::npos)
            {
                outType = DeviceType::Pigeon2;
                return true;
            }
            if (lowerPrefix.find("spark") != std::string::npos)
            {
                outType = DeviceType::SparkMax;
                return true;
            }
            if (lowerPrefix.find("navx") != std::string::npos)
            {
                outType = DeviceType::NavX;
                return true;
            }
        }
        else if (deviceName.find("navX") != std::string_view::npos)
        {
            outType = DeviceType::NavX;
            outId = 0;
            return true;
        }

        return false;
    }

    bool SimDeviceDispatcher::DispatchIncomingFrame(std::string_view jsonStr, Control::SimInputs& inputs)
    {
        try
        {
            auto root = nlohmann::json::parse(jsonStr);
            if (!root.contains("type") || root["type"] != "SimDevice" || !root.contains("device") || !root.contains("data"))
            {
                return false;
            }

            std::string deviceName = root["device"].get<std::string>();
            const auto& data = root["data"];

            DeviceType type = DeviceType::Unknown;
            uint32_t id = 0;
            std::string bus = "rio";
            if (!ParseDeviceString(deviceName, type, id, bus))
            {
                return false;
            }

            auto it = motorsByCanId.find(id);
            if (it == motorsByCanId.end())
            {
                return false;
            }

            const auto& binding = it->second;
            uint32_t slot = binding.motorSlot;
            if (slot >= Control::MAX_MOTOR_SLOTS)
            {
                return false;
            }

            // CTRE TalonFX input extraction
            if (type == DeviceType::TalonFX)
            {
                double voltage = 0.0;
                bool hasVoltage = false;

                if (data.contains(">motorVoltage"))
                {
                    voltage = data[">motorVoltage"].get<double>();
                    hasVoltage = true;
                }
                else if (data.contains(">motorOutputLead"))
                {
                    voltage = data[">motorOutputLead"].get<double>() * 12.0;
                    hasVoltage = true;
                }
                else if (data.contains(">BridgeOutput"))
                {
                    voltage = data[">BridgeOutput"].get<double>() * 12.0;
                    hasVoltage = true;
                }

                if (hasVoltage)
                {
                    double sign = binding.inverted ? -1.0 : 1.0;
                    if (binding.isFollower)
                    {
                        // Add follower contribution to primary mechanism slot
                        inputs.motorCommands[slot].demand += (sign * voltage);
                    }
                    else
                    {
                        inputs.motorCommands[slot].mode = Control::MotorControlMode::Voltage;
                        inputs.motorCommands[slot].demand = (sign * voltage);
                    }
                    return true;
                }
            }
            // REV SPARK MAX / SPARK Flex input extraction
            else if (type == DeviceType::SparkMax)
            {
                if (data.contains(">Applied Output"))
                {
                    double dutyCycle = data[">Applied Output"].get<double>();
                    double sign = binding.inverted ? -1.0 : 1.0;
                    double voltage = sign * dutyCycle * 12.0;

                    if (binding.isFollower)
                    {
                        inputs.motorCommands[slot].demand += voltage;
                    }
                    else
                    {
                        inputs.motorCommands[slot].mode = Control::MotorControlMode::Voltage;
                        inputs.motorCommands[slot].demand = voltage;
                    }
                    return true;
                }
            }

            return false;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    std::string SimDeviceDispatcher::SerializeDeviceState(const std::string& deviceName,
        const Control::SimOutputs& outputs) const
    {
        DeviceType type = DeviceType::Unknown;
        uint32_t id = 0;
        std::string bus = "rio";
        if (!ParseDeviceString(deviceName, type, id, bus))
        {
            return "";
        }

        nlohmann::json root;
        root["type"] = "SimDevice";
        root["device"] = deviceName;
        nlohmann::json data;

        if (type == DeviceType::TalonFX)
        {
            auto it = motorsByCanId.find(id);
            if (it == motorsByCanId.end())
            {
                return "";
            }

            const auto& binding = it->second;
            uint32_t slot = binding.motorSlot;
            double sign = binding.inverted ? -1.0 : 1.0;

            data["<supplyVoltage"] = outputs.batteryTerminalVoltage;
            data["<rawRotorPosition"] = sign * outputs.rotorPositionsRot[slot];
            data["<rotorVelocity"] = sign * outputs.rotorVelocitiesRps[slot];
            data["<rotorAcceleration"] = 0.0;
            data["<torqueCurrent"] = outputs.motorCurrents[slot];
            data["<supplyCurrent"] = outputs.motorSupplyCurrents[slot];
            data["<forwardLimit"] = outputs.forwardLimitTriggered[slot];
            data["<reverseLimit"] = outputs.reverseLimitTriggered[slot];
        }
        else if (type == DeviceType::CANcoder)
        {
            auto it = cancodersByCanId.find(id);
            if (it == cancodersByCanId.end())
            {
                return "";
            }

            uint32_t slot = it->second.cancoderSlot;
            data["<supplyVoltage"] = outputs.batteryTerminalVoltage;
            data["<rawPosition"] = outputs.cancoderPositionsRot[slot];
            data["<velocity"] = outputs.cancoderVelocitiesRps[slot];
        }
        else if (type == DeviceType::Pigeon2)
        {
            constexpr double radToDeg = 180.0 / std::numbers::pi;
            data["<supplyVoltage"] = outputs.batteryTerminalVoltage;
            data["<rawYaw"] = outputs.gyroYaw * radToDeg;
            data["<pitch"] = outputs.gyroPitch * radToDeg;
            data["<roll"] = outputs.gyroRoll * radToDeg;
            data["<angularVelocityZ"] = outputs.gyroAngularVelocityRadPerSec.z * radToDeg;
            data["<angularVelocityY"] = outputs.gyroAngularVelocityRadPerSec.y * radToDeg;
            data["<angularVelocityX"] = outputs.gyroAngularVelocityRadPerSec.x * radToDeg;
            data["<accelX"] = outputs.accelSpecificForceMetersPerSec2.x / 9.80665;
            data["<accelY"] = outputs.accelSpecificForceMetersPerSec2.y / 9.80665;
            data["<accelZ"] = outputs.accelSpecificForceMetersPerSec2.z / 9.80665;
        }
        else if (type == DeviceType::SparkMax)
        {
            auto it = motorsByCanId.find(id);
            if (it == motorsByCanId.end())
            {
                return "";
            }

            const auto& binding = it->second;
            uint32_t slot = binding.motorSlot;
            double sign = binding.inverted ? -1.0 : 1.0;

            data["<Bus Voltage"] = outputs.batteryTerminalVoltage;
            data["<Motor Current"] = outputs.motorCurrents[slot];
            data["<Position"] = sign * outputs.rotorPositionsRot[slot];
            data["<Velocity"] = sign * outputs.rotorVelocitiesRps[slot] * 60.0; // RPS -> RPM
        }
        else if (type == DeviceType::NavX)
        {
            constexpr double radToDeg = 180.0 / std::numbers::pi;
            data["<yaw"] = -outputs.gyroYaw * radToDeg; // navX CW+ = -yaw_NWU
            data["<pitch"] = outputs.gyroPitch * radToDeg;
            data["<roll"] = outputs.gyroRoll * radToDeg;
            data["<LinearWorldAccelX"] = outputs.accelSpecificForceMetersPerSec2.x;
            data["<LinearWorldAccelY"] = outputs.accelSpecificForceMetersPerSec2.y;
            data["<LinearWorldAccelZ"] = outputs.accelSpecificForceMetersPerSec2.z - 9.80665;
        }

        root["data"] = data;
        return root.dump();
    }

    std::vector<std::string> SimDeviceDispatcher::DispatchOutgoingFrames(const Control::SimOutputs& outputs)
    {
        std::vector<std::string> frames;
        frames.reserve(motorsByCanId.size() + cancodersByCanId.size() + imuBindings.size());

        // 1. Motor Controllers
        for (const auto& pair : motorsByCanId)
        {
            std::string deviceName;
            if (pair.second.type == DeviceType::SparkMax)
            {
                deviceName = "SPARK MAX [" + std::to_string(pair.first) + "]";
            }
            else
            {
                deviceName = "TalonFX[" + std::to_string(pair.first) + "]";
            }

            std::string jsonStr = SerializeDeviceState(deviceName, outputs);
            if (!jsonStr.empty())
            {
                frames.push_back(std::move(jsonStr));
            }
        }

        // 2. Standalone CANcoders
        for (const auto& pair : cancodersByCanId)
        {
            std::string deviceName = "CANcoder[" + std::to_string(pair.first) + "]";
            std::string jsonStr = SerializeDeviceState(deviceName, outputs);
            if (!jsonStr.empty())
            {
                frames.push_back(std::move(jsonStr));
            }
        }

        // 3. IMUs
        for (const auto& binding : imuBindings)
        {
            std::string deviceName;
            if (binding.type == DeviceType::NavX)
            {
                deviceName = "navX-Sensor[0]";
            }
            else
            {
                deviceName = "Pigeon2[" + std::to_string(binding.canId) + "]";
            }

            std::string jsonStr = SerializeDeviceState(deviceName, outputs);
            if (!jsonStr.empty())
            {
                frames.push_back(std::move(jsonStr));
            }
        }

        return frames;
    }

    void SimDeviceDispatcher::StepClosedLoopControllers(Control::SimInputs& inputs,
        const Control::SimOutputs& outputs, double dt)
    {
        for (size_t slot = 0; slot < Control::MAX_MOTOR_SLOTS; ++slot)
        {
            auto& cmd = inputs.motorCommands[slot];
            if (cmd.mode == Control::MotorControlMode::PositionVoltage)
            {
                pidControllers[slot].SetGains(cmd.gains);
                double voltage = pidControllers[slot].Calculate(cmd.demand, outputs.rotorPositionsRot[slot], dt, false);
                cmd.demand = voltage;
                cmd.mode = Control::MotorControlMode::Voltage;
            }
            else if (cmd.mode == Control::MotorControlMode::VelocityVoltage)
            {
                pidControllers[slot].SetGains(cmd.gains);
                double voltage = pidControllers[slot].Calculate(cmd.demand, outputs.rotorVelocitiesRps[slot], dt, true);
                cmd.demand = voltage;
                cmd.mode = Control::MotorControlMode::Voltage;
            }
        }
    }

    Control::OnControllerPIDModel* SimDeviceDispatcher::GetPIDModel(size_t slot)
    {
        if (slot < Control::MAX_MOTOR_SLOTS)
        {
            return &pidControllers[slot];
        }
        return nullptr;
    }
}