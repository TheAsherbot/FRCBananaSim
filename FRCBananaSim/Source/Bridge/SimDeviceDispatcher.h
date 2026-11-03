/**
 * @file SimDeviceDispatcher.h
 * @brief Dispatcher mapping incoming and outgoing WPILib SimDevice WebSocket frames to SimControlBoundary slots.
 */

#pragma once

#include "Control/SimControlBoundary.h"
#include "Control/OnControllerPIDModel.h"
#include "Robot/Schema/RobotSidecar.h"

#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <array>
#include <memory>

namespace FRCBananaSim::Bridge
{
    /**
     * @enum DeviceType
     * @brief Recognized vendor SimDevice hardware types.
     */
    enum class DeviceType : uint8_t
    {
        Unknown = 0,
        TalonFX = 1,
        CANcoder = 2,
        Pigeon2 = 3,
        SparkMax = 4,
        NavX = 5
    };

    /**
     * @struct MotorBinding
     * @brief Hardware mapping for an individual motor controller.
     */
    struct MotorBinding
    {
        uint32_t canId = 0;
        uint32_t motorSlot = 0;
        DeviceType type = DeviceType::TalonFX;
        std::string canBus = "rio";
        bool inverted = false;
        bool isFollower = false;
        uint32_t leaderCanId = 0;
    };

    /**
     * @struct CANcoderBinding
     * @brief Hardware mapping for a standalone CANcoder.
     */
    struct CANcoderBinding
    {
        uint32_t canId = 0;
        uint32_t cancoderSlot = 0;
        std::string canBus = "rio";
    };

    /**
     * @struct IMUBinding
     * @brief Hardware mapping for an IMU sensor (Pigeon2 or navX).
     */
    struct IMUBinding
    {
        uint32_t canId = 0;
        DeviceType type = DeviceType::Pigeon2;
        std::string canBus = "rio";
    };

    /**
     * @class SimDeviceDispatcher
     * @brief Serializes SimOutputs and deserializes SimInputs for CTRE, REV, and Kauai Labs SimDevice frames.
     */
    class SimDeviceDispatcher
    {
    public:
        /**
         * @brief Constructs the SimDevice dispatcher.
         */
        SimDeviceDispatcher();

        /**
         * @brief Destructor.
         */
        ~SimDeviceDispatcher();

        /**
         * @brief Configures motor, encoder, and IMU slot mappings from a parsed sidecar.json.
         * @param sidecar Parsed electromechanical sidecar specification.
         */
        void ConfigureFromSidecar(const FRCBananaSim::Robot::Schema::RobotSidecar& sidecar);

        /**
         * @brief Loads custom keymap aliases from a JSON file.
         * @param keymapPath Path to simdevice_keymap.json.
         * @return True if loaded successfully; false if fallback defaults used.
         */
        bool LoadKeymap(const std::string& keymapPath);

        /**
         * @brief Registers a motor controller binding manually.
         * @param canId CAN ID of the motor controller.
         * @param motorSlot SimControlBoundary motor slot index [0..23].
         * @param type Controller type ("talonfx", "sparkmax", "sparkflex").
         * @param bus CAN bus name ("rio", "canivore").
         * @param inverted Inversion flag.
         */
        void RegisterMotor(uint32_t canId, uint32_t motorSlot,
            const std::string& type = "talonfx",
            const std::string& bus = "rio",
            bool inverted = false);

        /**
         * @brief Registers a follower motor controller referencing a leader CAN ID.
         * @param followerCanId CAN ID of the follower.
         * @param leaderCanId CAN ID of the leader motor.
         * @param inverted Follower inversion flag relative to leader.
         */
        void RegisterFollower(uint32_t followerCanId, uint32_t leaderCanId, bool inverted = false);

        /**
         * @brief Registers a standalone CANcoder binding manually.
         * @param canId CAN ID of the CANcoder.
         * @param cancoderSlot SimControlBoundary CANcoder slot index [0..11].
         * @param bus CAN bus name ("rio", "canivore").
         */
        void RegisterCANcoder(uint32_t canId, uint32_t cancoderSlot, const std::string& bus = "rio");

        /**
         * @brief Registers an IMU sensor binding manually.
         * @param canId CAN ID of the IMU (or 0 for navX).
         * @param type Sensor type ("pigeon2" or "navx").
         * @param bus CAN bus name ("rio", "canivore").
         */
        void RegisterIMU(uint32_t canId, const std::string& type = "pigeon2", const std::string& bus = "rio");

        /**
         * @brief Parses an incoming WebSocket JSON message frame and routes commands to SimInputs.
         * @param jsonStr UTF-8 JSON text received from robot code.
         * @param inputs Mutable reference to SimInputs receiving parsed commands.
         * @return True if message was a recognized SimDevice frame and processed.
         */
        bool DispatchIncomingFrame(std::string_view jsonStr, Control::SimInputs& inputs);

        /**
         * @brief Serializes all dirty or active registered devices into outgoing WebSocket JSON frames.
         * @param outputs Sensory feedback from physical simulation.
         * @return Array of JSON strings ready for WebSocket transmission.
         */
        std::vector<std::string> DispatchOutgoingFrames(const Control::SimOutputs& outputs);

        /**
         * @brief Serializes state for an individual device by exact SimDevice name.
         * @param deviceName Formatted device string (e.g. "TalonFX[10]").
         * @param outputs Sensory feedback.
         * @return JSON string, or empty string if device is not registered.
         */
        std::string SerializeDeviceState(const std::string& deviceName, const Control::SimOutputs& outputs) const;

        /**
         * @brief Executes sim-side closed-loop control sub-step for motors in PositionVoltage or VelocityVoltage.
         * @param inputs Current commanded setpoints and gains.
         * @param outputs Current measured rotor feedback.
         * @param dt Sub-step duration in seconds (typically 0.00125 s).
         */
        void StepClosedLoopControllers(Control::SimInputs& inputs, const Control::SimOutputs& outputs, double dt);

        /**
         * @brief Direct access to an on-controller PID instance for a motor slot.
         * @param slot Motor slot [0..23].
         * @return Pointer to OnControllerPIDModel, or nullptr if slot out of bounds.
         */
        Control::OnControllerPIDModel* GetPIDModel(size_t slot);

    private:
        static bool ParseDeviceString(std::string_view deviceName, DeviceType& outType,
            uint32_t& outId, std::string& outBus);

        std::unordered_map<uint32_t, MotorBinding> motorsByCanId;
        std::unordered_map<uint32_t, CANcoderBinding> cancodersByCanId;
        std::vector<IMUBinding> imuBindings;

        std::array<Control::OnControllerPIDModel, Control::MAX_MOTOR_SLOTS> pidControllers;
    };
}