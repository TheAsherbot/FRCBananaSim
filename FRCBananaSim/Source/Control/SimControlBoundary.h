/**
 * @file SimControlBoundary.h
 * @brief Decoupled control boundary structs exchanged with WPILib HALSim or test harnesses.
 */

#pragma once

#include <array>
#include <cstdint>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Control
{
    constexpr size_t MAX_MOTOR_SLOTS = 24;   ///< [0..3] Steer, [4..7] Drive, [8..23] Mechanisms
    constexpr size_t MAX_CANCODER_SLOTS = 12;
    constexpr size_t MAX_ANALOG_INPUTS = 8;  ///< Standard RoboRIO Analog In channels [0..7] (0.0V to 5.0V)

    /**
     * @enum MotorControlMode
     * @brief Control mode for an individual actuator demand.
     */
    enum class MotorControlMode : uint8_t
    {
        Voltage = 0,         ///< Commanded in Volts (-12.0V to +12.0V).
        DutyCycle = 1,       ///< Fractional duty cycle (-1.0 to +1.0).
        TorqueCurrent = 2,   ///< Commanded stator current in Amperes.
        PositionVoltage = 3, ///< Sim-side 1 kHz closed-loop position PID + FF.
        VelocityVoltage = 4, ///< Sim-side 1 kHz closed-loop velocity PID + FF.
        Disabled = 5         ///< Motor output disabled.
    };

    /**
     * @enum NeutralMode
     * @brief Neutral behavior when commanded voltage is zero or motor is disabled.
     */
    enum class NeutralMode : uint8_t
    {
        Brake = 0, ///< Dynamic back-EMF electrical damping applied to joint.
        Coast = 1  ///< Free unactuated rotation (zero torque).
    };

    /**
     * @struct ClosedLoopGains
     * @brief PID and feedforward gains for sim-side closed-loop motor control.
     */
    struct ClosedLoopGains
    {
        double kP = 0.0; ///< Proportional gain.
        double kI = 0.0; ///< Integral gain.
        double kD = 0.0; ///< Derivative gain.
        double kS = 0.0; ///< Static friction feedforward.
        double kV = 0.0; ///< Velocity feedforward.
        double kA = 0.0; ///< Acceleration feedforward.
        double kG = 0.0; ///< Gravity feedforward.
    };

    /**
     * @struct MotorCommand
     * @brief Per-motor control demand and current limit settings.
     */
    struct MotorCommand
    {
        MotorControlMode mode = MotorControlMode::Voltage; ///< Commanded control mode.
        double demand = 0.0;                               ///< Volts, fraction (-1..1), Amps, or target setpoint.
        double statorCurrentLimit = 60.0;                  ///< Stator current clamp threshold (A).
        NeutralMode neutralMode = NeutralMode::Brake;      ///< Neutral mode setting.
        ClosedLoopGains gains = {};                        ///< Optional closed-loop PID gains for sim-side control.
    };

    /**
     * @struct SimInputs
     * @brief Control demands sent to the simulation core.
     * Injected by either HALSim WebSocket Server (live robot code)
     * or Headless Test Harness / Parameter Calibrator (direct sweep).
     */
    struct SimInputs
    {
        // 24 motors: [0..3] = Steer (FL, FR, BL, BR), [4..7] = Drive (FL, FR, BL, BR), [8..23] = Mechanisms
        std::array<MotorCommand, MAX_MOTOR_SLOTS> motorCommands;
        uint32_t digitalOutputs = 0; ///< Solenoids / pneumatic bitfield.
        bool robotEnabled = false;   ///< DriverStation enable state.
        bool isAutonomous = false;  ///< Autonomous vs Teleop mode.
        double matchTime = 0.0;      ///< Elapsed match time (s).
    };

    /**
     * @struct SimOutputs
     * @brief Sensory feedback read back from the physical simulation core.
     * Sent to WPILib HALSim client or recorded by TelemetryLogger.
     */
    struct SimOutputs
    {
        // Rotor & Joint Feedback per Motor Slot [0..23]
        std::array<double, MAX_MOTOR_SLOTS> encoderPositions = { 0.0 };      ///< Joint rad or meters.
        std::array<double, MAX_MOTOR_SLOTS> encoderVelocities = { 0.0 };     ///< Joint rad/s or m/s.
        std::array<double, MAX_MOTOR_SLOTS> rotorPositionsRot = { 0.0 };     ///< Ungeared rotor rotations.
        std::array<double, MAX_MOTOR_SLOTS> rotorVelocitiesRps = { 0.0 };    ///< Ungeared rotor RPS.
        std::array<double, MAX_MOTOR_SLOTS> motorCurrents = { 0.0 };         ///< Stator current (A).
        std::array<double, MAX_MOTOR_SLOTS> motorSupplyCurrents = { 0.0 };   ///< Supply current (A).
        std::array<bool, MAX_MOTOR_SLOTS> forwardLimitTriggered = { false }; ///< Forward hardware limit switch state.
        std::array<bool, MAX_MOTOR_SLOTS> reverseLimitTriggered = { false }; ///< Reverse hardware limit switch state.

        // Standalone Absolute/Relative Encoders (CANcoders)
        std::array<double, MAX_CANCODER_SLOTS> cancoderPositionsRot = { 0.0 };  ///< CANcoder raw position (rotations).
        std::array<double, MAX_CANCODER_SLOTS> cancoderVelocitiesRps = { 0.0 }; ///< CANcoder velocity (RPS).

        // CTRE Pigeon 2 / navX IMU (WPILib NWU Standard)
        double gyroYaw = 0.0;                                                ///< Continuous CCW-positive yaw (rad).
        double gyroPitch = 0.0;                                              ///< Pitch (rad).
        double gyroRoll = 0.0;                                               ///< Roll (rad).
        double gyroYawRate = 0.0;                                            ///< Yaw angular rate around vertical (rad/s).
        glm::dvec3 gyroAngularVelocityRadPerSec = { 0.0, 0.0, 0.0 };          ///< 3-axis angular velocity in NWU (rad/s).
        glm::dvec3 accelSpecificForceMetersPerSec2 = { 0.0, 0.0, 9.80665 };   ///< 3-axis specific force in NWU (m/s^2).
        glm::dquat imuOrientationQuat = { 1.0, 0.0, 0.0, 0.0 };               ///< Orientation quaternion in NWU.

        // Electrical & Digital/Analog I/O
        double batteryTerminalVoltage = 12.60;                         ///< Terminal voltage post-sag (V).
        double totalBusCurrent = 1.5;                                  ///< Instantaneous bus current draw (A).
        uint32_t digitalInputs = 0;                                    ///< Bit i = DIO channel i (beam breaks, limit switches).
        std::array<double, MAX_ANALOG_INPUTS> analogInputs = { 0.0 };  ///< Analog input voltages (0.0V to 5.0V).
        bool isBrownedOut = false;                                     ///< Brownout stage active.
    };
}