/**
 * @file SimControlBoundary.h
 * @brief Decoupled control boundary structs exchanged with WPILib HALSim or headless test harnesses.
 */

#pragma once

#include <array>
#include <cstdint>

namespace FRCBananaSim::Control
{
    /**
     * @enum MotorControlMode
     * @brief Control mode for an individual actuator demand.
     */
    enum class MotorControlMode : uint8_t
    {
        Voltage = 0,       ///< Commanded in Volts (-12.0V to +12.0V).
        DutyCycle = 1,     ///< Fractional duty cycle (-1.0 to +1.0).
        TorqueCurrent = 2, ///< Commanded stator current in Amperes (CTRE TorqueCurrentFOC).
        Disabled = 3       ///< Motor output disabled.
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
     * @struct MotorCommand
     * @brief Per-motor control demand and current limit settings.
     */
    struct MotorCommand
    {
        MotorControlMode mode = MotorControlMode::Voltage;
        double demand = 0.0;             ///< Volts, fraction (-1..1), or Amps depending on mode.
        double statorCurrentLimit = 60.0; ///< Stator current clamp threshold (A).
        NeutralMode neutralMode = NeutralMode::Brake; ///< Neutral mode setting.
    };

    /**
     * @struct SimInputs
     * @brief Control demands sent to the simulation core.
     * Injected by either HALSim WebSocket Server (live robot code)
     * or Headless Test Harness / Parameter Calibrator (direct sweep).
     */
    struct SimInputs
    {
        // 8 motors: [0..3] = Steer (FL, FR, BL, BR), [4..7] = Drive (FL, FR, BL, BR)
        std::array<MotorCommand, 8> motorCommands;
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
        // 8 encoders: [0..3] = Steer (rad, continuous unwrapped), [4..7] = Drive (rad, continuous)
        std::array<double, 8> encoderPositions = { 0.0 };
        std::array<double, 8> encoderVelocities = { 0.0 };
        std::array<double, 8> motorCurrents = { 0.0 }; ///< Instantaneous stator current (A).

        // CTRE Pigeon 2 IMU (WPILib NWU Standard: CCW positive)
        // Standard coordinates: +X Right, +Y Up, +Z Forward
        double gyroYaw = 0.0;     ///< Heading in radians (CCW positive: 0 facing +Z, +pi/2 facing -X left).
        double gyroPitch = 0.0;   ///< Pitch in radians (nose up positive).
        double gyroRoll = 0.0;    ///< Roll in radians (right bank positive).
        double gyroYawRate = 0.0; ///< Rotational velocity around vertical axis (rad/s, CCW positive).

        // Electrical telemetry
        double batteryTerminalVoltage = 12.60; ///< Terminal voltage post-sag (V).
        double totalBusCurrent = 1.5;          ///< Instantaneous bus current draw (A).
        uint32_t digitalInputs = 0;            ///< Beam breaks, limit switches bitfield.
        bool isBrownedOut = false;             ///< Brownout stage active.
    };
}