/**
 * @file ServoModel.h
 * @brief Datasheet-driven electromechanical servo model supporting positional and continuous modes.
 */

#pragma once

#include "Actuation/DCMotorModel.h"
#include "Core/Units/Units.h"
#include <string>

namespace FRCBananaSim::Actuation
{
    using namespace Core::Units::Literals;

    /**
     * @enum ServoMode
     * @brief Operating mode of the servo.
     */
    enum class ServoMode
    {
        Positional, ///< Tracks a commanded target angle or PWM fraction (0.0 to 1.0).
        Continuous  ///< Operates as a continuous variable-speed gearmotor (-1.0 to 1.0).
    };

    /**
     * @class ServoModel
     * @brief Models hobby/robotics servos using typical retailer datasheet parameters.
     *
     * Automatically constructs the underlying DC motor and gearbox model from high-level
     * specifications such as seconds-per-60-degrees, stall torque in kg*cm, and stall current.
     * Supports running at operating voltages different from the datasheet rated voltage (e.g. 4.8V or 5.0V).
     */
    class ServoModel
    {
    public:
        /**
         * @brief Default constructor initializing to a goBILDA Dual-Mode Torque servo.
         */
        ServoModel();

        /**
         * @brief Constructs a servo model from basic manufacturer datasheet specifications.
         * @param nominalVoltage Rated operating voltage (V) on datasheet, e.g. 6.0_V or 7.4_V.
         * @param stallTorque Stall torque in Newton-meters (N*m).
         * @param secPer60Deg No-load rotational speed in seconds per 60 degrees.
         * @param stallCurrent Peak current draw at stall (A).
         * @param freeCurrent Idle running current (A). Defaults to 0.15A if omitted.
         * @param totalTravelRange Total rotational travel (radians). Defaults to 300 degrees.
         * @param mode Operating mode (Positional or Continuous).
         */
        ServoModel(double nominalVoltage, double stallTorque, double secPer60Deg,
            double stallCurrent, double freeCurrent = 0.15_A,
            double totalTravelRange = 300.0_deg, ServoMode mode = ServoMode::Positional);

        // Retailer / Manufacturer Presets
        static ServoModel GoBildaTorque(ServoMode mode = ServoMode::Positional);
        static ServoModel GoBildaSpeed(ServoMode mode = ServoMode::Positional);
        static ServoModel GoBildaSuperSpeed(ServoMode mode = ServoMode::Positional);
        static ServoModel SwyftSpeed(ServoMode mode = ServoMode::Positional);
        static ServoModel SwyftTorque(ServoMode mode = ServoMode::Positional);
        static ServoModel SwyftBalanced(ServoMode mode = ServoMode::Positional);
        static ServoModel SwyftSlim(ServoMode mode = ServoMode::Positional);
        static ServoModel SwyftNano(ServoMode mode = ServoMode::Positional);

        /**
         * @brief Sets the actual supply voltage powering the servo rail (e.g. 4.8_V, 5.0_V, or 6.0_V).
         * When lower than rated voltage, stall torque and speed automatically scale down physically.
         * @param voltage Current rail voltage (V).
         */
        void SetSupplyVoltage(double voltage);

        /**
         * @brief Sets target rotational angle for positional mode.
         * @param targetAngleRadians Desired angle clamped between [minAngle, maxAngle].
         */
        void SetTargetAngle(double targetAngleRadians);

        /**
         * @brief Sets target position via normalized PWM fraction (0.0 to 1.0).
         * Maps 0.0 to minAngle, 0.5 to center, and 1.0 to maxAngle.
         * @param fraction0To1 Normalized input from WPILib Servo::Set().
         */
        void SetTargetNormalized(double fraction0To1);

        /**
         * @brief Sets commanded speed for continuous rotation mode (-1.0 to +1.0).
         * @param normalizedSpeed Duty cycle velocity command.
         */
        void SetSpeed(double normalizedSpeed);

        /**
         * @brief Sets the operating mode of the servo.
         * @param newMode Positional or Continuous.
         */
        void SetMode(ServoMode newMode);

        /**
         * @brief Calculates output torque delivered to the physical joint.
         * @param currentAngle Current physical angle of the joint (rad).
         * @param currentAngularVelocity Current angular velocity of the joint (rad/s).
         * @param externalInertia Connected load inertia (kg*m^2).
         * @param deltaTime Integration timestep duration (s).
         * @return Joint torque (N*m).
         */
        double CalculateTorque(double currentAngle, double currentAngularVelocity,
            double externalInertia, double deltaTime);

        /**
         * @brief Calculates instantaneous electrical current drawn by the servo.
         * @param currentAngularVelocity Current angular velocity of the joint (rad/s).
         * @return Electrical current in Amperes (A).
         */
        double CalculateCurrent(double currentAngularVelocity) const;

        // Accessors
        ServoMode GetMode() const;
        double GetTargetAngle() const;
        double GetMinAngle() const;
        double GetMaxAngle() const;
        double GetTotalTravelRange() const;
        double GetStallTorque() const;
        double GetFreeSpeed() const;
        double GetStallCurrent() const;
        double GetNominalVoltage() const;
        double GetSupplyVoltage() const;
        double GetLastAppliedVoltage() const;
        const DCMotorModel& GetInternalMotor() const;

    private:
        DCMotorModel motorModel;
        ServoMode mode;

        double supplyVoltage;
        double minAngle;
        double maxAngle;
        double targetAngle;
        double commandedSpeed;
        double positionGainKp;
        double lastAppliedVoltage;
    };
}