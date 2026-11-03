/**
 * @file OnControllerPIDModel.h
 * @brief Deterministic 1000 Hz sim-side PID and feedforward motor controller model.
 */

#pragma once

#include "Control/SimControlBoundary.h"

namespace FRCBananaSim::Control
{
    /**
     * @class OnControllerPIDModel
     * @brief Models on-hardware motor controller closed-loop control (PositionVoltage, VelocityVoltage).
     */
    class OnControllerPIDModel
    {
    public:
        /**
         * @brief Constructs an on-controller PID model.
         * @param gains PID and feedforward gains.
         */
        explicit OnControllerPIDModel(const ClosedLoopGains& gains = ClosedLoopGains{});

        /**
         * @brief Calculates commanded voltage output across timestep dt.
         * @param setpoint Target position (rotations) or velocity (RPS).
         * @param measurement Measured position (rotations) or velocity (RPS).
         * @param dt Timestep duration in seconds (typically 0.001 s or sub-step).
         * @param isVelocityMode True if controlling velocity; false if position.
         * @param feedforwardAccel Optional feedforward acceleration (RPS^2).
         * @return Commanded motor voltage in Volts (-12.0V to +12.0V).
         */
        double Calculate(double setpoint, double measurement, double dt,
            bool isVelocityMode = false, double feedforwardAccel = 0.0);

        /**
         * @brief Resets accumulated integral error and previous derivative history.
         * @param initialMeasurement Optional measurement offset.
         */
        void Reset(double initialMeasurement = 0.0);

        /**
         * @brief Sets PID and feedforward gains.
         * @param gains New gains.
         */
        void SetGains(const ClosedLoopGains& gains);

        /**
         * @brief Gets current PID and feedforward gains.
         * @return Const reference to ClosedLoopGains.
         */
        const ClosedLoopGains& GetGains() const;

        /**
         * @brief Enables continuous input wrapping for rotational joints (e.g. swerve steer).
         * @param enabled True to enable wrapping.
         * @param minVal Minimum boundary (e.g. -0.5 rot).
         * @param maxVal Maximum boundary (e.g. +0.5 rot).
         */
        void SetContinuousWrap(bool enabled, double minVal = -0.5, double maxVal = 0.5);

        /**
         * @brief Checks if continuous input wrapping is enabled.
         * @return True if wrapping.
         */
        bool IsContinuousWrapEnabled() const;

        /**
         * @brief Sets voltage output limits.
         * @param minVolts Minimum voltage limit (typically -12.0 V).
         * @param maxVolts Maximum voltage limit (typically +12.0 V).
         */
        void SetOutputLimits(double minVolts, double maxVolts);

        /**
         * @brief Sets integral deadband zone (iZone). Error outside this band zeroes integral accumulator.
         * @param zone Threshold error magnitude (0.0 to disable iZone gating).
         */
        void SetIZone(double zone);

    private:
        ClosedLoopGains gains;
        double minOutputVolts = -12.0;
        double maxOutputVolts = 12.0;
        double iZone = 0.0;

        bool continuousWrap = false;
        double wrapMin = -0.5;
        double wrapMax = 0.5;

        double integralAccumulator = 0.0;
        double previousError = 0.0;
        bool hasPreviousError = false;
    };
}