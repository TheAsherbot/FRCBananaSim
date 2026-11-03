/**
 * @file OnControllerPIDModel.cpp
 * @brief Implementation of deterministic 1000 Hz sim-side PID and feedforward motor controller.
 */

#include "Control/OnControllerPIDModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Control
{
    OnControllerPIDModel::OnControllerPIDModel(const ClosedLoopGains& gains)
        : gains(gains),
        minOutputVolts(-12.0),
        maxOutputVolts(12.0),
        iZone(0.0),
        continuousWrap(false),
        wrapMin(-0.5),
        wrapMax(0.5),
        integralAccumulator(0.0),
        previousError(0.0),
        hasPreviousError(false)
    {}

    double OnControllerPIDModel::Calculate(double setpoint, double measurement, double dt,
        bool isVelocityMode, double feedforwardAccel)
    {
        if (dt <= 1e-9)
        {
            return 0.0;
        }

        // 1. Error calculation with optional continuous wrapping
        double error = setpoint - measurement;
        if (continuousWrap && !isVelocityMode)
        {
            double span = wrapMax - wrapMin;
            if (span > 1e-9)
            {
                error = std::fmod(error, span);
                if (error > (span * 0.5))
                {
                    error -= span;
                }
                else if (error < (-span * 0.5))
                {
                    error += span;
                }
            }
        }

        // 2. Proportional term
        double pTerm = gains.kP * error;

        // 3. Integral term with anti-windup and iZone
        if (iZone > 1e-9 && std::abs(error) > iZone)
        {
            integralAccumulator = 0.0;
        }
        else
        {
            integralAccumulator += error * dt;
        }
        double iTerm = gains.kI * integralAccumulator;
        iTerm = std::clamp(iTerm, minOutputVolts, maxOutputVolts);

        // 4. Derivative term
        double dTerm = 0.0;
        if (hasPreviousError)
        {
            double dError = (error - previousError) / dt;
            dTerm = gains.kD * dError;
        }
        previousError = error;
        hasPreviousError = true;

        // 5. Feedforward term
        double ffTerm = 0.0;
        if (isVelocityMode)
        {
            double sign = (setpoint > 1e-6) ? 1.0 : ((setpoint < -1e-6) ? -1.0 : 0.0);
            ffTerm = (gains.kS * sign) + (gains.kV * setpoint) + (gains.kA * feedforwardAccel) + gains.kG;
        }
        else
        {
            double sign = (error > 1e-6) ? 1.0 : ((error < -1e-6) ? -1.0 : 0.0);
            ffTerm = (gains.kS * sign) + gains.kG;
        }

        // 6. Output summation and clamping
        double totalOutputVolts = pTerm + iTerm + dTerm + ffTerm;
        return std::clamp(totalOutputVolts, minOutputVolts, maxOutputVolts);
    }

    void OnControllerPIDModel::Reset(double initialMeasurement)
    {
        integralAccumulator = 0.0;
        previousError = 0.0;
        hasPreviousError = false;
    }

    void OnControllerPIDModel::SetGains(const ClosedLoopGains& newGains)
    {
        gains = newGains;
    }

    const ClosedLoopGains& OnControllerPIDModel::GetGains() const
    {
        return gains;
    }

    void OnControllerPIDModel::SetContinuousWrap(bool enabled, double minVal, double maxVal)
    {
        continuousWrap = enabled;
        wrapMin = minVal;
        wrapMax = maxVal;
    }

    bool OnControllerPIDModel::IsContinuousWrapEnabled() const
    {
        return continuousWrap;
    }

    void OnControllerPIDModel::SetOutputLimits(double minVolts, double maxVolts)
    {
        minOutputVolts = minVolts;
        maxOutputVolts = maxVolts;
    }

    void OnControllerPIDModel::SetIZone(double zone)
    {
        iZone = zone;
    }
}