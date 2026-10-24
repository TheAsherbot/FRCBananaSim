/**
 * @file ServoModel.cpp
 * @brief Implementation of datasheet-driven electromechanical servo model.
 */

#include "Actuation/ServoModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Actuation
{
    using namespace Core::Units::Literals;

    ServoModel::ServoModel()
        : ServoModel(GoBildaTorque())
    {}

    ServoModel::ServoModel(double nominalVoltage, double stallTorque, double secPer60Deg,
        double stallCurrent, double freeCurrent, double totalTravelRange, ServoMode mode)
        : motorModel(nominalVoltage, stallTorque, stallCurrent,
            (secPer60Deg > 1e-6) ? ((3.14159265358979323846 / 3.0) / secPer60Deg) : 0.0, freeCurrent,
            1.0e-5, 1.0, 1.0),
        mode(mode),
        supplyVoltage(nominalVoltage),
        minAngle(-totalTravelRange * 0.5),
        maxAngle(totalTravelRange * 0.5),
        targetAngle(0.0_rad),
        profiledAngle(0.0_rad),
        commandedSpeed(0.0),
        positionGainKp(0.0),
        velocityGainKd(0.0),
        lastAppliedVoltage(0.0_V)
    {
        // Internal proportional gain: reaches full motor voltage at ~8 degrees of tracking error
        constexpr double errorForFullVoltage = 8.0_deg;
        positionGainKp = nominalVoltage / errorForFullVoltage;

        // Active derivative damping opposes velocity near target to eliminate ringing
        double freeSpeed = motorModel.GetFreeSpeed();
        velocityGainKd = (freeSpeed > 1e-6) ? (nominalVoltage / freeSpeed) : 0.1;
    }

    ServoModel ServoModel::GoBildaTorque(ServoMode mode)
    {
        // goBILDA 2000 Series Dual Mode Servo (Torque)
        // 6.0V, 21.6 kg*cm, 0.20 s/60 deg, 2.5A stall, 300 deg range
        return ServoModel(6.0_V, 21.6_kgcm, 0.20, 2.5_A, 160_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::GoBildaSpeed(ServoMode mode)
    {
        // goBILDA 2000 Series Dual Mode Servo (Speed)
        return ServoModel(6.0_V, 9.3_kgcm, 0.09, 2.5_A, 200_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::GoBildaSuperSpeed(ServoMode mode)
    {
        // goBILDA 2000 Series Dual Mode Servo (Super Speed)
        return ServoModel(6.0_V, 4.7_kgcm, 0.08, 2.5_A, 200_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::SwyftSpeed(ServoMode mode)
    {
        // Swyft Robotics Speed Servo
        return ServoModel(6.0_V, 19.0_kgcm, 0.062, 2.7_A, 150_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::SwyftTorque(ServoMode mode)
    {
        // Swyft Robotics Torque Servo
        return ServoModel(6.0_V, 33.5_kgcm, 0.112, 2.7_A, 150_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::SwyftBalanced(ServoMode mode)
    {
        // Swyft Robotics Balanced Servo
        return ServoModel(6.0_V, 27.3_kgcm, 0.092, 2.7_A, 150_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::SwyftSlim(ServoMode mode)
    {
        // Swyft Robotics Slim Servo
        return ServoModel(6.0_V, 14.0_kgcm, 0.075, 2.4_A, 150_mA, 300.0_deg, mode);
    }

    ServoModel ServoModel::SwyftNano(ServoMode mode)
    {
        // Swyft Robotics Nano Servo
        return ServoModel(6.0_V, 10.2_kgcm, 0.110, 2.0_A, 150_mA, 300.0_deg, mode);
    }

    void ServoModel::SetSupplyVoltage(double voltage)
    {
        supplyVoltage = std::max(0.0_V, voltage);
    }

    void ServoModel::SetTargetAngle(double targetAngleRadians)
    {
        targetAngle = std::clamp(targetAngleRadians, minAngle, maxAngle);
    }

    void ServoModel::SetTargetNormalized(double fraction0To1)
    {
        double clamped = std::clamp(fraction0To1, 0.0, 1.0);
        targetAngle = minAngle + clamped * (maxAngle - minAngle);
    }

    void ServoModel::SetSpeed(double normalizedSpeed)
    {
        commandedSpeed = std::clamp(normalizedSpeed, -1.0, 1.0);
    }

    void ServoModel::SetMode(ServoMode newMode)
    {
        mode = newMode;
    }

    double ServoModel::CalculateTorque(double currentAngle, double currentAngularVelocity,
        double externalInertia, double deltaTime)
    {
        if (mode == ServoMode::Continuous)
        {
            lastAppliedVoltage = commandedSpeed * supplyVoltage;
        }
        else
        {
            // 1. Slew-rate limiter: advance the internal setpoint at the catalog speed
            // If running at 4.8V on a 6.0V servo, max speed scales proportionally
            double ratedFreeSpeed = motorModel.GetFreeSpeed() * (supplyVoltage / motorModel.GetNominalVoltage());
            double maxStepThisTick = ratedFreeSpeed * deltaTime;

            double trajectoryError = targetAngle - profiledAngle;
            profiledAngle += std::clamp(trajectoryError, -maxStepThisTick, maxStepThisTick);

            // 2. Deadband: if within 0.3 degrees of target, cut motor voltage to prevent hunting/buzzing
            double trackingError = profiledAngle - currentAngle;
            constexpr double deadband = 0.3_deg;

            if (std::abs(trackingError) < deadband)
            {
                lastAppliedVoltage = 0.0_V;
            }
            else
            {
                // Active PD drive tracking the moving profile
                double rawVoltage = (trackingError * positionGainKp) - (currentAngularVelocity * velocityGainKd);
                lastAppliedVoltage = std::clamp(rawVoltage, -supplyVoltage, supplyVoltage);
            }

            // Inhibit drive if pushed against travel limits
            if (currentAngle <= minAngle && lastAppliedVoltage < 0.0)
            {
                lastAppliedVoltage = 0.0_V;
            }
            else if (currentAngle >= maxAngle && lastAppliedVoltage > 0.0)
            {
                lastAppliedVoltage = 0.0_V;
            }
        }

        // Available stall torque scaled by actual supply voltage rail
        double availableTorque = motorModel.GetStallTorque() * (supplyVoltage / motorModel.GetNominalVoltage());
        double rawTorque = motorModel.CalculateTorque(lastAppliedVoltage, currentAngularVelocity, externalInertia, deltaTime);

        return std::clamp(rawTorque, -availableTorque, availableTorque);
    }

    double ServoModel::CalculateCurrent(double currentAngularVelocity) const
    {
        double rawCurrent = std::abs(motorModel.CalculateCurrent(lastAppliedVoltage, currentAngularVelocity));
        // Physical servos are powered via internal H-bridges limited to stall current
        return std::min(rawCurrent, motorModel.GetStallCurrent());
    }

    ServoMode ServoModel::GetMode() const { return mode; }
    double ServoModel::GetTargetAngle() const { return targetAngle; }
    double ServoModel::GetMinAngle() const { return minAngle; }
    double ServoModel::GetMaxAngle() const { return maxAngle; }
    double ServoModel::GetTotalTravelRange() const { return maxAngle - minAngle; }
    double ServoModel::GetStallTorque() const { return motorModel.GetStallTorque(); }
    double ServoModel::GetFreeSpeed() const { return motorModel.GetFreeSpeed(); }
    double ServoModel::GetStallCurrent() const { return motorModel.GetStallCurrent(); }
    double ServoModel::GetNominalVoltage() const { return motorModel.GetNominalVoltage(); }
    double ServoModel::GetSupplyVoltage() const { return supplyVoltage; }
    double ServoModel::GetLastAppliedVoltage() const { return lastAppliedVoltage; }
    const DCMotorModel& ServoModel::GetInternalMotor() const { return motorModel; }
}