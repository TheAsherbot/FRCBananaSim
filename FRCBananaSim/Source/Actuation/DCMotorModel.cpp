/**
 * @file DCMotorModel.cpp
 * @brief Implementation of semi-implicit DC motor and gearbox dynamics model.
 */

#include "Actuation/DCMotorModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Actuation
{
    using namespace Core::Units::Literals;

    DCMotorModel::DCMotorModel()
        : DCMotorModel(NEO1())
    {}

    DCMotorModel::DCMotorModel(double nominalVoltage, double stallTorque, double stallCurrent,
        double freeSpeedRadPerSec, double freeCurrent, double rotorInertia,
        double gearRatio, double gearboxEfficiency)
        : nominalVoltage(nominalVoltage),
        stallTorque(stallTorque),
        stallCurrent(stallCurrent),
        freeSpeedRadPerSec(freeSpeedRadPerSec),
        freeCurrent(freeCurrent),
        rotorInertia(rotorInertia),
        gearRatio(gearRatio),
        gearboxEfficiency(gearboxEfficiency),
        resistance(0.0),
        torqueConstantKt(0.0),
        backEmfConstantKe(0.0)
    {
        resistance = (stallCurrent > 1e-6) ? (nominalVoltage / stallCurrent) : 0.1;
        torqueConstantKt = (stallCurrent > 1e-6) ? (stallTorque / stallCurrent) : 0.01;

        double effectiveBackEmfVoltage = nominalVoltage - (freeCurrent * resistance);
        if (effectiveBackEmfVoltage <= 0.0)
        {
            effectiveBackEmfVoltage = nominalVoltage;
        }

        backEmfConstantKe = (freeSpeedRadPerSec > 1e-6) ? (effectiveBackEmfVoltage / freeSpeedRadPerSec) : 0.01;
    }

    DCMotorModel DCMotorModel::KrakenX60(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 7.09_Nm, 366.0_A, 6000.0_rpm, 2.0_A, 5.6e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::KrakenX44(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 4.11_Nm, 279.0_A, 7758.0_rpm, 3.0_A, 2.1e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::Minion(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 3.17_Nm, 211.15_A, 7700.0_rpm, 1.43_A, 3.5e-6, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::Falcon500(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 4.69_Nm, 257.0_A, 6380.0_rpm, 1.5_A, 4.2e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::NEO1(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 2.60_Nm, 105.0_A, 5676.0_rpm, 1.8_A, 1.8e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::NEO2(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 2.60_Nm, 105_A, 5676_rpm, 1.8_A, 1.6e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::NEO550(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 0.97_Nm, 100.0_A, 11000_rpm, 1.4_A, 2.9e-6, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::NEOVortex(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 3.60_Nm, 211.0_A, 6784.0_rpm, 3.6_A, 2.2e-5, gearRatio, efficiency);
    }

    DCMotorModel DCMotorModel::GoBilda5202(double gearRatio, double efficiency)
    {
        return DCMotorModel(12.0_V, 0.25_Nm, 9.2_A, 6000.0_rpm, 0.25_A, 1.0e-6, gearRatio, efficiency);
    }

    double DCMotorModel::CalculateTorque(double appliedVoltage, double currentAngularVelocity,
        double externalInertia, double deltaTime) const
    {
        if (deltaTime <= 1e-7)
        {
            return 0.0;
        }

        // Commanded stall torque at output shaft scaled by gear ratio and efficiency
        double commandedStallTorque = gearRatio * gearboxEfficiency * stallTorque * (appliedVoltage / nominalVoltage);

        // Reflected electrical damping coefficient: D = (Kt * Kv * G^2 * eta) / R
        double electricalDamping = (torqueConstantKt * backEmfConstantKe * gearRatio * gearRatio * gearboxEfficiency) / resistance;

        // Total effective rotational inertia at output shaft
        double effectiveInertia = GetReflectedInertia(externalInertia);

        // Semi-implicit integration formulation: unconditionally stable for any gear ratio and timestep
        double denominator = 1.0 + ((electricalDamping * deltaTime) / effectiveInertia);
        double outputTorque = (commandedStallTorque - (electricalDamping * currentAngularVelocity)) / denominator;

        return outputTorque;
    }

    double DCMotorModel::CalculateCurrent(double appliedVoltage, double currentAngularVelocity) const
    {
        double motorAngularVelocity = currentAngularVelocity * gearRatio;
        double backEmf = backEmfConstantKe * motorAngularVelocity;
        double armatureCurrent = (appliedVoltage - backEmf) / resistance;

        return armatureCurrent;
    }

    double DCMotorModel::GetReflectedInertia(double externalInertia) const
    {
        return (rotorInertia * gearRatio * gearRatio) + std::max(externalInertia, 1e-6);
    }

    double DCMotorModel::GetNominalVoltage() const { return nominalVoltage; }
    double DCMotorModel::GetStallTorque() const { return stallTorque; }
    double DCMotorModel::GetStallCurrent() const { return stallCurrent; }
    double DCMotorModel::GetFreeSpeed() const { return freeSpeedRadPerSec; }
    double DCMotorModel::GetFreeCurrent() const { return freeCurrent; }
    double DCMotorModel::GetRotorInertia() const { return rotorInertia; }
    double DCMotorModel::GetGearRatio() const { return gearRatio; }
    double DCMotorModel::GetGearboxEfficiency() const { return gearboxEfficiency; }
    double DCMotorModel::GetResistance() const { return resistance; }
    double DCMotorModel::GetTorqueConstantKt() const { return torqueConstantKt; }
    double DCMotorModel::GetBackEmfConstantKe() const { return backEmfConstantKe; }
}