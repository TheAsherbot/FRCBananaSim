/**
 * @file DCMotorModel.h
 * @brief Semi-implicit DC motor and gearbox dynamics model with reflected rotor inertia.
 */

#pragma once

#include "Core/Units/Units.h"
#include <string>

namespace FRCBananaSim::Actuation
{
    /**
     * @struct AffineMotorTorque
     * @brief Affine torque decomposition at the output shaft: tau = tau0 - damping * omega.
     */
    struct AffineMotorTorque
    {
        double stallTorqueCommanded = 0.0; ///< Commanded torque at zero output speed (N*m).
        double electricalDamping = 0.0;    ///< Back-EMF electrical damping coefficient D (N*m / (rad/s)).
    };

    /**
     * @class DCMotorModel
     * @brief Physics and electrical model for permanent-magnet DC motors coupled to gearboxes.
     *
     * Utilizes semi-implicit integration of back-EMF to guarantee unconditional stability
     * across high gear ratios (e.g., G = 26.09 or G = 100) at fixed physics timesteps.
     */
    class DCMotorModel
    {
    public:
        /**
         * @brief Constructs a DC motor model with explicit electromechanical parameters.
         * @param nominalVoltage Rated motor voltage (V).
         * @param stallTorque Stall torque at rated voltage (N*m).
         * @param stallCurrent Current draw at stall (A).
         * @param freeSpeedRadPerSec Free rotational speed at rated voltage in radians per second (rad/s).
         * @param freeCurrent Current draw at free speed (A).
         * @param rotorInertia Internal rotor rotational inertia (kg*m^2).
         * @param gearRatio Gearbox reduction ratio (output speed = rotor speed / gearRatio).
         * @param gearboxEfficiency Mechanical transmission efficiency (0.0 to 1.0).
         */
        DCMotorModel(double nominalVoltage, double stallTorque, double stallCurrent,
            double freeSpeedRadPerSec, double freeCurrent, double rotorInertia,
            double gearRatio = 1.0, double gearboxEfficiency = 1.0);

        /**
         * @brief Default constructor initializing to a standard NEO motor with 1:1 gearing.
         */
        DCMotorModel();

        // WPILib-aligned factory presets
        static DCMotorModel KrakenX60(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel KrakenX60FOC(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel KrakenX44(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel KrakenX44FOC(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel Minion(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel Falcon500(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel NEO1(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel NEO2(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel NEO550(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel NEOVortex(double gearRatio = 1.0, double efficiency = 1.0);
        static DCMotorModel GoBilda5202(double gearRatio = 1.0, double efficiency = 1.0);

        /**
         * @brief Calculates unconditionally stable semi-implicit torque at the gearbox output shaft.
         * @param appliedVoltage Voltage across the motor windings (V).
         * @param currentAngularVelocity Output shaft angular velocity at start of step (rad/s).
         * @param externalInertia Connected load inertia at output shaft (kg*m^2).
         * @param deltaTime Integration timestep duration (s).
         * @return Torque delivered to the joint at step t + dt (N*m).
         */
        double CalculateTorque(double appliedVoltage, double currentAngularVelocity,
            double externalInertia, double deltaTime) const;

        /**
         * @brief Decomposes motor output into affine linear terms: tau = tau0 - D * omega.
         * @param appliedVoltage Voltage across the motor windings (V).
         * @return AffineMotorTorque holding stall torque command tau0 and electrical damping D.
         */
        AffineMotorTorque CalculateAffineTorqueTerms(double appliedVoltage) const;

        /**
         * @brief Computes maximum output torque allowed by a stator current limit.
         * @param statorCurrentLimit Stator current threshold in Amperes (A).
         * @return Maximum permissible output torque magnitude (N*m).
         */
        double CalculateStatorTorqueLimit(double statorCurrentLimit) const;

        /**
         * @brief Calculates instantaneous motor winding (stator) current.
         * @param appliedVoltage Voltage across motor windings (V).
         * @param currentAngularVelocity Output shaft angular velocity (rad/s).
         * @return Stator current magnitude (A).
         */
        double CalculateCurrent(double appliedVoltage, double currentAngularVelocity) const;

        /**
         * @brief Computes effective reflected inertia at the gearbox output shaft.
         * J_eff = J_rotor * G^2 + J_load.
         * @param externalInertia Connected load inertia at output shaft (kg*m^2).
         * @return Total effective inertia (kg*m^2).
         */
        double GetReflectedInertia(double externalInertia) const;

        // Accessors
        double GetNominalVoltage() const;
        double GetStallTorque() const;
        double GetStallCurrent() const;
        double GetFreeSpeed() const;
        double GetFreeCurrent() const;
        double GetRotorInertia() const;
        double GetGearRatio() const;
        double GetGearboxEfficiency() const;
        double GetResistance() const;
        double GetTorqueConstantKt() const;
        double GetBackEmfConstantKe() const;

    private:
        double nominalVoltage;
        double stallTorque;
        double stallCurrent;
        double freeSpeedRadPerSec;
        double freeCurrent;
        double rotorInertia;
        double gearRatio;
        double gearboxEfficiency;

        double resistance;
        double torqueConstantKt;
        double backEmfConstantKe;
    };
}