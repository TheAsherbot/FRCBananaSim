/**
 * @file BatteryModel.h
 * @brief Lumped electrical model simulating internal resistance, voltage sag, and brownout protection.
 */

#pragma once

#include "Core/Units/Units.h"

namespace FRCBananaSim::Actuation
{
    using namespace Core::Units::Literals;

    /**
     * @class BatteryModel
     * @brief Computes bus voltage sag and manages supply current throttling and brownout thresholds.
     */
    class BatteryModel
    {
    public:
        /**
         * @brief Constructs a battery model with configurable internal parameters and unit literals.
         * @param nominalVoltage Fully charged open circuit voltage (V).
         * @param internalResistance Total series internal resistance including main breaker & wiring (ohms).
         * @param quiescentCurrent Idle baseline current draw from roboRIO, radio, and coprocessors (A).
         * @param statorCurrentLimit Peak motor stator current limit (A).
         * @param supplyCurrentLimit Continuous battery bus supply current limit per branch (A).
         * @param brownoutWarningVoltage Threshold triggering brownout telemetry warning (V).
         * @param brownoutCutoffVoltage Threshold triggering motor output shutoff (V).
         */
        BatteryModel(double nominalVoltage = 12.60_V, double internalResistance = 0.023_ohm,
            double quiescentCurrent = 1.5_A, double statorCurrentLimit = 60.0_A,
            double supplyCurrentLimit = 40.0_A, double brownoutWarningVoltage = 6.8_V,
            double brownoutCutoffVoltage = 6.3_V);

        // Standard presets
        static BatteryModel FRCStandard();
        static BatteryModel FTCStandard();

        /**
         * @brief Computes sagged terminal voltage from aggregate motor supply current.
         * @param totalMotorSupplyCurrent Sum of supply currents drawn by all active actuators (A).
         * @return Sagged terminal voltage V_terminal (V).
         */
        double CalculateTerminalVoltage(double totalMotorSupplyCurrent) const;

        /**
         * @brief Throttles commanded motor voltage to obey supply current limits without zero division.
         * @param commandedVoltage Voltage commanded by robot code (V).
         * @param statorCurrent Instantaneous winding current through motor (A).
         * @param terminalVoltage Current sagged battery bus voltage (V).
         * @return Safe, power-limited voltage output (V).
         */
        double ApplySupplyCurrentLimit(double commandedVoltage, double statorCurrent, double terminalVoltage) const;

        /**
         * @brief Checks if terminal voltage has dropped into the roboRIO brownout warning region.
         * @param terminalVoltage Current terminal voltage (V).
         * @return True if terminal voltage is below warning threshold.
         */
        bool IsBrownoutWarning(double terminalVoltage) const;

        /**
         * @brief Checks if terminal voltage has dropped below the hard motor cut-off threshold.
         * @param terminalVoltage Current terminal voltage (V).
         * @return True if terminal voltage is critically low.
         */
        bool IsBrownoutCritical(double terminalVoltage) const;

        // Accessors
        double GetNominalVoltage() const;
        double GetInternalResistance() const;
        double GetQuiescentCurrent() const;
        double GetStatorCurrentLimit() const;
        double GetSupplyCurrentLimit() const;
        double GetBrownoutWarningVoltage() const;
        double GetBrownoutCutoffVoltage() const;

    private:
        double nominalVoltage;
        double internalResistance;
        double quiescentCurrent;
        double statorCurrentLimit;
        double supplyCurrentLimit;
        double brownoutWarningVoltage;
        double brownoutCutoffVoltage;
    };
}