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
         * @param brownoutRecoveryVoltage Threshold for exiting brownout state (V).
         */
        BatteryModel(double nominalVoltage = 12.60_V, double internalResistance = 0.023_ohm,
            double quiescentCurrent = 1.5_A, double statorCurrentLimit = 60.0_A,
            double supplyCurrentLimit = 40.0_A, double brownoutWarningVoltage = 6.8_V,
            double brownoutCutoffVoltage = 6.3_V, double brownoutRecoveryVoltage = 7.5_V);

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
         * @brief Closed-form bus voltage solver eliminating algebraic loops:
         * Solves V_bus = 0.5 * (B + sqrt(B^2 - 4 * R_bat * P_total)) where B = V_oc - R_bat * I_q.
         * @param aggregateMotorPowerWatts Total active electrical power commanded by actuators (W).
         * @param isCurrentlyBrownedOut Current persistent brownout latch state.
         * @param outNewBrownoutState Output updated brownout state with Schmitt-trigger hysteresis.
         * @return Solved terminal bus voltage (V).
         */
        double SolveBusVoltageClosedForm(double aggregateMotorPowerWatts, bool isCurrentlyBrownedOut,
            bool& outNewBrownoutState) const;

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
         * @brief Evaluates brownout critical cutoff with Schmidt-trigger hysteresis.
         * @param terminalVoltage Current terminal voltage (V).
         * @return True if system is in brownout critical state.
         */
        bool IsBrownoutCritical(double terminalVoltage) const;

        /**
         * @brief Resets the internal brownout state machine.
         */
        void ResetBrownoutState();

        /**
         * @brief Queries whether brownout critical state is currently active.
         * @return True if browned out.
         */
        bool IsBrownoutActive() const;

        // Accessors
        double GetNominalVoltage() const;
        double GetInternalResistance() const;
        double GetQuiescentCurrent() const;
        double GetStatorCurrentLimit() const;
        double GetSupplyCurrentLimit() const;
        double GetBrownoutWarningVoltage() const;
        double GetBrownoutCutoffVoltage() const;
        double GetBrownoutRecoveryVoltage() const;

    private:
        double nominalVoltage;
        double internalResistance;
        double quiescentCurrent;
        double statorCurrentLimit;
        double supplyCurrentLimit;
        double brownoutWarningVoltage;
        double brownoutCutoffVoltage;
        double brownoutRecoveryVoltage;
        mutable bool isBrownedOutState = false;
    };
}