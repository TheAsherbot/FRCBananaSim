/**
 * @file BatteryModel.cpp
 * @brief Implementation of lumped electrical battery model and brownout hysteresis.
 */

#include "Actuation/BatteryModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Actuation
{
    using namespace Core::Units::Literals;

    BatteryModel::BatteryModel(double nominalVoltage, double internalResistance,
        double quiescentCurrent, double statorCurrentLimit,
        double supplyCurrentLimit, double brownoutWarningVoltage,
        double brownoutCutoffVoltage, double brownoutRecoveryVoltage)
        : nominalVoltage(nominalVoltage),
        internalResistance(internalResistance),
        quiescentCurrent(quiescentCurrent),
        statorCurrentLimit(statorCurrentLimit),
        supplyCurrentLimit(supplyCurrentLimit),
        brownoutWarningVoltage(brownoutWarningVoltage),
        brownoutCutoffVoltage(brownoutCutoffVoltage),
        brownoutRecoveryVoltage(brownoutRecoveryVoltage),
        isBrownedOutState(false)
    {}

    BatteryModel BatteryModel::FRCStandard()
    {
        return BatteryModel(12.60_V, 0.023_ohm, 1.5_A, 60.0_A, 40.0_A, 6.8_V, 6.3_V, 7.5_V);
    }

    BatteryModel BatteryModel::FTCStandard()
    {
        return BatteryModel(12.00_V, 0.120_ohm, 0.35_A, 20.0_A, 15.0_A, 8.5_V, 7.0_V, 9.0_V);
    }

    double BatteryModel::CalculateTerminalVoltage(double totalMotorSupplyCurrent) const
    {
        double totalCurrent = std::max(0.0, totalMotorSupplyCurrent) + quiescentCurrent;
        double saggedVoltage = nominalVoltage - (totalCurrent * internalResistance);
        return std::max(0.0, saggedVoltage);
    }

    double BatteryModel::SolveBusVoltageClosedForm(double aggregateMotorPowerWatts, bool isCurrentlyBrownedOut,
        bool& outNewBrownoutState) const
    {
        double openCircuitEffective = nominalVoltage - (quiescentCurrent * internalResistance);
        double discriminant = (openCircuitEffective * openCircuitEffective) - (4.0 * internalResistance * aggregateMotorPowerWatts);

        double terminalVoltage = 0.0;
        if (discriminant <= 0.0)
        {
            // Commanded power exceeds theoretical maximum deliverable by battery: voltage collapses
            terminalVoltage = 0.5 * openCircuitEffective;
            outNewBrownoutState = true;
            return terminalVoltage;
        }

        terminalVoltage = 0.5 * (openCircuitEffective + std::sqrt(discriminant));

        // Evaluate Schmitt-trigger brownout state
        if (isCurrentlyBrownedOut)
        {
            if (terminalVoltage >= brownoutRecoveryVoltage)
            {
                outNewBrownoutState = false;
            }
            else
            {
                outNewBrownoutState = true;
            }
        }
        else
        {
            if (terminalVoltage <= brownoutCutoffVoltage)
            {
                outNewBrownoutState = true;
            }
            else
            {
                outNewBrownoutState = false;
            }
        }

        return terminalVoltage;
    }

    double BatteryModel::ApplySupplyCurrentLimit(double commandedVoltage, double statorCurrent,
        double terminalVoltage) const
    {
        double absCommanded = std::abs(commandedVoltage);
        if (absCommanded < 1e-4 || terminalVoltage < 1e-3)
        {
            return 0.0;
        }

        double motorElectricalPower = std::abs(statorCurrent) * absCommanded;
        double maxAllowedPower = supplyCurrentLimit * terminalVoltage;

        if (motorElectricalPower > maxAllowedPower && motorElectricalPower > 1e-6)
        {
            double scaleFactor = maxAllowedPower / motorElectricalPower;
            return commandedVoltage * scaleFactor;
        }

        return commandedVoltage;
    }

    bool BatteryModel::IsBrownoutWarning(double terminalVoltage) const
    {
        return terminalVoltage <= brownoutWarningVoltage;
    }

    bool BatteryModel::IsBrownoutCritical(double terminalVoltage) const
    {
        if (isBrownedOutState)
        {
            if (terminalVoltage >= brownoutRecoveryVoltage)
            {
                isBrownedOutState = false;
            }
        }
        else
        {
            if (terminalVoltage <= brownoutCutoffVoltage)
            {
                isBrownedOutState = true;
            }
        }

        return isBrownedOutState;
    }

    void BatteryModel::ResetBrownoutState()
    {
        isBrownedOutState = false;
    }

    bool BatteryModel::IsBrownoutActive() const
    {
        return isBrownedOutState;
    }

    double BatteryModel::GetNominalVoltage() const { return nominalVoltage; }
    double BatteryModel::GetInternalResistance() const { return internalResistance; }
    double BatteryModel::GetQuiescentCurrent() const { return quiescentCurrent; }
    double BatteryModel::GetStatorCurrentLimit() const { return statorCurrentLimit; }
    double BatteryModel::GetSupplyCurrentLimit() const { return supplyCurrentLimit; }
    double BatteryModel::GetBrownoutWarningVoltage() const { return brownoutWarningVoltage; }
    double BatteryModel::GetBrownoutCutoffVoltage() const { return brownoutCutoffVoltage; }
    double BatteryModel::GetBrownoutRecoveryVoltage() const { return brownoutRecoveryVoltage; }
}