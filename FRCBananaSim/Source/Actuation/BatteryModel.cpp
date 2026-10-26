/**
 * @file BatteryModel.cpp
 * @brief Implementation of lumped battery model with voltage sag and brownout protection.
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
        return BatteryModel(12.0_V, 0.15_ohm, 0.35_A, 20.0_A, 15.0_A, 7.5_V, 6.5_V, 7.2_V);
    }

    double BatteryModel::CalculateTerminalVoltage(double totalMotorSupplyCurrent) const
    {
        double totalCurrent = std::max(0.0_A, totalMotorSupplyCurrent) + quiescentCurrent;
        double voltageSag = totalCurrent * internalResistance;
        double terminalVoltage = nominalVoltage - voltageSag;

        return std::max(0.0_V, terminalVoltage);
    }

    double BatteryModel::ApplySupplyCurrentLimit(double commandedVoltage, double statorCurrent,
        double terminalVoltage) const
    {
        if (terminalVoltage <= 0.001_V || std::abs(commandedVoltage) <= 0.001_V)
        {
            return 0.0_V;
        }

        // Power drawn at motor armature: P = |I_stator * V_cmd|
        double demandedPower = std::abs(statorCurrent * commandedVoltage);
        // Maximum electrical power allowed by supply breaker: P_max = I_supply_limit * V_terminal
        double allowedPower = supplyCurrentLimit * terminalVoltage;

        if (demandedPower > allowedPower && demandedPower > 1e-6_W)
        {
            double scale = allowedPower / demandedPower;
            return commandedVoltage * scale;
        }

        return commandedVoltage;
    }

    bool BatteryModel::IsBrownoutWarning(double terminalVoltage) const
    {
        return terminalVoltage < brownoutWarningVoltage;
    }

    bool BatteryModel::IsBrownoutCritical(double terminalVoltage) const
    {
        if (isBrownedOutState)
        {
            // Schmidt trigger recovery threshold: requires voltage to rise above recovery level
            if (terminalVoltage > brownoutRecoveryVoltage)
            {
                isBrownedOutState = false;
            }
        }
        else
        {
            // Schmidt trigger trip threshold: falls below critical cutoff
            if (terminalVoltage < brownoutCutoffVoltage)
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