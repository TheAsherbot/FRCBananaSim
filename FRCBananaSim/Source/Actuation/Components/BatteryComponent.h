/**
 * @file BatteryComponent.h
 * @brief ECS component tracking robot battery electrical state and brownout condition.
 */

#pragma once

#include "Actuation/BatteryModel.h"
#include "Core/Units/Units.h"

namespace FRCBananaSim::Actuation::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct BatteryComponent
     * @brief Singleton-style robot component managing electrical bus voltage.
     */
    struct BatteryComponent
    {
        BatteryModel batteryModel;              ///< Configured electrical battery parameters.
        double terminalVoltage = 12.60_V;       ///< Instantaneous bus terminal voltage (V).
        double totalCurrent = 1.5_A;            ///< Aggregate current drawn by all actuators plus idle load (A).
        bool isBrownoutWarning = false;         ///< RoboRIO brownout warning flag.
        bool isBrownoutCritical = false;        ///< RoboRIO motor shutoff threshold reached.
    };
}