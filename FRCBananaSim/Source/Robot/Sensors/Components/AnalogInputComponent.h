/**
 * @file AnalogInputComponent.h
 * @brief Component representing an analog 0.0V to 5.0V input transducer (potentiometers, pressure sensors).
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <cstdint>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @enum AnalogInputMode
     * @brief Input signal origin for the analog channel.
     */
    enum class AnalogInputMode : uint8_t
    {
        JointPosition = 0, ///< Linear/angular potentiometer reading joint travel.
        CustomVoltage = 1  ///< Custom programmatic voltage (e.g. pressure sensor).
    };

    /**
     * @struct AnalogInputComponent
     * @brief ECS component mapping mechanical travel or transducer voltage to SimOutputs::analogInputs[0..7].
     */
    struct AnalogInputComponent
    {
        std::string name;
        uint32_t analogChannel = 0; ///< Analog input channel [0..7].
        AnalogInputMode mode = AnalogInputMode::JointPosition;

        Core::ECS::Core::Entity targetJoint = Core::ECS::Core::invalidEntity; ///< JointComponent if in JointPosition mode.

        double minInputValue = 0.0;  ///< Mechanism travel coordinate at minimum voltage (m or rad).
        double maxInputValue = 1.0;  ///< Mechanism travel coordinate at maximum voltage (m or rad).
        double minVoltage = 0.0;     ///< Minimum output voltage (typically 0.0 V).
        double maxVoltage = 5.0;     ///< Maximum output voltage (typically 5.0 V).

        // Runtime Output
        double voltage = 0.0;        ///< Measured voltage output in Volts (0.0V to 5.0V).
    };
}