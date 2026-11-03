/**
 * @file LimitSwitchComponent.h
 * @brief Component representing a physical travel limit switch with hysteresis and motor stop flags.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <cstdint>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @enum LimitSwitchDirection
     * @brief Motion direction arrested when the limit switch is tripped.
     */
    enum class LimitSwitchDirection : uint8_t
    {
        Forward = 0, ///< Blocks positive motor commands (forwardLimitTriggered).
        Reverse = 1  ///< Blocks negative motor commands (reverseLimitTriggered).
    };

    /**
     * @struct LimitSwitchComponent
     * @brief ECS component binding a joint coordinate to a DIO channel and actuator limit flags.
     */
    struct LimitSwitchComponent
    {
        std::string name;

        // Kinematic Joint Binding
        Core::ECS::Core::Entity targetJoint = Core::ECS::Core::invalidEntity; ///< JointComponent to evaluate.

        // Channel & Motor Mapping
        uint32_t dioChannel = 0;              ///< Digital I/O channel [0..31] written to SimOutputs::digitalInputs.
        uint32_t attachToMotorSlot = 0;       ///< SimInputs motor slot index [0..23] receiving limit flags.
        LimitSwitchDirection direction = LimitSwitchDirection::Forward;

        // Trigger Envelope & Hysteresis
        double triggerMin = 0.0;              ///< Lower trip coordinate threshold (m or rad).
        double triggerMax = 0.0;              ///< Upper trip coordinate threshold (m or rad).
        double hysteresis = 0.0005;           ///< Hysteresis band (0.5 mm or 0.0035 rad) preventing contact chatter.
        bool normallyClosed = false;          ///< Inverted logic: true if switch is closed/high when not pressed.

        // Runtime State
        bool isTriggered = false;             ///< True if physical joint is currently inside the trip zone.
        bool dioState = false;                ///< Actual logic level delivered to digitalInputs bit.
    };
}