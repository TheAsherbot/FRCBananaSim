/**
 * @file BeamBreakComponent.h
 * @brief Component representing an optical beam-break sensor bound to a collision volume with debounce filtering.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <cstdint>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @struct BeamBreakComponent
     * @brief ECS component evaluating volume overlap events on ObjectLayers::SENSOR with debounce timing.
     */
    struct BeamBreakComponent
    {
        std::string name;

        // Collision Volume Binding
        Core::ECS::Core::Entity sensorEntity = Core::ECS::Core::invalidEntity; ///< Entity holding ColliderComponent (isSensor=true).

        // Channel & Configuration
        uint32_t dioChannel = 0;        ///< Digital I/O channel [0..31] written to SimOutputs::digitalInputs.
        bool normallyClosed = false;    ///< True if beam is unbroken when high (standard FRC beam-break logic).
        double debounceTimeS = 0.010;   ///< Debounce filter duration (default 10 ms).

        // Runtime State
        bool isBroken = false;          ///< Instantaneous physical occlusion state from physics contact overlaps.
        bool dioState = false;          ///< Filtered logic level delivered to digitalInputs bit.
        double debounceTimerS = 0.0;    ///< Accumulated time in current physical state.
    };
}