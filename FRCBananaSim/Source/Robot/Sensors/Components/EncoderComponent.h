/**
 * @file EncoderComponent.h
 * @brief Component representing an encoder sensor attached to a joint, motor slot, or standalone CANcoder.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Robot/Sensors/Models/EncoderSensorModel.h"
#include <string>
#include <cstdint>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @enum EncoderBindingType
     * @brief Output mapping target for the encoder transducer.
     */
    enum class EncoderBindingType : uint8_t
    {
        CANcoder = 0,    ///< Standalone CANcoder slot [0..11] (cancoderPositionsRot / cancoderVelocitiesRps).
        MotorRotor = 1,  ///< Ungeared motor rotor slot [0..23] (rotorPositionsRot / rotorVelocitiesRps).
        JointOnly = 2    ///< Custom joint feedback without direct SimOutputs slot binding.
    };

    /**
     * @struct EncoderComponent
     * @brief ECS component binding a joint or mechanism to an EncoderSensorModel and SimOutputs slot.
     */
    struct EncoderComponent
    {
        std::string name;
        EncoderBindingType bindingType = EncoderBindingType::CANcoder;
        uint32_t slot = 0; ///< CANcoder slot [0..11] or Motor slot [0..23].

        Core::ECS::Core::Entity targetJoint = Core::ECS::Core::invalidEntity; ///< Target JointComponent to track.

        Models::EncoderSensorModel model;
    };
}