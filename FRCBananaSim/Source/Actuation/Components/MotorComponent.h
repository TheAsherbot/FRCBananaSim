/**
 * @file MotorComponent.h
 * @brief ECS component holding DC motor properties, applied demands, and sensor feedback.
 */

#pragma once

#include "Actuation/DCMotorModel.h"
#include "Core/Units/Units.h"

namespace FRCBananaSim::Actuation::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct MotorComponent
     * @brief Attached to entities representing actuated joints or mechanism linkages.
     */
    struct MotorComponent
    {
        DCMotorModel motorModel;                ///< Configured physical motor parameters.
        double commandedVoltage = 0.0_V;        ///< Voltage requested by robot code / controller (V).
        double appliedVoltage = 0.0_V;          ///< Actual voltage applied post current-limit and brownout (V).
        double statorCurrent = 0.0_A;           ///< Active armature winding current (A).
        double supplyCurrent = 0.0_A;           ///< Current drawn from battery bus (A).
        double outputTorque = 0.0_Nm;           ///< Instantaneous output torque delivered to joint (N*m).
        double angularPosition = 0.0_rad;       ///< Joint position in radians.
        double angularVelocity = 0.0_radps;     ///< Joint velocity in rad/s.
        double statorCurrentLimit = 60.0_A;     ///< Stator current clamp threshold (A).
    };
}