/**
 * @file ServoComponent.h
 * @brief ECS component holding servo configuration, commanded targets, and sensor feedback.
 */

#pragma once

#include "Actuation/ServoModel.h"
#include "Core/Units/Units.h"

namespace FRCBananaSim::Actuation::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct ServoComponent
     * @brief Attached to entities representing servo-actuated linkages, grippers, or wrists.
     */
    struct ServoComponent
    {
        ServoModel servoModel;                 ///< Configured physical servo parameters.
        double targetFraction = 0.5;           ///< Normalized PWM demand (0.0 to 1.0).
        double currentAngle = 0.0_rad;         ///< Current physical angle (rad).
        double currentVelocity = 0.0_radps;    ///< Current physical angular velocity (rad/s).
        double outputTorque = 0.0_Nm;          ///< Torque delivered to joint (N*m).
        double currentDraw = 0.0_A;            ///< Instantaneous current draw (A).
        uint32_t pwmChannel = 0;               ///< Associated WPILib PWM channel index.
    };
}