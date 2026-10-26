/**
 * @file SwerveModuleComponent.h
 * @brief ECS component holding physical entity references and state for an individual swerve module.
 */

#pragma once

#include "Actuation/DCMotorModel.h"
#include "Core/ECS/Core/Entity.h"
#include "Core/Units/Units.h"
#include <glm/vec2.hpp>

namespace FRCBananaSim::Drivetrain::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct SwerveModuleComponent
     * @brief Attached to each of the 4 swerve module root entities.
     */
    struct SwerveModuleComponent
    {
        uint32_t moduleIndex = 0;              ///< 0: FrontLeft, 1: FrontRight, 2: BackLeft, 3: BackRight.
        glm::vec2 moduleLocation{ 0.0f, 0.0f }; ///< Offset from chassis center in meters (+X forward, +Y left).

        // Physical Entity References
        Core::ECS::Core::Entity steerEntity = Core::ECS::Core::invalidEntity;      ///< Steer housing rigid body entity.
        Core::ECS::Core::Entity wheelEntity = Core::ECS::Core::invalidEntity;      ///< Wheel cylinder rigid body entity.
        Core::ECS::Core::Entity steerJointEntity = Core::ECS::Core::invalidEntity; ///< Vertical steer hinge joint entity.
        Core::ECS::Core::Entity driveJointEntity = Core::ECS::Core::invalidEntity; ///< Horizontal wheel axle joint entity.

        // Electromechanical Motor Models
        Actuation::DCMotorModel steerMotor = Actuation::DCMotorModel::Falcon500(21.43, 0.92);
        Actuation::DCMotorModel driveMotor = Actuation::DCMotorModel::KrakenX60(6.12, 0.96);

        // Physical Parameters
        double wheelRadius = 0.0508_m; ///< Standard 2-inch radius (4-inch wheel).

        // Hardware IDs for CTRE Phoenix 6 / WPILib Bridge
        uint32_t steerCANId = 1;
        uint32_t driveCANId = 2;
        uint32_t canCoderId = 1;

        // Runtime Feedback Telemetry
        double currentSteerAngle = 0.0_rad;         ///< Azimuth angle in radians.
        double currentSteerVelocity = 0.0_radps;    ///< Azimuth angular rate in rad/s.
        double currentWheelPosition = 0.0_rad;      ///< Accumulated wheel axle rotation in radians.
        double currentWheelVelocity = 0.0_radps;    ///< Wheel axle angular rate in rad/s.
        double currentLinearVelocity = 0.0_mps;     ///< Ground tangential velocity in m/s (omega * r).
        double distanceTraveled = 0.0_m;            ///< Ground distance traveled in meters (theta * r).
    };
}