/**
 * @file SwerveChassisComponent.h
 * @brief ECS component holding chassis dimensions, module entity handles, and battery linkage.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Core/Units/Units.h"
#include <array>
#include <glm/vec2.hpp>

namespace FRCBananaSim::Drivetrain::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct SwerveChassisComponent
     * @brief Attached to the root chassis entity.
     */
    struct SwerveChassisComponent
    {
        // 4 modules: [0] FrontLeft, [1] FrontRight, [2] BackLeft, [3] BackRight
        std::array<Core::ECS::Core::Entity, 4> moduleEntities;

        // Battery linkage entity (for bus voltage sag calculation)
        Core::ECS::Core::Entity batteryEntity = Core::ECS::Core::invalidEntity;

        // Chassis Geometry
        double trackWidth = 0.56_m;  ///< Left-to-right wheel spacing in meters.
        double wheelBase = 0.56_m;   ///< Front-to-back wheel spacing in meters.
        double chassisMass = 50.0_kg; ///< Total nominal robot mass in kg.

        // Runtime IMU Telemetry (CTRE Pigeon 2)
        double gyroYaw = 0.0_rad;
        double gyroPitch = 0.0_rad;
        double gyroRoll = 0.0_rad;
        double gyroYawRate = 0.0_radps;
    };
}