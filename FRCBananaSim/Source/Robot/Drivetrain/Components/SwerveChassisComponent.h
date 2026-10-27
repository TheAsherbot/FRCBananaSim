/**
 * @file SwerveChassisComponent.h
 * @brief ECS component holding chassis dimensions, module entity handles, battery linkage, and simulation model instance.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Core/Units/Units.h"
#include <array>
#include <memory>
#include <glm/vec2.hpp>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    class SwerveDrivetrainModel;
}

namespace FRCBananaSim::Robot::Drivetrain::Components
{
    using namespace Core::Units::Literals;

    /**
     * @struct SwerveChassisComponent
     * @brief Attached to the root chassis entity.
     */
    struct SwerveChassisComponent
    {
        // 4 modules: [0] FrontLeft, [1] FrontRight, [2] BackLeft, [3] BackRight
        std::array<Core::ECS::Core::Entity, 4> moduleEntities{
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity,
            Core::ECS::Core::invalidEntity
        };

        // Battery linkage entity
        Core::ECS::Core::Entity batteryEntity = Core::ECS::Core::invalidEntity;

        // Chassis Geometry
        double trackWidth = 0.56_m;   ///< Left-to-right wheel spacing in meters.
        double wheelBase = 0.56_m;    ///< Front-to-back wheel spacing in meters.
        double chassisMass = 50.0_kg; ///< Total nominal robot mass in kg.

        // Model instance owned by this chassis
        std::shared_ptr<Model::SwerveDrivetrainModel> drivetrainModel;

        // Runtime IMU Telemetry (CTRE Pigeon 2 in NWU)
        double gyroYaw = 0.0_rad;
        double gyroPitch = 0.0_rad;
        double gyroRoll = 0.0_rad;
        double gyroYawRate = 0.0_radps;
    };
}