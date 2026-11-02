/**
 * @file IMUComponent.h
 * @brief Component representing a 6-axis IMU (Pigeon 2 / navX) mounted to the robot chassis.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Robot/Sensors/Models/IMUSensorModel.h"
#include <string>

namespace FRCBananaSim::Robot::Sensors::Components
{
    /**
     * @struct IMUComponent
     * @brief ECS component encapsulating an IMUSensorModel mounted to a chassis rigid body.
     */
    struct IMUComponent
    {
        std::string name;
        Core::ECS::Core::Entity parentChassis = Core::ECS::Core::invalidEntity; ///< Chassis rigid body entity.

        Models::IMUSensorModel model;

        // Kinematic feed override (used in testing or when physics step is synthetic)
        bool useKinematicOverride = false;
        glm::dvec3 overrideAngularVelocityNWU = { 0.0, 0.0, 0.0 };
        glm::dvec3 overrideLinearAccelNWU = { 0.0, 0.0, 0.0 };
        glm::dquat overrideOrientationNWU = { 1.0, 0.0, 0.0, 0.0 };
    };
}