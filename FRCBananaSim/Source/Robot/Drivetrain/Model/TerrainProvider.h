/**
 * @file TerrainProvider.h
 * @brief Abstract interface for querying ground terrain geometry and contact normals under swerve wheels.
 */

#pragma once

#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct WheelContactProbe
     * @brief Result of querying ground terrain under an individual swerve wheel.
     * All vectors are expressed in standard WPILib NWU frame (+X Forward, +Y Left, +Z Up).
     */
    struct WheelContactProbe
    {
        glm::vec3 contactPoint{ 0.0f, 0.0f, 0.0f };  ///< World contact patch location (m).
        glm::vec3 surfaceNormal{ 0.0f, 0.0f, 1.0f }; ///< Upward unit normal of the ground surface.
        double penetrationDepth = 0.0;               ///< Signed radial penetration in meters (>0 is compressed).
        bool hasContact = false;                     ///< True if wheel touches or penetrates the surface.
    };

    /**
     * @class ITerrainProvider
     * @brief Pure interface decoupled from physics backends (implemented by Jolt probe or analytic tests).
     */
    class ITerrainProvider
    {
    public:
        /**
         * @brief Virtual destructor.
         */
        virtual ~ITerrainProvider() = default;

        /**
         * @brief Queries surface penetration and normal for a wheel positioned at hubPosition.
         * @param hubPosition Center of wheel hub in world coordinates (+X Forward, +Y Left, +Z Up).
         * @param wheelRadius Nominal undeformed outer wheel radius (m).
         * @param wheelWidth Wheel tread width (m).
         * @return WheelContactProbe struct containing surface normal and compression depth.
         */
        virtual WheelContactProbe QueryWheelContact(const glm::vec3& hubPosition,
            double wheelRadius, double wheelWidth) const = 0;
    };
}