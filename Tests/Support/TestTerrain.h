/**
 * @file TestTerrain.h
 * @brief Analytic terrain geometry providers (flat carpet, tilted ramps, step bumps) for headless testing.
 */

#pragma once

#include "Robot/Drivetrain/Model/TerrainProvider.h"
#include <glm/vec3.hpp>

namespace FRCBananaSim::Tests::Support
{
    /**
     * @class FlatCarpetTerrain
     * @brief Perfectly flat horizontal ground plane at Z = 0 in NWU coordinates.
     */
    class FlatCarpetTerrain : public Robot::Drivetrain::Model::ITerrainProvider
    {
    public:
        /**
         * @brief Constructs flat carpet terrain at specified ground elevation.
         * @param groundElevationZ Ground elevation along vertical +Z axis in meters (default 0.0).
         */
        explicit FlatCarpetTerrain(double groundElevationZ = 0.0);

        /**
         * @brief Queries contact penetration and upward normal for a wheel.
         * @param hubPosition Center of wheel hub in world coordinates (+X Forward, +Y Left, +Z Up).
         * @param wheelRadius Outer radius of the wheel in meters.
         * @param wheelWidth Width of wheel tread in meters.
         * @return WheelContactProbe struct.
         */
        virtual Robot::Drivetrain::Model::WheelContactProbe QueryWheelContact(const glm::vec3& hubPosition,
            double wheelRadius, double wheelWidth) const override;

    private:
        double elevationZ;
    };

    /**
     * @class TiltedRampTerrain
     * @brief Planar ramp inclined by pitch angle theta along the forward (+X) axis.
     */
    class TiltedRampTerrain : public Robot::Drivetrain::Model::ITerrainProvider
    {
    public:
        /**
         * @brief Constructs tilted ramp terrain.
         * @param inclineAngleRad Incline pitch angle in radians (positive tilts upwards in +X).
         * @param pivotX Longitudinal X coordinate where elevation is zero (default 0.0).
         */
        explicit TiltedRampTerrain(double inclineAngleRad, double pivotX = 0.0);

        /**
         * @brief Queries contact penetration and normal on the tilted ramp surface.
         * @param hubPosition Center of wheel hub in world coordinates.
         * @param wheelRadius Outer radius of the wheel in meters.
         * @param wheelWidth Width of wheel tread in meters.
         * @return WheelContactProbe struct.
         */
        virtual Robot::Drivetrain::Model::WheelContactProbe QueryWheelContact(const glm::vec3& hubPosition,
            double wheelRadius, double wheelWidth) const override;

    private:
        double angleRad;
        double pivotX;
        glm::vec3 surfaceNormal;
    };
}