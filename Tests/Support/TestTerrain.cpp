/**
 * @file TestTerrain.cpp
 * @brief Implementation of analytic terrain geometry providers.
 */

#include "Support/TestTerrain.h"
#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Tests::Support
{
    using namespace Robot::Drivetrain::Model;

    FlatCarpetTerrain::FlatCarpetTerrain(double groundElevationZ)
        : elevationZ(groundElevationZ)
    {}

    WheelContactProbe FlatCarpetTerrain::QueryWheelContact(const glm::vec3& hubPosition,
        double wheelRadius, double wheelWidth) const
    {
        WheelContactProbe probe;
        probe.surfaceNormal = glm::vec3(0.0f, 0.0f, 1.0f); // Upward vertical normal in NWU

        double lowestWheelZ = hubPosition.z - wheelRadius;
        probe.penetrationDepth = elevationZ - lowestWheelZ;
        probe.hasContact = (probe.penetrationDepth > 0.0);

        probe.contactPoint = glm::vec3(hubPosition.x, hubPosition.y, static_cast<float>(elevationZ));
        return probe;
    }

    TiltedRampTerrain::TiltedRampTerrain(double inclineAngleRad, double pivotX)
        : angleRad(inclineAngleRad),
        pivotX(pivotX)
    {
        // Normal tilted in X-Z plane: (-sin(theta), 0, cos(theta))
        surfaceNormal = glm::vec3(
            static_cast<float>(-std::sin(angleRad)),
            0.0f,
            static_cast<float>(std::cos(angleRad))
        );
    }

    WheelContactProbe TiltedRampTerrain::QueryWheelContact(const glm::vec3& hubPosition,
        double wheelRadius, double wheelWidth) const
    {
        WheelContactProbe probe;
        probe.surfaceNormal = surfaceNormal;

        // Plane equation passing through (pivotX, 0, 0): n . (p - p0) = 0
        glm::vec3 p0(static_cast<float>(pivotX), 0.0f, 0.0f);
        double distanceToPlane = glm::dot(hubPosition - p0, surfaceNormal);

        probe.penetrationDepth = wheelRadius - distanceToPlane;
        probe.hasContact = (probe.penetrationDepth > 0.0);

        probe.contactPoint = hubPosition - (surfaceNormal * static_cast<float>(distanceToPlane));
        return probe;
    }
}