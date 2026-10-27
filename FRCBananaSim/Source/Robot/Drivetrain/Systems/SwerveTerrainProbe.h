/**
 * @file SwerveTerrainProbe.h
 * @brief Jolt CollideShape implementation of the ITerrainProvider interface for wheel terrain queries.
 */

#pragma once

#include "Robot/Drivetrain/Model/TerrainProvider.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>

namespace FRCBananaSim::Robot::Drivetrain::Systems
{
    /**
     * @class SwerveTerrainProbe
     * @brief Queries Jolt static geometry using convex cylinder collision tests under each swerve wheel.
     */
    class SwerveTerrainProbe : public Model::ITerrainProvider
    {
    public:
        /**
         * @brief Constructs terrain probe linked to a Jolt simulation context.
         * @param joltContext Reference to the active Jolt physics context.
         * @param nominalRadius Nominal wheel radius (m) for pre-allocating the query cylinder shape.
         * @param nominalWidth Nominal wheel tread width (m).
         * @param ignoredBody Optional chassis body ID to exclude from ground queries.
         */
        explicit SwerveTerrainProbe(Physics::JoltContext& joltContext,
            double nominalRadius = 0.0508, double nominalWidth = 0.0381,
            JPH::BodyID ignoredBody = JPH::BodyID());

        /**
         * @brief Queries ground surface penetration and normal for a wheel hub position.
         * @param hubPosition Center of wheel hub in world coordinates (+X Forward, +Y Left, +Z Up).
         * @param wheelRadius Nominal undeformed outer wheel radius (m).
         * @param wheelWidth Wheel tread width (m).
         * @return WheelContactProbe struct with surface normal and signed compression.
         */
        Model::WheelContactProbe QueryWheelContact(const glm::vec3& hubPosition,
            double wheelRadius, double wheelWidth) const override;

        /**
         * @brief Sets the chassis body handle to ignore during collision queries.
         * @param bodyID Jolt body handle of the chassis.
         */
        void SetIgnoredBody(JPH::BodyID bodyID);

        /**
         * @brief Sets query separation distance envelope (default 0.005 m = 5 mm).
         * @param separationDistance Maximum separation distance to probe.
         */
        void SetMaxSeparationDistance(float separationDistance);

    private:
        Physics::JoltContext& context;
        JPH::BodyID ignoredBodyID;
        float maxSeparationDistance = 0.005f;

        // Cached reusable cylinder shape
        JPH::RefConst<JPH::Shape> wheelShape;
        double cachedRadius = 0.0;
        double cachedWidth = 0.0;
    };
}