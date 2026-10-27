/**
 * @file SwerveTerrainProbe.cpp
 * @brief Implementation of Jolt CollideShape terrain query provider.
 */

#include "Robot/Drivetrain/Systems/SwerveTerrainProbe.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Systems
{
    namespace
    {
        class StaticObjectLayerFilter : public JPH::ObjectLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer inLayer) const override
            {
                return inLayer == Physics::ObjectLayers::NonMoving;
            }
        };

        class StaticBroadPhaseLayerFilter : public JPH::BroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override
            {
                return inLayer == Physics::BroadPhaseLayers::NonMoving;
            }
        };
    }

    SwerveTerrainProbe::SwerveTerrainProbe(Physics::JoltContext& joltContext,
        double nominalRadius, double nominalWidth, JPH::BodyID ignoredBody)
        : context(joltContext),
        ignoredBodyID(ignoredBody),
        maxSeparationDistance(0.005f),
        cachedRadius(nominalRadius),
        cachedWidth(nominalWidth)
    {
        // JPH::CylinderShape is oriented along vertical Y: top at +halfHeight, bottom at -halfHeight
        JPH::CylinderShapeSettings shapeSettings(
            static_cast<float>(nominalWidth * 0.5),
            static_cast<float>(nominalRadius),
            0.0f // Zero convex radius for exact mathematical cylinder profile
        );

        auto result = shapeSettings.Create();
        if (result.IsValid())
        {
            wheelShape = result.Get();
        }
    }

    void SwerveTerrainProbe::SetIgnoredBody(JPH::BodyID bodyID)
    {
        ignoredBodyID = bodyID;
    }

    void SwerveTerrainProbe::SetMaxSeparationDistance(float separationDistance)
    {
        maxSeparationDistance = separationDistance;
    }

    Model::WheelContactProbe SwerveTerrainProbe::QueryWheelContact(const glm::vec3& hubPosition,
        double wheelRadius, double wheelWidth) const
    {
        SIM_PROFILE_SCOPE("SwerveTerrainProbe::QueryWheelContact");

        Model::WheelContactProbe probe;
        probe.surfaceNormal = glm::vec3(0.0f, 0.0f, 1.0f);
        probe.contactPoint = hubPosition - glm::vec3(0.0f, 0.0f, static_cast<float>(wheelRadius));
        probe.penetrationDepth = 0.0;
        probe.hasContact = false;

        if (!context.IsInitialized() || !wheelShape)
        {
            return probe;
        }

        // Coordinate conversion NWU -> Jolt Engine (+X Left, +Y Up, +Z Forward):
        JPH::RVec3 joltHubPos(
            static_cast<double>(hubPosition.y),
            static_cast<double>(hubPosition.z),
            static_cast<double>(hubPosition.x)
        );
        // In Jolt, Cylinder axis is along Y (+Y Up).
        // A rolling wheel axle lies horizontally along engine X (+X Right).
        // Rotate 90 degrees around Z to orient cylinder axis along +X Right:
        constexpr float halfSqrt2 = 0.70710678f;
        JPH::Quat joltWheelRot(0.0f, 0.0f, -halfSqrt2, halfSqrt2);

        JPH::RMat44 queryTransform = JPH::RMat44::sRotationTranslation(joltWheelRot, joltHubPos);

        JPH::CollideShapeSettings settings;
        settings.mMaxSeparationDistance = maxSeparationDistance;
        settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;

        StaticBroadPhaseLayerFilter broadphaseFilter;
        StaticObjectLayerFilter objectFilter;
        JPH::IgnoreSingleBodyFilter bodyFilter(ignoredBodyID);

        JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> collector;

        context.GetPhysicsSystem().GetNarrowPhaseQuery().CollideShape(
            wheelShape.GetPtr(),
            JPH::Vec3::sReplicate(1.0f),
            queryTransform,
            settings,
            joltHubPos,
            collector,
            broadphaseFilter,
            objectFilter,
            bodyFilter
        );

        if (collector.HadHit())
        {
            const JPH::CollideShapeResult& hit = collector.mHit;

            // Penetration depth: > 0 means overlapping, <= 0 means within separation distance
            double depth = static_cast<double>(hit.mPenetrationDepth);

            // Direction ground pushes wheel upwards out of collision: -mPenetrationAxis
            JPH::Vec3 groundNormalEng = -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3(0.0f, 1.0f, 0.0f));

            // Ensure normal points upwards towards the wheel hub
            if (groundNormalEng.GetY() < 0.0f)
            {
                groundNormalEng = -groundNormalEng;
            }

            // Convert normal from Jolt Engine back to NWU:
            glm::vec3 normalNWU(
                groundNormalEng.GetZ(),
                groundNormalEng.GetX(),
                groundNormalEng.GetY()
            );

            float len = glm::length(normalNWU);
            if (len > 1e-5f)
            {
                normalNWU /= len;
            }
            else
            {
                normalNWU = glm::vec3(0.0f, 0.0f, 1.0f);
            }

            probe.surfaceNormal = normalNWU;
            probe.penetrationDepth = depth;
            probe.hasContact = (depth > 0.0);

            // Contact point on wheel mid-plane: c = h - n * (R - d)
            probe.contactPoint = hubPosition - normalNWU * static_cast<float>(wheelRadius - depth);
        }

        return probe;
    }
}