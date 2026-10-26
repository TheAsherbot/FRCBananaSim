/**
 * @file JointComponent.h
 * @brief Component representing a physical mechanical joint connecting two rigid bodies.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Core/Units/Units.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <glm/vec3.hpp>

#include <cstdint>

namespace FRCBananaSim::Physics::Components
{
    using namespace Core::Units::Literals;

    /**
     * @enum JointType
     * @brief Mechanical degrees of freedom for the constraint.
     */
    enum class JointType : uint8_t
    {
        Hinge = 0,  ///< 1 rotational degree of freedom (swerve steer, wheel axle, arm).
        Slider = 1, ///< 1 translational degree of freedom (elevator stage).
        Fixed = 2   ///< 0 degrees of freedom (rigid mounting).
    };

    /**
     * @enum JointMotorMode
     * @brief Active drive mode applied to the joint constraint.
     */
    enum class JointMotorMode : uint8_t
    {
        Off = 0,      ///< Passive free-spinning joint with optional friction.
        Velocity = 1, ///< Velocity motor driving towards target angular velocity (wheel drive).
        Position = 2, ///< Position servo motor driving towards target angle (swerve steer, arm).
        Torque = 3    ///< Direct torque drive applied along hinge axis.
    };

    /**
     * @enum JointConstraintSpace
     * @brief Coordinate space used for pivot and axis definitions.
     */
    enum class JointConstraintSpace : uint8_t
    {
        LocalToBodyCOM = 0, ///< Anchors defined relative to each body's center of mass.
        WorldSpace = 1      ///< Anchors defined in world coordinates at spawn time.
    };

    /**
     * @struct JointComponent
     * @brief Configures and tracks physical constraints between ECS rigid bodies.
     */
    struct JointComponent
    {
        JointType type = JointType::Hinge;
        JointConstraintSpace constraintSpace = JointConstraintSpace::LocalToBodyCOM;

        Core::ECS::Core::Entity parentEntity = Core::ECS::Core::invalidEntity; ///< Parent body (or invalidEntity for world).
        Core::ECS::Core::Entity childEntity = Core::ECS::Core::invalidEntity;  ///< Dynamic child body.

        // Anchor points
        glm::vec3 parentAnchor{ 0.0f, 0.0f, 0.0f }; ///< Pivot relative to parent COM (or world pivot).
        glm::vec3 childAnchor{ 0.0f, 0.0f, 0.0f };  ///< Pivot relative to child COM.

        // Alignment axes
        glm::vec3 parentAxis{ 0.0f, 1.0f, 0.0f };   ///< Hinge rotation axis in parent space.
        glm::vec3 childAxis{ 0.0f, 1.0f, 0.0f };    ///< Hinge rotation axis in child space.
        glm::vec3 parentNormal{ 1.0f, 0.0f, 0.0f }; ///< Normal axis perpendicular to hinge axis in parent space.
        glm::vec3 childNormal{ 1.0f, 0.0f, 0.0f };  ///< Normal axis perpendicular to hinge axis in child space.

        // Rotational Limits
        bool hasLimits = false;                      ///< If false, continuous 360-degree rotation.
        float minLimit = -3.14159265f;               ///< Minimum angle in radians.
        float maxLimit = 3.14159265f;                ///< Maximum angle in radians.

        // Motor Controls
        JointMotorMode motorMode = JointMotorMode::Off;
        float targetVelocity = 0.0f;                 ///< Target angular velocity in rad/s (Velocity mode).
        float targetAngle = 0.0f;                    ///< Target angle in radians (Position mode).
        float maxMotorTorque = 1000.0f;              ///< Maximum torque limit in N*m.
        float appliedTorque = 0.0f;                  ///< Direct torque in N*m applied to child 
        float springFrequency = 10.0f;               ///< Spring oscillation frequency for position mode (Hz).
        float springDamping = 1.0f;                  ///< Critical damping ratio for position mode.
        float maxFrictionTorque = 0.0f;              ///< Passive bearing friction torque (N*m).

        // Runtime Telemetry & State
        JPH::Ref<JPH::Constraint> constraint;        ///< Allocated Jolt constraint handle.
        float currentAngle = 0.0f;                   ///< Instantaneous relative rotation angle (rad).
        float currentVelocity = 0.0f;                ///< Instantaneous relative angular velocity (rad/s).
        float currentDeflection = 0.0f;              ///< Distance between body pivots in meters (joint separation).
        bool isCreated = false;                      ///< True once instantiated in Jolt PhysicsSystem.

        /**
         * @brief Configures this joint using world-space coordinates.
         * @param worldPivot World coordinate of the hinge pivot.
         * @param worldHingeAxis Normalized rotation axis in world coordinates.
         */
        void SetWorldSpace(const glm::vec3& worldPivot, const glm::vec3& worldHingeAxis);

        /**
         * @brief Configures this joint using local center-of-mass offsets.
         */
        void SetLocalSpace(const glm::vec3& localPoint1, const glm::vec3& localAxis1,
            const glm::vec3& localPoint2, const glm::vec3& localAxis2);
    };
}