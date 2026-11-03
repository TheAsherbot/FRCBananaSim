/**
 * @file JointComponent.h
 * @brief Component representing a physical mechanical joint connecting two rigid bodies.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Core/Units/Units.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/Constraint.h>
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
        Off = 0,      ///< Passive free joint with optional friction.
        Velocity = 1, ///< Velocity motor driving towards target speed (rad/s or m/s).
        Position = 2, ///< Position servo driving towards target angle/position (rad or m).
        Torque = 3    ///< Direct force or torque drive applied along the joint axis.
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
        glm::vec3 parentAnchor{ 0.0f, 0.0f, 0.0f }; ///< Pivot/anchor relative to parent COM (or world anchor).
        glm::vec3 childAnchor{ 0.0f, 0.0f, 0.0f };  ///< Pivot/anchor relative to child COM.

        // Alignment axes
        glm::vec3 parentAxis{ 0.0f, 1.0f, 0.0f };   ///< Hinge rotation axis or slider axis in parent space.
        glm::vec3 childAxis{ 0.0f, 1.0f, 0.0f };    ///< Hinge rotation axis or slider axis in child space.
        glm::vec3 parentNormal{ 1.0f, 0.0f, 0.0f }; ///< Normal axis perpendicular to joint axis in parent space.
        glm::vec3 childNormal{ 1.0f, 0.0f, 0.0f };  ///< Normal axis perpendicular to joint axis in child space.

        // Travel Limits (rad for Hinge, meters for Slider)
        bool hasLimits = false;                      ///< If false, unlimited travel.
        float minLimit = -3.14159265f;               ///< Minimum angle (rad) or slider travel (m).
        float maxLimit = 3.14159265f;                ///< Maximum angle (rad) or slider travel (m).
        bool hasLimitSpring = true;                  ///< If true, applies soft spring limits at the travel stops.
        float limitSpringFrequency = 40.0f;          ///< Hard stop spring frequency (Hz) for critically damped bounce reduction.
        float limitSpringDamping = 1.0f;            ///< Hard stop spring damping ratio (1.0 = critically damped).

        // Motor Controls
        JointMotorMode motorMode = JointMotorMode::Off;
        float targetVelocity = 0.0f;                 ///< Target speed in rad/s (Hinge) or m/s (Slider).
        float targetPosition = 0.0f;                 ///< Target position in radians (Hinge) or meters (Slider).
        float targetAngle = 0.0f;                    ///< Rotational alias for targetPosition.
        float maxMotorTorque = 1000.0f;              ///< Maximum torque limit in N*m (Hinge).
        float maxMotorForce = 1000.0f;               ///< Maximum force limit in N (Slider).
        float appliedTorque = 0.0f;                  ///< Direct torque in N*m applied to child hinge body.
        float appliedForce = 0.0f;                   ///< Direct linear force in N applied to child slider body.
        float springFrequency = 10.0f;               ///< Motor drive position spring frequency (Hz).
        float springDamping = 1.0f;                  ///< Motor drive position damping ratio.
        float maxFrictionTorque = 0.0f;              ///< Passive bearing friction torque (N*m).
        float maxFrictionForce = 0.0f;               ///< Passive linear sliding friction force (N).

        // Runtime Telemetry & State
        JPH::Ref<JPH::Constraint> constraint;        ///< Allocated Jolt constraint handle.
        float currentPosition = 0.0f;                ///< Instantaneous relative position (rad for hinge, m for slider).
        float currentAngle = 0.0f;                   ///< Instantaneous relative rotation angle in radians (Hinge).
        float currentVelocity = 0.0f;                ///< Instantaneous relative velocity (rad/s for hinge, m/s for slider).
        float currentDeflection = 0.0f;              ///< Joint pivot deflection / constraint separation in meters.
        bool isCreated = false;                      ///< True once instantiated in Jolt PhysicsSystem.

        /**
         * @brief Configures this joint as a hinge using world-space coordinates.
         * @param worldPivot World coordinate of the hinge pivot.
         * @param worldHingeAxis Normalized rotation axis in world coordinates.
         */
        void SetWorldSpace(const glm::vec3& worldPivot, const glm::vec3& worldHingeAxis);

        /**
         * @brief Configures this joint as a slider using world-space coordinates.
         * @param worldAnchor World coordinate of the slider rest point.
         * @param worldSliderAxis Normalized translation axis in world coordinates.
         * @param minTravelM Minimum slider limit in meters.
         * @param maxTravelM Maximum slider limit in meters.
         */
        void SetWorldSpaceSlider(const glm::vec3& worldAnchor, const glm::vec3& worldSliderAxis,
            float minTravelM, float maxTravelM);

        /**
         * @brief Configures this joint using local center-of-mass offsets.
         */
        void SetLocalSpace(const glm::vec3& localPoint1, const glm::vec3& localAxis1,
            const glm::vec3& localPoint2, const glm::vec3& localAxis2);

        /**
         * @brief Configures this joint as a slider using local center-of-mass offsets.
         */
        void SetLocalSpaceSlider(const glm::vec3& localPoint1, const glm::vec3& localAxis1,
            const glm::vec3& localPoint2, const glm::vec3& localAxis2,
            float minTravelM, float maxTravelM);
    };
}