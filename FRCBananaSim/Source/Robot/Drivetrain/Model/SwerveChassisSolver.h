/**
 * @file SwerveChassisSolver.h
 * @brief 6-DOF implicit dynamics solver for chassis velocities under multi-wheel tire contact and normal loads.
 */

#pragma once

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include "Robot/Drivetrain/Model/SwerveModuleModel.h"
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct ChassisMotionState
     * @brief 6-DOF spatial velocity and transform state of the robot chassis in world NWU frame.
     */
    struct ChassisMotionState
    {
        glm::vec3 position{ 0.0f, 0.0f, 0.20f };       ///< World CG location (+X Forward, +Y Left, +Z Up) in meters.
        glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f }; ///< World orientation quaternion (Hamilton W, X, Y, Z).
        glm::vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };   ///< Linear velocity v in world coordinates (m/s).
        glm::vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };  ///< Angular velocity omega in world coordinates (rad/s).
    };

    /**
     * @struct SolverWheelContact
     * @brief Geometry and contact impedance for an individual wheel during the 6x6 solve.
     */
    struct SolverWheelContact
    {
        glm::vec3 contactPoint{ 0.0f, 0.0f, 0.0f };  ///< World coordinates of contact patch (m).
        glm::vec3 surfaceNormal{ 0.0f, 0.0f, 1.0f }; ///< Terrain upward unit normal vector.
        glm::vec3 wheelHeading{ 1.0f, 0.0f, 0.0f };  ///< Steering wheel forward rolling unit vector.
        double penetrationDepth = 0.0;               ///< Static compression depth (m).
        double springStiffnessKt = 0.0;              ///< Radial spring stiffness kt (N/m).
        double normalStiffnessKn = 0.0;              ///< Effective normal stiffness Kn = kt * dt + ct.
        EffectiveTireStiffness tangentialStiffness;  ///< Kx, Ky, and intercept force vector f0.
        double muX = 1.20;                           ///< Longitudinal friction coefficient.
        double muY = 1.20;                           ///< Lateral friction coefficient.
        bool hasContact = false;                     ///< True if wheel is touching terrain.
    };

    /**
     * @class SwerveChassisSolver
     * @brief Solves implicit 6-DOF equations of motion using an SPD Cholesky decomposition.
     */
    class SwerveChassisSolver
    {
    public:
        /**
         * @brief Constructs 6-DOF chassis solver.
         * @param chassis Mass properties and inertia tensor of the robot.
         */
        explicit SwerveChassisSolver(const ChassisParams& chassis);

        /**
         * @brief Default constructor.
         */
        SwerveChassisSolver();

        /**
         * @brief Solves 6-DOF linear velocity and angular velocity at step t + dt via implicit integration.
         * Incorporates gravity, gyroscopic torque, and multi-pass tire friction saturation.
         * @param currentState Current 6-DOF pose and spatial velocity.
         * @param wheelContacts Contact geometric properties and linearized impedance from each wheel.
         * @param deltaTime Integration timestep duration (s).
         * @param outNewLinearVelocity Output resolved linear velocity (m/s).
         * @param outNewAngularVelocity Output resolved angular velocity (rad/s).
         */
        void SolveStep(const ChassisMotionState& currentState,
            const std::array<SolverWheelContact, 4>& wheelContacts,
            double deltaTime,
            glm::vec3& outNewLinearVelocity,
            glm::vec3& outNewAngularVelocity) const;

        /**
         * @brief Updates chassis mass and geometric inertia properties.
         * @param chassis New chassis configuration parameters.
         */
        void SetChassisParams(const ChassisParams& chassis);

        /**
         * @brief Accessor for current chassis parameters.
         * @return Const reference to ChassisParams.
         */
        const ChassisParams& GetChassisParams() const;

    private:
        ChassisParams chassisParams;
    };
}