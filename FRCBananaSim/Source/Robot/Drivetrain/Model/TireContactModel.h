/**
 * @file TireContactModel.h
 * @brief Normal compliance, elastic bristle friction, and radial-return plasticity model for swerve tires.
 */

#pragma once

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct TireContactState
     * @brief Persistent contact patch dynamic states per wheel across integration steps.
     */
    struct TireContactState
    {
        glm::vec2 bristleDeflection{ 0.0, 0.0 }; ///< 2D tangential elastic bristle deflection in contact frame (m).
        double normalForce = 0.0;                ///< Normal reaction force N (Newtons).
        double penetrationDepth = 0.0;           ///< Ground radial penetration depth d (m).
        double rollingDistance = 0.0;            ///< Accumulated linear rolling distance (m).
        bool isGrounded = false;                 ///< True if wheel has non-zero positive normal load.
        double saturationRatio = 0.0;            ///< Elliptical friction saturation ratio n (n > 1 indicates active slip).
    };

    /**
     * @struct EffectiveTireStiffness
     * @brief Linearized contact patch stiffness and intercept force for the implicit chassis solver.
     */
    struct EffectiveTireStiffness
    {
        double stiffnessX = 0.0;      ///< Effective longitudinal stiffness Kx (N / (m/s)).
        double stiffnessY = 0.0;      ///< Effective lateral stiffness Ky (N / (m/s)).
        glm::vec2 interceptForce{ 0.0, 0.0 }; ///< Intercept force vector f0 (N).
    };

    /**
     * @class TireContactModel
     * @brief Computes radial normal compliance and tangential elastoplastic friction forces on carpet.
     */
    class TireContactModel
    {
    public:
        /**
         * @brief Constructs tire model with configurable physical parameters.
         * @param params Tire geometry, radial stiffness, and friction envelope settings.
         */
        explicit TireContactModel(const TireParams& params);

        /**
         * @brief Default constructor using reference 4-inch swerve tire settings.
         */
        TireContactModel();

        /**
         * @brief Calculates normal contact force via radial spring-damper compliance.
         * N' = max(0.0, kt * d + ct * v_penetration).
         * @param penetrationDepth Ground radial penetration depth in meters (>0 is compressed).
         * @param penetrationVelocity Velocity of hub towards ground along surface normal (m/s).
         * @param deltaTime Integration sub-step duration in seconds.
         * @return Normal load N in Newtons.
         */
        double CalculateNormalForce(double penetrationDepth, double penetrationVelocity, double deltaTime) const;

        /**
         * @brief Calculates effective linearized tangential stiffness terms for the implicit chassis solver.
         * @param normalForce Resolved normal force N (N).
         * @param wheelAxleInertia Rotational inertia of wheel and reflected drive rotor (kg*m^2).
         * @param driveElectricalDamping Reflected back-EMF damping coefficient D (N*m / (rad/s)).
         * @param driveStallTorqueOffset Commanded stall torque contribution tau0 (N*m).
         * @param currentWheelSpin Angular velocity of wheel axle at start of sub-step (rad/s).
         * @param bristleDeflection Current 2D bristle deflection vector z (m).
         * @param deltaTime Integration sub-step duration in seconds.
         * @return EffectiveTireStiffness struct containing Kx, Ky, and intercept vector f0.
         */
        EffectiveTireStiffness CalculateEffectiveTangentialStiffness(double normalForce,
            double wheelAxleInertia, double driveElectricalDamping, double driveStallTorqueOffset,
            double currentWheelSpin, const glm::vec2& bristleDeflection, double deltaTime) const;

        /**
         * @brief Evaluates 2D contact friction force and updates persistent bristle state using radial return.
         * @param normalForce Normal reaction force N (N).
         * @param slipVelocity Contact patch slip velocity vector s = (sx, sy) in contact frame (m/s).
         * @param deltaTime Integration sub-step duration in seconds.
         * @param inOutState Mutable reference to module's persistent contact state.
         * @return Tangential friction force vector applied to the robot in contact frame (N).
         */
        glm::vec2 UpdateFrictionAndState(double normalForce, const glm::vec2& slipVelocity,
            double deltaTime, TireContactState& inOutState) const;

        /**
         * @brief Computes effective friction coefficient accounting for Stribeck slip velocity and normal load.
         * @param slipSpeed Magnitude of tangential slip velocity (m/s).
         * @param normalForce Current normal load N (N).
         * @return Scalar friction coefficient mu.
         */
        double CalculateEffectiveFrictionCoefficient(double slipSpeed, double normalForce) const;

        /**
         * @brief Retrieves configured tire physical parameters.
         * @return Const reference to TireParams.
         */
        const TireParams& GetParams() const;

    private:
        TireParams params;
    };
}