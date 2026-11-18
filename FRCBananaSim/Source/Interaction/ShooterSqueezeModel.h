/**
 * @file ShooterSqueezeModel.h
 * @brief Sub-stepped Hunt-Crossley normal squeeze force and tangential traction physics model.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace FRCBananaSim::Interaction
{
    /**
     * @struct SqueezeParameters
     * @brief Geometry, material compliance, and friction constants for shooter compression.
     */
    struct SqueezeParameters
    {
        double stiffnessK = 25000.0;        ///< Hunt-Crossley contact stiffness (N/m^1.5)
        double restitutionE = 0.25;         ///< Coefficient of restitution (0.0 to 1.0)
        double frictionMu = 0.68;           ///< Flywheel roller surface Coulomb friction
        double hoodFrictionMu = 0.15;       ///< Opposing guide/hood friction (0.0 for dual active flywheels)
        double rollerRadiusM = 0.0508;      ///< Flywheel roller radius (m)
        double nominalGapM = 0.09;          ///< Shooter nip clearance gap (m)
        double uncompressedSizeM = 0.12;    ///< Uncompressed dimension in compression axis (m)
        double contactLengthM = 0.18;       ///< Total transit distance through contact zone (m)
        double pieceMassKg = 0.235;         ///< Mass of the game piece (kg)
        bool isDualFlywheel = false;        ///< True if dual active driven rollers, false if single + hood
    };

    /**
     * @struct SqueezeState
     * @brief Runtime state of a game piece passing through shooter compression.
     */
    struct SqueezeState
    {
        double progressM = 0.0;             ///< Current position along contact zone [0..contactLengthM]
        double velocityMPerS = 0.0;         ///< Forward speed along tangent direction (m/s)
        double compressionDepthM = 0.0;     ///< Instantaneous squeeze depth delta (m)
        double compressionVelocityMPerS = 0.0; ///< Rate of squeeze delta_dot = d(delta)/dt (m/s)
        double normalForceN = 0.0;          ///< Instantaneous Hunt-Crossley normal force F_n (N)
        double tractionForceN = 0.0;        ///< Instantaneous tangential traction force F_t (N)
        double loadTorqueNm = 0.0;          ///< Braking load torque on flywheel rotor (N*m)
        double dissipatedEnergyJ = 0.0;     ///< Cumulative energy lost to slip friction and damping (J)
    };

    /**
     * @struct EnergyBalanceReport
     * @brief Energy accounting across flywheel deceleration, projectile exit kinetic energy, and dissipation.
     */
    struct EnergyBalanceReport
    {
        double initialFlywheelRPM = 0.0;
        double finalFlywheelRPM = 0.0;
        double flywheelDeltaEnergyJ = 0.0;
        double projectileKineticEnergyJ = 0.0;
        double totalDissipatedEnergyJ = 0.0;
        double balanceResidualJ = 0.0;
        double balanceErrorPercent = 0.0;
        bool isEnergyConserved = false;     ///< True if Delta E_fw >= E_proj and balance within tolerance
    };

    /**
     * @class ShooterSqueezeModel
     * @brief Mathematical engine evaluating non-linear compliant contact forces and load torque.
     */
    class ShooterSqueezeModel
    {
    public:
        /**
         * @brief Evaluates the Hunt-Crossley compliant normal contact force:
         *        F_n = max(0.0, k * delta^n * (1 + alpha * delta_dot))
         *        where alpha = 3 * (1 - e) / (2 * v_approach).
         * @param depthM Instantaneous compression depth delta (m).
         * @param depthVelocityMPerS Compression rate delta_dot (m/s).
         * @param stiffnessK Contact stiffness k (N/m^n).
         * @param restitutionE Coefficient of restitution e (0.0 to 1.0).
         * @param approachVelocityMPerS Characteristic approach speed (m/s).
         * @param exponentN Hertzian non-linear geometric exponent (default 1.5).
         * @return Normal force F_n in Newtons (strictly >= 0.0).
         */
        static double CalculateHuntCrossleyNormalForce(
            double depthM,
            double depthVelocityMPerS,
            double stiffnessK,
            double restitutionE,
            double approachVelocityMPerS = 1.0,
            double exponentN = 1.5);

        /**
         * @brief Evaluates tangential traction force with smooth slip regularization:
         *        F_t = mu * F_n * tanh(v_slip / v_thresh).
         * @param normalForceN Instantaneous normal force F_n (N).
         * @param slipVelocityMPerS Relative surface slip speed v_roller - v_piece (m/s).
         * @param frictionMu Coulomb friction coefficient.
         * @param slipThresholdMPerS Chattering regularization threshold (default 0.05 m/s).
         * @return Traction force F_t in Newtons.
         */
        static double CalculateTangentialTractionForce(
            double normalForceN,
            double slipVelocityMPerS,
            double frictionMu,
            double slipThresholdMPerS = 0.05);

        /**
         * @brief Computes continuous squeeze depth delta(x) and squeeze rate delta_dot(x, v)
         *        over a compression nip of length L with peak compression delta_max.
         * @param progressM Current position x along tangent [0..contactLengthM].
         * @param velocityMPerS Current projectile speed v = dx/dt (m/s).
         * @param contactLengthM Length L of the compression nip (m).
         * @param maxCompressionM Peak compression delta_max = uncompressedSize - nominalGap (m).
         * @param outDepthM Output instantaneous compression depth delta (m).
         * @param outDepthVelocityMPerS Output instantaneous compression rate delta_dot (m/s).
         */
        static void CalculateCompressionProfile(
            double progressM,
            double velocityMPerS,
            double contactLengthM,
            double maxCompressionM,
            double& outDepthM,
            double& outDepthVelocityMPerS);

        /**
         * @brief Numerically integrates one time step dt of projectile squeeze through the shooter.
         * @param params Constant geometric and material parameters.
         * @param state In-out projectile progress, velocity, forces, and accumulated dissipation.
         * @param flywheelSurfaceSpeedMPerS Tangential velocity omega_fw * r_roller of the driving wheel.
         * @param dt Timestep duration in seconds.
         * @param subSteps Number of internal sub-steps (default 4 = 1.25 ms for 200 Hz).
         */
        static void StepSqueeze(
            const SqueezeParameters& params,
            SqueezeState& state,
            double flywheelSurfaceSpeedMPerS,
            double dt,
            uint32_t subSteps = 4);

        /**
         * @brief Validates analytical work-energy conservation across a simulated shot.
         * @param flywheelInertiaKgm2 Total lumped rotational inertia J_total of the flywheel.
         * @param initialOmegaRadPerSec Flywheel speed before squeeze entry (rad/s).
         * @param finalOmegaRadPerSec Flywheel speed after muzzle exit (rad/s).
         * @param pieceMassKg Mass of the fired projectile (kg).
         * @param pieceExitSpeedMPerS Velocity of the projectile upon exit (m/s).
         * @param totalDissipatedEnergyJ Total slip and damping energy accumulated during transit (J).
         * @return Structured energy balance report.
         */
        static EnergyBalanceReport ComputeEnergyBalance(
            double flywheelInertiaKgm2,
            double initialOmegaRadPerSec,
            double finalOmegaRadPerSec,
            double pieceMassKg,
            double pieceExitSpeedMPerS,
            double totalDissipatedEnergyJ);
    };
}
