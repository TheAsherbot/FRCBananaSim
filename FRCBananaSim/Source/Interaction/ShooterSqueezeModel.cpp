/**
 * @file ShooterSqueezeModel.cpp
 * @brief Implementation of sub-stepped Hunt-Crossley squeeze dynamics, slip traction, and energy balance.
 */

#include "Interaction/ShooterSqueezeModel.h"

namespace FRCBananaSim::Interaction
{
    double ShooterSqueezeModel::CalculateHuntCrossleyNormalForce(
        double depthM,
        double depthVelocityMPerS,
        double stiffnessK,
        double restitutionE,
        double approachVelocityMPerS,
        double exponentN)
    {
        if (depthM <= 1e-9 || stiffnessK <= 1e-9)
        {
            return 0.0;
        }

        double e = std::clamp(restitutionE, 0.0, 1.0);
        double vApp = std::max(std::abs(approachVelocityMPerS), 0.05);
        double alpha = (1.5 * (1.0 - e)) / vApp;

        double dampingTerm = 1.0 + alpha * depthVelocityMPerS;
        if (dampingTerm <= 0.0)
        {
            return 0.0;
        }

        double force = stiffnessK * std::pow(depthM, exponentN) * dampingTerm;
        return std::max(0.0, force);
    }

    double ShooterSqueezeModel::CalculateTangentialTractionForce(
        double normalForceN,
        double slipVelocityMPerS,
        double frictionMu,
        double slipThresholdMPerS)
    {
        if (normalForceN <= 1e-9 || frictionMu <= 1e-9)
        {
            return 0.0;
        }

        double thresh = std::max(slipThresholdMPerS, 1e-4);
        return frictionMu * normalForceN * std::tanh(slipVelocityMPerS / thresh);
    }

    void ShooterSqueezeModel::CalculateCompressionProfile(
        double progressM,
        double velocityMPerS,
        double contactLengthM,
        double maxCompressionM,
        double& outDepthM,
        double& outDepthVelocityMPerS)
    {
        if (progressM < 0.0 || progressM > contactLengthM || contactLengthM <= 1e-6 || maxCompressionM <= 1e-6)
        {
            outDepthM = 0.0;
            outDepthVelocityMPerS = 0.0;
            return;
        }

        constexpr double pi = 3.14159265358979323846;
        double theta = pi * (progressM / contactLengthM);
        outDepthM = maxCompressionM * std::sin(theta);
        outDepthVelocityMPerS = maxCompressionM * (pi / contactLengthM) * std::cos(theta) * velocityMPerS;
    }

    void ShooterSqueezeModel::StepSqueeze(
        const SqueezeParameters& params,
        SqueezeState& state,
        double flywheelSurfaceSpeedMPerS,
        double dt,
        uint32_t subSteps)
    {
        if (dt <= 1e-9)
        {
            return;
        }

        uint32_t nSub = std::clamp(subSteps, 1u, 16u);
        double subDt = dt / static_cast<double>(nSub);
        double maxCompression = std::max(0.0, params.uncompressedSizeM - params.nominalGapM);
        double mass = std::max(params.pieceMassKg, 0.01);

        double accumulatedTorque = 0.0;

        for (uint32_t i = 0; i < nSub; ++i)
        {
            // 1. Current compression depth and velocity
            double depth = 0.0;
            double depthVel = 0.0;
            CalculateCompressionProfile(state.progressM, state.velocityMPerS, params.contactLengthM, maxCompression, depth, depthVel);

            state.compressionDepthM = depth;
            state.compressionVelocityMPerS = depthVel;

            // 2. Normal force
            double vApproach = std::max(flywheelSurfaceSpeedMPerS, 0.5);
            double fn = CalculateHuntCrossleyNormalForce(depth, depthVel, params.stiffnessK, params.restitutionE, vApproach);
            state.normalForceN = fn;

            // 3. Tangential traction from active flywheel
            double slipVel = flywheelSurfaceSpeedMPerS - state.velocityMPerS;
            double ft = CalculateTangentialTractionForce(fn, slipVel, params.frictionMu);
            state.tractionForceN = ft;

            // 4. Opposing friction from stationary guide/hood (if single flywheel)
            double fHood = 0.0;
            if (!params.isDualFlywheel && params.hoodFrictionMu > 1e-6 && state.velocityMPerS > 1e-4)
            {
                fHood = -params.hoodFrictionMu * fn * std::tanh(state.velocityMPerS / 0.05);
            }

            // Net forward force on projectile (dual flywheels provide 2x driven traction)
            double fNet = params.isDualFlywheel ? (2.0 * ft) : (ft + fHood);

            // 5. Accelerate projectile
            double accel = fNet / mass;
            double vNext = state.velocityMPerS + accel * subDt;
            if (vNext < 0.0) vNext = 0.0;

            // Advance position (trapezoidal integration)
            double avgV = 0.5 * (state.velocityMPerS + vNext);
            state.progressM += avgV * subDt;
            state.velocityMPerS = vNext;

            // 6. Flywheel load torque and energy dissipation
            double subTorque = ft * params.rollerRadiusM;
            accumulatedTorque += subTorque;

            // Work dissipated by slip: P_slip = |F_t * v_slip|
            double slipPower = std::abs(ft * slipVel);
            state.dissipatedEnergyJ += slipPower * subDt;
        }

        state.loadTorqueNm = accumulatedTorque / static_cast<double>(nSub);
    }

    EnergyBalanceReport ShooterSqueezeModel::ComputeEnergyBalance(
        double flywheelInertiaKgm2,
        double initialOmegaRadPerSec,
        double finalOmegaRadPerSec,
        double pieceMassKg,
        double pieceExitSpeedMPerS,
        double totalDissipatedEnergyJ)
    {
        EnergyBalanceReport report;
        constexpr double radPerSecToRpm = 60.0 / (2.0 * 3.14159265358979323846);
        report.initialFlywheelRPM = initialOmegaRadPerSec * radPerSecToRpm;
        report.finalFlywheelRPM = finalOmegaRadPerSec * radPerSecToRpm;

        double eFwInit = 0.5 * flywheelInertiaKgm2 * (initialOmegaRadPerSec * initialOmegaRadPerSec);
        double eFwFinal = 0.5 * flywheelInertiaKgm2 * (finalOmegaRadPerSec * finalOmegaRadPerSec);
        report.flywheelDeltaEnergyJ = std::max(0.0, eFwInit - eFwFinal);

        report.projectileKineticEnergyJ = 0.5 * pieceMassKg * (pieceExitSpeedMPerS * pieceExitSpeedMPerS);
        report.totalDissipatedEnergyJ = totalDissipatedEnergyJ;

        report.balanceResidualJ = report.flywheelDeltaEnergyJ - (report.projectileKineticEnergyJ + report.totalDissipatedEnergyJ);

        if (report.flywheelDeltaEnergyJ > 1e-4)
        {
            report.balanceErrorPercent = 100.0 * std::abs(report.balanceResidualJ) / report.flywheelDeltaEnergyJ;
        }
        else
        {
            report.balanceErrorPercent = 0.0;
        }

        report.isEnergyConserved = (report.flywheelDeltaEnergyJ >= report.projectileKineticEnergyJ - 1e-3)
            && (report.balanceErrorPercent <= 10.0);

        return report;
    }
}
