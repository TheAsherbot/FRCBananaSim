/**
 * @file TireContactModel.cpp
 * @brief Implementation of tire radial compliance, Stribeck friction, and radial return plasticity.
 */

#include "Robot/Drivetrain/Model/TireContactModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    TireContactModel::TireContactModel(const TireParams& params)
        : params(params)
    {}

    TireContactModel::TireContactModel()
        : params(TireParams{})
    {}

    double TireContactModel::CalculateNormalForce(double penetrationDepth, double penetrationVelocity, double deltaTime) const
    {
        if (penetrationDepth <= 1e-6)
        {
            return 0.0;
        }

        // Implicit normal force: N' = max(0.0, kt * d + ct * v_pen)
        double springForce = params.radialStiffness * penetrationDepth;
        double damperForce = params.radialDamping * penetrationVelocity;

        double normalForce = springForce + damperForce;
        return std::max(0.0, normalForce);
    }

    double TireContactModel::CalculateEffectiveFrictionCoefficient(double slipSpeed, double normalForce) const
    {
        // 1. Stribeck curve: transition from static mu to kinetic mu
        double stribeckRatio = std::abs(slipSpeed) / std::max(1e-4, params.stribeckVelocity);
        double decay = std::exp(-stribeckRatio * stribeckRatio);
        double muVelocity = params.muKinetic + (params.muStatic - params.muKinetic) * decay;

        // 2. Normal load sensitivity: mu(N) = mu0 * (N / N_ref)^(-beta)
        constexpr double referenceLoad = 125.0; // Nominal ~1/4 robot weight (~12.5 kg * 9.81 m/s^2)
        double normalizedLoad = std::max(1.0, normalForce) / referenceLoad;
        double loadFactor = std::pow(normalizedLoad, -params.loadSensitivityBeta);

        return std::clamp(muVelocity * loadFactor, 0.1, 2.5);
    }

    EffectiveTireStiffness TireContactModel::CalculateEffectiveTangentialStiffness(double normalForce,
        double wheelAxleInertia, double driveElectricalDamping, double driveStallTorqueOffset,
        double currentWheelSpin, const glm::vec2& bristleDeflection, double deltaTime) const
    {
        EffectiveTireStiffness result;

        if (normalForce <= 1e-3 || deltaTime <= 1e-7)
        {
            return result;
        }

        double mu = CalculateEffectiveFrictionCoefficient(0.0, normalForce);
        double kBristle = (mu * normalForce) / std::max(1e-5, params.preSlideDisplacement);

        // Critical damping for bristle: c = 2 * zeta * sqrt(k * m_eff)
        constexpr double effectiveQuarterMass = 12.5; // kg
        constexpr double bristleZeta = 0.707;
        double cBristle = 2.0 * bristleZeta * std::sqrt(kBristle * effectiveQuarterMass);
        double kEff = (kBristle * deltaTime) + cBristle;

        // Drive motor affine coupling elimination:
        // Lambda = J_w / dt + D + R^2 * k_eff
        double r = params.wheelRadius;
        double lambda = (wheelAxleInertia / deltaTime) + driveElectricalDamping + (r * r * kEff);

        // a = (J_w * omega / dt + tau0 + k * R * zx) / Lambda
        double numeratorA = (wheelAxleInertia * currentWheelSpin / deltaTime) + driveStallTorqueOffset + (kBristle * r * bristleDeflection.x);
        double a = numeratorA / lambda;

        // Longitudinal (X): Kx = kEff * (J_w / dt + D) / Lambda
        result.stiffnessX = kEff * ((wheelAxleInertia / deltaTime) + driveElectricalDamping) / lambda;
        result.interceptForce.x = (kEff * r * a) - (kBristle * bristleDeflection.x);

        // Lateral (Y): Ky = kEff * lateralFrictionRatio
        result.stiffnessY = kEff * params.lateralFrictionRatio;
        result.interceptForce.y = -kBristle * bristleDeflection.y;

        return result;
    }

    glm::vec2 TireContactModel::UpdateFrictionAndState(double normalForce, const glm::vec2& slipVelocity,
        double deltaTime, TireContactState& inOutState) const
    {
        if (normalForce <= 1e-3 || deltaTime <= 1e-7)
        {
            inOutState.normalForce = 0.0;
            inOutState.isGrounded = false;
            inOutState.saturationRatio = 0.0;
            inOutState.bristleDeflection = glm::vec2(0.0);
            return glm::vec2(0.0);
        }

        inOutState.normalForce = normalForce;
        inOutState.isGrounded = true;

        double slipSpeed = glm::length(slipVelocity);
        double muX = CalculateEffectiveFrictionCoefficient(slipSpeed, normalForce);
        double muY = muX * params.lateralFrictionRatio;

        // Bristle dynamic parameters
        double kBristleX = (muX * normalForce) / std::max(1e-5, params.preSlideDisplacement);
        double kBristleY = (muY * normalForce) / std::max(1e-5, params.preSlideDisplacement);

        constexpr double effectiveQuarterMass = 12.5;
        constexpr double bristleZeta = 0.707;
        double cBristleX = 2.0 * bristleZeta * std::sqrt(kBristleX * effectiveQuarterMass);
        double cBristleY = 2.0 * bristleZeta * std::sqrt(kBristleY * effectiveQuarterMass);

        double kEffX = (kBristleX * deltaTime) + cBristleX;
        double kEffY = (kBristleY * deltaTime) + cBristleY;

        // Trial elastic force (reaction force on robot, opposing slip)
        glm::vec2 trialForce;
        trialForce.x = -((kBristleX * inOutState.bristleDeflection.x) + (kEffX * slipVelocity.x));
        trialForce.y = -((kBristleY * inOutState.bristleDeflection.y) + (kEffY * slipVelocity.y));

        // Subtract rolling resistance in longitudinal rolling direction
        double fRollingResistance = params.rollingResistance * normalForce * std::clamp(slipVelocity.x / 0.05, -1.0, 1.0);
        trialForce.x -= fRollingResistance;

        // Elliptical yield surface evaluation
        double limitX = std::max(1e-3, muX * normalForce);
        double limitY = std::max(1e-3, muY * normalForce);

        double normX = trialForce.x / limitX;
        double normY = trialForce.y / limitY;
        double saturationNorm = std::sqrt((normX * normX) + (normY * normY));
        inOutState.saturationRatio = saturationNorm;

        glm::vec2 finalForce = trialForce;

        if (saturationNorm > 1.0)
        {
            // Radial return plasticity projection
            finalForce.x /= saturationNorm;
            finalForce.y /= saturationNorm;

            // Invert spring-damper to project consistent bristle deflection state
            inOutState.bristleDeflection.x = -(finalForce.x + (cBristleX * slipVelocity.x)) / kBristleX;
            inOutState.bristleDeflection.y = -(finalForce.y + (cBristleY * slipVelocity.y)) / kBristleY;
        }
        else
        {
            // Elastic stick phase: step bristle deflection
            inOutState.bristleDeflection.x += deltaTime * slipVelocity.x;
            inOutState.bristleDeflection.y += deltaTime * slipVelocity.y;
        }

        return finalForce;
    }

    const TireParams& TireContactModel::GetParams() const
    {
        return params;
    }
}