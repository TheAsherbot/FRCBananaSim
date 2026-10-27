/**
 * @file SwerveModuleModel.cpp
 * @brief Implementation of individual swerve module electromechanics and contact force evaluation.
 */

#include "Robot/Drivetrain/Model/SwerveModuleModel.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    SwerveModuleModel::SwerveModuleModel(const ModulePlacement& placement, const SteerModuleParams& steerParams,
        const DriveModuleParams& driveParams, const TireParams& tireParams)
        : placement(placement),
        steerParams(steerParams),
        driveParams(driveParams),
        tireModel(tireParams)
    {}

    SwerveModuleModel::SwerveModuleModel()
        : placement(ModulePlacement{}),
        steerParams(SteerModuleParams{}),
        driveParams(DriveModuleParams{}),
        tireModel(TireParams{})
    {}

    void SwerveModuleModel::StepSteer(const Control::MotorCommand& steerCommand, double busVoltage,
        double chassisYawRate, double deltaTime)
    {
        if (deltaTime <= 1e-7)
        {
            return;
        }

        // 1. Determine commanded motor voltage
        double appliedVolts = 0.0;
        switch (steerCommand.mode)
        {
        case Control::MotorControlMode::Voltage:
            appliedVolts = std::clamp(steerCommand.demand, -busVoltage, busVoltage);
            break;
        case Control::MotorControlMode::DutyCycle:
            appliedVolts = std::clamp(steerCommand.demand, -1.0, 1.0) * busVoltage;
            break;
        case Control::MotorControlMode::TorqueCurrent:
        {
            double r = steerParams.motor.GetResistance();
            double ke = steerParams.motor.GetBackEmfConstantKe();
            double motorRadps = state.steerVelocity * steerParams.motor.GetGearRatio();
            appliedVolts = std::clamp(steerCommand.demand * r + ke * motorRadps, -busVoltage, busVoltage);
            break;
        }
        case Control::MotorControlMode::Disabled:
            appliedVolts = 0.0;
            break;
        }

        // 2. Compute affine motor torque terms: tau = tau0 - D * omega
        Actuation::AffineMotorTorque affine = steerParams.motor.CalculateAffineTorqueTerms(appliedVolts);
        double maxStatorTorque = steerParams.motor.CalculateStatorTorqueLimit(steerCommand.statorCurrentLimit);

        // Stator current limit clamping on affine model:
        double naturalTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.steerVelocity);
        if (naturalTorque > maxStatorTorque)
        {
            affine.stallTorqueCommanded = maxStatorTorque;
            affine.electricalDamping = 0.0;
        }
        else if (naturalTorque < -maxStatorTorque)
        {
            affine.stallTorqueCommanded = -maxStatorTorque;
            affine.electricalDamping = 0.0;
        }

        if (steerCommand.mode == Control::MotorControlMode::Disabled &&
            steerCommand.neutralMode == Control::NeutralMode::Coast)
        {
            affine.electricalDamping = 0.0;
        }

        // 3. Total effective inertia at azimuth shaft: J_eff = J_housing + J_rotor * G^2
        double jEff = steerParams.steerHousingInertia +
            (steerParams.motor.GetRotorInertia() * steerParams.motor.GetGearRatio() * steerParams.motor.GetGearRatio());

        // 4. Free velocity without friction
        double totalDamping = affine.electricalDamping + steerParams.viscousDamping;
        double denominator = (jEff / deltaTime) + totalDamping;
        double freeVelocity = ((jEff * state.steerVelocity / deltaTime) + affine.stallTorqueCommanded) / denominator;

        // 5. Exact 1D Coulomb friction stick projection
        double availableTorque = std::abs(freeVelocity) * denominator;
        double tauCoulomb = steerParams.coulombFrictionTorque;

        if (availableTorque <= tauCoulomb)
        {
            state.steerVelocity = 0.0;
            state.steerAppliedTorque = affine.stallTorqueCommanded;
        }
        else
        {
            double sign = (freeVelocity > 0.0) ? 1.0 : -1.0;
            state.steerVelocity = freeVelocity - (sign * tauCoulomb / denominator);
            state.steerAppliedTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.steerVelocity);
        }

        // 6. Integrate azimuth angle
        state.steerAngle += state.steerVelocity * deltaTime;

        constexpr double pi = 3.14159265358979323846;
        while (state.steerAngle > pi)
        {
            state.steerAngle -= 2.0 * pi;
        }
        while (state.steerAngle < -pi)
        {
            state.steerAngle += 2.0 * pi;
        }

        // 7. Continuous unwrapped angle tracking
        double deltaAngle = state.steerAngle - state.lastSteerAngle;
        while (deltaAngle > pi)
        {
            deltaAngle -= 2.0 * pi;
        }
        while (deltaAngle < -pi)
        {
            deltaAngle += 2.0 * pi;
        }
        state.unwrappedSteerAngle += deltaAngle;
        state.lastSteerAngle = state.steerAngle;

        state.steerStatorCurrent = steerParams.motor.CalculateCurrent(appliedVolts, state.steerVelocity);
    }

    EffectiveTireStiffness SwerveModuleModel::ComputeTangentialStiffness(const Control::MotorCommand& driveCommand,
        double busVoltage, double normalForce, double deltaTime) const
    {
        double appliedVolts = 0.0;
        switch (driveCommand.mode)
        {
        case Control::MotorControlMode::Voltage:
            appliedVolts = std::clamp(driveCommand.demand, -busVoltage, busVoltage);
            break;
        case Control::MotorControlMode::DutyCycle:
            appliedVolts = std::clamp(driveCommand.demand, -1.0, 1.0) * busVoltage;
            break;
        case Control::MotorControlMode::TorqueCurrent:
        {
            double r = driveParams.motor.GetResistance();
            double ke = driveParams.motor.GetBackEmfConstantKe();
            double motorRadps = state.wheelVelocity * driveParams.motor.GetGearRatio();
            appliedVolts = std::clamp(driveCommand.demand * r + ke * motorRadps, -busVoltage, busVoltage);
            break;
        }
        case Control::MotorControlMode::Disabled:
            appliedVolts = 0.0;
            break;
        }

        Actuation::AffineMotorTorque affine = driveParams.motor.CalculateAffineTorqueTerms(appliedVolts);
        double maxStatorTorque = driveParams.motor.CalculateStatorTorqueLimit(driveCommand.statorCurrentLimit);

        double naturalTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.wheelVelocity);
        if (naturalTorque > maxStatorTorque)
        {
            affine.stallTorqueCommanded = maxStatorTorque;
            affine.electricalDamping = 0.0;
        }
        else if (naturalTorque < -maxStatorTorque)
        {
            affine.stallTorqueCommanded = -maxStatorTorque;
            affine.electricalDamping = 0.0;
        }

        if (driveCommand.mode == Control::MotorControlMode::Disabled &&
            driveCommand.neutralMode == Control::NeutralMode::Coast)
        {
            affine.electricalDamping = 0.0;
        }

        // Locked axle stick check: when motor demand is zero and neutral mode is Brake at near-zero velocity
        if (std::abs(driveCommand.demand) < 1e-4 && driveCommand.neutralMode == Control::NeutralMode::Brake &&
            std::abs(state.wheelVelocity) < 1e-3)
        {
            return tireModel.CalculateLockedTangentialStiffness(normalForce, state.contactState.bristleDeflection, deltaTime);
        }

        double jAxle = tireModel.GetParams().wheelAxleInertia +
            (driveParams.motor.GetRotorInertia() * driveParams.motor.GetGearRatio() * driveParams.motor.GetGearRatio());

        return tireModel.CalculateEffectiveTangentialStiffness(normalForce, jAxle, affine.electricalDamping,
            affine.stallTorqueCommanded, state.wheelVelocity, state.contactState.bristleDeflection, deltaTime);
    }

    glm::vec3 SwerveModuleModel::FinalizeDriveStep(const Control::MotorCommand& driveCommand, double busVoltage,
        double normalForce, const glm::vec3& groundPatchVelocity,
        const glm::vec3& surfaceNormal, const glm::vec3& wheelHeadingWorld, double deltaTime)
    {
        if (deltaTime <= 1e-7)
        {
            return glm::vec3(0.0f);
        }

        // 1. Orthogonalize rolling axis along terrain surface normal in world NWU
        glm::vec3 eX = wheelHeadingWorld - surfaceNormal * glm::dot(wheelHeadingWorld, surfaceNormal);
        float lenEx = glm::length(eX);
        if (lenEx > 1e-5f)
        {
            eX /= lenEx;
        }
        else
        {
            eX = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        glm::vec3 eY = glm::cross(surfaceNormal, eX);

        // 2. Project chassis contact velocity onto terrain tangent frame
        double patchUx = glm::dot(groundPatchVelocity, eX);
        double patchUy = glm::dot(groundPatchVelocity, eY);

        // 3. Locked axle stick check
        bool isAxleStick = false;
        if (std::abs(driveCommand.demand) < 1e-4 && driveCommand.neutralMode == Control::NeutralMode::Brake &&
            std::abs(state.wheelVelocity) < 1e-3)
        {
            isAxleStick = true;
        }

        double r = tireModel.GetParams().wheelRadius;

        if (isAxleStick)
        {
            state.wheelVelocity = 0.0;
            state.linearVelocity = 0.0;

            glm::vec2 slipVel(patchUx, patchUy);
            glm::vec2 fTangential = tireModel.UpdateFrictionAndState(normalForce, slipVel, deltaTime, state.contactState);

            state.driveAppliedTorque = -r * fTangential.x;
            state.driveStatorCurrent = 0.0;

            glm::vec3 worldForce = (eX * static_cast<float>(fTangential.x)) +
                (eY * static_cast<float>(fTangential.y)) +
                (surfaceNormal * static_cast<float>(normalForce));
            return worldForce;
        }

        // 4. Commanded motor voltage
        double appliedVolts = 0.0;
        switch (driveCommand.mode)
        {
        case Control::MotorControlMode::Voltage:
            appliedVolts = std::clamp(driveCommand.demand, -busVoltage, busVoltage);
            break;
        case Control::MotorControlMode::DutyCycle:
            appliedVolts = std::clamp(driveCommand.demand, -1.0, 1.0) * busVoltage;
            break;
        case Control::MotorControlMode::TorqueCurrent:
        {
            double res = driveParams.motor.GetResistance();
            double ke = driveParams.motor.GetBackEmfConstantKe();
            double motorRadps = state.wheelVelocity * driveParams.motor.GetGearRatio();
            appliedVolts = std::clamp(driveCommand.demand * res + ke * motorRadps, -busVoltage, busVoltage);
            break;
        }
        case Control::MotorControlMode::Disabled:
            appliedVolts = 0.0;
            break;
        }

        Actuation::AffineMotorTorque affine = driveParams.motor.CalculateAffineTorqueTerms(appliedVolts);
        double maxStatorTorque = driveParams.motor.CalculateStatorTorqueLimit(driveCommand.statorCurrentLimit);

        double naturalTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.wheelVelocity);
        if (naturalTorque > maxStatorTorque)
        {
            affine.stallTorqueCommanded = maxStatorTorque;
            affine.electricalDamping = 0.0;
        }
        else if (naturalTorque < -maxStatorTorque)
        {
            affine.stallTorqueCommanded = -maxStatorTorque;
            affine.electricalDamping = 0.0;
        }

        if (driveCommand.mode == Control::MotorControlMode::Disabled &&
            driveCommand.neutralMode == Control::NeutralMode::Coast)
        {
            affine.electricalDamping = 0.0;
        }

        double jAxle = tireModel.GetParams().wheelAxleInertia +
            (driveParams.motor.GetRotorInertia() * driveParams.motor.GetGearRatio() * driveParams.motor.GetGearRatio());

        double mu = tireModel.CalculateEffectiveFrictionCoefficient(0.0, normalForce);
        double kBristle = (mu * normalForce) / std::max(1e-5, tireModel.GetParams().preSlideDisplacement);
        constexpr double effectiveQuarterMass = 12.5;
        constexpr double bristleZeta = 0.707;
        double cBristle = 2.0 * bristleZeta * std::sqrt(kBristle * effectiveQuarterMass);
        double kEff = (kBristle * deltaTime) + cBristle;

        // 5. Implicit wheel axle velocity update
        double lambda = (jAxle / deltaTime) + affine.electricalDamping + (r * r * kEff);
        double numA = (jAxle * state.wheelVelocity / deltaTime) + affine.stallTorqueCommanded + (kBristle * r * state.contactState.bristleDeflection.x);
        double a = numA / lambda;
        state.wheelVelocity = a + (r * kEff / lambda) * patchUx;

        // 6. Update friction forces and bristle state via radial return plasticity
        glm::vec2 slipVel(patchUx - (r * state.wheelVelocity), patchUy);
        glm::vec2 fTangential = tireModel.UpdateFrictionAndState(normalForce, slipVel, deltaTime, state.contactState);

        state.driveAppliedTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.wheelVelocity) - (r * fTangential.x);
        state.wheelPosition += state.wheelVelocity * deltaTime;
        state.linearVelocity = state.wheelVelocity * r;
        state.distanceTraveled += state.linearVelocity * deltaTime;

        state.driveStatorCurrent = driveParams.motor.CalculateCurrent(appliedVolts, state.wheelVelocity);

        // 7. Total 3D force delivered to chassis in world NWU coordinates
        glm::vec3 worldForce = (eX * static_cast<float>(fTangential.x)) +
            (eY * static_cast<float>(fTangential.y)) +
            (surfaceNormal * static_cast<float>(normalForce));

        return worldForce;
    }

    const SwerveModuleState& SwerveModuleModel::GetState() const { return state; }
    SwerveModuleState& SwerveModuleModel::GetState() { return state; }
    const ModulePlacement& SwerveModuleModel::GetPlacement() const { return placement; }
    const Actuation::DCMotorModel& SwerveModuleModel::GetDriveMotor() const { return driveParams.motor; }
    const Actuation::DCMotorModel& SwerveModuleModel::GetSteerMotor() const { return steerParams.motor; }
}