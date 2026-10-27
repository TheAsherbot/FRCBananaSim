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
        affine.stallTorqueCommanded = std::clamp(affine.stallTorqueCommanded, -maxStatorTorque, maxStatorTorque);

        // In NeutralMode::Coast with zero voltage, back-EMF damping is zero
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
            // Static stick phase: zero velocity
            state.steerVelocity = 0.0;
            state.steerAppliedTorque = affine.stallTorqueCommanded;
        }
        else
        {
            // Dynamic kinetic slip phase: subtract kinetic friction in motion direction
            double sign = (freeVelocity > 0.0) ? 1.0 : -1.0;
            state.steerVelocity = freeVelocity - (sign * tauCoulomb / denominator);
            state.steerAppliedTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.steerVelocity);
        }

        // 6. Integrate azimuth angle
        state.steerAngle += state.steerVelocity * deltaTime;

        // Wrap to [-pi, +pi]
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

        // Current telemetry
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
        affine.stallTorqueCommanded = std::clamp(affine.stallTorqueCommanded, -maxStatorTorque, maxStatorTorque);

        if (driveCommand.mode == Control::MotorControlMode::Disabled &&
            driveCommand.neutralMode == Control::NeutralMode::Coast)
        {
            affine.electricalDamping = 0.0;
        }

        double jAxle = tireModel.GetParams().wheelAxleInertia +
            (driveParams.motor.GetRotorInertia() * driveParams.motor.GetGearRatio() * driveParams.motor.GetGearRatio());

        return tireModel.CalculateEffectiveTangentialStiffness(normalForce, jAxle, affine.electricalDamping,
            affine.stallTorqueCommanded, state.wheelVelocity, state.contactState.bristleDeflection, deltaTime);
    }

    glm::vec3 SwerveModuleModel::FinalizeDriveStep(const Control::MotorCommand& driveCommand, double busVoltage,
        double normalForce, const glm::vec3& groundPatchVelocity,
        const glm::vec3& surfaceNormal, double deltaTime)
    {
        if (deltaTime <= 1e-7)
        {
            return glm::vec3(0.0f);
        }

        // 1. Wheel local rolling and lateral axes in world NWU
        double cosA = std::cos(state.steerAngle);
        double sinA = std::sin(state.steerAngle);
        glm::vec3 wheelHeading(cosA, sinA, 0.0); // +X Forward, +Y Left

        // Orthogonalize rolling axis along terrain surface normal
        glm::vec3 eX = glm::normalize(wheelHeading - surfaceNormal * glm::dot(wheelHeading, surfaceNormal));
        glm::vec3 eY = glm::cross(surfaceNormal, eX);

        // 2. Project chassis contact velocity onto terrain tangent frame
        double patchUx = glm::dot(groundPatchVelocity, eX);
        double patchUy = glm::dot(groundPatchVelocity, eY);

        // 3. Advance wheel angular velocity from resolved drive motor torque
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
        affine.stallTorqueCommanded = std::clamp(affine.stallTorqueCommanded, -maxStatorTorque, maxStatorTorque);

        double jAxle = tireModel.GetParams().wheelAxleInertia +
            (driveParams.motor.GetRotorInertia() * driveParams.motor.GetGearRatio() * driveParams.motor.GetGearRatio());

        // Slip velocity at contact patch
        double r = tireModel.GetParams().wheelRadius;
        glm::vec2 slipVel(patchUx - (r * state.wheelVelocity), patchUy);

        // 4. Update friction forces and bristle state via radial return plasticity
        glm::vec2 fTangential = tireModel.UpdateFrictionAndState(normalForce, slipVel, deltaTime, state.contactState);

        // 5. Wheel angular acceleration: J_axle * dOmega/dt = tau_motor - r * Fx
        double motorTorque = affine.stallTorqueCommanded - (affine.electricalDamping * state.wheelVelocity);
        double netAxleTorque = motorTorque - (r * fTangential.x);

        state.driveAppliedTorque = netAxleTorque;
        state.wheelVelocity += (netAxleTorque / jAxle) * deltaTime;
        state.wheelPosition += state.wheelVelocity * deltaTime;
        state.linearVelocity = state.wheelVelocity * r;
        state.distanceTraveled += state.linearVelocity * deltaTime;

        state.driveStatorCurrent = driveParams.motor.CalculateCurrent(appliedVolts, state.wheelVelocity);

        // 6. Total 3D force delivered to chassis in world NWU coordinates
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