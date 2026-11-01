/**
 * @file ElevatorSystem.cpp
 * @brief Implementation of linear elevator semi-implicit actuation, Coulomb stick projection, and rigging.
 */

#include "Robot/Mechanisms/Systems/ElevatorSystem.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    using namespace Components;
    using Physics::Components::JointComponent;
    using Physics::Components::JointMotorMode;
    using Physics::Components::JointType;
    using Core::ECS::Core::invalidEntity;

    ElevatorSystem::ElevatorSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
        joltContext(jolt)
    {}

    ElevatorSystem::~ElevatorSystem()
    {}

    void ElevatorSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("ElevatorSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        auto entities = scene.Query<ElevatorComponent>();
        for (auto entity : entities)
        {
            auto* elevator = scene.GetComponent<ElevatorComponent>(entity);
            if (elevator)
            {
                UpdateElevator(*elevator, dt);
            }
        }
    }

    void ElevatorSystem::UpdateElevator(ElevatorComponent& elevator, double dt)
    {
        if (elevator.stages.empty())
        {
            return;
        }

        // 1. Multi-Motor Scaling & Reflected Linear Mass Computation
        Actuation::DCMotorModel motor = elevator.motorModel.WithMotorCount(elevator.motorCount);
        double rDrum = std::max(elevator.drumRadiusM, 1e-4);
        double gRatio = elevator.gearRatio;

        // m_reflected = N * J_rotor * (G / r_drum)^2
        elevator.reflectedLinearMassKg = elevator.motorCount * motor.GetRotorInertia() * (gRatio / rDrum) * (gRatio / rDrum);

        // 2. Lumped Effective Mass and Gravity Load Determination
        double lumpedStageMass = 0.0;
        double effectiveGravityLoadN = 0.0;

        if (elevator.riggingMode == ElevatorRiggingMode::Cascade)
        {
            // Cascade Rigging: M_eff = m_reflected + sum(k^2 * m_k), F_grav = sum(k * m_k * g)
            double effSum = 0.0;
            double gravSum = 0.0;
            for (size_t k = 0; k < elevator.stages.size(); ++k)
            {
                double multiplier = static_cast<double>(k + 1);
                effSum += multiplier * multiplier * elevator.stages[k].stageMassKg;
                gravSum += multiplier * elevator.stages[k].stageMassKg;
            }
            elevator.lumpedEffectiveMassKg = elevator.reflectedLinearMassKg + effSum;
            effectiveGravityLoadN = -gravSum * 9.80665;
            lumpedStageMass = gravSum;
        }
        else // SingleStage or Continuous
        {
            double stageMass = elevator.stages[0].stageMassKg;
            for (size_t k = 1; k < elevator.stages.size(); ++k)
            {
                stageMass += elevator.stages[k].stageMassKg;
            }
            elevator.lumpedEffectiveMassKg = elevator.reflectedLinearMassKg + stageMass;
            effectiveGravityLoadN = -stageMass * 9.80665;
            lumpedStageMass = stageMass;
        }

        elevator.lumpedEffectiveMassKg = std::max(elevator.lumpedEffectiveMassKg, 0.1);

        // 3. Read Current Kinematics of Primary Stage (Stage 1)
        auto* stage1Joint = scene.GetComponent<JointComponent>(elevator.stages[0].jointEntity);
        if (!stage1Joint)
        {
            return;
        }

        double x1 = stage1Joint->currentPosition;
        double v1 = stage1Joint->currentVelocity;
        elevator.stages[0].currentPositionM = x1;
        elevator.stages[0].currentVelocityMps = v1;

        // 4. Commanded Force and Back-EMF Linear Damping
        double appliedV = elevator.commandedVoltage;
        auto affine = motor.CalculateAffineTorqueTerms(appliedV);

        // F_cmd = tau_cmd / r_drum
        double fCmd = affine.stallTorqueCommanded / rDrum;

        // D_lin = (k_t * k_e * G^2 * eta) / (R * r_drum^2) = D_rot / r_drum^2
        double dLin = elevator.brakeMode ? (affine.electricalDamping / (rDrum * rDrum)) : 0.0;
        double dTotal = dLin + elevator.viscousDampingNspm;

        // 5. Semi-Implicit Linear Actuation Force Formulation:
        // F^{t+dt} = (F_cmd + F_spring - D_total * v1) / (1 + (D_total * dt) / M_eff)
        double denominator = 1.0 + ((dTotal * dt) / elevator.lumpedEffectiveMassKg);
        double fSemiImplicit = (fCmd + elevator.springPreloadForceN - (dTotal * v1)) / denominator;

        // 6. Stator Current Limit Clamp
        double maxStatorTorque = motor.CalculateStatorTorqueLimit(elevator.statorCurrentLimitA);
        double maxLinearForce = maxStatorTorque / rDrum;
        fSemiImplicit = std::clamp(fSemiImplicit, -maxLinearForce, maxLinearForce);

        // 7. 1D Coulomb Friction & Static Stick Projection (Zero-Creep Holding)
        double fCoulomb = elevator.coulombFrictionN;
        double fHoldingExt = fSemiImplicit + effectiveGravityLoadN;
        double fDelivered = 0.0;

        if (std::abs(appliedV) < 1e-3 && std::abs(v1) < 1e-3)
        {
            // Static stick regime: if net load is within Coulomb friction threshold, hold zero velocity exactly
            if (std::abs(fHoldingExt) <= fCoulomb)
            {
                fDelivered = -effectiveGravityLoadN; // Balance gravity load exactly
                v1 = 0.0;
                stage1Joint->currentVelocity = 0.0f;
            }
            else
            {
                // Breakaway: Coulomb friction opposes impending motion
                double sign = (fHoldingExt > 0.0) ? 1.0 : -1.0;
                fDelivered = fSemiImplicit - (fCoulomb * sign);
            }
        }
        else
        {
            // Dynamic sliding: Coulomb friction opposes instantaneous velocity direction
            double sign = (v1 > 1e-5) ? 1.0 : ((v1 < -1e-5) ? -1.0 : ((fHoldingExt > 0.0) ? 1.0 : -1.0));
            fDelivered = fSemiImplicit - (fCoulomb * sign);
        }

        elevator.appliedLinearForceN = fDelivered;
        elevator.appliedVoltage = appliedV;

        // 8. Deliver Force to Stage 1 Slider Constraint
        stage1Joint->appliedForce = static_cast<float>(fDelivered);
        stage1Joint->motorMode = JointMotorMode::Torque;

        // 9. Rigging Dispatch
        if (elevator.riggingMode == ElevatorRiggingMode::Cascade)
        {
            ProcessCascadeRigging(elevator, x1, v1);
        }
        else if (elevator.riggingMode == ElevatorRiggingMode::Continuous)
        {
            ProcessContinuousRigging(elevator, fDelivered);
        }
        else // SingleStage
        {
            elevator.totalCarriageHeightM = x1;
            elevator.totalCarriageVelocityMps = v1;
        }

        // 10. Electrical Telemetry Feedback
        double omegaMotor = (gRatio / rDrum) * v1;
        elevator.statorCurrentA = motor.CalculateCurrent(appliedV, omegaMotor);
    }

    void ElevatorSystem::ProcessCascadeRigging(ElevatorComponent& elevator, double x1, double v1)
    {
        // Cascade Rigging: each stage k relative to stage k-1 is locked to x1 (total carriage = N * x1)
        for (size_t k = 1; k < elevator.stages.size(); ++k)
        {
            auto* stageJoint = scene.GetComponent<JointComponent>(elevator.stages[k].jointEntity);
            if (stageJoint)
            {
                // Lock pose relative to stage k-1 to match Stage 1
                stageJoint->targetPosition = static_cast<float>(x1);
                stageJoint->motorMode = JointMotorMode::Position;
                stageJoint->springFrequency = 40.0f;
                stageJoint->springDamping = 1.0f;
                stageJoint->maxMotorForce = 50000.0f; // Stiff kinematic lock
            }

            elevator.stages[k].currentPositionM = x1;
            elevator.stages[k].currentVelocityMps = v1;
        }

        double stageCount = static_cast<double>(elevator.stages.size());
        elevator.totalCarriageHeightM = stageCount * x1;
        elevator.totalCarriageVelocityMps = stageCount * v1;
    }

    void ElevatorSystem::ProcessContinuousRigging(ElevatorComponent& elevator, double netLinearForce)
    {
        double upwardTension = std::max(0.0, netLinearForce);
        double totalHeight = 0.0;
        double totalVel = 0.0;

        // In continuous rigging, cable tension acts sequentially on the lowest un-extended stage
        bool tensionConsumed = false;

        for (size_t k = 0; k < elevator.stages.size(); ++k)
        {
            auto* stageJoint = scene.GetComponent<JointComponent>(elevator.stages[k].jointEntity);
            if (!stageJoint)
            {
                continue;
            }

            double stagePos = stageJoint->currentPosition;
            double stageLimit = elevator.stages[k].maxTravelM;

            if (!tensionConsumed && (stageLimit <= 0.0 || stagePos < stageLimit - 0.002))
            {
                // This stage is actively lifting under cable tension
                stageJoint->appliedForce = static_cast<float>(upwardTension);
                stageJoint->motorMode = JointMotorMode::Torque;
                tensionConsumed = true;
            }
            else
            {
                // Stage is either fully extended or waiting its turn
                stageJoint->appliedForce = 0.0f;
                stageJoint->motorMode = JointMotorMode::Off;
            }

            elevator.stages[k].currentPositionM = stagePos;
            elevator.stages[k].currentVelocityMps = stageJoint->currentVelocity;
            totalHeight += stagePos;
            totalVel += stageJoint->currentVelocity;
        }

        elevator.totalCarriageHeightM = totalHeight;
        elevator.totalCarriageVelocityMps = totalVel;
    }
}