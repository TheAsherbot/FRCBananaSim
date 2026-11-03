/**
 * @file MechanismSystem.cpp
 * @brief Implementation of articulated arm semi-implicit torques, four-bar loop closure, and power couplings.
 */

#include "Robot/Mechanisms/Systems/MechanismSystem.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Core/ECS/Core/Scene.h"

#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    using namespace Components;
    using Physics::Components::JointComponent;
    using Physics::Components::JointMotorMode;
    using Physics::Components::RigidBodyComponent;
    using Core::ECS::Components::WorldTransformComponent;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Core::invalidEntity;

    MechanismSystem::MechanismSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
        joltContext(jolt)
    {}

    MechanismSystem::~MechanismSystem()
    {}

    void MechanismSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("MechanismSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        // 1. Articulated Arm Updates
        auto armEntities = scene.Query<ArmComponent>();
        for (auto entity : armEntities)
        {
            auto* arm = scene.GetComponent<ArmComponent>(entity);
            if (arm)
            {
                UpdateArm(*arm, dt);
            }
        }

        // 2. Closed-Loop Four-Bar Linkage Tracking
        auto fourBarEntities = scene.Query<FourBarComponent>();
        for (auto entity : fourBarEntities)
        {
            auto* fourBar = scene.GetComponent<FourBarComponent>(entity);
            if (fourBar)
            {
                UpdateFourBar(*fourBar);
            }
        }

        // 3. Power Coupling Transmission Dispatch
        auto couplingEntities = scene.Query<CouplingComponent>();
        for (auto entity : couplingEntities)
        {
            auto* coupling = scene.GetComponent<CouplingComponent>(entity);
            if (coupling)
            {
                UpdateCoupling(*coupling, dt);
            }
        }
    }

    void MechanismSystem::UpdateArm(ArmComponent& arm, double dt)
    {
        auto* jointComp = scene.GetComponent<JointComponent>(arm.jointEntity);
        if (!jointComp)
        {
            return;
        }

        // 1. Motor Scaling & Reflected Rotor Inertia
        Actuation::DCMotorModel baseMotor = arm.motorModel;
        Actuation::DCMotorModel motor(
            baseMotor.GetNominalVoltage(),
            baseMotor.GetStallTorque(),
            baseMotor.GetStallCurrent(),
            baseMotor.GetFreeSpeed(),
            baseMotor.GetFreeCurrent(),
            baseMotor.GetRotorInertia(),
            arm.gearRatio,
            arm.efficiency
        );
        motor = motor.WithMotorCount(arm.motorCount);

        double gRatio = arm.gearRatio;
        arm.reflectedRotorInertiaKgm2 = motor.GetRotorInertia() * gRatio * gRatio;

        // If lumped effective inertia has not been set, estimate from link mass and COM
        if (arm.lumpedEffectiveInertiaKgm2 <= 1e-4)
        {
            double linkInertia = arm.linkMassKg * arm.centerOfMassDistanceM * arm.centerOfMassDistanceM + (0.05 * arm.linkMassKg);
            arm.lumpedEffectiveInertiaKgm2 = arm.reflectedRotorInertiaKgm2 + linkInertia;
        }
        arm.lumpedEffectiveInertiaKgm2 = std::max(arm.lumpedEffectiveInertiaKgm2, 0.005);

        // 2. Read Live Angular Kinematics
        double theta = jointComp->currentAngle;
        double omega = jointComp->currentVelocity;
        arm.currentAngleRad = theta;
        arm.currentVelocityRadPerSec = omega;

        // 3. Commanded Torque & Electrical Back-EMF Damping
        double appliedV = arm.commandedVoltage;
        auto affine = motor.CalculateAffineTorqueTerms(appliedV);
        double tauCmd = affine.stallTorqueCommanded;

        double dRot = arm.brakeMode ? affine.electricalDamping : 0.0;
        double dTotal = dRot + arm.viscousDampingNms;

        // 4. Counterbalance Torque
        double tauSpring = 0.0;
        switch (arm.springType)
        {
        case ArmSpringType::ConstantTorque:
            tauSpring = arm.springPreloadTorqueNm;
            break;
        case ArmSpringType::TorsionalSpring:
            tauSpring = arm.springPreloadTorqueNm - arm.torsionalSpringRateNmPerRad * (theta - arm.springRestAngleRad);
            break;
        case ArmSpringType::GasStrut:
            tauSpring = arm.springPreloadTorqueNm * std::cos(theta - arm.springRestAngleRad);
            break;
        default:
            break;
        }

        // 5. Gravity Torque Calculation
        double tauGrav = 0.0;
        if (arm.linkMassKg > 0.0 && arm.centerOfMassDistanceM > 0.0)
        {
            // Vertical plane gravity torque: tau_g = -m * g * L_cg * cos(theta)
            tauGrav = -arm.linkMassKg * 9.80665 * arm.centerOfMassDistanceM * std::cos(theta);
        }

        // 6. Semi-Implicit Rotational Torque Formulation:
        // tau^{t+dt} = (tau_cmd + tau_spring + tau_grav - D_total * omega) / (1 + (D_total * dt) / J_eff) - tau_grav
        bool joltGravActive = true;
        if (arm.linkEntity != invalidEntity)
        {
            auto* armRb = scene.GetComponent<RigidBodyComponent>(arm.linkEntity);
            if (armRb && armRb->gravityFactor == 0.0f)
            {
                joltGravActive = false;
            }
        }
        double joltGravTerm = joltGravActive ? tauGrav : 0.0;

        double denominator = 1.0 + ((dTotal * dt) / arm.lumpedEffectiveInertiaKgm2);
        double tauNetConstant = tauCmd + tauSpring + tauGrav;
        double tauSemi = (tauNetConstant - (dTotal * omega)) / denominator - joltGravTerm;

        // 7. Stator Current Limit Clamp
        double maxStatorTorque = motor.CalculateStatorTorqueLimit(arm.statorCurrentLimitA);
        tauSemi = std::clamp(tauSemi, -maxStatorTorque, maxStatorTorque);

        // 8. 1D Coulomb Friction & Static Stick Latch
        double tauCoulomb = arm.coulombFrictionNm;
        double tauHolding = tauSemi + joltGravTerm;
        double tauDelivered = 0.0;
        double dOmegaCoulomb = (arm.lumpedEffectiveInertiaKgm2 > 1e-6) ? ((tauCoulomb / arm.lumpedEffectiveInertiaKgm2) * dt) : 0.0;
        double omegaCapture = std::max(1e-3, dOmegaCoulomb);

        if (std::abs(appliedV) < 1e-3 && (arm.isStickLocked || std::abs(omega) <= omegaCapture))
        {
            if (std::abs(tauNetConstant) <= tauCoulomb)
            {
                // Static Stick Regime: Latch position once to eliminate floating walk
                if (!arm.isStickLocked)
                {
                    arm.isStickLocked = true;
                    arm.lockedAngleRad = std::clamp(theta, arm.minAngleRad, arm.maxAngleRad);
                }

                tauDelivered = -joltGravTerm; // Balance static gravity exactly
                omega = 0.0;

                jointComp->targetAngle = static_cast<float>(arm.lockedAngleRad);
                jointComp->targetPosition = jointComp->targetAngle;
                jointComp->motorMode = JointMotorMode::Position;
                jointComp->springFrequency = 100.0f;
                jointComp->springDamping = 1.0f;
                jointComp->maxMotorTorque = 50000.0f;
                jointComp->appliedTorque = static_cast<float>(tauDelivered);
                jointComp->currentVelocity = 0.0f;
            }
            else
            {
                arm.isStickLocked = false;
                double sign = (tauHolding > 0.0) ? 1.0 : -1.0;
                tauDelivered = tauSemi - (tauCoulomb * sign);

                jointComp->appliedTorque = static_cast<float>(tauDelivered);
                jointComp->motorMode = JointMotorMode::Torque;
            }
        }
        else
        {
            arm.isStickLocked = false;
            double sign = (omega > 1e-4) ? 1.0 : ((omega < -1e-4) ? -1.0 : ((tauHolding > 0.0) ? 1.0 : -1.0));
            double maxArrestTorque = (std::abs(omega) > 1e-4 && dt > 1e-9) ? ((std::abs(omega) * arm.lumpedEffectiveInertiaKgm2) / dt) : tauCoulomb;
            double tauFriction = std::min(tauCoulomb, maxArrestTorque);
            tauDelivered = tauSemi - (tauFriction * sign);

            jointComp->appliedTorque = static_cast<float>(tauDelivered);
            jointComp->motorMode = JointMotorMode::Torque;
        }

        arm.appliedTorqueNm = tauDelivered;
        arm.appliedVoltage = appliedV;

        // 9. Electrical Telemetry Feedback
        double rawCurrent = motor.CalculateCurrent(appliedV, omega);
        arm.statorCurrentA = std::clamp(rawCurrent, -arm.statorCurrentLimitA, arm.statorCurrentLimitA);
    }

    void MechanismSystem::UpdateFourBar(FourBarComponent& fourBar)
    {
        if (fourBar.crankJoint != invalidEntity && fourBar.followerJoint != invalidEntity)
        {
            auto* crankJc = scene.GetComponent<JointComponent>(fourBar.crankJoint);
            auto* followerJc = scene.GetComponent<JointComponent>(fourBar.followerJoint);
            if (crankJc && followerJc)
            {
                if (crankJc->hasLimits && crankJc->motorMode == JointMotorMode::Torque && std::abs(crankJc->currentVelocity) < 0.25f)
                {
                    if (crankJc->currentAngle <= crankJc->minLimit + 0.01f && crankJc->appliedTorque < 0.0f)
                    {
                        crankJc->targetAngle = crankJc->minLimit;
                        crankJc->targetPosition = crankJc->minLimit;
                        crankJc->motorMode = JointMotorMode::Position;
                        crankJc->springFrequency = 100.0f;
                        crankJc->springDamping = 1.0f;
                        crankJc->maxMotorTorque = 50000.0f;
                        crankJc->appliedTorque = 0.0f;
                    }
                    else if (crankJc->currentAngle >= crankJc->maxLimit - 0.01f && crankJc->appliedTorque > 0.0f)
                    {
                        crankJc->targetAngle = crankJc->maxLimit;
                        crankJc->targetPosition = crankJc->maxLimit;
                        crankJc->motorMode = JointMotorMode::Position;
                        crankJc->springFrequency = 100.0f;
                        crankJc->springDamping = 1.0f;
                        crankJc->maxMotorTorque = 50000.0f;
                        crankJc->appliedTorque = 0.0f;
                    }
                }

                if (crankJc->motorMode == JointMotorMode::Position)
                {
                    followerJc->targetAngle = crankJc->targetAngle;
                    followerJc->targetPosition = crankJc->targetAngle;
                    followerJc->motorMode = JointMotorMode::Position;
                    followerJc->springFrequency = 100.0f;
                    followerJc->springDamping = 1.0f;
                    followerJc->maxMotorTorque = 50000.0f;
                    followerJc->appliedTorque = 0.0f;

                    if (followerJc->isCreated && followerJc->constraint)
                    {
                        auto* followerHinge = static_cast<JPH::HingeConstraint*>(followerJc->constraint.GetPtr());
                        followerHinge->SetMotorState(JPH::EMotorState::Position);
                        followerHinge->SetTargetAngle(followerJc->targetAngle);
                        followerHinge->GetMotorSettings().SetTorqueLimit(followerJc->maxMotorTorque);
                        followerHinge->GetMotorSettings().mSpringSettings.mFrequency = followerJc->springFrequency;
                        followerHinge->GetMotorSettings().mSpringSettings.mDamping = followerJc->springDamping;
                    }
                }
                else
                {
                    float angleErr = crankJc->currentAngle - followerJc->currentAngle;
                    float velErr = crankJc->currentVelocity - followerJc->currentVelocity;
                    followerJc->motorMode = JointMotorMode::Torque;
                    followerJc->appliedTorque = (crankJc->appliedTorque * 0.8f) + (angleErr * 150.0f) + (velErr * 8.0f);

                    if (followerJc->isCreated && followerJc->constraint)
                    {
                        auto* followerHinge = static_cast<JPH::HingeConstraint*>(followerJc->constraint.GetPtr());
                        followerHinge->SetMotorState(JPH::EMotorState::Off);
                    }
                }
            }
        }

        if (fourBar.closureJoint == invalidEntity)
        {
            return;
        }

        auto* jointComp = scene.GetComponent<JointComponent>(fourBar.closureJoint);
        if (!jointComp)
        {
            return;
        }

        // Measure live distance between loop closure anchors
        glm::vec3 worldPoint1{ 0.0f };
        glm::vec3 worldPoint2{ 0.0f };

        if (joltContext && joltContext->IsInitialized())
        {
            JPH::BodyInterface& bi = joltContext->GetBodyInterface();

            if (fourBar.followerEntity != invalidEntity)
            {
                auto* rb1 = scene.GetComponent<RigidBodyComponent>(fourBar.followerEntity);
                if (rb1 && !rb1->bodyID.IsInvalid())
                {
                    JPH::RMat44 com1 = bi.GetCenterOfMassTransform(rb1->bodyID);
                    JPH::RVec3 p1 = com1 * JPH::Vec3(fourBar.closureParentAnchor.x, fourBar.closureParentAnchor.y, fourBar.closureParentAnchor.z);
                    worldPoint1 = glm::vec3(static_cast<float>(p1.GetX()), static_cast<float>(p1.GetY()), static_cast<float>(p1.GetZ()));
                }
            }

            if (fourBar.couplerEntity != invalidEntity)
            {
                auto* rb2 = scene.GetComponent<RigidBodyComponent>(fourBar.couplerEntity);
                if (rb2 && !rb2->bodyID.IsInvalid())
                {
                    JPH::RMat44 com2 = bi.GetCenterOfMassTransform(rb2->bodyID);
                    JPH::RVec3 p2 = com2 * JPH::Vec3(fourBar.closureChildAnchor.x, fourBar.closureChildAnchor.y, fourBar.closureChildAnchor.z);
                    worldPoint2 = glm::vec3(static_cast<float>(p2.GetX()), static_cast<float>(p2.GetY()), static_cast<float>(p2.GetZ()));
                }
            }
        }
        else
        {
            auto* wt1 = scene.GetComponent<WorldTransformComponent>(fourBar.followerEntity);
            auto* wt2 = scene.GetComponent<WorldTransformComponent>(fourBar.couplerEntity);
            if (wt1 && wt2)
            {
                worldPoint1 = wt1->worldPosition + wt1->worldRotation * fourBar.closureParentAnchor;
                worldPoint2 = wt2->worldPosition + wt2->worldRotation * fourBar.closureChildAnchor;
            }
        }

        fourBar.loopClosureErrorM = glm::length(worldPoint1 - worldPoint2);
    }

    void MechanismSystem::UpdateCoupling(CouplingComponent& coupling, double dt)
    {
        if (coupling.masterJoint == invalidEntity || coupling.slaveJoint == invalidEntity)
        {
            return;
        }

        auto* masterJointComp = scene.GetComponent<JointComponent>(coupling.masterJoint);
        auto* slaveJointComp = scene.GetComponent<JointComponent>(coupling.slaveJoint);
        if (!masterJointComp || !slaveJointComp)
        {
            return;
        }

        // If the slave joint has its own dedicated motorized ArmComponent (e.g., an independently
        // actuated wrist motor), do not fight the motor with a rigid coupling spring.
        auto armEntities = scene.Query<ArmComponent>();
        for (auto armEntity : armEntities)
        {
            auto* armComp = scene.GetComponent<ArmComponent>(armEntity);
            if (armComp && armComp->jointEntity == coupling.slaveJoint)
            {
                coupling.transmittedTorque = 0.0;
                return;
            }
        }

        if (masterJointComp->motorMode == JointMotorMode::Position &&
            slaveJointComp->motorMode == JointMotorMode::Position)
        {
            coupling.transmittedTorque = 0.0;
            return;
        }

        double thetaMaster = masterJointComp->currentAngle;
        double omegaMaster = masterJointComp->currentVelocity;

        double thetaSlave = slaveJointComp->currentAngle;
        double omegaSlave = slaveJointComp->currentVelocity;

        double s = coupling.inverted ? -1.0 : 1.0;
        double targetThetaSlave = s * coupling.ratio * thetaMaster;
        double targetOmegaSlave = s * coupling.ratio * omegaMaster;

        double error = targetThetaSlave - thetaSlave;

        // Apply deadband play window for mechanical backlash
        double effectiveError = 0.0;
        double halfBacklash = 0.5 * std::max(0.0, coupling.backlashRad);

        if (std::abs(error) > halfBacklash)
        {
            effectiveError = (error > 0.0) ? (error - halfBacklash) : (error + halfBacklash);
        }

        // Estimate slave link inertia for numerical stability
        double slaveInertia = 0.05;
        auto* slaveRb = scene.GetComponent<RigidBodyComponent>(slaveJointComp->childEntity);
        if (slaveRb && slaveRb->mass > 0.0f)
        {
            slaveInertia = std::max(0.01, static_cast<double>(slaveRb->mass) * 0.05);
        }

        // Critically damped coupling with stability limit for fixed dt
        double maxOmegaN = 0.5 / dt; // Stability limit for explicit/semi-implicit stepping
        double targetOmegaN = std::min(40.0, maxOmegaN);
        double kStiff = std::min(coupling.couplingStiffness, slaveInertia * targetOmegaN * targetOmegaN);
        double cDamp = std::min(coupling.couplingDamping, 2.0 * slaveInertia * targetOmegaN);

        // Semi-implicit penalty torque formulation to prevent forward Euler divergence
        double denominator = 1.0 + ((cDamp * dt) / slaveInertia);
        double tauCouple = ((kStiff * effectiveError) + cDamp * (targetOmegaSlave - omegaSlave)) / denominator;

        // Clamp transmission torque to prevent physical explosion
        double maxTorqueClamp = 150.0; // N*m
        tauCouple = std::clamp(tauCouple, -maxTorqueClamp, maxTorqueClamp);

        // Deliver torque to slave joint
        if (slaveJointComp->motorMode == JointMotorMode::Off)
        {
            slaveJointComp->motorMode = JointMotorMode::Torque;
            slaveJointComp->appliedTorque = 0.0f;
        }
        slaveJointComp->appliedTorque += static_cast<float>(tauCouple);

        // Apply Newton's third law reaction torque to master joint with transmission efficiency
        double eff = std::clamp(coupling.efficiency, 0.1, 1.0);
        double tauReaction = -s * (1.0 / (coupling.ratio * eff)) * tauCouple;

        if (masterJointComp->motorMode == JointMotorMode::Off)
        {
            masterJointComp->motorMode = JointMotorMode::Torque;
            masterJointComp->appliedTorque = 0.0f;
        }
        masterJointComp->appliedTorque += static_cast<float>(tauReaction);

        coupling.transmittedTorque = tauCouple;
    }
}