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
        double denominator = 1.0 + ((dTotal * dt) / arm.lumpedEffectiveInertiaKgm2);
        double tauNetConstant = tauCmd + tauSpring + tauGrav;
        double tauSemi = (tauNetConstant - (dTotal * omega)) / denominator - tauGrav;

        // 7. Stator Current Limit Clamp
        double maxStatorTorque = motor.CalculateStatorTorqueLimit(arm.statorCurrentLimitA);
        tauSemi = std::clamp(tauSemi, -maxStatorTorque, maxStatorTorque);

        // 8. 1D Coulomb Friction & Static Stick Latch
        double tauCoulomb = arm.coulombFrictionNm;
        double tauHolding = tauSemi + tauGrav;
        double tauDelivered = 0.0;

        if (std::abs(appliedV) < 1e-3 && std::abs(omega) < 1e-3)
        {
            if (std::abs(tauHolding) <= tauCoulomb)
            {
                // Static Stick Regime: Latch position once to eliminate floating walk
                if (!arm.isStickLocked)
                {
                    arm.isStickLocked = true;
                    arm.lockedAngleRad = theta;
                }

                tauDelivered = -tauGrav; // Balance static gravity exactly
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
            tauDelivered = tauSemi - (tauCoulomb * sign);

            jointComp->appliedTorque = static_cast<float>(tauDelivered);
            jointComp->motorMode = JointMotorMode::Torque;
        }

        arm.appliedTorqueNm = tauDelivered;
        arm.appliedVoltage = appliedV;

        // 9. Electrical Telemetry Feedback
        arm.statorCurrentA = motor.CalculateCurrent(appliedV, omega);
    }

    void MechanismSystem::UpdateFourBar(FourBarComponent& fourBar)
    {
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

        // Bilateral penalty torque
        double tauCouple = (coupling.couplingStiffness * effectiveError) + (coupling.couplingDamping * (targetOmegaSlave - omegaSlave));

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