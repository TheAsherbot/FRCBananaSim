/**
 * @file DifferentialMechanismSystem.cpp
 * @brief Implementation of 2-DOF coupled semi-implicit differential dynamics and Jolt constraint velocity delivery.
 */

#include "Robot/Mechanisms/Systems/DifferentialMechanismSystem.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
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

    DifferentialMechanismSystem::DifferentialMechanismSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
        joltContext(jolt)
    {}

    DifferentialMechanismSystem::~DifferentialMechanismSystem()
    {}

    void DifferentialMechanismSystem::ConfigureSymmetricBevel(DifferentialComponent& diff,
        double gearRatioPitch, double gearRatioRoll, bool invertRoll)
    {
        diff.differentialType = DifferentialType::SymmetricBevel;

        double gp = std::max(1e-4, gearRatioPitch);
        double gr = std::max(1e-4, gearRatioRoll);
        double rollSign = invertRoll ? -1.0 : 1.0;

        // T_mq: [omega_mA, omega_mB]^T = T_mq * [dq_pitch, dq_roll]^T
        diff.t_mq[0][0] = gp;
        diff.t_mq[1][0] = rollSign * gr;
        diff.t_mq[0][1] = gp;
        diff.t_mq[1][1] = -rollSign * gr;

        // Inverse T_qm = T_mq^-1
        diff.t_qm[0][0] = 0.5 / gp;
        diff.t_qm[1][0] = 0.5 / gp;
        diff.t_qm[0][1] = 0.5 * rollSign / gr;
        diff.t_qm[1][1] = -0.5 * rollSign / gr;

        RecalculateReflectedInertia(diff);
    }

    void DifferentialMechanismSystem::ConfigureCustomMatrix(DifferentialComponent& diff,
        const glm::dmat2& matrixMq)
    {
        diff.differentialType = DifferentialType::CustomMatrix;
        diff.t_mq = matrixMq;

        double det = (diff.t_mq[0][0] * diff.t_mq[1][1]) - (diff.t_mq[0][1] * diff.t_mq[1][0]);
        if (std::abs(det) > 1e-9)
        {
            diff.t_qm = (1.0 / det) * glm::dmat2(
                diff.t_mq[1][1], -diff.t_mq[0][1],
                -diff.t_mq[1][0], diff.t_mq[0][0]
            );
        }
        else
        {
            diff.t_qm = glm::dmat2(1.0);
        }

        RecalculateReflectedInertia(diff);
    }

    void DifferentialMechanismSystem::RecalculateReflectedInertia(DifferentialComponent& diff)
    {
        double jRotorA = diff.motorModelA.GetRotorInertia() * diff.motorCountA;
        double jRotorB = diff.motorModelB.GetRotorInertia() * diff.motorCountB;

        glm::dmat2 jRotorMat = glm::dmat2(jRotorA, 0.0, 0.0, jRotorB);

        // M_reflected = T_mq^T * J_rotor * T_mq
        diff.reflectedInertia = glm::transpose(diff.t_mq) * jRotorMat * diff.t_mq;

        // Force effectiveInertia to rebuild with new reflected inertia on next FixedUpdate
        diff.effectiveInertia = glm::dmat2(0.0);
    }

    void DifferentialMechanismSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("DifferentialMechanismSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        auto entities = scene.Query<DifferentialComponent>();
        for (auto entity : entities)
        {
            auto* diff = scene.GetComponent<DifferentialComponent>(entity);
            if (diff)
            {
                UpdateDifferential(*diff, dt);
            }
        }
    }

    void DifferentialMechanismSystem::UpdateDifferential(DifferentialComponent& diff, double dt)
    {
        SIM_PROFILE_SCOPE("DifferentialMechanismSystem::UpdateDifferential");

        auto* jointCompA = scene.GetComponent<JointComponent>(diff.jointEntityA);
        auto* jointCompB = scene.GetComponent<JointComponent>(diff.jointEntityB);
        if (!jointCompA || !jointCompB)
        {
            return;
        }

        // 1. Read Current Physical Joint Kinematics from Jolt Constraints
        diff.jointPositionsRad = { jointCompA->currentAngle, jointCompB->currentAngle };
        diff.jointVelocitiesRadS = { jointCompA->currentVelocity, jointCompB->currentVelocity };

        // 2. Map Joint Kinematics to Motor Shaft Space: omega_m = T_mq * dq, theta_m = T_mq * q
        diff.motorRotorSpeedsRadS = diff.t_mq * diff.jointVelocitiesRadS;
        diff.motorRotorAnglesRad = diff.t_mq * diff.jointPositionsRad;

        // 3. Ensure Lumped Effective Inertia Matrix is Initialized
        if (std::abs(diff.effectiveInertia[0][0]) < 1e-6)
        {
            RecalculateReflectedInertia(diff);

            double linkInertiaA = 0.05;
            double linkInertiaB = 0.02;

            auto* rbA = scene.GetComponent<RigidBodyComponent>(diff.linkEntityA);
            if (rbA && rbA->mass > 0.0f)
            {
                linkInertiaA = std::max(0.01, static_cast<double>(rbA->mass) * 0.03);
            }

            auto* rbB = scene.GetComponent<RigidBodyComponent>(diff.linkEntityB);
            if (rbB && rbB->mass > 0.0f)
            {
                linkInertiaB = std::max(0.005, static_cast<double>(rbB->mass) * 0.02);
            }

            diff.effectiveInertia = diff.reflectedInertia;
            diff.effectiveInertia[0][0] += linkInertiaA;
            diff.effectiveInertia[1][1] += linkInertiaB;
        }

        // 4. Evaluate Motor Commanded Torques, Stator Limits & Electrical Damping
        auto affineA = diff.motorModelA.CalculateAffineTorqueTerms(diff.commandedVoltageA);
        auto affineB = diff.motorModelB.CalculateAffineTorqueTerms(diff.commandedVoltageB);

        double maxStatorTorqueA = diff.motorModelA.CalculateStatorTorqueLimit(diff.statorCurrentLimitA) * diff.motorCountA;
        double maxStatorTorqueB = diff.motorModelB.CalculateStatorTorqueLimit(diff.statorCurrentLimitB) * diff.motorCountB;

        glm::dvec2 tauCmdMotor = {
            std::clamp(affineA.stallTorqueCommanded * diff.motorCountA, -maxStatorTorqueA, maxStatorTorqueA),
            std::clamp(affineB.stallTorqueCommanded * diff.motorCountB, -maxStatorTorqueB, maxStatorTorqueB)
        };

        glm::dvec2 dMotor = {
            (diff.brakeModeA ? affineA.electricalDamping : 0.0) * diff.motorCountA,
            (diff.brakeModeB ? affineB.electricalDamping : 0.0) * diff.motorCountB
        };
        glm::dmat2 dMotorMat = glm::dmat2(dMotor.x, 0.0, 0.0, dMotor.y);

        // 5. Project Damping & Commanded Torques to Joint Space: D_q = T_mq^T * D_m * T_mq
        glm::dmat2 dJoint = glm::transpose(diff.t_mq) * dMotorMat * diff.t_mq;
        dJoint[0][0] += std::max(0.01, diff.viscousDampingNms.x);
        dJoint[1][1] += std::max(0.01, diff.viscousDampingNms.y);

        double eff = std::clamp(diff.efficiency, 0.1, 1.0);
        glm::dvec2 tauCmdJoint = glm::transpose(diff.t_mq) * (tauCmdMotor * eff);

        // 6. Analytic Gravity Torques
        glm::dvec2 tauGravJoint = CalculateAnalyticGravity(diff);

        // 7. Net Driving Torque & Coulomb Friction Clamping
        glm::dvec2 tauDrive = tauCmdJoint + tauGravJoint - (dJoint * diff.jointVelocitiesRadS);
        ApplyFrictionClamping(diff, tauDrive);

        // 8. Solve 2x2 Semi-Implicit System for Velocity Increment:
        // (M_eff + dt * D_q) * delta_dq = dt * tauDrive
        glm::dmat2 aMatrix = diff.effectiveInertia + (dt * dJoint);
        double detA = (aMatrix[0][0] * aMatrix[1][1]) - (aMatrix[0][1] * aMatrix[1][0]);
        if (std::abs(detA) < 1e-9)
        {
            return;
        }

        glm::dmat2 aInv = (1.0 / detA) * glm::dmat2(
            aMatrix[1][1], -aMatrix[0][1],
            -aMatrix[1][0], aMatrix[0][0]
        );

        glm::dvec2 deltaJointVel = aInv * (dt * tauDrive);
        glm::dvec2 nextJointVel = diff.jointVelocitiesRadS + deltaJointVel;

        // Static stiction deadband when uncommanded near zero speed
        if (std::abs(diff.commandedVoltageA) < 1e-3 && std::abs(diff.commandedVoltageB) < 1e-3)
        {
            if (std::abs(nextJointVel.x) < 1e-3 && std::abs(tauCmdJoint.x + tauGravJoint.x) <= diff.coulombFrictionNm.x)
            {
                nextJointVel.x = 0.0;
            }
            if (std::abs(nextJointVel.y) < 1e-3 && std::abs(tauCmdJoint.y + tauGravJoint.y) <= diff.coulombFrictionNm.y)
            {
                nextJointVel.y = 0.0;
            }
        }

        // Prevent driving further into hard stops when at joint limits
        if (jointCompA->hasLimits)
        {
            if (jointCompA->currentAngle <= jointCompA->minLimit && nextJointVel.x < 0.0)
            {
                nextJointVel.x = 0.0;
            }
            else if (jointCompA->currentAngle >= jointCompA->maxLimit && nextJointVel.x > 0.0)
            {
                nextJointVel.x = 0.0;
            }
        }

        if (jointCompB->hasLimits)
        {
            if (jointCompB->currentAngle <= jointCompB->minLimit && nextJointVel.y < 0.0)
            {
                nextJointVel.y = 0.0;
            }
            else if (jointCompB->currentAngle >= jointCompB->maxLimit && nextJointVel.y > 0.0)
            {
                nextJointVel.y = 0.0;
            }
        }

        // 9. Compute Delivered Joint Torque & Max Stator Torque Bounds for Telemetry and Jolt
        glm::dvec2 tauDelivered = (1.0 / dt) * (diff.effectiveInertia * (nextJointVel - diff.jointVelocitiesRadS));

        double maxJointTorquePitch = (std::abs(diff.t_mq[0][0]) * maxStatorTorqueA) + (std::abs(diff.t_mq[0][1]) * maxStatorTorqueB);
        double maxJointTorqueRoll = (std::abs(diff.t_mq[1][0]) * maxStatorTorqueA) + (std::abs(diff.t_mq[1][1]) * maxStatorTorqueB);
        maxJointTorquePitch = std::max(1.0, maxJointTorquePitch * eff);
        maxJointTorqueRoll = std::max(1.0, maxJointTorqueRoll * eff);

        tauDelivered.x = std::clamp(tauDelivered.x, -maxJointTorquePitch, maxJointTorquePitch);
        tauDelivered.y = std::clamp(tauDelivered.y, -maxJointTorqueRoll, maxJointTorqueRoll);
        diff.deliveredJointTorquesNm = tauDelivered;

        // 10. Drive Physical Jolt Constraints via Coupled Velocity Motor Targets
        jointCompA->motorMode = JointMotorMode::Velocity;
        jointCompA->targetVelocity = static_cast<float>(nextJointVel.x);
        jointCompA->maxMotorTorque = static_cast<float>(maxJointTorquePitch);
        jointCompA->appliedTorque = static_cast<float>(tauDelivered.x);

        jointCompB->motorMode = JointMotorMode::Velocity;
        jointCompB->targetVelocity = static_cast<float>(nextJointVel.y);
        jointCompB->maxMotorTorque = static_cast<float>(maxJointTorqueRoll);
        jointCompB->appliedTorque = static_cast<float>(tauDelivered.y);

        // 11. Stator Current Calculation (Clamped to Configured Stator Limits)
        double rawCurrentA = diff.motorModelA.CalculateCurrent(diff.commandedVoltageA, diff.motorRotorSpeedsRadS.x) * diff.motorCountA;
        double rawCurrentB = diff.motorModelB.CalculateCurrent(diff.commandedVoltageB, diff.motorRotorSpeedsRadS.y) * diff.motorCountB;
        diff.motorStatorCurrentsA.x = std::clamp(rawCurrentA, -diff.statorCurrentLimitA, diff.statorCurrentLimitA);
        diff.motorStatorCurrentsA.y = std::clamp(rawCurrentB, -diff.statorCurrentLimitB, diff.statorCurrentLimitB);
    }

    glm::dvec2 DifferentialMechanismSystem::CalculateAnalyticGravity(const DifferentialComponent& diff) const
    {
        double tauGravPitch = 0.0;
        double tauGravRoll = 0.0;

        auto* rbA = scene.GetComponent<RigidBodyComponent>(diff.linkEntityA);
        auto* rbB = scene.GetComponent<RigidBodyComponent>(diff.linkEntityB);

        bool gravityActive = false;
        if ((rbA && rbA->gravityFactor > 0.0f) || (rbB && rbB->gravityFactor > 0.0f))
        {
            gravityActive = true;
        }

        if (!gravityActive)
        {
            return { 0.0, 0.0 };
        }

        double massA = (rbA && rbA->mass > 0.0f) ? static_cast<double>(rbA->mass) : 0.8;
        double massB = (rbB && rbB->mass > 0.0f) ? static_cast<double>(rbB->mass) : 0.5;

        // Pitch axis vertical gravity: tau_g = -(m_A * L_A + m_B * L_B) * g * cos(theta_pitch)
        double comDistA = 0.08;
        double comDistB = 0.15;
        tauGravPitch = -((massA * comDistA) + (massB * comDistB)) * 9.80665 * std::cos(diff.jointPositionsRad.x);

        return { tauGravPitch, tauGravRoll };
    }

    void DifferentialMechanismSystem::ApplyFrictionClamping(const DifferentialComponent& diff, glm::dvec2& netTorques) const
    {
        // Joint 1 Coulomb Clamping
        double fc1 = diff.coulombFrictionNm.x;
        if (fc1 > 0.0)
        {
            if (std::abs(diff.jointVelocitiesRadS.x) < 1e-3 && std::abs(netTorques.x) <= fc1)
            {
                netTorques.x = 0.0;
            }
            else
            {
                double s1 = (diff.jointVelocitiesRadS.x > 1e-3) ? 1.0 : ((diff.jointVelocitiesRadS.x < -1e-3) ? -1.0 : (netTorques.x > 0.0 ? 1.0 : -1.0));
                netTorques.x -= s1 * fc1;
            }
        }

        // Joint 2 Coulomb Clamping
        double fc2 = diff.coulombFrictionNm.y;
        if (fc2 > 0.0)
        {
            if (std::abs(diff.jointVelocitiesRadS.y) < 1e-3 && std::abs(netTorques.y) <= fc2)
            {
                netTorques.y = 0.0;
            }
            else
            {
                double s2 = (diff.jointVelocitiesRadS.y > 1e-3) ? 1.0 : ((diff.jointVelocitiesRadS.y < -1e-3) ? -1.0 : (netTorques.y > 0.0 ? 1.0 : -1.0));
                netTorques.y -= s2 * fc2;
            }
        }
    }
}