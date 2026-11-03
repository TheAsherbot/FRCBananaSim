/**
 * @file DifferentialComponent.h
 * @brief Pure data component representing a generic 2-DOF coupled differential mechanism.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Actuation/DCMotorModel.h"
#include <string>
#include <vector>
#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @enum DifferentialType
     * @brief Kinematic configuration topology for differential coupling.
     */
    enum class DifferentialType : uint8_t
    {
        SymmetricBevel = 0, ///< Symmetric pitch/roll or yaw/pitch bevel gear cluster.
        CoreXY = 1,         ///< 2D Cartesian timing belt differential gantry.
        CoupledArm = 2,     ///< Base-driven concentric belt/chain 2-link arm.
        CustomMatrix = 3    ///< Arbitrary 2x2 forward and reverse coupling matrices.
    };

    /**
     * @struct DifferentialComponent
     * @brief Pure ECS data component storing 2-DOF differential transmission matrices, motor models, and state.
     */
    struct DifferentialComponent
    {
        std::string name;
        DifferentialType differentialType = DifferentialType::SymmetricBevel;

        // Kinematic Hierarchy Joint Entities
        Core::ECS::Core::Entity jointEntityA = Core::ECS::Core::invalidEntity; ///< Joint 1 (e.g. Pitch hinge).
        Core::ECS::Core::Entity jointEntityB = Core::ECS::Core::invalidEntity; ///< Joint 2 (e.g. Roll hinge).

        // Kinematic Hierarchy Link Entities (for inertia and mass inspection)
        Core::ECS::Core::Entity linkEntityA = Core::ECS::Core::invalidEntity;  ///< Link 1 rigid body.
        Core::ECS::Core::Entity linkEntityB = Core::ECS::Core::invalidEntity;  ///< Link 2 rigid body.

        // Actuation Motor Models
        Actuation::DCMotorModel motorModelA;
        Actuation::DCMotorModel motorModelB;
        uint32_t motorCountA = 1;
        uint32_t motorCountB = 1;

        // Kinematic Transformation Matrices
        glm::dmat2 t_mq = glm::dmat2(1.0); ///< Maps joint velocity [dq1, dq2] -> motor velocity [w_mA, w_mB].
        glm::dmat2 t_qm = glm::dmat2(1.0); ///< Inverse: maps motor velocity [w_mA, w_mB] -> joint velocity [dq1, dq2].

        // Gearing & Transmission Parameters
        double efficiency = 0.95;
        glm::dvec2 coulombFrictionNm = { 0.0, 0.0 }; ///< Joint-space Coulomb friction [tau_c1, tau_c2] (N*m).
        glm::dvec2 viscousDampingNms = { 0.0, 0.0 };  ///< Joint-space viscous damping [b_1, b_2] (N*m*s/rad).
        double statorCurrentLimitA = 60.0;            ///< Motor A stator current limit (A).
        double statorCurrentLimitB = 60.0;            ///< Motor B stator current limit (A).

        // Reflected & Effective Inertia Matrices
        glm::dmat2 reflectedInertia = glm::dmat2(0.0); ///< T_mq^T * J_rotor * T_mq (kg*m^2).
        glm::dmat2 effectiveInertia = glm::dmat2(0.0); ///< Total lumped joint inertia matrix (kg*m^2).

        // Runtime Command Demands (from SimInputs or UI)
        double commandedVoltageA = 0.0; ///< Motor A voltage in Volts (-12.0V to +12.0V).
        double commandedVoltageB = 0.0; ///< Motor B voltage in Volts (-12.0V to +12.0V).
        bool brakeModeA = true;         ///< Motor A electrical back-EMF damping active.
        bool brakeModeB = true;         ///< Motor B electrical back-EMF damping active.

        // Runtime Joint-Space State Tracking
        glm::dvec2 jointPositionsRad = { 0.0, 0.0 };       ///< Current joint angles [q1, q2] (rad).
        glm::dvec2 jointVelocitiesRadS = { 0.0, 0.0 };     ///< Current joint angular speeds [dq1, dq2] (rad/s).
        glm::dvec2 deliveredJointTorquesNm = { 0.0, 0.0 }; ///< Delivered actuation torques [tau_q1, tau_q2] (N*m).

        // Runtime Motor-Space State Tracking
        glm::dvec2 motorRotorAnglesRad = { 0.0, 0.0 };     ///< Ungeared rotor angles [theta_mA, theta_mB] (rad).
        glm::dvec2 motorRotorSpeedsRadS = { 0.0, 0.0 };    ///< Ungeared rotor speeds [omega_mA, omega_mB] (rad/s).
        glm::dvec2 motorStatorCurrentsA = { 0.0, 0.0 };    ///< Instantaneous stator currents [I_mA, I_mB] (A).

        // Control Slot Allocations in SimInputs / SimOutputs
        uint32_t motorSlotA = 8; ///< Motor A slot index.
        uint32_t motorSlotB = 9; ///< Motor B slot index.
    };
}