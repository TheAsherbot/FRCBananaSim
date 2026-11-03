/**
 * @file CouplingComponent.h
 * @brief Component representing a unified mechanical power transmission coupling between two joints.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include <string>
#include <cstdint>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @enum TransmissionType
     * @brief Mechanical transmission hardware topology.
     */
    enum class TransmissionType : uint8_t
    {
        Gear = 0,             ///< Spur, helical, bevel , or planetary gear mesh.
        TimingBelt = 1,       ///< HTD or GT2 synchronous timing belt.
        RollerChain = 2,      ///< #25 or #35 roller chain drive.
        Cable = 3,            ///< Rigging wire rope or Dyneema cable loop.
        VirtualFourBar = 4    ///< 1:1 parallelogram virtual four-bar timing loop.
    };

    /**
     * @enum CouplingSolverMode
     * @brief Computational approach used to enforce transmission velocity and torque constraints.
     */
    enum class CouplingSolverMode : uint8_t
    {
        AnalyticCoupled = 0,  ///< Stiff bilateral spring-damper penalty torque applied symmetrically.
        JoltConstraint = 1    ///< Direct JPH::GearConstraint or physical constraint in Jolt solver.
    };

    /**
     * @struct CouplingComponent
     * @brief ECS component holding mechanical transmission parameters between two joint entities.
     */
    struct CouplingComponent
    {
        std::string name;
        TransmissionType transmissionType = TransmissionType::Gear;
        CouplingSolverMode solverMode = CouplingSolverMode::AnalyticCoupled;

        Core::ECS::Core::Entity masterJoint = Core::ECS::Core::invalidEntity; ///< Master/driver joint entity.
        Core::ECS::Core::Entity slaveJoint = Core::ECS::Core::invalidEntity;  ///< Slave/driven joint entity.

        double ratio = 1.0;          ///< Kinematic velocity ratio rho = omega_slave / omega_master.
        bool inverted = false;       ///< Direction multiplier s = inverted ? -1.0 : +1.0.
        double efficiency = 0.95;    ///< Mechanical efficiency eta (0.0 to 1.0).
        double backlashRad = 0.0;    ///< Mechanical backlash / deadband play (rad or meters).

        // Runtime penalty coupling parameters for AnalyticCoupled mode
        double couplingStiffness = 50000.0; ///< Bilateral coupling spring stiffness k_couple (N*m/rad or N/m).
        double couplingDamping = 1000.0;    ///< Bilateral coupling damping c_couple (N*m*s/rad or N*s/m).

        // Runtime state tracking
        double accumulatedBacklash = 0.0;   ///< Current position inside backlash window [-delta/2, +delta/2].
        double transmittedTorque = 0.0;     ///< Torque/force currently transmitted to slave joint (N*m or N).
    };
}