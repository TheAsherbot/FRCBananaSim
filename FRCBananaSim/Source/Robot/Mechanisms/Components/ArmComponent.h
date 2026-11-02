/**
 * @file ArmComponent.h
 * @brief Component representing a single or multi-jointed articulated arm with semi-implicit drive and gravity torque.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Actuation/DCMotorModel.h"
#include <string>
#include <cstdint>
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @enum ArmSpringType
     * @brief Mechanical counterbalance assist mechanism topology.
     */
    enum class ArmSpringType : uint8_t
    {
        None = 0,             ///< No mechanical counterbalance.
        ConstantTorque = 1,   ///< Constant-torque spring or gas strut assist.
        TorsionalSpring = 2,  ///< Linear torsional spring tau = -k * (theta - theta0).
        GasStrut = 3          ///< Non-linear gas strut linkage.
    };

    /**
     * @struct ArmComponent
     * @brief ECS component holding articulated arm motor parameters, reflected inertia, and counterbalance.
     */
    struct ArmComponent
    {
        std::string name;

        // Kinematic Hierarchy Links
        Core::ECS::Core::Entity jointEntity = Core::ECS::Core::invalidEntity;  ///< JointComponent entity (HingeConstraint).
        Core::ECS::Core::Entity linkEntity = Core::ECS::Core::invalidEntity;   ///< Dynamic link rigid body entity.

        // Actuation & Transmission
        Actuation::DCMotorModel motorModel;
        uint32_t motorCount = 1;
        double gearRatio = 1.0;                 ///< Overall reduction ratio G = omega_motor / omega_joint.
        double efficiency = 0.95;               ///< Transmission efficiency (0.0 to 1.0).

        // Counterbalance & Friction
        ArmSpringType springType = ArmSpringType::None;
        double springPreloadTorqueNm = 0.0;      ///< Preload torque in N*m.
        double torsionalSpringRateNmPerRad = 0.0;///< Torsional spring stiffness in N*m/rad.
        double springRestAngleRad = 0.0;         ///< Rest/equilibrium angle where spring torque is zero (rad).
        double coulombFrictionNm = 0.0;          ///< Static and dynamic Coulomb friction torque threshold (N*m).
        double viscousDampingNms = 0.0;          ///< Viscous friction damping coefficient (N*m*s/rad).
        double statorCurrentLimitA = 60.0;       ///< Stator current clamp limit in Amperes.

        // Reflected Rotor Inertia & Effective Inertia
        double reflectedRotorInertiaKgm2 = 0.0;  ///< J_reflected = N * J_rotor * G^2 (kg*m^2).
        double lumpedEffectiveInertiaKgm2 = 0.0; ///< J_effective = J_reflected + J_link (kg*m^2).

        // Link Mass & Gravity Geometry
        double linkMassKg = 0.0;                 ///< Mass of the articulating arm link (kg).
        double centerOfMassDistanceM = 0.0;      ///< Distance from hinge axis to link center of mass (m).
        glm::vec3 hingeAxis = { 0.0f, 0.0f, 1.0f }; ///< Hinge axis in local body frame.

        // Motion Limits
        double minAngleRad = -3.14159265;        ///< Lower hard-stop angle in radians.
        double maxAngleRad = 3.14159265;         ///< Upper hard-stop angle in radians.

        // Runtime Command Demands
        double commandedVoltage = 0.0;           ///< Commanded motor voltage in Volts (-12.0V to +12.0V).
        double appliedVoltage = 0.0;             ///< Actual voltage post-sag/limits.
        double appliedTorqueNm = 0.0;            ///< Delivered semi-implicit torque tau^{t+dt} (N*m).
        double statorCurrentA = 0.0;             ///< Instantaneous stator current (A).
        bool brakeMode = true;                   ///< Dynamic back-EMF electrical damping enabled when true.

        // Static Stick Projection Latch
        bool isStickLocked = false;              ///< True when arm is latched in 1D Coulomb static stick.
        double lockedAngleRad = 0.0;             ///< Coordinate captured upon entering stick regime.

        // Telemetry Readouts
        double currentAngleRad = 0.0;            ///< Current relative joint angle theta (rad).
        double currentVelocityRadPerSec = 0.0;   ///< Current relative joint velocity omega (rad/s).

        // Control Slot Mapping
        uint32_t motorSlot = 9;                  ///< SimInputs motor slot index [8..23].
    };
}