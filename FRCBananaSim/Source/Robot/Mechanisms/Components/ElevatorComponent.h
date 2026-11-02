/**
 * @file ElevatorComponent.h
 * @brief Component representing a multi-stage elevator mechanism with linear motor drive and rigging coupling.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Actuation/DCMotorModel.h"
#include <vector>
#include <string>
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Mechanisms::Components
{
    /**
     * @enum ElevatorRiggingMode
     * @brief Mechanical cable rigging coupling topology.
     */
    enum class ElevatorRiggingMode : uint8_t
    {
        SingleStage = 0, ///< Single moving stage on a prismatic slider.
        Cascade = 1,     ///< 2+ stage cascade rigging: stage k moves at k * x_1 (analytic coupled).
        Continuous = 2   ///< 2+ stage continuous cable rigging: stages lift sequentially.
    };

    /**
     * @struct ElevatorStageInfo
     * @brief Per-stage kinematic tracking and physical properties.
     */
    struct ElevatorStageInfo
    {
        Core::ECS::Core::Entity stageEntity = Core::ECS::Core::invalidEntity; ///< Rigid body entity of the stage.
        Core::ECS::Core::Entity jointEntity = Core::ECS::Core::invalidEntity; ///< JointComponent entity (SliderConstraint).
        double stageMassKg = 0.0;                                             ///< Physical mass of this stage (kg).
        double maxTravelM = 0.0;                                              ///< Travel limit for this stage (m).
        double currentPositionM = 0.0;                                        ///< Displacement relative to parent stage (m).
        double currentVelocityMps = 0.0;                                      ///< Linear velocity relative to parent stage (m/s).
    };

    /**
     * @struct ElevatorComponent
     * @brief ECS component holding elevator motor parameters, rigging state, and reflected inertia.
     */
    struct ElevatorComponent
    {
        std::string name;
        ElevatorRiggingMode riggingMode = ElevatorRiggingMode::Cascade;

        // Drive Actuation & Transmission
        Actuation::DCMotorModel motorModel;
        uint32_t motorCount = 1;
        double gearRatio = 1.0;                  ///< Gearbox reduction ratio G (motor revs per drum rev).
        double efficiency = 0.95;                ///< Gearbox and pulley transmission efficiency (0.0 to 1.0).
        double drumRadiusM = 0.01905;            ///< Drum/sprocket pitch radius r_drum (m).

        // Counterbalance & Friction
        double springPreloadForceN = 0.0;        ///< Constant-force counterbalance spring force (N).
        double coulombFrictionN = 0.0;           ///< Static/Coulomb mechanical sliding friction (N).
        double viscousDampingNspm = 0.0;         ///< Linear viscous friction damping coefficient (N/(m/s)).
        double statorCurrentLimitA = 80.0;       ///< Stator current clamp threshold (A).

        // Reflected Linear Mass
        double reflectedLinearMassKg = 0.0;      ///< m_reflected = N * J_rotor * (G / r_drum)^2 (kg).
        double lumpedEffectiveMassKg = 0.0;      ///< Total effective mass including reflected rotor inertia (kg).

        // Motion Limits & Geometry
        glm::vec3 liftAxis = { 0.0f, 1.0f, 0.0f }; ///< Direction of stage extension in world frame (typically +Y engine).
        double minHeightM = 0.0;                 ///< Lower hard stop position (m).
        double maxHeightM = 1.2;                 ///< Total extension travel across all stages (m).

        // Runtime Command Demands
        double commandedVoltage = 0.0;           ///< Motor drive command in Volts (-12.0V to +12.0V).
        double appliedVoltage = 0.0;             ///< Actual applied motor voltage post-sag/limit.
        double appliedLinearForceN = 0.0;        ///< Net linear actuation force F^{t+dt} (N).
        double statorCurrentA = 0.0;             ///< Instantaneous motor stator current (A).
        bool brakeMode = true;                   ///< If true, dynamic electrical back-EMF damping active.

        // Multistage Rigging Hierarchy
        std::vector<ElevatorStageInfo> stages;

        // Total Mechanism State
        double totalCarriageHeightM = 0.0;       ///< Total height of topmost carriage above base (m).
        double totalCarriageVelocityMps = 0.0;   ///< Total linear velocity of topmost carriage (m/s).

        // Control Slot Mapping
        uint32_t motorSlot = 8;                  ///< SimInputs motor slot index [8..23].

        // Static Stick Holding State
        bool isStickLocked = false;              ///< True if Coulomb friction is currently locking the mechanism in static stick.
        double lockedPositionM = 0.0;            ///< Latched displacement holding position during static stick (m).
    };
}