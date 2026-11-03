/**
 * @file PlaygroundScene.h
 * @brief Interactive visual sandbox scene for prototyping and tuning 2-DOF differential mechanisms.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Systems/DifferentialMechanismSystem.h"

#include <memory>

namespace FRCBananaSim::Playground
{
    /**
     * @enum PlaygroundControlMode
     * @brief Drive command generation modes for playground testing.
     */
    enum class PlaygroundControlMode : uint8_t
    {
        ManualSliders = 0, ///< Direct motor voltage sliders (-12V to +12V).
        PitchTest = 1,     ///< Coordinated symmetric pitch drive (+V, +V).
        RollTest = 2,      ///< Coordinated anti-symmetric roll drive (+V, -V).
        SineSweep = 3,     ///< Continuous sinusoidal oscillation test.
        BrakeHold = 4      ///< Zero volts with dynamic back-EMF brake active.
    };

    /**
     * @class PlaygroundScene
     * @brief Manages the 3D test rig, physical joints, and interactive HUD for differential mechanisms.
     */
    class PlaygroundScene
    {
    public:
        /**
         * @brief Constructs the playground scene manager.
         */
        PlaygroundScene();

        /**
         * @brief Destructor.
         */
        ~PlaygroundScene();

        /**
         * @brief Spawns the 3D test pedestal, pitch yoke, roll tool, Jolt joints, and differential component.
         * @param scene Reference to governing ECS scene.
         * @param jolt Reference to initialized JoltContext.
         * @return True if setup succeeded.
         */
        bool Initialize(Core::ECS::Core::Scene& scene, Physics::JoltContext& jolt);

        /**
         * @brief Advances physics, evaluates control modes, and steps the differential system.
         * @param dt Timestep in seconds (e.g. 0.005 for 200 Hz).
         */
        void FixedUpdate(double dt);

        /**
         * @brief Renders the interactive ImGui control panel, motor gauges, and telemetry curves.
         */
        void RenderUI();

        /**
         * @brief Resets the wrist joints and rigid body velocities back to zero rest pose.
         */
        void ResetPose();
        void Shutdown();

    private:
        Core::ECS::Core::Scene* activeScene;
        Physics::JoltContext* activeJolt;

        // Kinematic Hierarchy Entities
        Core::ECS::Core::Entity groundEntity;
        Core::ECS::Core::Entity mountStandEntity;
        Core::ECS::Core::Entity pitchYokeEntity;
        Core::ECS::Core::Entity rollToolEntity;
        Core::ECS::Core::Entity pitchJointEntity;
        Core::ECS::Core::Entity rollJointEntity;
        Core::ECS::Core::Entity diffMechanismEntity;

        // Dedicated Subsystem Pointers
        std::unique_ptr<Physics::Systems::PhysicsSystem> physicsSystem;
        std::unique_ptr<Physics::Systems::JointSystem> jointSystem;
        std::unique_ptr<Robot::Mechanisms::Systems::DifferentialMechanismSystem> diffSystem;

        // Interactive GUI & Control State
        PlaygroundControlMode controlMode;
        float manualVoltageA;
        float manualVoltageB;
        float testVoltageMagnitude;
        float testFrequencyHz;
        double testTimeSeconds;
        bool brakeModeA;
        bool brakeModeB;

        // Gearing & Transmission Tuning Parameters
        float gearRatioPitch;
        float gearRatioRoll;
        float transmissionEfficiency;
        float frictionPitchNm;
        float frictionRollNm;
    };
}