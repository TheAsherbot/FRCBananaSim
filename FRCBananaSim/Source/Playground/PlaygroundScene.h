/**
 * @file PlaygroundScene.h
 * @brief Interactive visual sandbox scene for prototyping differential mechanisms and shooter squeeze dynamics.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Physics/Systems/JointSystem.h"
#include "Robot/Mechanisms/Systems/DifferentialMechanismSystem.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/Sensors/Systems/SensorSystem.h"
#include "Interaction/ShooterSqueezeModel.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Renderer/Camera.h"

#include <memory>
#include <vector>
#include <array>
#include <string>

namespace FRCBananaSim::Playground
{
    /**
     * @enum PlaygroundSceneType
     * @brief Dedicated sandbox scenes available in the launcher.
     */
    enum class PlaygroundSceneType : int
    {
        BallShooterHood = 0,   ///< Dedicated Ball Shooter with Curved Polycarbonate Hood
        NoteShooterDual = 1,   ///< Dedicated 2024 Note Shooter with Dual Counter-Rotating Flywheels
        DifferentialWrist = 2  ///< Dedicated 2-DOF Differential Wrist Mechanism
    };

    /**
     * @enum PlaygroundControlMode
     * @brief Drive command generation modes for differential wrist testing.
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
     * @struct ProjectileInstance
     * @brief Managed physical projectile in the shooter pool (Ball or Note).
     */
    struct ProjectileInstance
    {
        Core::ECS::Core::Entity entity{ Core::ECS::Core::invalidEntity };
        bool active{ false };
        bool inSqueeze{ false };
        double progressM{ 0.0 };
        double velocityMPerS{ 2.5 };
        double lifeTimeS{ 0.0 };
        bool hasHitGround{ false };
        glm::vec3 landingPos{ 0.0f };
        double preImpactSpeedMPerS{ 0.0 };
        double postImpactSpeedMPerS{ 0.0 };
        double spinRadPerSec{ 0.0 };
        std::vector<glm::vec3> trajectoryTrail; ///< High-contrast 3D flight path breadcrumbs
    };

    /**
     * @class PlaygroundScene
     * @brief Manages the 3D test rigs, physical mechanisms, and interactive HUD for differential wrists and shooter squeeze.
     */
    class PlaygroundScene
    {
    public:
        PlaygroundScene();
        ~PlaygroundScene();

        /**
         * @brief Initializes baseline physics and environment floor. Rigs are spawned when a scene is started.
         * @param scene Reference to governing ECS scene.
         * @param jolt Reference to initialized JoltContext.
         * @return True if setup succeeded.
         */
        bool Initialize(Core::ECS::Core::Scene& scene, Physics::JoltContext& jolt);

        /**
         * @brief Binds camera for screen-space meter marker projection.
         */
        void SetCamera(Renderer::Camera* cam) { activeCamera = cam; }

        /**
         * @brief Advances physics, evaluates controls, and steps mechanisms at 200 Hz.
         * @param dt Timestep in seconds (e.g. 0.005 for 200 Hz).
         */
        void FixedUpdate(double dt);

        /**
         * @brief Renders the interactive ImGui launcher dialog or active control panel, telemetry scopes, and ground distance labels.
         */
        void RenderUI();

        /**
         * @brief Fires a game piece into the shooter compression zone.
         */
        void FireGamePiece();

        /**
         * @brief Resets the differential wrist joints and velocities back to zero rest pose.
         */
        void ResetWristPose();

        /**
         * @brief Loads the specified scene mode, spawning its entities and positioning the camera.
         */
        void LoadScene(PlaygroundSceneType type);

        /**
         * @brief Unloads current rig entities and clears projectile pool.
         */
        void UnloadScene();

        /**
         * @brief Cleans up entities and systems.
         */
        void Shutdown();

        // Scene Launcher State
        bool sceneStarted = false;
        PlaygroundSceneType selectedScene = PlaygroundSceneType::BallShooterHood;
        PlaygroundSceneType activeSceneType = PlaygroundSceneType::BallShooterHood;

        // Camera focus requests
        bool cameraFocusRequested = false;
        glm::vec3 cameraFocusTarget = { 0.0f, 1.0f, 0.2f };
        float cameraFocusDistance = 2.4f;

    private:
        void SpawnBallShooterHoodRig();
        void SpawnNoteShooterDualRig();
        void SpawnDifferentialWristRig();
        void UpdateHoodGeometry();
        void UpdateNoteShooterGeometry();
        void DestroyTrackedEntity(Core::ECS::Core::Entity& entity);
        void DrawOrientedBox(const glm::vec3& center, const glm::quat& rot, const glm::vec3& halfExtents, const glm::vec3& color);
        void RenderDistanceFieldHUD();
        void RenderBallisticOracleHUD(bool isBallShooter);

        Core::ECS::Core::Scene* activeScene;
        Physics::JoltContext* activeJolt;
        Renderer::Camera* activeCamera;

        // Shared Environment Entities
        Core::ECS::Core::Entity groundEntity;

        // 1. 2-DOF Differential Wrist Rig Entities
        Core::ECS::Core::Entity mountStandEntity;
        Core::ECS::Core::Entity pitchYokeEntity;
        Core::ECS::Core::Entity rollToolEntity;
        Core::ECS::Core::Entity pitchJointEntity;
        Core::ECS::Core::Entity rollJointEntity;
        Core::ECS::Core::Entity diffMechanismEntity;

        // 2. Shooter Rig Entities
        Core::ECS::Core::Entity shooterStandEntity;
        Core::ECS::Core::Entity topFlywheelEntity;
        Core::ECS::Core::Entity bottomFlywheelEntity;
        Core::ECS::Core::Entity contactZoneEntity;
        Core::ECS::Core::Entity beamBreakEntity;

        // Active Intake Rig Entities & State
        Core::ECS::Core::Entity intakeRollerEntity;
        Core::ECS::Core::Entity intakeZoneEntity;
        Core::ECS::Core::Entity intakeBeamBreakEntity;
        float intakeVoltage;
        float intakeRollerRadiusM;
        float intakeGripNormalForceN;
        float intakeViscousDampingNsPerM;
        float intakeMaxGripForceN;
        bool intakeEnabled;
        bool autoFeedToShooter;
        double lastIntakeTorqueNm;
        double currentIntakeRpm;
        size_t storedPiecesCount;

        void SpawnGroundGamePiece();
        void FeedStoredPiece();

        // Curved Hood Entities (Ball Shooter)
        static constexpr size_t kHoodSegments = 16;
        std::vector<Core::ECS::Core::Entity> hoodSegmentEntities;
        Core::ECS::Core::Entity hoodLeftCheekEntity;
        Core::ECS::Core::Entity hoodRightCheekEntity;

        // Projectile Pool (Multiple dynamic bouncing balls / notes)
        static constexpr size_t kMaxProjectiles = 24;
        std::array<ProjectileInstance, kMaxProjectiles> projectilePool{};
        size_t nextProjectileIdx = 0;

        // Dedicated Subsystem Pointers
        std::unique_ptr<Physics::Systems::PhysicsSystem> physicsSystem;
        std::unique_ptr<Physics::Systems::JointSystem> jointSystem;
        std::unique_ptr<Robot::Mechanisms::Systems::DifferentialMechanismSystem> diffSystem;
        std::unique_ptr<Robot::Mechanisms::Systems::FlywheelSystem> flywheelSystem;
        std::unique_ptr<Robot::Sensors::Systems::SensorSystem> sensorSystem;

        // Differential Wrist State
        PlaygroundControlMode controlMode;
        float manualVoltageA;
        float manualVoltageB;
        float testVoltageMagnitude;
        float testFrequencyHz;
        double testTimeSeconds;
        bool brakeModeA;
        bool brakeModeB;
        float gearRatioPitch;
        float gearRatioRoll;
        float transmissionEfficiency;
        float frictionPitchNm;
        float frictionRollNm;

        // Shooter Tuning Parameters
        float commandedFlywheelVoltage;
        float targetFlywheelRPM;
        bool useFlywheelClosedLoop;
        float complianceK;
        float restitutionE;
        float frictionMu;
        float hoodFrictionMu;
        float nominalGapM;
        float uncompressedSizeM;
        float contactLengthM;
        float pieceMassKg;
        float flywheelInertiaKgm2;
        float rollerRadiusM;
        bool isDualFlywheel;
        bool rapidFireEnabled;
        double rapidFireTimer;

        // Ball Shooter Curved Hood Angles
        bool isOverhandShooter;
        float hoodStartAngleDeg;
        float hoodEndAngleDeg;

        // Note Shooter Parameters
        float notePitchDeg;

        // Spin Aerodynamics & Ground Bite Physics
        bool enableSpinPhysics;
        float ballSpinMultiplier;
        float groundBiteFriction;
        double currentBallSpinRpm;
        double lastImpactSpinRpm;
        double lastPreImpactSpeedMPerS;
        double lastPostImpactSpeedMPerS;
        double lastMagnusLiftN;

        // Shot Telemetry & Distance Tracking
        bool pieceActive;
        double initialShotRpm;
        double lastExitSpeedMPerS;
        double currentFlightSpeedMPerS;
        double lastRpmSag;
        double peakNormalForceN;
        double peakTractionForceN;
        double currentSqueezeDepthM;
        double currentNormalForceN;
        double currentTractionForceN;
        double shotTransitTimeMs;
        Interaction::EnergyBalanceReport lastEnergyReport;
        bool hasShotFired;

        // Ground Landing & Carry Distance Tracking
        float lastShotDistanceM;
        glm::vec3 lastLandingPos;
        bool hasLandingRecorded;

        // Visual Overlays & Diagnostic Toggles
        bool showTrajectoryTrails = true;
        bool showBeamBreakLasers = true;
        bool showContactZones = true;
        bool showGamePieceStateBadges = true;
        bool showAeroForceVectors = true;

        // Ballistic Shot Oracle & Theoretical Energy Metrics
        double lastTheoreticalExitSpeedMPerS = 0.0;
        double lastTheoreticalRpmSag = 0.0;
        double lastTheoreticalEnergyTransferredJ = 0.0;
        double lastActualEnergyTransferredJ = 0.0;
        double lastRecoilImpulseNs = 0.0;

        // Telemetry Ring Buffers for ImGui Scopes
        static constexpr size_t kHistorySize = 150;
        std::array<float, kHistorySize> rpmHistory{};
        std::array<float, kHistorySize> torqueHistory{};
        std::array<float, kHistorySize> forceHistory{};
        size_t historyIndex = 0;
    };
}
