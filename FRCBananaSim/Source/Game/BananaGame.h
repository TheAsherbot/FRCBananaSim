/**
 * @file BananaGame.h
 * @brief Standalone arcade shootout mini-game featuring Robot Comp 2, swerve drive, intake, and goal shooting.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"
#include "Physics/Systems/JointSystem.h"
#include "Renderer/Renderer.h"
#include "Renderer/Camera.h"
#include "Robot/RobotAssembler.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Interaction/Components/GamePieceComponent.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <string>
#include <memory>

namespace FRCBananaSim::Game
{
    /**
     * @enum GameState
     * @brief States for the arcade shootout game loop.
     */
    enum class GameState
    {
        StartScreen,
        Countdown,
        Playing,
        Paused,
        GameOver
    };

    /**
     * @enum GameMode
     * @brief Mode of play: Timed 60s arcade rush or unlimited free practice.
     */
    enum class GameMode
    {
        ArcadeRush,  ///< 60-second score attack
        FreePractice ///< Unlimited time sandbox
    };

    /**
     * @enum CameraMode
     * @brief Visual perspective modes during gameplay.
     */
    enum class CameraMode
    {
        FollowRobot,   ///< Third-person dynamic chase camera
        DriverStation, ///< Fixed view from behind the alliance wall
        FreeOrbit      ///< Free orbit around target
    };

    /**
     * @struct ScorePopup
     * @brief Animated floating point text displayed in 3D world space.
     */
    struct ScorePopup
    {
        std::string text;
        glm::vec3 worldPos{ 0.0f };
        float lifetime = 0.0f;
        float maxLifetime = 1.2f;
        glm::vec3 color{ 1.0f, 0.9f, 0.2f };
    };

    /**
     * @class BananaGame
     * @brief Encapsulates the complete game loop, arena, physical balls, robot control, scoring, and UI.
     */
    class BananaGame
    {
    public:
        BananaGame(Core::ECS::Core::Scene& scene,
                   Physics::JoltContext& joltContext,
                   Renderer::Renderer& renderer);
        ~BananaGame();

        /**
         * @brief Loads Robot Comp 2, builds the arena walls & goal, and spawns the balls.
         * @return True if initialized successfully.
         */
        bool Initialize();

        /**
         * @brief Fixed 200 Hz physics tick for robot control, ball interaction, and goal triggers.
         * @param dt Fixed timestep delta (0.005 s = 5 ms).
         */
        void FixedUpdate(double dt);

        /**
         * @brief Variable frame tick for camera follow, input polling, timers, and particle cleanup.
         * @param dt Frame delta time in seconds.
         */
        void Update(double dt);

        /**
         * @brief Draws Dear ImGui overlays (Start Screen, In-Game HUD, Pause Menu, Game Over Screen).
         */
        void RenderUI();

        /**
         * @brief Renders 3D debug visuals (ballistic trajectory trails, goal rim highlights).
         */
        void Render3D();

        /**
         * @brief Starts or restarts a game session.
         */
        void StartGame(GameMode mode);

        /**
         * @brief Resets all balls onto the field and resets robot position.
         */
        void ResetField();

        /**
         * @brief Spawns a physical bouncy foam ball at a specified world position.
         */
        Core::ECS::Core::Entity SpawnBall(const glm::vec3& position, bool inShooter = false);

        /**
         * @brief Fires the currently indexed/loaded ball from the shooter.
         */
        void FireShooter();

        /**
         * @brief Checks if the game is currently requesting exit to desktop.
         */
        bool ShouldExit() const { return m_shouldExit; }

    private:
        void SetupArena();
        void SetupRobotVisuals();
        void UpdateArcadeRobotController(double dt);
        void CheckGoalScoring();
        void UpdateCamera(double dt);
        void DrawStartScreenUI();
        void DrawHUDUI();
        void DrawPauseUI();
        void DrawGameOverUI();
        void TriggerGoalScored(const glm::vec3& ballPos);

        // Core Engine References
        Core::ECS::Core::Scene& m_scene;
        Physics::JoltContext& m_joltContext;
        Renderer::Renderer& m_renderer;

        // Robot Subsystems
        Robot::AssembledRobot m_robot;
        std::unique_ptr<Physics::Systems::JointSystem> m_jointSystem;

        // Mechanism Component Pointers on Robot Comp 2
        Robot::Mechanisms::Components::ArmComponent* m_turretArm = nullptr;
        Robot::Mechanisms::Components::ArmComponent* m_intakePivotArm = nullptr;
        Robot::Mechanisms::Components::FlywheelComponent* m_flywheel = nullptr;
        Robot::Mechanisms::Components::FlywheelComponent* m_frontIntake = nullptr;
        Robot::Mechanisms::Components::FlywheelComponent* m_backIntake = nullptr;
        Robot::Mechanisms::Components::FlywheelComponent* m_bottomBasket = nullptr;
        Robot::Mechanisms::Components::FlywheelComponent* m_funnel = nullptr;

        // Game State & Rules
        GameState m_state = GameState::StartScreen;
        GameMode m_mode = GameMode::ArcadeRush;
        CameraMode m_camMode = CameraMode::FollowRobot;
        bool m_shouldExit = false;

        // Timers & Scores
        float m_sessionTimer = 60.0f;
        float m_countdownTimer = 3.0f;
        int m_score = 0;
        int m_highScore = 0;
        int m_comboStreak = 0;
        int m_maxCombo = 0;
        int m_ballsCollected = 0;
        int m_ballsFired = 0;
        int m_goalsScored = 0;

        // Goal Structure
        glm::vec3 m_goalCenter{ 0.0f, 2.20f, 6.0f }; ///< Target rim center in world coordinates
        float m_goalRadius = 0.50f;                   ///< Target rim radius
        float m_goalFlashTimer = 0.0f;                ///< Visual flash effect countdown

        // Driving Demands & Gyro
        float m_fieldCentricHeadingRad = 0.0f;
        bool m_fieldCentric = true;
        bool m_intakeDeployed = false;
        bool m_flywheelSpinning = false;
        float m_targetTurretAngle = 0.0f;

        // Balls & Trajectories
        std::vector<Core::ECS::Core::Entity> m_ballEntities;
        std::vector<ScorePopup> m_scorePopups;
        std::unordered_map<Core::ECS::Core::Entity, std::vector<glm::vec3>> m_ballTrails;
        float m_ballAutoDropTimer = 0.0f;

        // Follow Camera Smoothing
        glm::vec3 m_smoothCameraPos{ 0.0f, 2.0f, -4.0f };
        glm::vec3 m_smoothCameraTarget{ 0.0f, 0.4f, 0.0f };
    };
}
