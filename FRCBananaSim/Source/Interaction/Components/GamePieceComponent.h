/**
 * @file GamePieceComponent.h
 * @brief ECS component holding game-piece interaction state machine, kinematics, and Jolt binding.
 */

#pragma once

#include "Core/ECS/Core/Entity.h"
#include "Interaction/GamePieceSpec.h"
#include <Jolt/Physics/Body/BodyID.h>
#include <glm/vec3.hpp>
#include <cstdint>
#include <ostream>

namespace FRCBananaSim::Interaction::Components
{
    /**
     * @enum GamePieceState
     * @brief Discrete lifecycle states for interactive game pieces.
     */
    enum class GamePieceState : uint8_t
    {
        Free = 0,   ///< Dynamic rigid body in world solver.
        Intaking,   ///< Compliant surface traction pull toward throat.
        Stored,     ///< Kinematically latched inside indexer/hopper.
        Engaged,    ///< Hunt-Crossley normal squeeze and flywheel traction.
        Launched    ///< Decoupled free flight with quadratic drag & Magnus lift.
    };

    inline std::ostream& operator<<(std::ostream& os, const GamePieceState& state)
    {
        switch (state)
        {
        case GamePieceState::Free: return os << "Free";
        case GamePieceState::Intaking: return os << "Intaking";
        case GamePieceState::Stored: return os << "Stored";
        case GamePieceState::Engaged: return os << "Engaged";
        case GamePieceState::Launched: return os << "Launched";
        default: return os << "Unknown";
        }
    }

    /**
     * @struct GamePieceComponent
     * @brief Attached to all game-piece entities in scene to drive hybrid physics overrides.
     */
    struct GamePieceComponent
    {
        GamePieceState state = GamePieceState::Free;
        GamePieceSpec spec;
        JPH::BodyID bodyID;

        // Robot Binding (valid only while Intaking, Stored, or Engaged)
        Core::ECS::Core::Entity capturingRobot = Core::ECS::Core::invalidEntity;
        Core::ECS::Core::Entity boundContactZone = Core::ECS::Core::invalidEntity;

        // Kinematics & Recoil
        glm::vec3 linearVelocity = { 0.0f, 0.0f, 0.0f };
        glm::vec3 angularVelocity = { 0.0f, 0.0f, 0.0f };
        double stateTimer = 0.0;
        uint32_t muzzleCollisionMaskTicks = 0; ///< Countdown ticks for muzzle collision ignore

        // Shooter / Intake Squeeze Progress
        double transitProgressM = 0.0;         ///< Distance traveled through active compression zone [0..L]
        double transitVelocityMPerS = 0.0;     ///< Tangential speed along contact zone axis
        double currentCompressionM = 0.0;      ///< Instantaneous squeeze depth delta (m)
        double currentNormalForceN = 0.0;      ///< Instantaneous Hunt-Crossley normal force (N)
        double currentTractionForceN = 0.0;    ///< Instantaneous tangential traction force (N)
        double cumulativeWorkJ = 0.0;          ///< Cumulative work / energy transferred to piece (J)
    };
}
