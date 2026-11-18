/**
 * @file RobotGamePieceSystem.h
 * @brief Robot-local game-piece interaction passes for shooter squeeze and flywheel load torque coupling.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Robot/RobotAssembler.h"

namespace FRCBananaSim::Interaction::Systems
{
    /**
     * @class RobotGamePieceSystem
     * @brief Coordinates game-piece state transitions and injects physical interaction forces into the robot pipeline.
     */
    class RobotGamePieceSystem
    {
    public:
        /**
         * @brief Stage 3c: Evaluates active shooter squeeze, computes Hunt-Crossley normal force and slip traction,
         *        evaluates active intake zones, and injects braking load torque into FlywheelComponent::externalLoadTorqueNm.
         * @param scene ECS scene containing all entities.
         * @param robot Assembled robot data.
         * @param dt Physics timestep duration (s).
         */
        static void PreFlywheelUpdate(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt);

        /**
         * @brief Evaluates active intake zones: detects overlap, pulls pieces inward with compliant traction,
         *        applies motor reaction torque, and executes kinematic capture handoff to Stored state.
         * @param scene ECS scene containing all entities.
         * @param robot Assembled robot data.
         * @param dt Physics timestep duration (s).
         */
        static void UpdateIntake(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt);

        /**
         * @brief Advances stored game pieces from Stored state into the shooter compression zone (Engaged).
         * @param scene ECS scene containing all entities.
         * @param robot Assembled robot data.
         */
        static void TriggerFeedToShooter(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot);

        /**
         * @brief Stage 4b: Advances transit progress, checks for muzzle exit, updates ContactZoneComponent::isEngaged,
         *        and transitions projectile to Launched state.
         * @param scene ECS scene containing all entities.
         * @param robot Assembled robot data.
         * @param dt Physics timestep duration (s).
         */
        static void PostFlywheelUpdate(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt);
    };
}
