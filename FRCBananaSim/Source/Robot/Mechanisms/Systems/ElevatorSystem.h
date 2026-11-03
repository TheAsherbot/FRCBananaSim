/**
 * @file ElevatorSystem.h
 * @brief FixedUpdate system solving linear semi-implicit motor forces, Coulomb friction stick, and rigging.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Physics/JoltContext.h"

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    /**
     * @class ElevatorSystem
     * @brief Fixed-step physics system driving linear elevator actuation and multi-stage rigging.
     */
    class ElevatorSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the elevator simulation system.
         * @param scene Reference to the governing ECS scene.
         * @param jolt Optional pointer to JoltContext for direct physical queries.
         */
        ElevatorSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~ElevatorSystem() override;

        /**
         * @brief Advances elevator dynamics across fixed timestep dt.
         * @param dt Timestep duration in seconds.
         */
        virtual void FixedUpdate(double dt) override;

    private:
        /**
         * @brief Solves actuation forces and updates multi-stage rigging for a single elevator instance.
         * @param elevator Mutable reference to ElevatorComponent.
         * @param dt Timestep duration in seconds.
         */
        void UpdateElevator(Components::ElevatorComponent& elevator, double dt);

        /**
         * @brief Enforces analytic coupled cascade elevator stage constraints (x_k = k * x_1).
         * @param elevator Mutable reference to ElevatorComponent.
         * @param x1 Measured displacement of Stage 1 in meters.
         * @param v1 Measured velocity of Stage 1 in m/s.
         * @param predV Forward-predicted velocity of Stage 1 for the end of the step in m/s.
         */
        void ProcessCascadeRigging(Components::ElevatorComponent& elevator, double x1, double v1, double predV);

        /**
         * @brief Enforces sequential lifting for continuous cable rigging.
         * @param elevator Mutable reference to ElevatorComponent.
         * @param netLinearForce Actuation force delivered by the lift cable in Newtons.
         */
        void ProcessContinuousRigging(Components::ElevatorComponent& elevator, double netLinearForce);

        Physics::JoltContext* joltContext;
    };
}