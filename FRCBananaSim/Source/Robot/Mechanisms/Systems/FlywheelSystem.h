/**
 * @file FlywheelSystem.h
 * @brief System driving 1D high-RPM flywheel dynamics, chassis recoil, and visual render synchronization.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Physics/JoltContext.h"

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    /**
     * @class FlywheelSystem
     * @brief FixedUpdate and RenderUpdate system sub-stepping 1D semi-implicit flywheel ODEs and applying recoil.
     */
    class FlywheelSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the flywheel simulation system.
         * @param scene Reference to the governing ECS scene.
         * @param jolt Optional pointer to JoltContext for applying recoil torque to parent rigid bodies.
         */
        FlywheelSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~FlywheelSystem() override;

        /**
         * @brief Advances flywheel rotational ODEs across fixed timestep dt using sub-steps and applies recoil.
         * @param dt Timestep duration in seconds (typically 0.005 s).
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Synchronizes flywheel visual rotation quaternion in RenderUpdate.
         */
        virtual void RenderUpdate() override;

    private:
        /**
         * @brief Solves 1D semi-implicit rotational ODE across sub-steps for a single flywheel instance.
         * @param flywheel Mutable reference to FlywheelComponent.
         * @param dt Timestep duration in seconds.
         */
        void UpdateFlywheel(Components::FlywheelComponent& flywheel, double dt);

        Physics::JoltContext* joltContext;
    };
}