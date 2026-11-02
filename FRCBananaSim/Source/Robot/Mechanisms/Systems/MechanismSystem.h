/**
 * @file MechanismSystem.h
 * @brief System driving articulated arm dynamics, four-bar loop tracking, and power couplings.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FourBarComponent.h"
#include "Robot/Mechanisms/Components/CouplingComponent.h"
#include "Physics/JoltContext.h"

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    /**
     * @class MechanismSystem
     * @brief Fixed-step physics system calculating semi-implicit arm torques, 4-bar drift, and power transmissions.
     */
    class MechanismSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the mechanism simulation system.
         * @param scene Reference to the governing ECS scene.
         * @param jolt Optional pointer to JoltContext for reading physical body transforms.
         */
        MechanismSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~MechanismSystem() override;

        /**
         * @brief Advances arm dynamics, four-bar loop checks, and power transmissions across fixed timestep dt.
         * @param dt Timestep duration in seconds.
         */
        virtual void FixedUpdate(double dt) override;

    private:
        /**
         * @brief Evaluates semi-implicit motor torques, Coulomb stick, and limits for a single arm instance.
         * @param arm Mutable reference to ArmComponent.
         * @param dt Timestep duration in seconds.
         */
        void UpdateArm(Components::ArmComponent& arm, double dt);

        /**
         * @brief Evaluates loop closure drift and constraint integrity for a planar four-bar linkage.
         * @param fourBar Mutable reference to FourBarComponent.
         */
        void UpdateFourBar(Components::FourBarComponent& fourBar);

        /**
         * @brief Enforces velocity ratio, efficiency, and backlash deadband across a mechanical power coupling.
         * @param coupling Mutable reference to CouplingComponent.
         * @param dt Timestep duration in seconds.
         */
        void UpdateCoupling(Components::CouplingComponent& coupling, double dt);

        Physics::JoltContext* joltContext;
    };
}