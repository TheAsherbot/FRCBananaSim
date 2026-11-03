/**
 * @file DifferentialMechanismSystem.h
 * @brief Standalone physics system solving 2-DOF coupled differential dynamics.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Mechanisms/Components/DifferentialComponent.h"
#include "Physics/JoltContext.h"
#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    /**
     * @class DifferentialMechanismSystem
     * @brief Fixed-step physics system calculating semi-implicit 2x2 coupled motor torques and back-EMF damping.
     */
    class DifferentialMechanismSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the differential mechanism simulation system.
         * @param scene Reference to the governing ECS scene.
         * @param jolt Optional pointer to JoltContext for reading physical body transforms.
         */
        DifferentialMechanismSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~DifferentialMechanismSystem() override;

        /**
         * @brief Advances differential dynamics across fixed timestep dt.
         * @param dt Timestep duration in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Configures symmetric bevel differential matrices (pitch/roll) on a component.
         * @param diff Mutable reference to DifferentialComponent.
         * @param gearRatioPitch Reduction from motor shaft to pitch axis.
         * @param gearRatioRoll Reduction from motor shaft to roll axis.
         * @param invertRoll Inverts roll direction if true.
         */
        static void ConfigureSymmetricBevel(Components::DifferentialComponent& diff,
            double gearRatioPitch, double gearRatioRoll, bool invertRoll = false);

        /**
         * @brief Configures custom 2x2 forward transformation matrix on a component.
         * @param diff Mutable reference to DifferentialComponent.
         * @param matrixMq Matrix mapping [dq1, dq2] to [omega_mA, omega_mB].
         */
        static void ConfigureCustomMatrix(Components::DifferentialComponent& diff,
            const glm::dmat2& matrixMq);

        /**
         * @brief Recalculates reflected rotor inertia matrix based on component motor models.
         * @param diff Mutable reference to DifferentialComponent.
         */
        static void RecalculateReflectedInertia(Components::DifferentialComponent& diff);

    private:
        /**
         * @brief Evaluates coupled semi-implicit torques, damping, and Coulomb friction for a single differential mechanism.
         * @param diff Mutable reference to DifferentialComponent.
         * @param dt Timestep duration in seconds.
         */
        void UpdateDifferential(Components::DifferentialComponent& diff, double dt);

        /**
         * @brief Calculates vertical plane analytic gravity torques for the coupled joints.
         * @param diff Const reference to DifferentialComponent.
         * @return Vector of joint gravity torques [tau_g1, tau_g2] in N*m.
         */
        glm::dvec2 CalculateAnalyticGravity(const Components::DifferentialComponent& diff) const;

        /**
         * @brief Applies Coulomb friction clamping and stick thresholding in joint space.
         * @param diff Const reference to DifferentialComponent.
         * @param netTorques In/out vector of net delivered joint torques.
         */
        void ApplyFrictionClamping(const Components::DifferentialComponent& diff, glm::dvec2& netTorques) const;

        Physics::JoltContext* joltContext;
    };
}