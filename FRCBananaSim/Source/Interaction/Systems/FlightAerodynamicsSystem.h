/**
 * @file FlightAerodynamicsSystem.h
 * @brief Global scene-level aerodynamic simulation pass for airborne game pieces (quadratic drag & Magnus lift).
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Core/ECS/Core/Scene.h"
#include "Physics/JoltContext.h"

#include <glm/vec3.hpp>

namespace FRCBananaSim::Interaction::Systems
{
    /**
     * @class FlightAerodynamicsSystem
     * @brief Fixed-step scene-level system that applies realistic aerodynamic drag and Magnus rotational lift
     *        to game pieces in the Launched state, counts down muzzle collision masking, and detects carpet rest.
     */
    class FlightAerodynamicsSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the flight aerodynamics system.
         * @param scene ECS scene governing all entities.
         * @param jolt Pointer to active JoltContext (may be nullptr for standalone/headless unit tests).
         */
        FlightAerodynamicsSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt);

        virtual ~FlightAerodynamicsSystem() override = default;

        /**
         * @brief Advances aerodynamic simulation for all airborne game pieces (Stage 6b).
         * @param dt Fixed delta time in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Calculates quadratic air drag force: F_d = -0.5 * rho * Cd * A * ||v|| * v.
         * @param rhoAir Air density in kg/m^3.
         * @param Cd Drag coefficient.
         * @param area Cross-sectional area in m^2.
         * @param velocity Linear velocity vector in m/s.
         * @return Drag force vector in Newtons.
         */
        static glm::vec3 CalculateDragForce(double rhoAir, double Cd, double area, const glm::vec3& velocity);

        /**
         * @brief Calculates non-singular Magnus rotational lift force: F_m = S_m * (omega x v)
         *        where S_m = 0.5 * rho * ClAlpha * A * radius (kg).
         * @param rhoAir Air density in kg/m^3.
         * @param ClAlpha Lift-curve slope coefficient.
         * @param area Cross-sectional area in m^2.
         * @param radius Outer radius of game piece in meters.
         * @param angularVel Angular velocity vector in rad/s.
         * @param linearVel Linear velocity vector in m/s.
         * @return Magnus lift force vector in Newtons.
         */
        static glm::vec3 CalculateMagnusForce(double rhoAir, double ClAlpha, double area, double radius,
                                              const glm::vec3& angularVel, const glm::vec3& linearVel);

    private:
        Physics::JoltContext* joltContext;
    };
}
