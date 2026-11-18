/**
 * @file FlightAerodynamicsSystem.cpp
 * @brief Implementation of global aerodynamic simulation for airborne game pieces.
 */

#include "Interaction/Systems/FlightAerodynamicsSystem.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Interaction::Systems
{
    using Components::GamePieceComponent;
    using Components::GamePieceState;
    using Physics::Components::RigidBodyComponent;
    using Core::ECS::Components::TransformComponent;
    using Core::ECS::Components::WorldTransformComponent;

    FlightAerodynamicsSystem::FlightAerodynamicsSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
          joltContext(jolt)
    {
    }

    glm::vec3 FlightAerodynamicsSystem::CalculateDragForce(double rhoAir, double Cd, double area, const glm::vec3& velocity)
    {
        if (std::isnan(velocity.x) || std::isnan(velocity.y) || std::isnan(velocity.z))
        {
            return glm::vec3(0.0f);
        }

        float speed = glm::length(velocity);
        if (speed < 1e-6f)
        {
            return glm::vec3(0.0f);
        }

        // F_d = -0.5 * rho * Cd * A * ||v|| * v
        double dragCoeff = 0.5 * rhoAir * Cd * area * static_cast<double>(speed);
        return -static_cast<float>(dragCoeff) * velocity;
    }

    glm::vec3 FlightAerodynamicsSystem::CalculateMagnusForce(double rhoAir, double ClAlpha, double area, double radius,
                                                            const glm::vec3& angularVel, const glm::vec3& linearVel)
    {
        if (std::isnan(angularVel.x) || std::isnan(angularVel.y) || std::isnan(angularVel.z) ||
            std::isnan(linearVel.x) || std::isnan(linearVel.y) || std::isnan(linearVel.z))
        {
            return glm::vec3(0.0f);
        }

        // S_m = 0.5 * rho * ClAlpha * A * radius (units: kg)
        // F_m = S_m * (omega x v)
        double Sm = 0.5 * rhoAir * ClAlpha * area * radius;
        glm::vec3 crossW = glm::cross(angularVel, linearVel);
        return static_cast<float>(Sm) * crossW;
    }

    void FlightAerodynamicsSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("FlightAerodynamicsSystem::FixedUpdate");

        if (dt <= 1e-9)
        {
            return;
        }

        constexpr double kStandardAirDensity = 1.225; // kg/m^3

        JPH::BodyInterface* bi = nullptr;
        if (joltContext && joltContext->IsInitialized())
        {
            bi = &joltContext->GetBodyInterface();
        }

        auto entities = scene.Query<GamePieceComponent>();
        for (auto entity : entities)
        {
            auto* piece = scene.GetComponent<GamePieceComponent>(entity);
            if (!piece || piece->state != GamePieceState::Launched)
            {
                continue;
            }

            // 1. Muzzle collision masking countdown (3 ticks = 15 ms at 200 Hz)
            if (piece->muzzleCollisionMaskTicks > 0)
            {
                piece->muzzleCollisionMaskTicks--;
                if (piece->muzzleCollisionMaskTicks == 0)
                {
                    // Restore full mutual collision with robot links
                    if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
                    {
                        bi->SetCollisionGroup(piece->bodyID, JPH::CollisionGroup());
                    }

                    auto* rb = scene.GetComponent<RigidBodyComponent>(entity);
                    if (rb)
                    {
                        rb->collisionGroupID = 0;
                        rb->collisionSubGroupID = 0;
                    }
                }
            }

            // 2. Synchronize kinematics from Jolt if body exists
            glm::vec3 linVel = piece->linearVelocity;
            glm::vec3 angVel = piece->angularVelocity;
            glm::vec3 worldPos(0.0f);
            bool hasJoltBody = false;

            if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
            {
                hasJoltBody = true;
                JPH::Vec3 jv = bi->GetLinearVelocity(piece->bodyID);
                JPH::Vec3 ja = bi->GetAngularVelocity(piece->bodyID);
                JPH::RVec3 jp = bi->GetPosition(piece->bodyID);

                linVel = glm::vec3(jv.GetX(), jv.GetY(), jv.GetZ());
                angVel = glm::vec3(ja.GetX(), ja.GetY(), ja.GetZ());
                worldPos = glm::vec3(static_cast<float>(jp.GetX()), static_cast<float>(jp.GetY()), static_cast<float>(jp.GetZ()));

                piece->linearVelocity = linVel;
                piece->angularVelocity = angVel;
            }
            else
            {
                auto* tc = scene.GetComponent<TransformComponent>(entity);
                if (tc)
                {
                    worldPos = tc->position;
                }
            }

            // 3. Evaluate Aerodynamics (Quadratic Drag & Magnus Lift)
            double Cd = piece->spec.dragCoefficientCd > 0.0 ? piece->spec.dragCoefficientCd : 0.45;
            double area = piece->spec.crossSectionalAreaM2 > 0.0 ? piece->spec.crossSectionalAreaM2 : 0.045;
            double ClAlpha = piece->spec.liftSlopeClAlpha > 0.0 ? piece->spec.liftSlopeClAlpha : 1.20;
            double rPiece = piece->spec.outerRadiusM > 0.0 ? piece->spec.outerRadiusM : 0.1778;

            glm::vec3 F_drag = CalculateDragForce(kStandardAirDensity, Cd, area, linVel);
            glm::vec3 F_magnus = CalculateMagnusForce(kStandardAirDensity, ClAlpha, area, rPiece, angVel, linVel);
            glm::vec3 F_aero = F_drag + F_magnus;

            // 4. Apply Forces
            if (hasJoltBody && bi)
            {
                bi->AddForce(piece->bodyID, JPH::Vec3(F_aero.x, F_aero.y, F_aero.z));

                // Aerodynamic skin-friction spin decay
                bi->AddTorque(piece->bodyID, JPH::Vec3(-0.00015f * angVel.x, -0.00015f * angVel.y, -0.00015f * angVel.z));
            }
            else
            {
                // Analytical fallback when operating without Jolt
                double mass = piece->spec.massKg > 0.0 ? piece->spec.massKg : 0.235;
                glm::vec3 gravity(0.0f, -9.80665f, 0.0f);
                glm::vec3 accel = gravity + (F_aero / static_cast<float>(mass));

                auto* tc = scene.GetComponent<TransformComponent>(entity);
                float floorY = static_cast<float>(piece->spec.thicknessM > 0.01 ? piece->spec.thicknessM * 0.5 : 0.025);
                if (tc && tc->position.y <= floorY && accel.y < 0.0f)
                {
                    accel.y = 0.0f;
                    piece->linearVelocity.y = 0.0f;
                    float horizSpeed = glm::length(glm::vec2(piece->linearVelocity.x, piece->linearVelocity.z));
                    if (horizSpeed > 1e-4f)
                    {
                        float frictionDecel = 9.8f * static_cast<float>(piece->spec.surfaceFriction > 0.01 ? piece->spec.surfaceFriction : 0.5);
                        float deltaV = frictionDecel * static_cast<float>(dt);
                        float newHorizSpeed = std::max(0.0f, horizSpeed - deltaV);
                        float scale = newHorizSpeed / horizSpeed;
                        piece->linearVelocity.x *= scale;
                        piece->linearVelocity.z *= scale;
                    }
                }

                piece->linearVelocity += accel * static_cast<float>(dt);
                if (tc)
                {
                    tc->position += piece->linearVelocity * static_cast<float>(dt);
                    if (tc->position.y < floorY)
                    {
                        tc->position.y = floorY;
                    }
                    worldPos = tc->position;
                }
            }

            // 5. Rest / Landing Detection -> Transition to Free
            piece->stateTimer += dt;
            float speed = glm::length(piece->linearVelocity);
            float groundRestThreshold = static_cast<float>(piece->spec.thicknessM > 0.01 ? piece->spec.thicknessM * 1.5 : 0.1);

            bool atRestOnGround = (worldPos.y <= groundRestThreshold && speed < 0.1f && piece->stateTimer > 0.15);
            bool atRestGeneral = (speed < 0.05f && piece->stateTimer > 0.5);

            if (atRestOnGround || atRestGeneral)
            {
                piece->state = GamePieceState::Free;
                piece->stateTimer = 0.0;
            }
        }
    }
}
