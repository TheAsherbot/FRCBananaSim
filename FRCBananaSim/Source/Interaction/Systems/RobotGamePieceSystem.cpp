/**
 * @file RobotGamePieceSystem.cpp
 * @brief Implementation of PreFlywheel and PostFlywheel passes in the robot execution pipeline.
 */

#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/ShooterSqueezeModel.h"
#include "Interaction/IntakeSurfaceModel.h"
#include "Interaction/Components/GamePieceComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

namespace FRCBananaSim::Interaction::Systems
{
    using Components::GamePieceComponent;
    using Components::GamePieceState;
    using Robot::Sensors::Components::ContactZoneComponent;
    using Robot::Mechanisms::Components::FlywheelComponent;

    static double GetContactLength(const ContactZoneComponent& cz)
    {
        glm::vec3 absDims = glm::abs(cz.dimensions);
        glm::vec3 absTan = glm::abs(cz.tangentAxis);
        double len = glm::dot(absDims, absTan);
        return len > 0.01 ? len : 0.18;
    }

    void RobotGamePieceSystem::PreFlywheelUpdate(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt)
    {
        SIM_PROFILE_SCOPE("RobotGamePieceSystem::PreFlywheelUpdate");

        if (dt <= 1e-9)
        {
            return;
        }

        // 1. Locate primary shooter flywheel mechanism on the robot
        FlywheelComponent* primaryFlywheel = nullptr;
        for (const auto& [name, entity] : robot.mechanismEntities)
        {
            auto* fw = scene.GetComponent<FlywheelComponent>(entity);
            if (fw && fw->name.find("intake") == std::string::npos)
            {
                primaryFlywheel = fw;
                break;
            }
        }

        if (!primaryFlywheel)
        {
            return;
        }

        // 2. Iterate all game pieces actively in the ENGAGED state
        auto pieceEntities = scene.Query<GamePieceComponent>();
        for (auto pEntity : pieceEntities)
        {
            auto* piece = scene.GetComponent<GamePieceComponent>(pEntity);
            if (!piece || piece->state != GamePieceState::Engaged)
            {
                continue;
            }

            // Auto-bind to robot shooter contact zone if not already bound
            ContactZoneComponent* cz = nullptr;
            if (piece->boundContactZone != Core::ECS::Core::invalidEntity)
            {
                cz = scene.GetComponent<ContactZoneComponent>(piece->boundContactZone);
            }
            else
            {
                for (const auto& [name, cze] : robot.contactZoneEntities)
                {
                    auto* candidate = scene.GetComponent<ContactZoneComponent>(cze);
                    if (candidate && candidate->role == "shooter")
                    {
                        piece->boundContactZone = cze;
                        cz = candidate;
                        break;
                    }
                }
            }

            if (!cz)
            {
                continue;
            }

            // Mark zone as actively engaged
            cz->isEngaged = true;

            // 3. Configure squeeze parameters from game piece and contact zone definitions
            SqueezeParameters params;
            params.stiffnessK = piece->spec.complianceK > 1.0 ? piece->spec.complianceK : 25000.0;
            params.restitutionE = piece->spec.restitutionE;
            params.frictionMu = piece->spec.surfaceFriction > 0.01 ? piece->spec.surfaceFriction : 0.68;
            params.hoodFrictionMu = 0.05; // Low-friction hood backing
            params.rollerRadiusM = primaryFlywheel->rollerRadiusM > 0.001 ? primaryFlywheel->rollerRadiusM : 0.0508;
            params.nominalGapM = cz->nominalGapM > 0.001 ? cz->nominalGapM : 0.08;
            params.uncompressedSizeM = piece->spec.thicknessM > 0.01 ? piece->spec.thicknessM : 0.12;
            params.contactLengthM = GetContactLength(*cz);
            params.pieceMassKg = piece->spec.massKg > 0.01 ? piece->spec.massKg : 0.235;
            params.isDualFlywheel = false;

            // 4. Calculate flywheel roller surface speed
            double vRoller = primaryFlywheel->angularVelocityRadPerSec * params.rollerRadiusM;

            // 5. Integrate squeeze step
            SqueezeState state;
            state.progressM = piece->transitProgressM;
            state.velocityMPerS = piece->transitVelocityMPerS;
            state.dissipatedEnergyJ = piece->cumulativeWorkJ;

            uint32_t subSteps = primaryFlywheel->subSteps > 0 ? primaryFlywheel->subSteps : 4;
            ShooterSqueezeModel::StepSqueeze(params, state, vRoller, dt, subSteps);

            // 6. Update piece state
            piece->transitProgressM = state.progressM;
            piece->transitVelocityMPerS = state.velocityMPerS;
            piece->currentCompressionM = state.compressionDepthM;
            piece->currentNormalForceN = state.normalForceN;
            piece->currentTractionForceN = state.tractionForceN;
            piece->cumulativeWorkJ = state.dissipatedEnergyJ;

            // 7. Inject braking load torque into flywheel
            primaryFlywheel->externalLoadTorqueNm += state.loadTorqueNm;
        }
    }

    void RobotGamePieceSystem::UpdateIntake(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt)
    {
        SIM_PROFILE_SCOPE("RobotGamePieceSystem::UpdateIntake");

        if (dt <= 1e-9)
        {
            return;
        }

        // 1. Locate active intake mechanism and contact zone
        FlywheelComponent* intakeFlywheel = nullptr;
        for (const auto& [name, entity] : robot.mechanismEntities)
        {
            auto* fw = scene.GetComponent<FlywheelComponent>(entity);
            if (fw && (fw->name.find("intake") != std::string::npos || name.find("intake") != std::string::npos))
            {
                intakeFlywheel = fw;
                break;
            }
        }

        Core::ECS::Core::Entity intakeZoneEntity = Core::ECS::Core::invalidEntity;
        ContactZoneComponent* czIntake = nullptr;
        for (const auto& [name, cze] : robot.contactZoneEntities)
        {
            auto* candidate = scene.GetComponent<ContactZoneComponent>(cze);
            if (candidate && candidate->role == "intake")
            {
                intakeZoneEntity = cze;
                czIntake = candidate;
                break;
            }
        }

        // Fallback search across scene if not in robot named maps
        if (!czIntake)
        {
            auto czEntities = scene.Query<ContactZoneComponent>();
            for (auto cze : czEntities)
            {
                auto* candidate = scene.GetComponent<ContactZoneComponent>(cze);
                if (candidate && candidate->role == "intake")
                {
                    intakeZoneEntity = cze;
                    czIntake = candidate;
                    break;
                }
            }
        }

        if (!czIntake)
        {
            return;
        }

        JPH::BodyInterface* bi = nullptr;
        if (robot.physicsSystem && robot.physicsSystem->GetJoltContext().IsInitialized())
        {
            bi = &robot.physicsSystem->GetJoltContext().GetBodyInterface();
        }

        // 2. Resolve Intake Zone World Transform & Tangent Direction
        Core::ECS::Core::Entity parentLink = czIntake->parentLink != Core::ECS::Core::invalidEntity
            ? czIntake->parentLink
            : (robot.chassisEntity != Core::ECS::Core::invalidEntity ? robot.chassisEntity : robot.rootLinkEntity);

        glm::vec3 linkPos(0.0f);
        glm::quat linkRot(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 parentLinkVel(0.0f);

        if (parentLink != Core::ECS::Core::invalidEntity)
        {
            auto* wt = scene.GetComponent<Core::ECS::Components::WorldTransformComponent>(parentLink);
            auto* tc = scene.GetComponent<Core::ECS::Components::TransformComponent>(parentLink);
            if (wt)
            {
                linkPos = wt->worldPosition;
                linkRot = wt->worldRotation;
            }
            else if (tc)
            {
                linkPos = tc->position;
                linkRot = tc->rotation;
            }

            auto* rbLink = scene.GetComponent<Physics::Components::RigidBodyComponent>(parentLink);
            if (bi && rbLink && !rbLink->bodyID.IsInvalid() && bi->IsAdded(rbLink->bodyID))
            {
                JPH::Vec3 pv = bi->GetLinearVelocity(rbLink->bodyID);
                parentLinkVel = glm::vec3(pv.GetX(), pv.GetY(), pv.GetZ());
            }
            else if (rbLink)
            {
                parentLinkVel = rbLink->linearVelocity;
            }
        }

        glm::vec3 localTan = glm::length(czIntake->tangentAxis) > 1e-4f ? glm::normalize(czIntake->tangentAxis) : glm::vec3(0.0f, 0.0f, 1.0f);
        glm::vec3 tangentWorld = glm::normalize(linkRot * localTan);
        glm::vec3 zoneWorldPos = linkPos + linkRot * czIntake->localOrigin;
        glm::quat zoneWorldRot = linkRot * czIntake->localRotation;

        double rollerSpeedRadS = intakeFlywheel ? intakeFlywheel->angularVelocityRadPerSec : 0.0;
        double rollerRadiusM = intakeFlywheel ? (intakeFlywheel->rollerRadiusM > 0.001 ? intakeFlywheel->rollerRadiusM : 0.0508) : 0.0508;
        glm::vec3 rollerSurfaceVel = IntakeSurfaceModel::CalculateRollerSurfaceVelocity(rollerSpeedRadS, rollerRadiusM, tangentWorld, parentLinkVel);

        // 3. Process All Game Pieces
        auto pieceEntities = scene.Query<GamePieceComponent>();
        bool anyPieceStoredOrEngaged = false;

        for (auto pEntity : pieceEntities)
        {
            auto* piece = scene.GetComponent<GamePieceComponent>(pEntity);
            if (!piece)
            {
                continue;
            }

            // Retrieve Piece World Position and Velocity
            glm::vec3 piecePos(0.0f);
            glm::vec3 pieceVel = piece->linearVelocity;

            if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
            {
                JPH::RVec3 jp = bi->GetPosition(piece->bodyID);
                JPH::Vec3 jv = bi->GetLinearVelocity(piece->bodyID);
                piecePos = glm::vec3(static_cast<float>(jp.GetX()), static_cast<float>(jp.GetY()), static_cast<float>(jp.GetZ()));
                pieceVel = glm::vec3(jv.GetX(), jv.GetY(), jv.GetZ());
            }
            else
            {
                auto* tcPiece = scene.GetComponent<Core::ECS::Components::TransformComponent>(pEntity);
                if (tcPiece)
                {
                    piecePos = tcPiece->position;
                }
            }

            double rPiece = piece->spec.outerRadiusM > 0.001 ? piece->spec.outerRadiusM : 0.1778;
            bool overlaps = IntakeSurfaceModel::TestZoneOverlap(piecePos, rPiece, zoneWorldPos, zoneWorldRot, czIntake->dimensions);

            // A. State: Free -> Overlaps Active Intake Zone -> Intaking
            if (piece->state == GamePieceState::Free)
            {
                if (overlaps && rollerSpeedRadS > 1.0)
                {
                    piece->state = GamePieceState::Intaking;
                    piece->capturingRobot = parentLink;
                    piece->boundContactZone = intakeZoneEntity;
                    piece->stateTimer = 0.0;
                }
            }

            // B. State: Intaking -> Compliant Surface Velocity Traction
            if (piece->state == GamePieceState::Intaking)
            {
                if (!overlaps)
                {
                    // Released or escaped before capture
                    piece->state = GamePieceState::Free;
                    piece->capturingRobot = Core::ECS::Core::invalidEntity;
                    piece->boundContactZone = Core::ECS::Core::invalidEntity;
                }
                else
                {
                    float progressAlongTan = glm::dot(piecePos - zoneWorldPos, tangentWorld);

                    IntakeParameters params;
                    params.rollerRadiusM = rollerRadiusM;
                    params.frictionMu = piece->spec.surfaceFriction > 0.01 ? piece->spec.surfaceFriction : 0.70;
                    params.normalForceN = 45.0;
                    params.viscousDampingNsPerM = 8.0;
                    params.maxGripForceN = 120.0;
                    params.centerlineCaptureThresholdM = 0.0; // Centerline threshold

                    auto traction = IntakeSurfaceModel::CalculateIntakeTraction(params, rollerSurfaceVel, pieceVel, tangentWorld, progressAlongTan);

                    // Apply gripping traction force to game piece
                    if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
                    {
                        bi->AddForce(piece->bodyID, JPH::Vec3(traction.intakeForceN.x, traction.intakeForceN.y, traction.intakeForceN.z));
                    }
                    else
                    {
                        double m = piece->spec.massKg > 0.01 ? piece->spec.massKg : 0.235;
                        piece->linearVelocity += (traction.intakeForceN / static_cast<float>(m)) * static_cast<float>(dt);
                    }

                    // Apply equal-and-opposite braking reaction torque to intake roller
                    if (intakeFlywheel)
                    {
                        intakeFlywheel->externalLoadTorqueNm += traction.loadTorqueNm;
                    }

                    // Centerline Crossing Threshold -> Kinematic Capture Handoff
                    if (traction.isPastCenterline)
                    {
                        piece->state = GamePieceState::Stored;
                        piece->linearVelocity = glm::vec3(0.0f);
                        piece->angularVelocity = glm::vec3(0.0f);
                        czIntake->isEngaged = true;

                        if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
                        {
                            bi->SetMotionType(piece->bodyID, JPH::EMotionType::Kinematic, JPH::EActivation::DontActivate);
                            bi->SetLinearVelocity(piece->bodyID, JPH::Vec3(0, 0, 0));
                            bi->SetAngularVelocity(piece->bodyID, JPH::Vec3(0, 0, 0));
                        }

                        auto* rb = scene.GetComponent<Physics::Components::RigidBodyComponent>(pEntity);
                        if (rb)
                        {
                            rb->motionType = Physics::Components::BodyMotionType::Kinematic;
                            rb->linearVelocity = glm::vec3(0.0f);
                            rb->angularVelocity = glm::vec3(0.0f);
                        }
                    }
                }
            }

            // C. State: Stored -> Retain in Hopper & Assert Sensor Engagement
            if (piece->state == GamePieceState::Stored)
            {
                anyPieceStoredOrEngaged = true;
                czIntake->isEngaged = true;

                // Lock position inside hopper relative to parent link
                glm::vec3 storedPos = zoneWorldPos + tangentWorld * 0.05f;
                auto* tcPiece = scene.GetComponent<Core::ECS::Components::TransformComponent>(pEntity);
                if (tcPiece)
                {
                    tcPiece->position = storedPos;
                    tcPiece->rotation = zoneWorldRot;
                }

                if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
                {
                    bi->SetPositionAndRotation(piece->bodyID, JPH::RVec3(storedPos.x, storedPos.y, storedPos.z),
                        JPH::Quat(zoneWorldRot.x, zoneWorldRot.y, zoneWorldRot.z, zoneWorldRot.w),
                        JPH::EActivation::DontActivate);
                    bi->SetLinearVelocity(piece->bodyID, JPH::Vec3(0, 0, 0));
                    bi->SetAngularVelocity(piece->bodyID, JPH::Vec3(0, 0, 0));
                }
            }
        }

        if (!pieceEntities.empty() && !anyPieceStoredOrEngaged && czIntake->isEngaged)
        {
            // Verify if any piece is still intaking/stored; if none, clear isEngaged
            bool hasActive = false;
            for (auto pEntity : pieceEntities)
            {
                auto* p = scene.GetComponent<GamePieceComponent>(pEntity);
                if (p && (p->state == GamePieceState::Stored || p->state == GamePieceState::Engaged))
                {
                    hasActive = true;
                    break;
                }
            }
            if (!hasActive)
            {
                czIntake->isEngaged = false;
            }
        }
    }

    void RobotGamePieceSystem::TriggerFeedToShooter(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot)
    {
        SIM_PROFILE_SCOPE("RobotGamePieceSystem::TriggerFeedToShooter");

        Core::ECS::Core::Entity shooterZoneEntity = Core::ECS::Core::invalidEntity;
        ContactZoneComponent* czShooter = nullptr;
        for (const auto& [name, cze] : robot.contactZoneEntities)
        {
            auto* candidate = scene.GetComponent<ContactZoneComponent>(cze);
            if (candidate && candidate->role == "shooter")
            {
                shooterZoneEntity = cze;
                czShooter = candidate;
                break;
            }
        }

        auto pieceEntities = scene.Query<GamePieceComponent>();
        for (auto pEntity : pieceEntities)
        {
            auto* piece = scene.GetComponent<GamePieceComponent>(pEntity);
            if (piece && piece->state == GamePieceState::Stored)
            {
                if (piece->boundContactZone != Core::ECS::Core::invalidEntity)
                {
                    auto* czOld = scene.GetComponent<ContactZoneComponent>(piece->boundContactZone);
                    if (czOld)
                    {
                        czOld->isEngaged = false;
                    }
                }

                piece->state = GamePieceState::Engaged;
                piece->boundContactZone = shooterZoneEntity;
                piece->transitProgressM = 0.0;
                piece->transitVelocityMPerS = 2.5; // Feed velocity into nip

                if (czShooter)
                {
                    czShooter->isEngaged = true;
                }
                break; // Feed one piece at a time
            }
        }
    }

    void RobotGamePieceSystem::PostFlywheelUpdate(Core::ECS::Core::Scene& scene, Robot::AssembledRobot& robot, double dt)
    {
        SIM_PROFILE_SCOPE("RobotGamePieceSystem::PostFlywheelUpdate");

        if (dt <= 1e-9)
        {
            return;
        }

        JPH::BodyInterface* bi = nullptr;
        if (robot.physicsSystem && robot.physicsSystem->GetJoltContext().IsInitialized())
        {
            bi = &robot.physicsSystem->GetJoltContext().GetBodyInterface();
        }

        auto pieceEntities = scene.Query<GamePieceComponent>();
        for (auto pEntity : pieceEntities)
        {
            auto* piece = scene.GetComponent<GamePieceComponent>(pEntity);
            if (!piece || piece->state != GamePieceState::Engaged)
            {
                continue;
            }

            ContactZoneComponent* cz = nullptr;
            if (piece->boundContactZone != Core::ECS::Core::invalidEntity)
            {
                cz = scene.GetComponent<ContactZoneComponent>(piece->boundContactZone);
            }

            double contactLength = cz ? GetContactLength(*cz) : 0.18;

            // Check if projectile reached the muzzle threshold
            if (piece->transitProgressM >= (contactLength - 1e-4))
            {
                // 1. Resolve parent link transform and geometric axes
                Core::ECS::Core::Entity parentLink = cz ? cz->parentLink : Core::ECS::Core::invalidEntity;
                if (parentLink == Core::ECS::Core::invalidEntity)
                {
                    parentLink = robot.chassisEntity != Core::ECS::Core::invalidEntity ? robot.chassisEntity : robot.rootLinkEntity;
                }

                glm::vec3 linkPos(0.0f);
                glm::quat linkRot(1.0f, 0.0f, 0.0f, 0.0f);
                if (parentLink != Core::ECS::Core::invalidEntity)
                {
                    auto* wt = scene.GetComponent<Core::ECS::Components::WorldTransformComponent>(parentLink);
                    auto* tc = scene.GetComponent<Core::ECS::Components::TransformComponent>(parentLink);
                    if (wt)
                    {
                        linkPos = wt->worldPosition;
                        linkRot = wt->worldRotation;
                    }
                    else if (tc)
                    {
                        linkPos = tc->position;
                        linkRot = tc->rotation;
                    }
                }

                glm::vec3 localTan = (cz && glm::length(cz->tangentAxis) > 1e-4f) ? glm::normalize(cz->tangentAxis) : glm::vec3(0.0f, 0.0f, 1.0f);
                glm::vec3 localComp = (cz && glm::length(cz->compressionAxis) > 1e-4f) ? glm::normalize(cz->compressionAxis) : glm::vec3(0.0f, 1.0f, 0.0f);
                glm::vec3 localOrigin = cz ? cz->localOrigin : glm::vec3(0.0f);
                glm::quat localRot = cz ? cz->localRotation : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

                glm::vec3 tangentWorld = glm::normalize(linkRot * localTan);
                glm::vec3 compWorld = glm::normalize(linkRot * localComp);
                glm::vec3 spinAxisWorld = glm::cross(compWorld, tangentWorld);
                if (glm::length(spinAxisWorld) > 1e-4f)
                {
                    spinAxisWorld = glm::normalize(spinAxisWorld);
                }
                else
                {
                    spinAxisWorld = glm::vec3(1.0f, 0.0f, 0.0f);
                }

                glm::vec3 zoneCenterWorld = linkPos + linkRot * localOrigin;
                glm::quat zoneRotWorld = linkRot * localRot;
                glm::vec3 muzzlePos = zoneCenterWorld + tangentWorld * (0.5f * static_cast<float>(contactLength));

                // 2. Identify shooter rigid body for linear recoil impulse
                Physics::Components::RigidBodyComponent* shooterRB = nullptr;
                if (parentLink != Core::ECS::Core::invalidEntity)
                {
                    shooterRB = scene.GetComponent<Physics::Components::RigidBodyComponent>(parentLink);
                }
                if (!shooterRB && robot.chassisEntity != Core::ECS::Core::invalidEntity)
                {
                    shooterRB = scene.GetComponent<Physics::Components::RigidBodyComponent>(robot.chassisEntity);
                }
                if (!shooterRB && robot.rootLinkEntity != Core::ECS::Core::invalidEntity)
                {
                    shooterRB = scene.GetComponent<Physics::Components::RigidBodyComponent>(robot.rootLinkEntity);
                }

                // 3. Compute robot velocity at muzzle position: v_robot,muzzle
                glm::vec3 vRobotMuzzle(0.0f);
                if (bi && shooterRB && !shooterRB->bodyID.IsInvalid() && bi->IsAdded(shooterRB->bodyID))
                {
                    JPH::Vec3 pv = bi->GetPointVelocity(shooterRB->bodyID, JPH::RVec3(muzzlePos.x, muzzlePos.y, muzzlePos.z));
                    vRobotMuzzle = glm::vec3(pv.GetX(), pv.GetY(), pv.GetZ());
                }
                else if (shooterRB)
                {
                    glm::vec3 r = muzzlePos - linkPos;
                    vRobotMuzzle = shooterRB->linearVelocity + glm::cross(shooterRB->angularVelocity, r);
                }

                // 4. Compute exit velocity and linear recoil impulse
                // v_rel = v_piece_rel * t_world
                // v_exit = v_robot_muzzle + v_rel
                // J_recoil = -m_piece * v_rel
                float vRel = static_cast<float>(piece->transitVelocityMPerS);
                glm::vec3 vRelVec = tangentWorld * vRel;
                glm::vec3 vExit = vRobotMuzzle + vRelVec;

                double pieceMass = piece->spec.massKg > 0.01 ? piece->spec.massKg : 0.235;
                glm::vec3 J_recoil = -static_cast<float>(pieceMass) * vRelVec;

                // Apply linear recoil impulse to shooter link
                if (bi && shooterRB && !shooterRB->bodyID.IsInvalid() && bi->IsAdded(shooterRB->bodyID))
                {
                    bi->AddImpulse(shooterRB->bodyID, JPH::Vec3(J_recoil.x, J_recoil.y, J_recoil.z),
                                   JPH::RVec3(muzzlePos.x, muzzlePos.y, muzzlePos.z));
                }
                else if (shooterRB && shooterRB->mass > 0.0f)
                {
                    shooterRB->linearVelocity += (J_recoil / shooterRB->mass);
                }

                // 5. Compute exit spin: omega_exit = [(v_top - v_bottom) / (2 * r_piece)] * s_world
                double rollerRadius = 0.0508;
                double vRoller = 0.0;
                for (const auto& [name, entity] : robot.mechanismEntities)
                {
                    auto* fw = scene.GetComponent<FlywheelComponent>(entity);
                    if (fw)
                    {
                        if (fw->rollerRadiusM > 0.001)
                        {
                            rollerRadius = fw->rollerRadiusM;
                        }
                        vRoller = fw->angularVelocityRadPerSec * rollerRadius;
                        break;
                    }
                }
                double rPiece = piece->spec.outerRadiusM > 0.01 ? piece->spec.outerRadiusM : 0.1778;
                double omegaSpin = vRoller / (2.0 * rPiece);
                glm::vec3 omegaExit = spinAxisWorld * static_cast<float>(omegaSpin);

                // 6. Transition state to Launched
                piece->state = GamePieceState::Launched;
                piece->linearVelocity = vExit;
                piece->angularVelocity = omegaExit;
                piece->stateTimer = 0.0;
                piece->muzzleCollisionMaskTicks = 3; // 3 ticks (15 ms) collision masking
                piece->boundContactZone = Core::ECS::Core::invalidEntity;
                piece->capturingRobot = Core::ECS::Core::invalidEntity;

                if (cz)
                {
                    cz->isEngaged = false;
                }

                // 7. Update Projectile ECS components and Jolt body
                auto* pieceRB = scene.GetComponent<Physics::Components::RigidBodyComponent>(pEntity);
                if (pieceRB)
                {
                    pieceRB->motionType = Physics::Components::BodyMotionType::Dynamic;
                    pieceRB->linearVelocity = vExit;
                    pieceRB->angularVelocity = omegaExit;
                    if (shooterRB && shooterRB->collisionGroupID != 0)
                    {
                        pieceRB->collisionGroupID = shooterRB->collisionGroupID;
                        pieceRB->collisionSubGroupID = 30;
                    }
                    if (piece->bodyID.IsInvalid() && !pieceRB->bodyID.IsInvalid())
                    {
                        piece->bodyID = pieceRB->bodyID;
                    }
                }

                if (bi && !piece->bodyID.IsInvalid() && bi->IsAdded(piece->bodyID))
                {
                    bi->SetPositionAndRotation(piece->bodyID, JPH::RVec3(muzzlePos.x, muzzlePos.y, muzzlePos.z),
                        JPH::Quat(zoneRotWorld.x, zoneRotWorld.y, zoneRotWorld.z, zoneRotWorld.w), JPH::EActivation::Activate);
                    bi->SetMotionType(piece->bodyID, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                    bi->SetMotionQuality(piece->bodyID, JPH::EMotionQuality::LinearCast);
                    bi->SetLinearVelocity(piece->bodyID, JPH::Vec3(vExit.x, vExit.y, vExit.z));
                    bi->SetAngularVelocity(piece->bodyID, JPH::Vec3(omegaExit.x, omegaExit.y, omegaExit.z));

                    if (robot.physicsSystem && shooterRB && shooterRB->collisionGroupID != 0)
                    {
                        bi->SetCollisionGroup(piece->bodyID, JPH::CollisionGroup(
                            robot.physicsSystem->GetGroupFilterTable(), shooterRB->collisionGroupID, 30));
                    }
                }

                auto* tcPiece = scene.GetComponent<Core::ECS::Components::TransformComponent>(pEntity);
                if (tcPiece)
                {
                    tcPiece->position = muzzlePos;
                    tcPiece->rotation = zoneRotWorld;
                }
                auto* wtPiece = scene.GetComponent<Core::ECS::Components::WorldTransformComponent>(pEntity);
                if (wtPiece)
                {
                    wtPiece->worldPosition = muzzlePos;
                    wtPiece->worldRotation = zoneRotWorld;
                }
            }
        }
    }
}
