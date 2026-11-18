/**
 * @file BananaGame.cpp
 * @brief Implementation of the Robot Comp 2 arcade shootout mini-game.
 */

#include "Game/BananaGame.h"

#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Renderer/Components/MeshComponent.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Interaction/Systems/RobotGamePieceSystem.h"
#include "Interaction/Systems/FlightAerodynamicsSystem.h"
#include "Utils/DebugDraw.h"
#include "Utils/Log.h"

#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

using namespace FRCBananaSim::Core::ECS::Core;
using namespace FRCBananaSim::Core::ECS::Components;
using namespace FRCBananaSim::Physics::Components;
using namespace FRCBananaSim::Renderer::Components;
using namespace FRCBananaSim::Control;
using namespace FRCBananaSim::Robot::Mechanisms::Components;
using namespace FRCBananaSim::Robot::Sensors::Components;
using namespace FRCBananaSim::Interaction::Components;

namespace FRCBananaSim::Game
{
    BananaGame::BananaGame(Scene& scene, Physics::JoltContext& joltContext, Renderer::Renderer& renderer)
        : m_scene(scene),
          m_joltContext(joltContext),
          m_renderer(renderer)
    {
    }

    BananaGame::~BananaGame()
    {
    }

    bool BananaGame::Initialize()
    {
        SIM_LOG_INFO("[BananaGame] Initializing Robot Comp 2 Arcade Shootout Game...");

        // 1. Locate Robot Comp 2 CAD & Sidecar resources
        std::string manifestPath = "FRCBananaSim/Resources/Robots/Robot Comp 2/manifest.json";
        std::string sidecarPath = "FRCBananaSim/Resources/Robots/Robot Comp 2/sidecar.json";
        std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

        const std::vector<std::string> searchPrefixes = {
            "", "../", "../../", "../../../", "../../../../", "FRCBananaSim/"
        };

        bool foundRobot = false;
        for (const auto& prefix : searchPrefixes)
        {
            std::string candidate = prefix + manifestPath;
            if (std::filesystem::exists(candidate))
            {
                manifestPath = candidate;
                sidecarPath = prefix + "FRCBananaSim/Resources/Robots/Robot Comp 2/sidecar.json";
                materialsPath = prefix + "FRCBananaSim/Resources/Config/materials.json";
                foundRobot = true;
                break;
            }
            std::string directCandidate = prefix + "Resources/Robots/Robot Comp 2/manifest.json";
            if (std::filesystem::exists(directCandidate))
            {
                manifestPath = directCandidate;
                sidecarPath = prefix + "Resources/Robots/Robot Comp 2/sidecar.json";
                materialsPath = prefix + "Resources/Config/materials.json";
                foundRobot = true;
                break;
            }
        }

        if (!foundRobot)
        {
            SIM_LOG_FATAL("[BananaGame] Could not locate Robot Comp 2 resources at: " << manifestPath);
            return false;
        }

        // 2. Assemble Robot Comp 2 in World Space
        Robot::RobotAssembler assembler(m_scene, &m_joltContext);
        const glm::vec3 spawnPos(0.0f, 0.20f, -2.5f); // Centered on carpet, facing forward toward goal (+Z)
        const glm::quat spawnRot(1.0f, 0.0f, 0.0f, 0.0f);

        m_robot = assembler.AssembleFromFiles(manifestPath, sidecarPath, materialsPath, spawnPos, spawnRot);
        if (!m_robot.isSuccess)
        {
            SIM_LOG_FATAL("[BananaGame] Failed to assemble Robot Comp 2: " << m_robot.errorMessage);
            return false;
        }

        // 3. Allocate Joint System for Jolt constraints management
        m_jointSystem = std::make_unique<Physics::Systems::JointSystem>(m_scene, m_joltContext);

        // Warmup: Allocate all Jolt rigid bodies and joint constraints
        m_scene.FixedUpdateSystems();
        if (m_jointSystem)
        {
            m_jointSystem->FixedUpdate(Physics::PhysicsConfig::fixedDeltaTime);
        }

        // 4. Cache Mechanism Component Pointers
        for (const auto& [name, entity] : m_robot.mechanismEntities)
        {
            if (name == "Turret")
            {
                m_turretArm = m_scene.GetComponent<ArmComponent>(entity);
            }
            else if (name == "IntakePivot")
            {
                m_intakePivotArm = m_scene.GetComponent<ArmComponent>(entity);
            }
            else if (name == "Flywheel")
            {
                m_flywheel = m_scene.GetComponent<FlywheelComponent>(entity);
            }
            else if (name == "FrontIntake" || name == "FronIntake")
            {
                m_frontIntake = m_scene.GetComponent<FlywheelComponent>(entity);
            }
            else if (name == "BackIntake")
            {
                m_backIntake = m_scene.GetComponent<FlywheelComponent>(entity);
            }
            else if (name == "BottomBasket")
            {
                m_bottomBasket = m_scene.GetComponent<FlywheelComponent>(entity);
            }
            else if (name == "Funnel")
            {
                m_funnel = m_scene.GetComponent<FlywheelComponent>(entity);
            }
        }

        // 5. Construct Arena Environment & Visuals
        SetupArena();
        SetupRobotVisuals();

        // 6. Spawn Initial Field Balls
        ResetField();

        // Configure Initial Camera Position
        auto& camera = m_renderer.GetCamera();
        camera.SetMode(Renderer::Camera::Mode::Orbit);
        camera.orbitTarget = spawnPos + glm::vec3(0.0f, 0.4f, 0.0f);
        m_smoothCameraPos = spawnPos + glm::vec3(0.0f, 2.2f, -3.8f);
        m_smoothCameraTarget = spawnPos + glm::vec3(0.0f, 0.4f, 1.0f);

        SIM_LOG_INFO("[BananaGame] Robot Comp 2 Arcade Shootout Game initialized successfully!");
        return true;
    }

    void BananaGame::SetupArena()
    {
        auto assignTransforms = [&](Entity entity, const glm::vec3& pos, const glm::vec3& scale,
                                    const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        {
            TransformComponent t;
            t.position = pos;
            t.scale = scale;
            t.rotation = rot;
            m_scene.AddComponent(entity, t);

            WorldTransformComponent wt;
            wt.worldPosition = pos;
            wt.worldScale = scale;
            wt.worldRotation = rot;
            m_scene.AddComponent(entity, wt);
        };

        // A. FRC Field Carpet (18m x 14m, Surface at Y = 0.0m)
        Entity carpet = m_scene.CreateEntity();
        {
            assignTransforms(carpet, glm::vec3(0.0f, -0.05f, 0.0f), glm::vec3(18.0f, 0.10f, 14.0f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Plane;
            mesh.color = glm::vec3(0.14f, 0.40f, 0.18f); // FRC Carpet Green
            m_scene.AddComponent(carpet, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.85f;
            rb.restitution = 0.05f;
            m_scene.AddComponent(carpet, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(9.0f, 0.05f, 7.0f);
            m_scene.AddComponent(carpet, col);
        }

        // B. Perimeter Arena Barrier Walls
        auto createWall = [&](const glm::vec3& pos, const glm::vec3& size, const glm::vec3& color)
        {
            Entity wall = m_scene.CreateEntity();
            assignTransforms(wall, pos, size);

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = color;
            m_scene.AddComponent(wall, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.60f;
            rb.restitution = 0.10f;
            m_scene.AddComponent(wall, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = size * 0.5f;
            m_scene.AddComponent(wall, col);
        };

        const glm::vec3 wallColor(0.22f, 0.32f, 0.45f); // Steel Blue Perimeter Walls
        // North Wall (+Z far end behind goal)
        createWall(glm::vec3(0.0f, 0.6f, 7.0f), glm::vec3(18.0f, 1.2f, 0.3f), wallColor);
        // South Wall (-Z driver station side)
        createWall(glm::vec3(0.0f, 0.6f, -7.0f), glm::vec3(18.0f, 1.2f, 0.3f), glm::vec3(0.20f, 0.28f, 0.60f));
        // East Wall (+X right side)
        createWall(glm::vec3(9.0f, 0.6f, 0.0f), glm::vec3(0.3f, 1.2f, 14.0f), wallColor);
        // West Wall (-X left side)
        createWall(glm::vec3(-9.0f, 0.6f, 0.0f), glm::vec3(0.3f, 1.2f, 14.0f), wallColor);

        // C. Target High Goal Tower & Hoop (Centered at X = 0.0m, Y = 2.2m, Z = 5.5m)
        m_goalCenter = glm::vec3(0.0f, 2.20f, 5.5f);

        // 1. Goal Support Post
        Entity goalPost = m_scene.CreateEntity();
        {
            assignTransforms(goalPost, glm::vec3(0.0f, 1.0f, 5.9f), glm::vec3(0.20f, 2.0f, 0.20f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.30f, 0.32f, 0.35f);
            m_scene.AddComponent(goalPost, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            m_scene.AddComponent(goalPost, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.10f, 1.0f, 0.10f);
            m_scene.AddComponent(goalPost, col);
        }

        // 2. Goal Target Backboard
        Entity backboard = m_scene.CreateEntity();
        {
            assignTransforms(backboard, glm::vec3(0.0f, 2.35f, 5.95f), glm::vec3(1.80f, 1.10f, 0.08f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.85f, 0.18f, 0.22f); // Alliance Red Backboard
            m_scene.AddComponent(backboard, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            m_scene.AddComponent(backboard, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.90f, 0.55f, 0.04f);
            m_scene.AddComponent(backboard, col);
        }

        // 3. Goal Rim / Funnel Catch Bounding Structure
        Entity rimFront = m_scene.CreateEntity();
        {
            assignTransforms(rimFront, glm::vec3(0.0f, 2.15f, 5.0f), glm::vec3(1.10f, 0.12f, 0.08f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.98f, 0.85f, 0.12f); // Safety Yellow Rim
            m_scene.AddComponent(rimFront, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            m_scene.AddComponent(rimFront, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.55f, 0.06f, 0.04f);
            m_scene.AddComponent(rimFront, col);
        }

        Entity rimLeft = m_scene.CreateEntity();
        {
            assignTransforms(rimLeft, glm::vec3(-0.55f, 2.15f, 5.5f), glm::vec3(0.08f, 0.12f, 0.95f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.98f, 0.85f, 0.12f);
            m_scene.AddComponent(rimLeft, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            m_scene.AddComponent(rimLeft, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.04f, 0.06f, 0.48f);
            m_scene.AddComponent(rimLeft, col);
        }

        Entity rimRight = m_scene.CreateEntity();
        {
            assignTransforms(rimRight, glm::vec3(0.55f, 2.15f, 5.5f), glm::vec3(0.08f, 0.12f, 0.95f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.98f, 0.85f, 0.12f);
            m_scene.AddComponent(rimRight, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            m_scene.AddComponent(rimRight, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(0.04f, 0.06f, 0.48f);
            m_scene.AddComponent(rimRight, col);
        }
    }

    void BananaGame::SetupRobotVisuals()
    {
        auto attachVisual = [&](Entity linkEntity, PrimitiveShape shape, const glm::vec3& color,
                                const glm::vec3& fallbackScale = glm::vec3(0.04f),
                                const glm::vec3& fallbackOffset = glm::vec3(0.0f))
        {
            if (linkEntity == invalidEntity) return;

            auto* linkT = m_scene.GetComponent<TransformComponent>(linkEntity);
            if (!linkT) return;

            auto* col = m_scene.GetComponent<ColliderComponent>(linkEntity);
            glm::vec3 visualOffset = col ? col->localOffset : fallbackOffset;
            glm::quat visualRot = col ? col->localRotation : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 visualScale = fallbackScale;

            if (col && col->shape == ColliderShape::Box)
            {
                visualScale = col->boxHalfExtents * 2.0f;
            }

            Entity visual = m_scene.CreateEntity();
            TransformComponent vt;
            vt.position = visualOffset;
            vt.rotation = visualRot;
            vt.scale = visualScale;
            m_scene.AddComponent(visual, vt);

            WorldTransformComponent vwt;
            vwt.worldPosition = linkT->position + (linkT->rotation * visualOffset);
            vwt.worldRotation = linkT->rotation * visualRot;
            vwt.worldScale = visualScale;
            m_scene.AddComponent(visual, vwt);

            HierarchyComponent vh;
            vh.parent = linkEntity;
            m_scene.AddComponent(visual, vh);

            auto* linkH = m_scene.GetComponent<HierarchyComponent>(linkEntity);
            if (linkH)
            {
                linkH->children.push_back(visual);
            }
            else
            {
                HierarchyComponent newH;
                newH.children.push_back(visual);
                m_scene.AddComponent(linkEntity, newH);
            }

            MeshComponent mesh;
            mesh.shape = shape;
            mesh.color = color;
            m_scene.AddComponent(visual, mesh);
        };

        // Attach rich color-coded visuals to all Robot Comp 2 links
        for (const auto& [name, entity] : m_robot.linkEntities)
        {
            if (entity == invalidEntity) continue;

            if (name == "base_link")
            {
                // Dark Titanium Chassis Body
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.16f, 0.22f, 0.32f));
                // Add Red Perimeter Bumper Visual
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.85f, 0.15f, 0.18f),
                             glm::vec3(0.92f, 0.14f, 0.80f), glm::vec3(0.0f, 0.09f, 0.0f));
            }
            else if (name == "gear")
            {
                // Turret Ring Brass Gear
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.90f, 0.72f, 0.18f));
            }
            else if (name == "sidepannel")
            {
                // Basket Hopper Side Panels
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.24f, 0.27f, 0.31f));
            }
            else if (name == "moving_hood")
            {
                // Adjustable Shooter Hood (Neon Cyan)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.12f, 0.75f, 0.92f));
            }
            else if (name == "side_plates_2")
            {
                // Intake Pivot Arm (Anodized Orange)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.96f, 0.48f, 0.08f));
            }
            else if (name == "wheels")
            {
                // High-RPM Flywheel Rollers (Safety Yellow)
                attachVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.98f, 0.88f, 0.12f));
            }
            else if (name == "side_plates")
            {
                // Funnel Side Plates (Brushed Aluminum)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.70f, 0.74f, 0.80f));
            }
            else if (name.find("hex_shaft") != std::string::npos)
            {
                // Indexer Basket Rollers (Vibrant Green)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.20f, 0.82f, 0.35f));
            }
            else if (name.find("sprocket") != std::string::npos)
            {
                // Intake Grip Rollers (Grippy Purple/Red)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.78f, 0.25f, 0.40f));
            }
        }
    }

    Entity BananaGame::SpawnBall(const glm::vec3& position, bool inShooter)
    {
        Entity ballEntity = m_scene.CreateEntity();

        GamePieceComponent piece;
        piece.spec.name = "Cargo Ball (Bouncy Foam)";
        piece.spec.massKg = 0.270;
        piece.spec.thicknessM = 0.2413;
        piece.spec.outerRadiusM = 0.1206; // 9.5-inch diameter sphere (0.12m radius)
        piece.spec.dragCoefficientCd = 0.47;
        piece.spec.liftSlopeClAlpha = 0.85;
        piece.spec.crossSectionalAreaM2 = 0.0457;
        piece.spec.complianceK = 28000.0;
        piece.spec.restitutionE = 0.35;
        piece.spec.surfaceFriction = 0.72;

        TransformComponent tc;
        tc.position = position;
        tc.scale = glm::vec3(static_cast<float>(piece.spec.outerRadiusM * 2.0));
        m_scene.AddComponent(ballEntity, tc);

        WorldTransformComponent wt;
        wt.worldPosition = position;
        wt.worldScale = tc.scale;
        m_scene.AddComponent(ballEntity, wt);

        MeshComponent mesh;
        mesh.shape = PrimitiveShape::Sphere;
        mesh.color = glm::vec3(0.98f, 0.62f, 0.08f); // Bright High-Visibility Orange Foam
        m_scene.AddComponent(ballEntity, mesh);

        if (inShooter)
        {
            piece.state = GamePieceState::Engaged;
            auto czIt = m_robot.contactZoneEntities.find("FlyWHeelZone");
            if (czIt != m_robot.contactZoneEntities.end())
            {
                piece.boundContactZone = czIt->second;
            }
            piece.capturingRobot = m_robot.chassisEntity;
            piece.transitProgressM = 0.0;
            piece.transitVelocityMPerS = 2.0;
        }
        else
        {
            piece.state = GamePieceState::Free;
            piece.capturingRobot = invalidEntity;

            RigidBodyComponent rb;
            rb.mass = static_cast<float>(piece.spec.massKg);
            rb.motionType = BodyMotionType::Dynamic;
            rb.friction = 0.70f;
            rb.restitution = 0.35f;
            rb.allowSleeping = true;
            m_scene.AddComponent(ballEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Sphere;
            col.radius = static_cast<float>(piece.spec.outerRadiusM);
            m_scene.AddComponent(ballEntity, col);
        }

        m_scene.AddComponent(ballEntity, piece);
        m_ballEntities.push_back(ballEntity);

        return ballEntity;
    }

    void BananaGame::ResetField()
    {
        // 1. Remove Existing Balls
        for (Entity e : m_ballEntities)
        {
            if (m_scene.IsEntityValid(e))
            {
                auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                if (rb && !rb->bodyID.IsInvalid() && m_joltContext.IsInitialized())
                {
                    m_joltContext.GetBodyInterface().RemoveBody(rb->bodyID);
                    m_joltContext.GetBodyInterface().DestroyBody(rb->bodyID);
                }
                m_scene.DestroyEntity(e);
            }
        }
        m_ballEntities.clear();
        m_ballTrails.clear();

        // 2. Spawn 12 Foam Balls in Strategic Formations
        // Centerline Wing Clusters
        const std::vector<glm::vec3> spawnLocations = {
            // Midfield line
            { -3.0f, 0.15f, 1.0f },
            { -1.5f, 0.15f, 1.2f },
            {  0.0f, 0.15f, 1.5f },
            {  1.5f, 0.15f, 1.2f },
            {  3.0f, 0.15f, 1.0f },
            // Left Wing
            { -4.5f, 0.15f, -1.0f },
            { -3.5f, 0.15f, -2.2f },
            { -2.0f, 0.15f, -3.2f },
            // Right Wing
            {  4.5f, 0.15f, -1.0f },
            {  3.5f, 0.15f, -2.2f },
            {  2.0f, 0.15f, -3.2f },
            // Near Spawn
            {  0.0f, 0.15f, -1.0f }
        };

        for (const auto& pos : spawnLocations)
        {
            SpawnBall(pos);
        }

        // 3. Reset Robot Position & Motion
        const glm::vec3 startPos(0.0f, 0.20f, -4.5f);
        const glm::quat startRot(1.0f, 0.0f, 0.0f, 0.0f);

        auto* chassisRb = m_scene.GetComponent<RigidBodyComponent>(m_robot.chassisEntity);
        if (chassisRb && !chassisRb->bodyID.IsInvalid() && m_joltContext.IsInitialized())
        {
            JPH::BodyInterface& bi = m_joltContext.GetBodyInterface();
            bi.SetPositionAndRotation(chassisRb->bodyID,
                JPH::RVec3(startPos.x, startPos.y, startPos.z),
                JPH::Quat(startRot.x, startRot.y, startRot.z, startRot.w),
                JPH::EActivation::Activate);
            bi.SetLinearAndAngularVelocity(chassisRb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        }

        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (chassisT)
        {
            chassisT->position = startPos;
            chassisT->rotation = startRot;
        }
        auto* chassisWt = m_scene.GetComponent<WorldTransformComponent>(m_robot.chassisEntity);
        if (chassisWt)
        {
            chassisWt->worldPosition = startPos;
            chassisWt->worldRotation = startRot;
        }

        if (m_robot.swerveSystem && m_robot.swerveSystem->GetModel())
        {
            glm::vec3 posNWU = Robot::Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(startPos);
            glm::quat rotNWU = glm::quat(Robot::Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(glm::dquat(startRot)));
            m_robot.swerveSystem->GetModel()->ApplyExternalCorrections(posNWU, rotNWU, glm::vec3(0.0f), glm::vec3(0.0f));
        }

        m_fieldCentricHeadingRad = 0.0f;
    }

    void BananaGame::StartGame(GameMode mode)
    {
        m_mode = mode;
        m_score = 0;
        m_comboStreak = 0;
        m_maxCombo = 0;
        m_ballsCollected = 0;
        m_ballsFired = 0;
        m_goalsScored = 0;
        m_sessionTimer = (mode == GameMode::ArcadeRush) ? 60.0f : 9999.0f;
        m_countdownTimer = 3.0f;
        m_state = GameState::Countdown;

        ResetField();
    }

    void BananaGame::FireShooter()
    {
        // Check if there is a ball currently stored inside the robot indexer/throat
        for (Entity e : m_ballEntities)
        {
            if (!m_scene.IsEntityValid(e)) continue;

            auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
            if (!piece) continue;

            if (piece->state == GamePieceState::Stored || piece->state == GamePieceState::Intaking)
            {
                // Transition ball directly into shooter compression zone
                piece->state = GamePieceState::Engaged;
                auto czIt = m_robot.contactZoneEntities.find("FlyWHeelZone");
                if (czIt != m_robot.contactZoneEntities.end())
                {
                    piece->boundContactZone = czIt->second;
                }
                piece->capturingRobot = m_robot.chassisEntity;
                piece->transitProgressM = 0.0;
                piece->transitVelocityMPerS = 3.0;
                m_ballsFired++;
                return;
            }
        }

        // If no ball was currently captured inside, spawn one directly into the shooter muzzle for rapid fun!
        SpawnBall(glm::vec3(0.0f, 0.45f, 0.0f), true);
        m_ballsFired++;
    }

    void BananaGame::FixedUpdate(double dt)
    {
        // 1. Joint System pre-physics step
        if (m_jointSystem)
        {
            m_jointSystem->FixedUpdate(dt);
        }

        // 2. Arcade Robot Controller
        UpdateArcadeRobotController(dt);

        // 3. Step Official 8-Stage Robot Execution Pipeline
        Robot::RobotAssembler::StepPipeline(m_robot, dt);

        // 4. Joint System telemetry readback
        if (m_jointSystem)
        {
            m_jointSystem->ReadbackTelemetry();
        }

        // 5. Goal Trigger & Scoring Pass
        if (m_state == GameState::Playing || m_state == GameState::Countdown)
        {
            CheckGoalScoring();
        }

        // 6. Record Ballistic Trajectory Trails
        for (Entity e : m_ballEntities)
        {
            if (!m_scene.IsEntityValid(e)) continue;

            auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
            auto* tc = m_scene.GetComponent<TransformComponent>(e);
            if (!piece || !tc) continue;

            if (piece->state == GamePieceState::Launched)
            {
                auto& trail = m_ballTrails[e];
                if (trail.empty() || glm::distance(trail.back(), tc->position) > 0.04f)
                {
                    trail.push_back(tc->position);
                    if (trail.size() > 120)
                    {
                        trail.erase(trail.begin());
                    }
                }
            }
        }
    }

    void BananaGame::UpdateArcadeRobotController(double dt)
    {
        SimInputs inputs;
        inputs.robotEnabled = (m_state == GameState::Playing);

        if (m_state != GameState::Playing)
        {
            if (m_robot.swerveSystem)
            {
                m_robot.swerveSystem->SetInputs(inputs);
            }
            return;
        }

        auto& io = m_renderer.GetInput();

        // A. Keyboard Swerve Driving (WASD + Q/E)
        float fieldVx = 0.0f;
        float fieldVz = 0.0f;
        float spinRate = 0.0f;

        if (io.IsKeyDown(GLFW_KEY_W) || io.IsKeyDown(GLFW_KEY_UP))    fieldVz += 10.5f;
        if (io.IsKeyDown(GLFW_KEY_S) || io.IsKeyDown(GLFW_KEY_DOWN))  fieldVz -= 10.5f;
        if (io.IsKeyDown(GLFW_KEY_A) || io.IsKeyDown(GLFW_KEY_LEFT))  fieldVx += 10.5f;
        if (io.IsKeyDown(GLFW_KEY_D) || io.IsKeyDown(GLFW_KEY_RIGHT)) fieldVx -= 10.5f;
        if (io.IsKeyDown(GLFW_KEY_Q)) spinRate += 9.0f;
        if (io.IsKeyDown(GLFW_KEY_E)) spinRate -= 9.0f;

        // Toggle Field-Centric mode with H
        if (io.WasKeyPressed(GLFW_KEY_H))
        {
            m_fieldCentric = !m_fieldCentric;
        }

        // Zero Gyro Heading with Z
        if (io.WasKeyPressed(GLFW_KEY_Z))
        {
            auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
            if (chassisT)
            {
                glm::vec3 euler = glm::eulerAngles(chassisT->rotation);
                m_fieldCentricHeadingRad = euler.y;
            }
        }

        // Get Chassis Yaw for Field-Centric Translation
        float chassisYaw = 0.0f;
        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (chassisT)
        {
            glm::vec3 euler = glm::eulerAngles(chassisT->rotation);
            chassisYaw = euler.y - m_fieldCentricHeadingRad;
        }

        float cmdVx = fieldVz; // In NWU: +X is Forward (towards goal Z)
        float cmdVy = fieldVx; // In NWU: +Y is Left (towards -X)

        if (m_fieldCentric)
        {
            // Rotate field translation vector into robot chassis frame
            float cosY = std::cos(-chassisYaw);
            float sinY = std::sin(-chassisYaw);
            cmdVx = (fieldVz * cosY) - (fieldVx * sinY);
            cmdVy = (fieldVz * sinY) + (fieldVx * cosY);
        }

        // 4-Module Swerve Kinematics Demands
        constexpr double kP = 18.0;
        constexpr double kD = 0.6;
        constexpr double kInvSqrt2 = 0.7071067811865475;
        constexpr double modRx[4] = { +kInvSqrt2, +kInvSqrt2, -kInvSqrt2, -kInvSqrt2 };
        constexpr double modRy[4] = { +kInvSqrt2, -kInvSqrt2, +kInvSqrt2, -kInvSqrt2 };

        double modVx[4] = { 0.0, 0.0, 0.0, 0.0 };
        double modVy[4] = { 0.0, 0.0, 0.0, 0.0 };
        double modMag[4] = { 0.0, 0.0, 0.0, 0.0 };
        double maxModMag = 0.0;

        for (uint32_t i = 0; i < 4; ++i)
        {
            modVx[i] = cmdVx - (spinRate * modRy[i]);
            modVy[i] = cmdVy + (spinRate * modRx[i]);
            modMag[i] = std::hypot(modVx[i], modVy[i]);
            maxModMag = std::max(maxModMag, modMag[i]);
        }

        double desaturate = (maxModMag > 12.0) ? (12.0 / maxModMag) : 1.0;
        auto outputs = m_robot.sensorSystem ? m_robot.sensorSystem->GetOutputs() : SimOutputs{};

        for (uint32_t i = 0; i < 4; ++i)
        {
            double targetAngle = 0.0;
            double driveV = 0.0;

            if (modMag[i] > 1e-3)
            {
                targetAngle = std::atan2(modVy[i], modVx[i]);
                driveV = modMag[i] * desaturate;
            }

            double curAngle = outputs.encoderPositions[i];
            double curVel = outputs.encoderVelocities[i];
            double err = targetAngle - curAngle;

            while (err > 3.141592653589793) err -= 6.283185307179586;
            while (err < -3.141592653589793) err += 6.283185307179586;

            // Swerve 180-degree module azimuth optimization
            if (std::abs(driveV) > 1e-3)
            {
                if (err > 1.5707963267948966)
                {
                    err -= 3.141592653589793;
                    driveV = -driveV;
                }
                else if (err < -1.5707963267948966)
                {
                    err += 3.141592653589793;
                    driveV = -driveV;
                }
                driveV *= std::cos(err);
            }

            double steerV = std::clamp((err * kP) - (curVel * kD), -12.0, 12.0);

            inputs.motorCommands[i].mode = MotorControlMode::Voltage;
            inputs.motorCommands[i].demand = steerV;
            inputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
            inputs.motorCommands[4 + i].demand = driveV;
        }

        if (m_robot.swerveSystem)
        {
            m_robot.swerveSystem->SetInputs(inputs);
        }

        // B. Intake Mechanism Control (Spacebar / Shift)
        m_intakeDeployed = io.IsKeyDown(GLFW_KEY_SPACE) || io.IsKeyDown(GLFW_KEY_LEFT_SHIFT);

        if (m_intakePivotArm)
        {
            // Joint intake_1_Revolute 21 has limits [-2.705, 0.0] rad
            double targetPivot = m_intakeDeployed ? -1.65 : 0.0; // Deployed: -95 deg down to carpet
            double curAngle = m_intakePivotArm->currentAngleRad;
            double curVel = m_intakePivotArm->currentVelocityRadPerSec;
            double err = targetPivot - curAngle;
            double pivotVolts = std::clamp((err * 35.0) - (curVel * 2.8), -12.0, 12.0);
            m_intakePivotArm->commandedVoltage = pivotVolts;
        }

        // Front & Back Intake Rollers
        double intakeRollerV = m_intakeDeployed ? 12.0 : 0.0;
        if (m_frontIntake)  m_frontIntake->commandedVoltage = intakeRollerV;
        if (m_backIntake)   m_backIntake->commandedVoltage = intakeRollerV;

        // Bottom Basket & Funnel Feeders
        double indexerV = m_intakeDeployed ? 9.0 : 0.0;
        if (m_bottomBasket) m_bottomBasket->commandedVoltage = indexerV;
        if (m_funnel)       m_funnel->commandedVoltage = indexerV;

        // C. Shooter Flywheel & Fire Action (Left Click or F)
        bool firing = io.IsMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT) || io.IsKeyDown(GLFW_KEY_F);
        m_flywheelSpinning = firing;

        if (m_flywheel)
        {
            m_flywheel->commandedVoltage = firing ? 12.0 : 0.0;
        }

        // Rapid Fire Trigger on leading edge click
        if (io.WasKeyPressed(GLFW_KEY_F) || io.WasKeyPressed(GLFW_KEY_SPACE) && firing)
        {
            FireShooter();
        }

        // D. Reset Field (R key)
        if (io.WasKeyPressed(GLFW_KEY_R))
        {
            ResetField();
        }
    }

    void BananaGame::CheckGoalScoring()
    {
        // High Goal Trigger Bounding Cylinder around m_goalCenter (X=0, Y=2.2m, Z=5.5m)
        const float targetY = m_goalCenter.y;
        const float targetRadiusSq = m_goalRadius * m_goalRadius;

        for (Entity e : m_ballEntities)
        {
            if (!m_scene.IsEntityValid(e)) continue;

            auto* tc = m_scene.GetComponent<TransformComponent>(e);
            auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
            if (!tc || !piece) continue;

            // Only score balls that are in flight or descending
            if (piece->state == GamePieceState::Launched || piece->state == GamePieceState::Free)
            {
                float dx = tc->position.x - m_goalCenter.x;
                float dz = tc->position.z - m_goalCenter.z;
                float distSq = dx * dx + dz * dz;

                // Inside rim diameter and passing downward through net level
                if (distSq < targetRadiusSq && tc->position.y >= 1.95f && tc->position.y <= 2.35f)
                {
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                    bool isFalling = (rb && rb->linearVelocity.y < 0.2f) || (piece->linearVelocity.y < 0.2f);

                    if (isFalling)
                    {
                        TriggerGoalScored(tc->position);

                        // Recycle ball: drop through net back to floor
                        tc->position = glm::vec3(0.0f, 0.4f, 5.2f);
                        if (rb)
                        {
                            rb->linearVelocity = glm::vec3(0.0f, -1.0f, -1.0f);
                        }
                        piece->state = GamePieceState::Free;
                    }
                }
            }
        }
    }

    void BananaGame::TriggerGoalScored(const glm::vec3& ballPos)
    {
        m_goalsScored++;
        m_comboStreak++;
        if (m_comboStreak > m_maxCombo) m_maxCombo = m_comboStreak;

        int addedPoints = 100 * m_comboStreak;
        m_score += addedPoints;
        if (m_score > m_highScore) m_highScore = m_score;

        m_goalFlashTimer = 0.6f;

        // Floating Score Popup
        ScorePopup popup;
        if (m_comboStreak > 1)
        {
            popup.text = "+" + std::to_string(addedPoints) + " (COMBO x" + std::to_string(m_comboStreak) + "!)";
            popup.color = glm::vec3(1.0f, 0.35f, 0.10f); // Fiery Orange Combo
        }
        else
        {
            popup.text = "+100 PTS!";
            popup.color = glm::vec3(1.0f, 0.90f, 0.15f); // Bright Gold
        }
        popup.worldPos = ballPos + glm::vec3(0.0f, 0.3f, 0.0f);
        m_scorePopups.push_back(popup);

        SIM_LOG_INFO("[BananaGame] GOAL SCORED! +" << addedPoints << " PTS! Total: " << m_score);
    }

    void BananaGame::Update(double dt)
    {
        auto& io = m_renderer.GetInput();

        // Pause Toggle with ESC
        if (io.WasKeyPressed(GLFW_KEY_ESCAPE))
        {
            if (m_state == GameState::Playing)
            {
                m_state = GameState::Paused;
            }
            else if (m_state == GameState::Paused)
            {
                m_state = GameState::Playing;
            }
        }

        // Camera Perspective Mode Toggle with TAB
        if (io.WasKeyPressed(GLFW_KEY_TAB))
        {
            if (m_camMode == CameraMode::FollowRobot)
            {
                m_camMode = CameraMode::DriverStation;
            }
            else if (m_camMode == CameraMode::DriverStation)
            {
                m_camMode = CameraMode::FreeOrbit;
            }
            else
            {
                m_camMode = CameraMode::FollowRobot;
            }
        }

        // Game State Machine Transitions
        if (m_state == GameState::Countdown)
        {
            m_countdownTimer -= static_cast<float>(dt);
            if (m_countdownTimer <= 0.0f)
            {
                m_state = GameState::Playing;
            }
        }
        else if (m_state == GameState::Playing)
        {
            if (m_mode == GameMode::ArcadeRush)
            {
                m_sessionTimer -= static_cast<float>(dt);
                if (m_sessionTimer <= 0.0f)
                {
                    m_sessionTimer = 0.0f;
                    m_state = GameState::GameOver;
                }
            }

            // Continuous Field Ball Refresh (Keep at least 4 balls on carpet)
            m_ballAutoDropTimer += static_cast<float>(dt);
            if (m_ballAutoDropTimer >= 4.0f)
            {
                m_ballAutoDropTimer = 0.0f;
                int freeBalls = 0;
                for (Entity e : m_ballEntities)
                {
                    if (!m_scene.IsEntityValid(e)) continue;
                    auto* p = m_scene.GetComponent<GamePieceComponent>(e);
                    if (p && p->state == GamePieceState::Free) freeBalls++;
                }

                if (freeBalls < 5)
                {
                    float dropX = static_cast<float>((rand() % 100 - 50) * 0.08f);
                    SpawnBall(glm::vec3(dropX, 1.8f, 1.5f));
                }
            }
        }

        // Animate Floating Score Popups
        for (auto it = m_scorePopups.begin(); it != m_scorePopups.end();)
        {
            it->lifetime += static_cast<float>(dt);
            it->worldPos.y += static_cast<float>(dt * 0.6f); // Float upward
            if (it->lifetime >= it->maxLifetime)
            {
                it = m_scorePopups.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Visual Goal Rim Flash Decay
        if (m_goalFlashTimer > 0.0f)
        {
            m_goalFlashTimer -= static_cast<float>(dt);
        }

        // Smooth Camera Follow
        UpdateCamera(dt);
    }

    void BananaGame::UpdateCamera(double dt)
    {
        auto& camera = m_renderer.GetCamera();
        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (!chassisT) return;

        glm::vec3 robotPos = chassisT->position;
        glm::quat robotRot = chassisT->rotation;
        glm::vec3 robotFwd = robotRot * glm::vec3(0.0f, 0.0f, 1.0f);

        if (m_camMode == CameraMode::FollowRobot)
        {
            camera.SetMode(Renderer::Camera::Mode::Orbit);

            // Position camera smoothly behind and above robot, looking forward
            glm::vec3 targetCamPos = robotPos - (robotFwd * 3.4f) + glm::vec3(0.0f, 1.85f, 0.0f);
            glm::vec3 targetLookAt = robotPos + (robotFwd * 1.5f) + glm::vec3(0.0f, 0.45f, 0.0f);

            float blend = std::clamp(static_cast<float>(dt * 7.5), 0.01f, 1.0f);
            m_smoothCameraPos = glm::mix(m_smoothCameraPos, targetCamPos, blend);
            m_smoothCameraTarget = glm::mix(m_smoothCameraTarget, targetLookAt, blend);

            camera.position = m_smoothCameraPos;
            camera.orbitTarget = m_smoothCameraTarget;
        }
        else if (m_camMode == CameraMode::DriverStation)
        {
            // Fixed Driver Station View from Alliance Wall
            camera.SetMode(Renderer::Camera::Mode::Orbit);
            camera.position = glm::vec3(0.0f, 2.10f, -6.8f);
            camera.orbitTarget = robotPos + glm::vec3(0.0f, 0.40f, 0.0f);
        }
        else
        {
            // Free Orbit
            camera.SetMode(Renderer::Camera::Mode::Orbit);
            camera.orbitTarget = robotPos + glm::vec3(0.0f, 0.40f, 0.0f);
        }
    }

    void BananaGame::Render3D()
    {
        // 1. Draw Ballistic Trajectory Trails for Flying Balls
        for (const auto& [entity, trail] : m_ballTrails)
        {
            if (trail.size() < 2) continue;
            for (size_t i = 0; i + 1 < trail.size(); ++i)
            {
                float alpha = static_cast<float>(i) / static_cast<float>(trail.size());
                glm::vec3 trailColor = glm::mix(glm::vec3(0.1f, 0.6f, 1.0f), glm::vec3(1.0f, 0.85f, 0.1f), alpha);
                Utils::DebugDraw::Line(trail[i], trail[i + 1], trailColor, true);
            }
        }

        // 2. Draw Target Goal Rim Highlight & Flash FX
        glm::vec3 hoopColor = (m_goalFlashTimer > 0.0f)
            ? glm::vec3(1.0f, 1.0f, 1.0f) // Bright white flash on score
            : glm::vec3(0.98f, 0.85f, 0.12f); // Vibrant Yellow Rim

        constexpr int rimSegments = 24;
        for (int i = 0; i < rimSegments; ++i)
        {
            float theta1 = (static_cast<float>(i) / rimSegments) * 6.2831853f;
            float theta2 = (static_cast<float>(i + 1) / rimSegments) * 6.2831853f;
            glm::vec3 p1 = m_goalCenter + glm::vec3(std::cos(theta1) * m_goalRadius, 0.0f, std::sin(theta1) * m_goalRadius);
            glm::vec3 p2 = m_goalCenter + glm::vec3(std::cos(theta2) * m_goalRadius, 0.0f, std::sin(theta2) * m_goalRadius);
            Utils::DebugDraw::Line(p1, p2, hoopColor, true);
        }
    }

    void BananaGame::RenderUI()
    {
        // Dispatch UI views based on GameState
        switch (m_state)
        {
        case GameState::StartScreen:
            DrawStartScreenUI();
            break;
        case GameState::Countdown:
            DrawHUDUI();
            break;
        case GameState::Playing:
            DrawHUDUI();
            break;
        case GameState::Paused:
            DrawHUDUI();
            DrawPauseUI();
            break;
        case GameState::GameOver:
            DrawHUDUI();
            DrawGameOverUI();
            break;
        }

        // Draw In-World Floating Score Text
        auto& camera = m_renderer.GetCamera();
        float aspect = (ImGui::GetIO().DisplaySize.y > 0.0f) ? (ImGui::GetIO().DisplaySize.x / ImGui::GetIO().DisplaySize.y) : (1280.0f / 720.0f);
        glm::mat4 vp = camera.GetProjectionMatrix(aspect) * camera.GetViewMatrix();
        ImDrawList* drawList = ImGui::GetForegroundDrawList();

        for (const auto& popup : m_scorePopups)
        {
            glm::vec4 clip = vp * glm::vec4(popup.worldPos, 1.0f);
            if (clip.w > 0.1f)
            {
                float ndcX = clip.x / clip.w;
                float ndcY = clip.y / clip.w;
                if (ndcX >= -1.0f && ndcX <= 1.0f && ndcY >= -1.0f && ndcY <= 1.0f)
                {
                    float screenX = (ndcX * 0.5f + 0.5f) * ImGui::GetIO().DisplaySize.x;
                    float screenY = (1.0f - (ndcY * 0.5f + 0.5f)) * ImGui::GetIO().DisplaySize.y;

                    float fade = 1.0f - (popup.lifetime / popup.maxLifetime);
                    ImU32 textColor = ImColor(popup.color.r, popup.color.g, popup.color.b, fade);
                    ImU32 shadowColor = ImColor(0.0f, 0.0f, 0.0f, fade * 0.8f);

                    drawList->AddText(ImVec2(screenX + 2, screenY + 2), shadowColor, popup.text.c_str());
                    drawList->AddText(ImVec2(screenX, screenY), textColor, popup.text.c_str());
                }
            }
        }
    }

    void BananaGame::DrawStartScreenUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(680, 560), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
        ImGui::Begin("##StartScreen", nullptr, flags);

        // Header Title Banner
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.88f, 0.15f, 1.0f));
        ImGui::SetWindowFontScale(1.8f);
        ImGui::Text(" FRC BANANA SIMULATOR");
        ImGui::SetWindowFontScale(1.2f);
        ImGui::PopStyleColor();

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.85f, 1.0f, 1.0f));
        ImGui::Text(" ROBOT COMP 2: ARCADE SHOOTOUT");
        ImGui::PopStyleColor();
        ImGui::SetWindowFontScale(1.0f);

        ImGui::Separator();
        ImGui::Spacing();

        ImGui::TextWrapped("Drive Robot Comp 2 around the arena, deploy the intake to vacuum bouncy foam balls into the basket, and fire balls through the target goal to rack up combos!");

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Controls Cheatsheet Card
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "CONTROLS & KEYBINDINGS:");
        ImGui::Columns(2, "controlsColumns", false);
        ImGui::SetColumnWidth(0, 240);

        ImGui::BulletText("W / A / S / D");
        ImGui::NextColumn();
        ImGui::Text("Field-Centric Swerve Translation");
        ImGui::NextColumn();

        ImGui::BulletText("Q / E");
        ImGui::NextColumn();
        ImGui::Text("Rotate Robot Heading");
        ImGui::NextColumn();

        ImGui::BulletText("Spacebar / Shift (Hold)");
        ImGui::NextColumn();
        ImGui::Text("Deploy Intake & Vacuum Rollers");
        ImGui::NextColumn();

        ImGui::BulletText("Left Mouse / F");
        ImGui::NextColumn();
        ImGui::Text("Spool Flywheel & Fire Ball");
        ImGui::NextColumn();

        ImGui::BulletText("Tab");
        ImGui::NextColumn();
        ImGui::Text("Cycle Camera (Follow / Cockpit / Orbit)");
        ImGui::NextColumn();

        ImGui::BulletText("H");
        ImGui::NextColumn();
        ImGui::Text("Toggle Field-Centric / Robot-Centric");
        ImGui::NextColumn();

        ImGui::BulletText("Z");
        ImGui::NextColumn();
        ImGui::Text("Zero Gyroscope Forward Heading");
        ImGui::NextColumn();

        ImGui::BulletText("R");
        ImGui::NextColumn();
        ImGui::Text("Respawn All Balls on Field");
        ImGui::NextColumn();

        ImGui::Columns(1);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (m_highScore > 0)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "HIGH SCORE: %d PTS", m_highScore);
            ImGui::Spacing();
        }

        // Action Buttons
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.80f, 0.32f, 1.0f));
        ImGui::SetWindowFontScale(1.25f);
        if (ImGui::Button("START 60s ARCADE CHALLENGE", ImVec2(640, 52)))
        {
            StartGame(GameMode::ArcadeRush);
        }
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor(2);

        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.38f, 0.65f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.50f, 0.80f, 1.0f));
        if (ImGui::Button("FREE PRACTICE (Unlimited Time)", ImVec2(640, 38)))
        {
            StartGame(GameMode::FreePractice);
        }
        ImGui::PopStyleColor(2);

        ImGui::End();
    }

    void BananaGame::DrawHUDUI()
    {
        ImGuiIO& io = ImGui::GetIO();

        // 1. Top Center Score & Timer Header
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, 18), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(520, 80), ImGuiCond_Always);
        ImGuiWindowFlags hudFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                                   ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav;

        ImGui::Begin("##TopHUD", nullptr, hudFlags);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        ImVec2 p0 = ImGui::GetWindowPos();
        ImVec2 p1 = ImVec2(p0.x + 520, p0.y + 75);

        // Dark translucent banner backing
        drawList->AddRectFilled(p0, p1, IM_COL32(18, 22, 28, 210), 12.0f);
        drawList->AddRect(p0, p1, IM_COL32(80, 140, 220, 160), 12.0f, 0, 2.0f);

        // Timer Display
        ImGui::SetCursorPos(ImVec2(24, 14));
        if (m_mode == GameMode::ArcadeRush)
        {
            int sec = static_cast<int>(m_sessionTimer);
            ImVec4 timeCol = (sec <= 10) ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, timeCol);
            ImGui::SetWindowFontScale(1.9f);
            ImGui::Text("%02d:%02d", sec / 60, sec % 60);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopStyleColor();
        }
        else
        {
            ImGui::SetWindowFontScale(1.4f);
            ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "PRACTICE");
            ImGui::SetWindowFontScale(1.0f);
        }

        // Score Display
        ImGui::SetCursorPos(ImVec2(200, 14));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.88f, 0.15f, 1.0f));
        ImGui::SetWindowFontScale(1.9f);
        ImGui::Text("%d PTS", m_score);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        // Combo Multiplier Display
        if (m_comboStreak > 1)
        {
            ImGui::SetCursorPos(ImVec2(390, 18));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.40f, 0.10f, 1.0f));
            ImGui::SetWindowFontScale(1.4f);
            ImGui::Text("x%d COMBO!", m_comboStreak);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopStyleColor();
        }

        ImGui::End();

        // 2. Countdown Banner
        if (m_state == GameState::Countdown)
        {
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.4f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowSize(ImVec2(400, 160), ImGuiCond_Always);
            ImGui::Begin("##CountdownWindow", nullptr, hudFlags);

            ImGui::SetWindowFontScale(4.0f);
            int count = static_cast<int>(std::ceil(m_countdownTimer));
            if (count > 0)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.15f, 1.0f), "   %d", count);
            }
            else
            {
                ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "  GO!");
            }
            ImGui::SetWindowFontScale(1.0f);
            ImGui::End();
        }

        // 3. Bottom-Left Robot Status Card
        ImGui::SetNextWindowPos(ImVec2(18, io.DisplaySize.y - 120), ImGuiCond_Always, ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(320, 105), ImGuiCond_Always);
        ImGui::Begin("##RobotHUD", nullptr, hudFlags);

        ImVec2 r0 = ImGui::GetWindowPos();
        ImVec2 r1 = ImVec2(r0.x + 320, r0.y + 105);
        drawList->AddRectFilled(r0, r1, IM_COL32(18, 22, 28, 200), 8.0f);
        drawList->AddRect(r0, r1, IM_COL32(60, 100, 160, 150), 8.0f);

        ImGui::SetCursorPos(ImVec2(12, 10));
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "ROBOT COMP 2 (SWERVE)");

        // Ball Loaded Sensor Indicator
        bool ballLoaded = false;
        for (Entity e : m_ballEntities)
        {
            if (!m_scene.IsEntityValid(e)) continue;
            auto* p = m_scene.GetComponent<GamePieceComponent>(e);
            if (p && (p->state == GamePieceState::Stored || p->state == GamePieceState::Engaged))
            {
                ballLoaded = true;
                break;
            }
        }

        ImGui::SetCursorPos(ImVec2(12, 32));
        if (ballLoaded)
        {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "LOADED: [ BALL READY TO FIRE ]");
        }
        else
        {
            ImGui::TextColored(ImVec4(0.6f, 0.65f, 0.7f, 1.0f), "LOADED: [ NO BALL (HOLD SPACE) ]");
        }

        // Flywheel RPM Indicator
        float rpm = m_flywheel ? static_cast<float>(m_flywheel->angularVelocityRpm) : 0.0f;
        ImGui::SetCursorPos(ImVec2(12, 54));
        ImGui::Text("FLYWHEEL: %.0f RPM", rpm);
        ImGui::SameLine();
        ImGui::ProgressBar(rpm / 5000.0f, ImVec2(120, 14), "");

        // Mode Status
        ImGui::SetCursorPos(ImVec2(12, 78));
        ImGui::TextDisabled("MODE: %s | CAM: %s",
            m_fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC",
            m_camMode == CameraMode::FollowRobot ? "FOLLOW" : (m_camMode == CameraMode::DriverStation ? "COCKPIT" : "ORBIT"));

        ImGui::End();
    }

    void BananaGame::DrawPauseUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(340, 240), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
        ImGui::Begin("GAME PAUSED##PauseMenu", nullptr, flags);

        ImGui::Spacing();
        ImGui::SetWindowFontScale(1.4f);
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "  GAME PAUSED");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button("RESUME (Esc)", ImVec2(310, 38)))
        {
            m_state = GameState::Playing;
        }

        ImGui::Spacing();
        if (ImGui::Button("RESTART SESSION", ImVec2(310, 38)))
        {
            StartGame(m_mode);
        }

        ImGui::Spacing();
        if (ImGui::Button("RETURN TO TITLE MENU", ImVec2(310, 38)))
        {
            m_state = GameState::StartScreen;
        }

        ImGui::End();
    }

    void BananaGame::DrawGameOverUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(480, 380), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
        ImGui::Begin("SESSION COMPLETE##GameOver", nullptr, flags);

        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.35f, 0.20f, 1.0f));
        ImGui::SetWindowFontScale(1.8f);
        ImGui::Text("  TIME'S UP!");
        ImGui::PopStyleColor();
        ImGui::SetWindowFontScale(1.0f);

        ImGui::Separator();
        ImGui::Spacing();

        ImGui::SetWindowFontScale(1.4f);
        ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.15f, 1.0f), "FINAL SCORE: %d PTS", m_score);
        ImGui::SetWindowFontScale(1.0f);

        if (m_score >= m_highScore && m_score > 0)
        {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "*** NEW HIGH SCORE! ***");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Goals Made:    %d", m_goalsScored);
        ImGui::Text("Shots Taken:   %d", m_ballsFired);
        float acc = (m_ballsFired > 0) ? (static_cast<float>(m_goalsScored) / m_ballsFired * 100.0f) : 0.0f;
        ImGui::Text("Accuracy:      %.1f %%", acc);
        ImGui::Text("Max Combo:     x%d", m_maxCombo);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.25f, 1.0f));
        if (ImGui::Button("PLAY AGAIN", ImVec2(440, 44)))
        {
            StartGame(m_mode);
        }
        ImGui::PopStyleColor();

        ImGui::Spacing();
        if (ImGui::Button("MAIN MENU", ImVec2(440, 32)))
        {
            m_state = GameState::StartScreen;
        }

        ImGui::End();
    }
}
