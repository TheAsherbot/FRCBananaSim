/**
 * @file BananaGame.cpp
 * @brief Implementation of the Robot Comp 2 arcade shootout mini-game.
 */

#include "Game/BananaGame.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
#include "Robot/Schema/EmbeddedRobotData.h"

#include "Core/ECS/Components/TransformComponent.h"
#include "Core/ECS/Components/HierarchyComponent.h"
#include "Physics/PhysicsConfig.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Renderer/Components/MeshComponent.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Drivetrain/Components/SwerveModuleComponent.h"
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
#include <fstream>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>

using namespace FRCBananaSim::Core::ECS::Core;
using namespace FRCBananaSim::Core::ECS::Components;
using namespace FRCBananaSim::Physics::Components;
using namespace FRCBananaSim::Renderer::Components;
using namespace FRCBananaSim::Control;
using namespace FRCBananaSim::Robot::Drivetrain::Components;
using namespace FRCBananaSim::Robot::Mechanisms::Components;
using namespace FRCBananaSim::Robot::Sensors::Components;
using namespace FRCBananaSim::Interaction::Components;

static std::string GetExecutableDirectory()
{
#ifdef _WIN32
    wchar_t exePath[MAX_PATH];
    DWORD len = GetModuleFileNameW(nullptr, exePath, MAX_PATH);
    if (len > 0)
    {
        std::error_code ec;
        std::filesystem::path p(exePath);
        return p.parent_path().string();
    }
#endif
    return "";
}

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
        SIM_LOG_INFO("[BananaGame] Shutting down and cleaning physics entities...");

        // 1. Destroy joint system first so all constraints are removed from Jolt before bodies
        m_jointSystem.reset();

        // 2. Clear all joint constraints in ECS scene so no JPH::Ref<JPH::Constraint> remains
        auto jointEntities = m_scene.Query<JointComponent>();
        for (Entity e : jointEntities)
        {
            auto* joint = m_scene.GetComponent<JointComponent>(e);
            if (joint)
            {
                joint->constraint = nullptr;
                joint->isCreated = false;
            }
            m_scene.DestroyEntity(e);
        }
        m_robot.jointEntities.clear();

        // 3. Destroy ball bodies cleanly
        for (Entity e : m_ballEntities)
        {
            if (m_scene.IsEntityValid(e))
            {
                if (m_robot.physicsSystem)
                {
                    m_robot.physicsSystem->DestroyBody(e);
                }
                else if (m_joltContext.IsInitialized())
                {
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                    if (rb && !rb->bodyID.IsInvalid() && m_joltContext.GetBodyInterface().IsAdded(rb->bodyID))
                    {
                        m_joltContext.GetBodyInterface().RemoveBody(rb->bodyID);
                        m_joltContext.GetBodyInterface().DestroyBody(rb->bodyID);
                        rb->bodyID = JPH::BodyID();
                    }
                }
                m_scene.DestroyEntity(e);
            }
        }
        m_ballEntities.clear();
        m_activeShots.clear();
        m_heldBalls.clear();

        // 4. Destroy arena bodies (floor, walls, goal posts, hoop)
        for (Entity e : m_arenaEntities)
        {
            if (m_scene.IsEntityValid(e))
            {
                if (m_robot.physicsSystem)
                {
                    m_robot.physicsSystem->DestroyBody(e);
                }
                else if (m_joltContext.IsInitialized())
                {
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                    if (rb && !rb->bodyID.IsInvalid() && m_joltContext.GetBodyInterface().IsAdded(rb->bodyID))
                    {
                        m_joltContext.GetBodyInterface().RemoveBody(rb->bodyID);
                        m_joltContext.GetBodyInterface().DestroyBody(rb->bodyID);
                        rb->bodyID = JPH::BodyID();
                    }
                }
                m_scene.DestroyEntity(e);
            }
        }
        m_arenaEntities.clear();

        // 4b. Destroy custom robot visual meshes cleanly
        for (Entity e : m_robotVisualEntities)
        {
            if (m_scene.IsEntityValid(e))
            {
                m_scene.DestroyEntity(e);
            }
        }
        m_robotVisualEntities.clear();

        // 5. Destroy robot link bodies cleanly via physics system and destroy entities
        for (auto& [name, entity] : m_robot.linkEntities)
        {
            if (m_scene.IsEntityValid(entity))
            {
                if (m_robot.physicsSystem)
                {
                    m_robot.physicsSystem->DestroyBody(entity);
                }
                else if (m_joltContext.IsInitialized())
                {
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(entity);
                    if (rb && !rb->bodyID.IsInvalid() && m_joltContext.GetBodyInterface().IsAdded(rb->bodyID))
                    {
                        m_joltContext.GetBodyInterface().RemoveBody(rb->bodyID);
                        m_joltContext.GetBodyInterface().DestroyBody(rb->bodyID);
                        rb->bodyID = JPH::BodyID();
                    }
                }
                m_scene.DestroyEntity(entity);
            }
        }
        m_robot.linkEntities.clear();

        // Destroy chassis entity if still valid
        if (m_scene.IsEntityValid(m_robot.chassisEntity))
        {
            if (m_robot.physicsSystem)
            {
                m_robot.physicsSystem->DestroyBody(m_robot.chassisEntity);
            }
            m_scene.DestroyEntity(m_robot.chassisEntity);
        }
        m_robot.chassisEntity = invalidEntity;

        // Destroy remaining robot entities (mechanisms, sensors, contact zones)
        for (auto& [name, entity] : m_robot.mechanismEntities)
        {
            if (m_scene.IsEntityValid(entity)) m_scene.DestroyEntity(entity);
        }
        m_robot.mechanismEntities.clear();

        for (auto& [name, entity] : m_robot.sensorEntities)
        {
            if (m_scene.IsEntityValid(entity)) m_scene.DestroyEntity(entity);
        }
        m_robot.sensorEntities.clear();

        for (auto& [name, entity] : m_robot.contactZoneEntities)
        {
            if (m_scene.IsEntityValid(entity)) m_scene.DestroyEntity(entity);
        }
        m_robot.contactZoneEntities.clear();

        // 6. Cleanly reset robot subsystems in reverse dependency order
        m_robot.physicsSystem.reset();
        m_robot.swerveSystem.reset();
        m_robot.flywheelSystem.reset();
        m_robot.mechanismSystem.reset();
        m_robot.electricalSystem.reset();
        m_robot.elevatorSystem.reset();
        m_robot.sensorSystem.reset();
        m_robot.flightAerodynamicsSystem.reset();

        SaveHighScore();

        SIM_LOG_INFO("[BananaGame] Teardown complete.");
    }

    bool BananaGame::Initialize()
    {
        SIM_LOG_INFO("[BananaGame] Initializing Robot Comp 2 Arcade Shootout Game...");

        // 1. Locate Robot Comp 2 CAD & Sidecar resources
        std::string manifestPath = "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/manifest.json";
        std::string sidecarPath = "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/sidecar.json";
        std::string materialsPath = "FRCBananaSim/Resources/Config/materials.json";

        std::string exeDir = GetExecutableDirectory();
        std::vector<std::string> searchPrefixes = {
            "", "../", "../../", "../../../", "../../../../", "FRCBananaSim/"
        };
        if (!exeDir.empty())
        {
            searchPrefixes.insert(searchPrefixes.begin(), exeDir + "/../../FRCBananaSim/");
            searchPrefixes.insert(searchPrefixes.begin(), exeDir + "/../");
            searchPrefixes.insert(searchPrefixes.begin(), exeDir + "/FRCBananaSim/");
            searchPrefixes.insert(searchPrefixes.begin(), exeDir + "/");
        }

        bool foundRobot = false;
        std::error_code ec;
        for (const auto& prefix : searchPrefixes)
        {
            std::string editCandidate = prefix + "Resources/Robots/Comp 2 Robot EDITED/manifest.json";
            std::string editPrefixCandidate = prefix + "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/manifest.json";
            if (std::filesystem::exists(editCandidate, ec))
            {
                manifestPath = editCandidate;
                sidecarPath = prefix + "Resources/Robots/Comp 2 Robot EDITED/sidecar.json";
                materialsPath = prefix + "Resources/Config/materials.json";
                foundRobot = true;
                SIM_LOG_INFO("[BananaGame] Loaded edited robot from disk: " << manifestPath);
                break;
            }
            if (std::filesystem::exists(editPrefixCandidate, ec))
            {
                manifestPath = editPrefixCandidate;
                sidecarPath = prefix + "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/sidecar.json";
                materialsPath = prefix + "FRCBananaSim/Resources/Config/materials.json";
                foundRobot = true;
                SIM_LOG_INFO("[BananaGame] Loaded edited robot from disk: " << manifestPath);
                break;
            }
            std::string candidate = prefix + "Resources/Robots/Robot Comp 2/manifest.json";
            if (std::filesystem::exists(candidate, ec))
            {
                manifestPath = candidate;
                sidecarPath = prefix + "Resources/Robots/Robot Comp 2/sidecar.json";
                materialsPath = prefix + "Resources/Config/materials.json";
                foundRobot = true;
                SIM_LOG_INFO("[BananaGame] Loaded robot from disk: " << manifestPath);
                break;
            }
        }

        // 2. Assemble Robot Comp 2 in World Space
        Robot::RobotAssembler assembler(m_scene, &m_joltContext);
        const glm::vec3 spawnPos(0.0f, 0.20f, -2.5f); // Centered on carpet, facing forward toward goal (+Z)
        const glm::quat spawnRot(1.0f, 0.0f, 0.0f, 0.0f);

        if (foundRobot)
        {
            SIM_LOG_INFO("[BananaGame] Assembling robot from disk files: " << manifestPath);
            m_robot = assembler.AssembleFromFiles(manifestPath, sidecarPath, materialsPath, spawnPos, spawnRot);
            if (!m_robot.isSuccess)
            {
                SIM_LOG_WARN("[BananaGame] Failed to assemble disk robot files (" << m_robot.errorMessage << "); falling back to embedded data.");
                foundRobot = false;
            }
        }

        if (!foundRobot)
        {
            SIM_LOG_INFO("[BananaGame] External robot files not found; assembling self-contained embedded Robot Comp 2 data.");
            m_robot = assembler.AssembleFromStrings(
                Robot::Schema::Embedded::kManifestJson,
                Robot::Schema::Embedded::kSidecarJson,
                Robot::Schema::Embedded::kMaterialsJson,
                spawnPos, spawnRot
            );
            if (!m_robot.isSuccess)
            {
                SIM_LOG_FATAL("[BananaGame] Failed to assemble embedded Robot Comp 2: " << m_robot.errorMessage);
                return false;
            }
        }

        // Cache initial assembled poses for all links to support flawless resets
        for (const auto& [name, entity] : m_robot.linkEntities)
        {
            auto* tc = m_scene.GetComponent<TransformComponent>(entity);
            if (tc)
            {
                m_initialLinkPoses[name] = { tc->position, tc->rotation };
            }
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

        // Cache Intake Pivot Hinge Joint for Jolt Position Motor control
        auto jIt = m_robot.jointEntities.find("intake_1_Revolute 21");
        if (jIt != m_robot.jointEntities.end())
        {
            m_intakeJoint = m_scene.GetComponent<Physics::Components::JointComponent>(jIt->second);
            if (m_intakeJoint)
            {
                m_intakeJoint->motorMode = Physics::Components::JointMotorMode::Position;
                m_intakeJoint->springFrequency = 8.0f;
                m_intakeJoint->springDamping = 1.0f;
                m_intakeJoint->maxMotorTorque = 300.0f;
                m_intakeJoint->targetAngle = 0.0f; // Stowed upright by default
            }
        }

        // Ensure Flywheel & Funnel Rollers are located at their true CAD throat coordinates
        if (m_flywheel)
        {
            m_flywheel->visualLocalOffset = glm::vec3(0.0f, 0.406618f, 0.156811f);
            m_flywheel->spinAxis = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        if (m_funnel)
        {
            m_funnel->visualLocalOffset = glm::vec3(0.0f, 0.301143f, 0.135771f);
            m_funnel->spinAxis = glm::vec3(1.0f, 0.0f, 0.0f);
        }

        // 5. Construct Arena Environment & Visuals
        SetupArena();
        SetupRobotVisuals();

        // 6. Spawn Initial Field Balls
        ResetField();

        // Configure Initial Camera Position
        auto& camera = m_renderer.GetCamera();
        camera.SetMode(Renderer::Camera::Mode::FreeFly);
        camera.SetKeyboardMovementEnabled(false);
        m_smoothCameraPos = spawnPos + glm::vec3(0.0f, 2.50f, -4.50f);
        m_smoothCameraTarget = spawnPos + glm::vec3(0.0f, 0.40f, 0.80f);
        camera.position = m_smoothCameraPos;
        camera.orbitTarget = m_smoothCameraTarget;
        glm::vec3 initDir = glm::normalize(camera.orbitTarget - camera.position);
        camera.pitchDegrees = glm::degrees(std::asin(std::clamp(initDir.y, -0.999f, 0.999f)));
        camera.yawDegrees = glm::degrees(std::atan2(initDir.z, initDir.x));

        // Load Persistent High Score from Disk
        LoadHighScore();

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
        m_arenaEntities.push_back(carpet);
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

        // B. Perimeter Arena Barrier Walls (Transparent Polycarbonate Walls)
        auto createWall = [&](const glm::vec3& visualPos, const glm::vec3& visualSize,
                              const glm::vec3& colOffset, const glm::vec3& colHalfExtents,
                              const glm::vec3& color, float alpha = 0.35f)
        {
            Entity wall = m_scene.CreateEntity();
            m_arenaEntities.push_back(wall);
            assignTransforms(wall, visualPos, visualSize);

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = color;
            mesh.alpha = alpha; // Translucent arena wall so user can always see through
            m_scene.AddComponent(wall, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.60f;
            rb.restitution = 0.10f;
            m_scene.AddComponent(wall, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.localOffset = colOffset;
            col.boxHalfExtents = colHalfExtents; // Extra thick collider so fast impacts cannot tunnel
            m_scene.AddComponent(wall, col);
        };

        const glm::vec3 wallColor(0.35f, 0.55f, 0.85f); // Sleek Ice-Blue Translucent Polycarbonate
        // North Wall (+Z far end behind goal): visual at Z=7.0m, thick collider extending outward
        createWall(glm::vec3(0.0f, 0.6f, 7.0f), glm::vec3(18.0f, 1.2f, 0.3f),
                   glm::vec3(0.0f, 0.0f, 0.6f), glm::vec3(10.0f, 1.5f, 0.8f), wallColor, 0.32f);
        // South Wall (-Z driver station side): visual at Z=-7.0m, thick collider extending outward
        createWall(glm::vec3(0.0f, 0.6f, -7.0f), glm::vec3(18.0f, 1.2f, 0.3f),
                   glm::vec3(0.0f, 0.0f, -0.6f), glm::vec3(10.0f, 1.5f, 0.8f), glm::vec3(0.30f, 0.45f, 0.85f), 0.32f);
        // East Wall (+X right side): visual at X=9.0m, thick collider extending outward
        createWall(glm::vec3(9.0f, 0.6f, 0.0f), glm::vec3(0.3f, 1.2f, 14.0f),
                   glm::vec3(0.6f, 0.0f, 0.0f), glm::vec3(0.8f, 1.5f, 8.0f), wallColor, 0.32f);
        // West Wall (-X left side): visual at X=-9.0m, thick collider extending outward
        createWall(glm::vec3(-9.0f, 0.6f, 0.0f), glm::vec3(0.3f, 1.2f, 14.0f),
                   glm::vec3(-0.6f, 0.0f, 0.0f), glm::vec3(0.8f, 1.5f, 8.0f), wallColor, 0.32f);

        // C. Target High Goal Tower & Hoop (Centered at X = 0.0m, Y = 2.2m, Z = 5.5m)
        m_goalCenter = glm::vec3(0.0f, 2.20f, 5.5f);

        // 1. Goal Support Post
        Entity goalPost = m_scene.CreateEntity();
        m_arenaEntities.push_back(goalPost);
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
        m_arenaEntities.push_back(backboard);
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
        m_arenaEntities.push_back(rimFront);
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
        m_arenaEntities.push_back(rimLeft);
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
        m_arenaEntities.push_back(rimRight);
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
        auto createCustomVisual = [&](Entity linkEntity, PrimitiveShape shape, const glm::vec3& color,
                                      const glm::vec3& scale, const glm::vec3& offset,
                                      const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        {
            if (linkEntity == invalidEntity) return;

            Entity visual = m_scene.CreateEntity();
            m_robotVisualEntities.push_back(visual);
            TransformComponent vt;
            vt.position = offset;
            vt.rotation = rot;
            vt.scale = scale;
            m_scene.AddComponent(visual, vt);

            auto* linkT = m_scene.GetComponent<TransformComponent>(linkEntity);
            WorldTransformComponent vwt;
            vwt.worldPosition = linkT ? (linkT->position + (linkT->rotation * offset)) : offset;
            vwt.worldRotation = linkT ? (linkT->rotation * rot) : rot;
            vwt.worldScale = scale;
            m_scene.AddComponent(visual, vwt);

            m_scene.SetParent(visual, linkEntity);

            MeshComponent mesh;
            mesh.shape = shape;
            mesh.color = color;
            m_scene.AddComponent(visual, mesh);
        };

        auto attachVisual = [&](Entity linkEntity, PrimitiveShape shape, const glm::vec3& color,
                                const glm::vec3& fallbackScale = glm::vec3(0.08f),
                                const glm::vec3& fallbackOffset = glm::vec3(0.0f))
        {
            if (linkEntity == invalidEntity) return;

            auto* linkT = m_scene.GetComponent<TransformComponent>(linkEntity);
            if (!linkT) return;

            linkT->scale = glm::vec3(1.0f);
            auto* linkWt = m_scene.GetComponent<WorldTransformComponent>(linkEntity);
            if (linkWt) linkWt->worldScale = glm::vec3(1.0f);

            auto* col = m_scene.GetComponent<ColliderComponent>(linkEntity);
            glm::vec3 visualOffset = col ? col->localOffset : fallbackOffset;
            glm::quat visualRot = col ? col->localRotation : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 visualScale = fallbackScale;

            if (col)
            {
                if (col->shape == ColliderShape::Box)
                {
                    visualScale = col->boxHalfExtents * 2.0f;
                }
                else if (col->shape == ColliderShape::Cylinder || col->shape == ColliderShape::Capsule)
                {
                    visualScale = glm::vec3(col->radius * 2.0f, col->halfHeight * 2.0f, col->radius * 2.0f);
                }
                else if (col->shape == ColliderShape::Sphere)
                {
                    visualScale = glm::vec3(col->radius * 2.0f);
                }
            }

            Entity visual = m_scene.CreateEntity();
            m_robotVisualEntities.push_back(visual);
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

            m_scene.SetParent(visual, linkEntity);

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
                // Sleek Chassis Baseplate (replaces the oversized auto-box CAD block that obstructed view)
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.18f, 0.22f, 0.28f),
                                   glm::vec3(0.72f, 0.04f, 0.72f), glm::vec3(0.0f, 0.02f, 0.0f));

                // Low-profile Red Perimeter Bumpers (keeps internal mechanisms & field visible)
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.85f, 0.15f, 0.18f),
                                   glm::vec3(0.82f, 0.09f, 0.06f), glm::vec3(0.0f, 0.05f, 0.38f));  // Front
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.85f, 0.15f, 0.18f),
                                   glm::vec3(0.82f, 0.09f, 0.06f), glm::vec3(0.0f, 0.05f, -0.38f)); // Back
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.85f, 0.15f, 0.18f),
                                   glm::vec3(0.06f, 0.09f, 0.70f), glm::vec3(-0.38f, 0.05f, 0.0f)); // Left
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.85f, 0.15f, 0.18f),
                                   glm::vec3(0.06f, 0.09f, 0.70f), glm::vec3(0.38f, 0.05f, 0.0f));  // Right
            }
            else if (name == "gear")
            {
                // Turret Ring Brass Gear
                createCustomVisual(entity, PrimitiveShape::Cylinder, glm::vec3(0.90f, 0.72f, 0.18f),
                                   glm::vec3(0.28f, 0.04f, 0.28f), glm::vec3(0.0f, 0.02f, 0.0f));
            }
            else if (name == "sidepannel")
            {
                // Basket Hopper Side Panels (Brushed Polycarbonate)
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.24f, 0.27f, 0.31f),
                                   glm::vec3(0.02f, 0.14f, 0.32f), glm::vec3(-0.24f, 0.06f, -0.05f));
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.24f, 0.27f, 0.31f),
                                   glm::vec3(0.02f, 0.14f, 0.32f), glm::vec3(0.24f, 0.06f, -0.05f));
            }
            else if (name == "moving_hood")
            {
                // Adjustable Shooter Hood (Neon Cyan)
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.12f, 0.75f, 0.92f),
                                   glm::vec3(0.20f, 0.03f, 0.16f), glm::vec3(0.0f, 0.02f, 0.06f));
            }
            else if (name == "side_plates_2")
            {
                // Intake Pivot Arm: Sleek twin anodized orange intake arms on Left & Right + Crossbar
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.96f, 0.48f, 0.08f),
                                   glm::vec3(0.025f, 0.05f, 0.30f), glm::vec3(-0.25f, 0.0f, -0.15f)); // Left arm
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.96f, 0.48f, 0.08f),
                                   glm::vec3(0.025f, 0.05f, 0.30f), glm::vec3(0.25f, 0.0f, -0.15f));  // Right arm
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.96f, 0.48f, 0.08f),
                                   glm::vec3(0.50f, 0.02f, 0.02f), glm::vec3(0.0f, 0.02f, -0.30f));   // Front tie bar
            }
            else if (name == "wheels")
            {
                // High-RPM Flywheel Rollers (Safety Yellow with Dark Anodized Hub Endcaps)
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.98f, 0.88f, 0.12f),
                                   glm::vec3(0.18f, 0.08f, 0.08f), glm::vec3(0.0f, 0.0f, 0.0f));
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.18f, 0.18f, 0.22f),
                                   glm::vec3(0.02f, 0.085f, 0.085f), glm::vec3(-0.09f, 0.0f, 0.0f));
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.18f, 0.18f, 0.22f),
                                   glm::vec3(0.02f, 0.085f, 0.085f), glm::vec3(0.09f, 0.0f, 0.0f));
            }
            else if (name == "side_plates")
            {
                // Funnel Feed Roller (Electric Cyan High-Traction Roller pushing balls upward into flywheel)
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.15f, 0.65f, 0.95f),
                                   glm::vec3(0.18f, 0.065f, 0.065f), glm::vec3(0.0f, 0.0f, 0.0f));
                // Funnel Cheek Guide Plates (Brushed Aluminum)
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.70f, 0.74f, 0.80f),
                                   glm::vec3(0.02f, 0.10f, 0.12f), glm::vec3(-0.10f, 0.0f, 0.0f));
                createCustomVisual(entity, PrimitiveShape::Cube, glm::vec3(0.70f, 0.74f, 0.80f),
                                   glm::vec3(0.02f, 0.10f, 0.12f), glm::vec3(0.10f, 0.0f, 0.0f));
            }
            else if (name.find("hex_shaft") != std::string::npos)
            {
                // Indexer Basket Rollers (Vibrant Green Rollers spanning across width)
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.20f, 0.82f, 0.35f),
                                   glm::vec3(0.46f, 0.035f, 0.035f), glm::vec3(0.0f, 0.0f, 0.0f));
            }
            else if (name.find("sprocket") != std::string::npos)
            {
                // Intake Grip Rollers (Grippy Purple/Red Rubberized Rollers)
                createCustomVisual(entity, PrimitiveShape::CylinderX, glm::vec3(0.78f, 0.25f, 0.40f),
                                   glm::vec3(0.48f, 0.05f, 0.05f), glm::vec3(0.0f, 0.0f, 0.0f));
            }
            else
            {
                // Other Robot Links (Anodized Dark Gray)
                attachVisual(entity, PrimitiveShape::Cube, glm::vec3(0.42f, 0.45f, 0.48f));
            }
        }

        // Add visual meshes to Swerve Steer Housings and Wheels so driving/steering is visible
        if (m_robot.chassisEntity != invalidEntity)
        {
            auto* chassisComp = m_scene.GetComponent<SwerveChassisComponent>(m_robot.chassisEntity);
            if (chassisComp)
            {
                for (uint32_t i = 0; i < 4; ++i)
                {
                    Entity modEnt = chassisComp->moduleEntities[i];
                    if (modEnt == invalidEntity) continue;

                    auto* modComp = m_scene.GetComponent<SwerveModuleComponent>(modEnt);
                    if (!modComp) continue;

                    if (modComp->steerEntity != invalidEntity && !m_scene.GetComponent<MeshComponent>(modComp->steerEntity))
                    {
                        MeshComponent steerMesh;
                        steerMesh.shape = PrimitiveShape::Cylinder;
                        steerMesh.color = glm::vec3(0.35f, 0.38f, 0.42f); // Anodized aluminum steer module
                        m_scene.AddComponent(modComp->steerEntity, steerMesh);
                    }

                    if (modComp->wheelEntity != invalidEntity && !m_scene.GetComponent<MeshComponent>(modComp->wheelEntity))
                    {
                        MeshComponent wheelMesh;
                        wheelMesh.shape = PrimitiveShape::CylinderX;
                        wheelMesh.color = glm::vec3(0.08f, 0.08f, 0.10f); // High-traction tread wheel
                        m_scene.AddComponent(modComp->wheelEntity, wheelMesh);
                    }
                }
            }
        }
    }

    Entity BananaGame::SpawnBall(const glm::vec3& position, bool inShooter)
    {
        Entity ballEntity = m_scene.CreateEntity();

        GamePieceComponent piece;
        piece.spec.name = "Cargo Ball (6-inch Foam)";
        piece.spec.massKg = 0.150;
        piece.spec.thicknessM = 0.1524;
        piece.spec.outerRadiusM = 0.0762; // 6.0-inch diameter sphere (0.0762m radius)
        piece.spec.dragCoefficientCd = 0.47;
        piece.spec.liftSlopeClAlpha = 0.85;
        piece.spec.crossSectionalAreaM2 = 0.01824;
        piece.spec.complianceK = 28000.0;
        piece.spec.restitutionE = 0.40;
        piece.spec.surfaceFriction = 0.72;

        TransformComponent tc;
        tc.position = position;
        tc.scale = glm::vec3(0.1524f); // 6.0 inches diameter
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
            rb.restitution = 0.40f;
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
        // 1. Clear held balls tracking, active shots, combo, and intake deployment state
        m_heldBalls.clear();
        m_activeShots.clear();
        m_comboStreak = 0;
        m_currentPivotAngle = -1.85f; // Stowed upright inside chassis frame
        m_intakeDeployed = false;

        // 2. Remove Existing Balls
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

        // 3. Spawn 28 6-inch Foam Balls in Strategic Formations
        const std::vector<glm::vec3> spawnLocations = {
            // Midfield arc (10 balls)
            { -3.6f, 0.12f, 1.0f },
            { -2.8f, 0.12f, 1.2f },
            { -2.0f, 0.12f, 1.4f },
            { -1.2f, 0.12f, 1.5f },
            { -0.4f, 0.12f, 1.6f },
            {  0.4f, 0.12f, 1.6f },
            {  1.2f, 0.12f, 1.5f },
            {  2.0f, 0.12f, 1.4f },
            {  2.8f, 0.12f, 1.2f },
            {  3.6f, 0.12f, 1.0f },
            // Left Wing Cluster (6 balls)
            { -4.5f, 0.12f, -0.6f },
            { -3.8f, 0.12f, -1.2f },
            { -4.2f, 0.12f, -1.8f },
            { -3.2f, 0.12f, -2.2f },
            { -2.5f, 0.12f, -2.8f },
            { -3.6f, 0.12f, -3.2f },
            // Right Wing Cluster (6 balls)
            {  4.5f, 0.12f, -0.6f },
            {  3.8f, 0.12f, -1.2f },
            {  4.2f, 0.12f, -1.8f },
            {  3.2f, 0.12f, -2.2f },
            {  2.5f, 0.12f, -2.8f },
            {  3.6f, 0.12f, -3.2f },
            // Near Arena Center / Goal Approach (6 balls)
            { -0.8f, 0.12f, -0.8f },
            {  0.0f, 0.12f, -0.6f },
            {  0.8f, 0.12f, -0.8f },
            { -0.6f, 0.12f, -1.8f },
            {  0.0f, 0.12f, -1.6f },
            {  0.6f, 0.12f, -1.8f }
        };

        for (const auto& pos : spawnLocations)
        {
            SpawnBall(pos);
        }

        // 3. Reset All Robot Links to Exact Initial Assembled Poses (Zero Relative Joint Strain)
        for (const auto& [name, entity] : m_robot.linkEntities)
        {
            auto it = m_initialLinkPoses.find(name);
            if (it == m_initialLinkPoses.end()) continue;

            auto* tc = m_scene.GetComponent<TransformComponent>(entity);
            if (tc)
            {
                tc->position = it->second.pos;
                tc->rotation = it->second.rot;
            }
            auto* wt = m_scene.GetComponent<WorldTransformComponent>(entity);
            if (wt)
            {
                wt->worldPosition = it->second.pos;
                wt->worldRotation = it->second.rot;
            }

            auto* rb = m_scene.GetComponent<RigidBodyComponent>(entity);
            if (rb && !rb->bodyID.IsInvalid() && m_joltContext.IsInitialized())
            {
                JPH::BodyInterface& bi = m_joltContext.GetBodyInterface();
                glm::vec3 joltPos = it->second.pos;
                glm::quat joltRot = it->second.rot;
                auto* col = m_scene.GetComponent<ColliderComponent>(entity);
                if (col)
                {
                    joltPos += it->second.rot * col->localOffset;
                    joltRot = glm::normalize(it->second.rot * col->localRotation);
                }
                bi.SetPositionAndRotation(rb->bodyID,
                    JPH::RVec3(joltPos.x, joltPos.y, joltPos.z),
                    JPH::Quat(joltRot.x, joltRot.y, joltRot.z, joltRot.w),
                    JPH::EActivation::Activate);
                bi.SetLinearAndAngularVelocity(rb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            }
        }

        if (m_robot.swerveSystem && m_robot.swerveSystem->GetModel())
        {
            const glm::vec3 spawnPos(0.0f, 0.20f, -2.5f);
            const glm::quat spawnRot(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 posNWU = Robot::Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(spawnPos);
            glm::quat rotNWU = glm::quat(Robot::Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(glm::dquat(spawnRot)));
            m_robot.swerveSystem->GetModel()->ApplyExternalCorrections(posNWU, rotNWU, glm::vec3(0.0f), glm::vec3(0.0f));
            for (uint32_t i = 0; i < 4; ++i)
            {
                m_robot.swerveSystem->GetModel()->GetModule(i).GetState() = Robot::Drivetrain::Model::SwerveModuleState{};
            }
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
        // Advance funnel ball into shooter compression nip if available
        for (auto& [entity, info] : m_heldBalls)
        {
            if (info.slot == BallSlot::Funnel && m_fireCooldown <= 0.0f)
            {
                info.slot = BallSlot::MuzzleExit;
                info.targetLocalPos = glm::vec3(0.0f, 0.46f, 0.24f);
                info.exitProgress = 0.0f;
                m_fireCooldown = 0.25f;
                return;
            }
        }
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

        // 4b. Lock Chassis Pitch and Roll (keep robot perfectly level on carpet)
        // Strictly zero out yaw spin unless user is actively pressing Q or E
        LockChassisPitchAndRoll(m_isSpinningHeading);

        // 5. Goal Trigger & Scoring Pass
        if (m_state == GameState::Playing || m_state == GameState::Countdown)
        {
            CheckGoalScoring(dt);
        }
    }

    void BananaGame::LockChassisPitchAndRoll(bool isSpinning)
    {
        if (m_robot.chassisEntity == invalidEntity) return;

        // 1. Enforce pure Yaw on Jolt RigidBody (zero Pitch around X and Roll around Z)
        auto* chassisRb = m_scene.GetComponent<RigidBodyComponent>(m_robot.chassisEntity);
        if (chassisRb && !chassisRb->bodyID.IsInvalid() && m_joltContext.IsInitialized())
        {
            JPH::BodyInterface& bi = m_joltContext.GetBodyInterface();

            // Zero out pitch (X) and roll (Z) angular velocities in Engine frame (+Y Up)
            // AND if not pressing Q or E, strictly zero out yaw rate so WASD translation NEVER turns the robot!
            JPH::Vec3 angVel = bi.GetAngularVelocity(chassisRb->bodyID);
            float yawRate = isSpinning ? angVel.GetY() : 0.0f;
            bi.SetAngularVelocity(chassisRb->bodyID, JPH::Vec3(0.0f, yawRate, 0.0f));

            // Extract pure yaw twist quaternion around Y axis
            JPH::Quat rot = bi.GetRotation(chassisRb->bodyID);
            float y = rot.GetY();
            float w = rot.GetW();
            float len = std::hypot(y, w);
            if (len > 1e-5f)
            {
                JPH::Quat yawOnly(0.0f, y / len, 0.0f, w / len);
                bi.SetRotation(chassisRb->bodyID, yawOnly, JPH::EActivation::DontActivate);
            }

            // Arena boundary clamp: prevent robot from ever penetrating or getting stuck in walls
            JPH::RVec3 curPos = bi.GetPosition(chassisRb->bodyID);
            float px = static_cast<float>(curPos.GetX());
            float pz = static_cast<float>(curPos.GetZ());
            constexpr float kMaxX = 8.40f;
            constexpr float kMaxZ = 6.40f;

            if (std::abs(px) > kMaxX || std::abs(pz) > kMaxZ)
            {
                float cx = std::clamp(px, -kMaxX, kMaxX);
                float cz = std::clamp(pz, -kMaxZ, kMaxZ);
                bi.SetPosition(chassisRb->bodyID, JPH::RVec3(cx, curPos.GetY(), cz), JPH::EActivation::Activate);
                JPH::Vec3 v = bi.GetLinearVelocity(chassisRb->bodyID);
                bi.SetLinearVelocity(chassisRb->bodyID, JPH::Vec3(
                    (cx != px) ? 0.0f : v.GetX(),
                    v.GetY(),
                    (cz != pz) ? 0.0f : v.GetZ()
                ));
            }
        }

        // 2. Sync pure Yaw to ECS Transform and WorldTransform
        auto* tc = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (tc)
        {
            float y = tc->rotation.y;
            float w = tc->rotation.w;
            float len = std::hypot(y, w);
            if (len > 1e-5f)
            {
                tc->rotation = glm::quat(w / len, 0.0f, y / len, 0.0f);
            }
            tc->position.x = std::clamp(tc->position.x, -8.40f, 8.40f);
            tc->position.z = std::clamp(tc->position.z, -6.40f, 6.40f);
        }
        auto* wt = m_scene.GetComponent<WorldTransformComponent>(m_robot.chassisEntity);
        if (wt)
        {
            float y = wt->worldRotation.y;
            float w = wt->worldRotation.w;
            float len = std::hypot(y, w);
            if (len > 1e-5f)
            {
                wt->worldRotation = glm::quat(w / len, 0.0f, y / len, 0.0f);
            }
            wt->worldPosition.x = std::clamp(wt->worldPosition.x, -8.40f, 8.40f);
            wt->worldPosition.z = std::clamp(wt->worldPosition.z, -6.40f, 6.40f);
        }

        // 3. Clear Roll and Pitch in underlying Swerve Drivetrain Model (in NWU: +Z is Up/Yaw)
        if (m_robot.swerveSystem && m_robot.swerveSystem->GetModel())
        {
            auto& state = m_robot.swerveSystem->GetModel()->GetChassisState();
            state.angularVelocity.x = 0.0f; // Roll
            state.angularVelocity.y = 0.0f; // Pitch
            if (!isSpinning)
            {
                state.angularVelocity.z = 0.0f; // Zero yaw rate when not pressing Q/E
            }

            float z = state.orientation.z;
            float w = state.orientation.w;
            float len = std::hypot(z, w);
            if (len > 1e-5f)
            {
                state.orientation = glm::quat(w / len, 0.0f, 0.0f, z / len);
            }
        }
    }

    void BananaGame::UpdateArcadeRobotController(double dt)
    {
        SimInputs inputs;
        inputs.robotEnabled = (m_state == GameState::Playing);

        if (m_state != GameState::Playing)
        {
            m_isSpinningHeading = false;
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

        m_isSpinningHeading = (std::abs(spinRate) > 1e-3f);

        // Zero Gyro Heading with Z
        if (io.WasKeyPressed(GLFW_KEY_Z))
        {
            if (m_robot.swerveSystem)
            {
                m_fieldCentricHeadingRad = static_cast<float>(m_robot.swerveSystem->GetOutputs().gyroYaw);
            }
        }

        // Get Chassis Yaw for Field-Centric Translation
        float chassisYaw = 0.0f;
        if (m_robot.swerveSystem)
        {
            chassisYaw = static_cast<float>(m_robot.swerveSystem->GetOutputs().gyroYaw) - m_fieldCentricHeadingRad;
        }

        float cmdVx = fieldVz; // In NWU: +X is Forward (towards goal Z)
        float cmdVy = fieldVx; // In NWU: +Y is Left (towards -X)

        // Permanent Field-Centric Drive Translation
        float cosY = std::cos(-chassisYaw);
        float sinY = std::sin(-chassisYaw);
        cmdVx = (fieldVz * cosY) - (fieldVx * sinY);
        cmdVy = (fieldVz * sinY) + (fieldVx * cosY);

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
        const auto& swerveOutputs = m_robot.swerveSystem ? m_robot.swerveSystem->GetOutputs() : SimOutputs{};

        for (uint32_t i = 0; i < 4; ++i)
        {
            if (modMag[i] <= 1e-3)
            {
                // When idle, supply 0V to prevent jitter and spurious steering torques
                inputs.motorCommands[i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[4 + i].demand = 0.0;
                continue;
            }

            double targetAngle = std::atan2(modVy[i], modVx[i]);
            double driveV = modMag[i] * desaturate;

            double curAngle = swerveOutputs.encoderPositions[i];
            double curVel = swerveOutputs.encoderVelocities[i];
            double err = targetAngle - curAngle;

            while (err > 3.141592653589793) err -= 6.283185307179586;
            while (err < -3.141592653589793) err += 6.283185307179586;

            // Swerve 180-degree module azimuth optimization
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

        // B. 3 Independent Mechanism Controls (Can all be pressed simultaneously)
        // #1 Intake: Runs intake rollers + bottom basket + deploys arm (Hold Spacebar or Shift)
        m_btnIntake = io.IsKeyDown(GLFW_KEY_SPACE) || io.IsKeyDown(GLFW_KEY_LEFT_SHIFT);

        // #2 Funnel: Runs funnel roller + bottom basket (Hold F or Right Mouse Button or V)
        m_btnFunnel = io.IsKeyDown(GLFW_KEY_F) || io.IsMouseButtonDown(GLFW_MOUSE_BUTTON_RIGHT) || io.IsKeyDown(GLFW_KEY_V);

        // #3 Flywheel: Runs shooter flywheel (Hold Left Mouse Button or C)
        m_btnFlywheel = io.IsMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT) || io.IsKeyDown(GLFW_KEY_C);
        m_flywheelSpinning = m_btnFlywheel;

        // Toggle Intake Pivot Deployment with I key
        if (io.WasKeyPressed(GLFW_KEY_I))
        {
            m_intakeDeployed = !m_intakeDeployed;
        }

        // Animate Intake Pivot deployment smoothly toward commanded angle
        // In Jolt / CAD: -1.85 rad is resting flat on carpet (DEPLOYED), 0.0 rad is folded upright into chassis frame (STOWED)
        constexpr float kPivotAngleDeployed = -1.85f;
        constexpr float kPivotAngleStowed = 0.0f;

        bool wantDeployed = (m_btnIntake || m_intakeDeployed);
        float targetPivotAngle = wantDeployed ? kPivotAngleDeployed : kPivotAngleStowed;
        float pivotSpeed = 4.8f; // rad/s realistic deployment speed
        if (m_currentPivotAngle < targetPivotAngle)
        {
            m_currentPivotAngle = std::min(m_currentPivotAngle + static_cast<float>(pivotSpeed * dt), targetPivotAngle);
        }
        else if (m_currentPivotAngle > targetPivotAngle)
        {
            m_currentPivotAngle = std::max(m_currentPivotAngle - static_cast<float>(pivotSpeed * dt), targetPivotAngle);
        }

        // Gate: ONLY allow intaking balls when the intake pivot is physically DOWN on the carpet
        bool isPivotDown = (m_currentPivotAngle <= -1.40f);

        // Drive Intake Pivot via Jolt Joint Position Motor
        if (m_intakeJoint)
        {
            m_intakeJoint->motorMode = Physics::Components::JointMotorMode::Position;
            m_intakeJoint->springFrequency = 12.0f;
            m_intakeJoint->springDamping = 1.0f;
            m_intakeJoint->maxMotorTorque = 450.0f;
            m_intakeJoint->targetAngle = m_currentPivotAngle;
        }

        if (m_intakePivotArm)
        {
            m_intakePivotArm->commandedVoltage = wantDeployed ? 8.0 : -8.0;
        }

        // Intake Rollers (Button #1)
        double intakeRollerV = m_btnIntake ? 12.0 : 0.0;
        if (m_frontIntake)  m_frontIntake->commandedVoltage = intakeRollerV;
        if (m_backIntake)   m_backIntake->commandedVoltage = intakeRollerV;

        // Bottom Basket (Button #1 OR Button #2)
        double basketV = (m_btnIntake || m_btnFunnel) ? 10.0 : 0.0;
        if (m_bottomBasket) m_bottomBasket->commandedVoltage = basketV;

        // Funnel Roller (Button #2)
        double funnelV = m_btnFunnel ? 12.0 : 0.0;
        if (m_funnel)       m_funnel->commandedVoltage = funnelV;

        // Flywheel (Button #3)
        double flywheelV = m_btnFlywheel ? 12.0 : 0.0;
        if (m_flywheel)     m_flywheel->commandedVoltage = flywheelV;

        // C. Multi-Slot Ball Queue & Physical Conveyor
        constexpr int kNumIntakeSlots = 3;
        constexpr int kNumBasketSlots = 18;

        // Waypoint Generator for Basket: 3 columns wide x 2 rows deep x 3 layers tall = 18 slots
        auto GetBasketSlotPos = [](int idx) -> glm::vec3 {
            idx = std::clamp(idx, 0, kNumBasketSlots - 1);
            int layer = idx / 6; // 0, 1, 2 (bottom, middle, top layer)
            int rem = idx % 6;
            int row = rem / 3;   // 0 (front row near funnel), 1 (rear row near intake)
            int col = rem % 3;   // 0..2 (3 wide across robot width)

            // 3 columns across: cleanly clears the hopper polycarbonate side panels (at X = +/- 0.22m)
            static const float colX[3] = { -0.115f, 0.0f, +0.115f };
            // 2 rows along length of basket (from funnel throat at Z = -0.095m to rear near intake at Z = -0.235m)
            static const float rowZ[2] = { -0.095f, -0.235f };
            // 3 vertical layers resting cleanly on bottom rollers (roller top is at Y = 0.206m)
            static const float layerY[3] = { 0.235f, 0.355f, 0.475f };

            glm::vec3 pos(colX[col], layerY[layer], rowZ[row]);

            // Stagger middle layer (layer 1) into the valleys between layer 0 balls
            if (layer == 1)
            {
                pos.x += (col == 1) ? 0.00f : ((col == 0) ? +0.025f : -0.025f);
                pos.z += (row == 0) ? -0.015f : +0.015f;
            }
            else if (layer == 2)
            {
                pos.x += (col == 0) ? +0.015f : ((col == 2) ? -0.015f : 0.0f);
            }

            // Organic deterministic offsets so balls look naturally packed into the hopper
            float jitterX = std::sin(static_cast<float>(idx) * 2.83f + 0.7f) * 0.008f;
            float jitterZ = std::cos(static_cast<float>(idx) * 3.41f + 1.3f) * 0.008f;
            float jitterY = std::sin(static_cast<float>(idx) * 5.17f + 2.1f) * 0.004f;

            return pos + glm::vec3(jitterX, jitterY, jitterZ);
        };

        // Intake Tray: 3 columns wide across intake mouth (fits cleanly between orange intake arms)
        auto GetIntakeTraySlotPos = [](int col) -> glm::vec3 {
            col = std::clamp(col, 0, kNumIntakeSlots - 1);
            static const float colX[3] = { -0.115f, 0.0f, +0.115f };
            return glm::vec3(colX[col], 0.14f, -0.52f);
        };

        const glm::vec3 P_Funnel(0.0f, 0.34f, 0.08f);       // Funnel throat beneath flywheel
        const glm::vec3 P_MuzzleExit(0.0f, 0.46f, 0.24f);   // Shooter compression nip

        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (!chassisT) return;
        glm::vec3 chassisPos = chassisT->position;
        glm::quat chassisRot = chassisT->rotation;
        JPH::BodyInterface* bi = m_joltContext.IsInitialized() ? &m_joltContext.GetBodyInterface() : nullptr;

        // Identify current slot occupancy across all slots
        std::array<Entity, kNumIntakeSlots> intakeSlots;
        intakeSlots.fill(invalidEntity);
        std::array<Entity, kNumBasketSlots> basketSlots;
        basketSlots.fill(invalidEntity);
        Entity ballInFunnel = invalidEntity;

        for (const auto& [entity, info] : m_heldBalls)
        {
            if (!m_scene.IsEntityValid(entity)) continue;
            switch (info.slot)
            {
            case BallSlot::IntakeTray:
                if (info.slotIndex >= 0 && info.slotIndex < kNumIntakeSlots)
                    intakeSlots[info.slotIndex] = entity;
                break;
            case BallSlot::Basket:
                if (info.slotIndex >= 0 && info.slotIndex < kNumBasketSlots)
                    basketSlots[info.slotIndex] = entity;
                break;
            case BallSlot::Funnel:
                ballInFunnel = entity;
                break;
            default:
                break;
            }
        }

        // 1. Button #1 (Intake): Vacuum balls on carpet into Intake Tray
        // Strictly gated: requires Button #1 held, pivot DOWN, and at least one empty Intake Tray slot!
        if (m_btnIntake && isPivotDown)
        {
            glm::vec3 intakeMouth = chassisPos + (chassisRot * glm::vec3(0.0f, 0.12f, -0.65f));

            // Check each free intake slot
            for (int col = 0; col < kNumIntakeSlots; ++col)
            {
                if (intakeSlots[col] != invalidEntity) continue; // slot occupied

                Entity bestBall = invalidEntity;
                float bestDistSq = 0.70f * 0.70f;
                float targetLocalX = (col == 0) ? -0.115f : (col == 1) ? 0.0f : 0.115f;

                for (Entity e : m_ballEntities)
                {
                    if (!m_scene.IsEntityValid(e)) continue;
                    if (m_heldBalls.find(e) != m_heldBalls.end()) continue; // already held

                    auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
                    auto* tc = m_scene.GetComponent<TransformComponent>(e);
                    if (!piece || !tc) continue;

                    if (piece->state == GamePieceState::Free)
                    {
                        glm::vec3 delta = tc->position - intakeMouth;
                        float dSq = glm::dot(delta, delta);
                        if (dSq < bestDistSq)
                        {
                            glm::vec3 localPos = glm::inverse(chassisRot) * (tc->position - chassisPos);
                            // Prefer balls closest to this column's lateral lane
                            float laneErr = std::abs(localPos.x - targetLocalX);
                            if (laneErr < 0.22f)
                            {
                                bestDistSq = dSq;
                                bestBall = e;
                            }
                        }
                    }
                }

                if (bestBall != invalidEntity)
                {
                    auto* tc = m_scene.GetComponent<TransformComponent>(bestBall);
                    auto* piece = m_scene.GetComponent<GamePieceComponent>(bestBall);
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(bestBall);

                    glm::vec3 localPos = glm::inverse(chassisRot) * (tc->position - chassisPos);
                    localPos.x = std::clamp(localPos.x, -0.22f, 0.22f);
                    localPos.y = std::clamp(localPos.y, 0.10f, 0.18f);
                    localPos.z = std::clamp(localPos.z, -0.72f, -0.58f);

                    if (bi && rb && !rb->bodyID.IsInvalid() && bi->IsAdded(rb->bodyID))
                    {
                        bi->SetMotionType(rb->bodyID, JPH::EMotionType::Kinematic, JPH::EActivation::DontActivate);
                        bi->SetLinearVelocity(rb->bodyID, JPH::Vec3(0, 0, 0));
                        bi->SetAngularVelocity(rb->bodyID, JPH::Vec3(0, 0, 0));
                    }

                    piece->state = GamePieceState::Intaking;
                    piece->capturingRobot = m_robot.chassisEntity;

                    HeldBallInfo info;
                    info.entity = bestBall;
                    info.slot = BallSlot::IntakeTray;
                    info.slotIndex = col;
                    info.currentLocalPos = localPos;
                    info.targetLocalPos = GetIntakeTraySlotPos(col);
                    m_heldBalls[bestBall] = info;
                    intakeSlots[col] = bestBall;
                    m_ballsCollected++;
                }
            }
        }

        // 2. Progression through bottom basket and funnel
        // A. Funnel -> MuzzleExit (Shooter firing)
        if (m_fireCooldown > 0.0f)
        {
            m_fireCooldown -= static_cast<float>(dt);
        }

        if (m_btnFlywheel && m_fireCooldown <= 0.0f && ballInFunnel != invalidEntity)
        {
            auto& info = m_heldBalls[ballInFunnel];
            info.slot = BallSlot::MuzzleExit;
            info.targetLocalPos = P_MuzzleExit;
            info.exitProgress = 0.0f;
            m_fireCooldown = 0.30f; // Controlled satisfying shooting cadence (~3.3 shots/sec)
            ballInFunnel = invalidEntity;
        }

        // B. Basket -> Funnel (Button #2 runs funnel)
        if (m_btnFunnel && ballInFunnel == invalidEntity)
        {
            // Find lowest index ball in basket (closest to funnel entry)
            for (int bIdx = 0; bIdx < kNumBasketSlots; ++bIdx)
            {
                Entity e = basketSlots[bIdx];
                if (e != invalidEntity)
                {
                    auto& info = m_heldBalls[e];
                    info.slot = BallSlot::Funnel;
                    info.slotIndex = 0;
                    info.targetLocalPos = P_Funnel;
                    auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
                    if (piece) piece->state = GamePieceState::Engaged;
                    ballInFunnel = e;
                    basketSlots[bIdx] = invalidEntity;
                    break;
                }
            }
        }

        // C. Internal Basket Cascade (Shift forward toward funnel)
        if (m_btnIntake || m_btnFunnel)
        {
            for (int dst = 0; dst < kNumBasketSlots; ++dst)
            {
                if (basketSlots[dst] == invalidEntity)
                {
                    // Find first ball in a higher slot to shift down
                    for (int src = dst + 1; src < kNumBasketSlots; ++src)
                    {
                        if (basketSlots[src] != invalidEntity)
                        {
                            Entity e = basketSlots[src];
                            auto& info = m_heldBalls[e];
                            info.slotIndex = dst;
                            info.targetLocalPos = GetBasketSlotPos(dst);
                            basketSlots[dst] = e;
                            basketSlots[src] = invalidEntity;
                            break;
                        }
                    }
                }
            }
        }

        // D. IntakeTray -> Basket (Button #1 runs intake rollers and basket)
        if (m_btnIntake)
        {
            for (int col = 0; col < kNumIntakeSlots; ++col)
            {
                Entity e = intakeSlots[col];
                if (e == invalidEntity) continue;

                auto& info = m_heldBalls[e];
                if (glm::distance(info.currentLocalPos, info.targetLocalPos) <= 0.14f)
                {
                    // Find first empty slot in basket (layer 0 first, then layer 1, then layer 2)
                    int emptyBasketIdx = -1;
                    for (int bIdx = 0; bIdx < kNumBasketSlots; ++bIdx)
                    {
                        if (basketSlots[bIdx] == invalidEntity)
                        {
                            emptyBasketIdx = bIdx;
                            break;
                        }
                    }

                    if (emptyBasketIdx != -1)
                    {
                        info.slot = BallSlot::Basket;
                        info.slotIndex = emptyBasketIdx;
                        info.targetLocalPos = GetBasketSlotPos(emptyBasketIdx);
                        auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
                        if (piece) piece->state = GamePieceState::Stored;
                        basketSlots[emptyBasketIdx] = e;
                        intakeSlots[col] = invalidEntity;
                    }
                    // If no empty basket slot, basket is 18/18 FULL: ball remains in intake slot!
                }
            }
        }

        // 3. Smooth Physical Translation of All Held Balls (No Teleportation)
        std::vector<Entity> ballsToErase;
        float baseTransferSpeed = 2.6f; // m/s realistic conveyor speed

        for (auto& [e, info] : m_heldBalls)
        {
            if (!m_scene.IsEntityValid(e))
            {
                ballsToErase.push_back(e);
                continue;
            }

            float speed = (info.slot == BallSlot::MuzzleExit) ? 15.0f : baseTransferSpeed;
            glm::vec3 diff = info.targetLocalPos - info.currentLocalPos;
            float dist = glm::length(diff);
            float maxStep = speed * static_cast<float>(dt);

            if (dist <= maxStep)
            {
                info.currentLocalPos = info.targetLocalPos;
            }
            else
            {
                info.currentLocalPos += (diff / dist) * maxStep;
            }

            // Check if muzzle exit is reached for dynamic launch
            if (info.slot == BallSlot::MuzzleExit && glm::distance(info.currentLocalPos, P_MuzzleExit) <= 0.02f)
            {
                glm::vec3 launchPos = chassisPos + (chassisRot * P_MuzzleExit);

                // Ballistic Trajectory targeting m_goalCenter (0.0f, 2.20f, 5.5f)
                glm::vec3 toGoal = m_goalCenter - launchPos;
                float horizDist = std::hypot(toGoal.x, toGoal.z);
                float deltaY = toGoal.y; // 2.20 - launchPos.y ~ 1.74m

                // Robot forward direction in world space (+Z in robot frame)
                glm::vec3 robotFwd = chassisRot * glm::vec3(0.0f, 0.0f, 1.0f);
                glm::vec2 fwd2D = glm::normalize(glm::vec2(robotFwd.x, robotFwd.z));
                glm::vec2 goal2D = (horizDist > 0.1f) ? glm::vec2(toGoal.x / horizDist, toGoal.z / horizDist) : fwd2D;

                float aimDot = glm::dot(fwd2D, goal2D);
                glm::vec3 exitVel;
                constexpr float g = 9.81f;

                // When robot is generally facing toward the goal (+/- 55 degrees) and within field range
                if (aimDot > 0.35f && horizDist > 1.0f && horizDist < 14.0f)
                {
                    // Auto-assist: gently guide launch azimuth towards goal center
                    float assistStrength = 0.70f;
                    glm::vec2 blended2D = glm::normalize(glm::mix(fwd2D, goal2D, assistStrength));

                    // Elevation angle tuned for high arching swish
                    float elevationDeg = std::clamp(54.0f - (horizDist - 2.0f) * 1.5f, 38.0f, 54.0f);
                    float theta = glm::radians(elevationDeg);
                    float cosT = std::cos(theta);
                    float tanT = std::tan(theta);

                    // Trajectory velocity: v^2 = g * D^2 / (2 * cos^2(theta) * (D * tan(theta) - deltaY))
                    float denom = 2.0f * (cosT * cosT) * (horizDist * tanT - deltaY);
                    float speed = 15.0f;
                    if (denom > 0.1f)
                    {
                        speed = std::sqrt((g * horizDist * horizDist) / denom);
                        speed *= 1.03f; // slight nudge to drop cleanly through hoop center
                    }
                    speed = std::clamp(speed, 7.5f, 22.0f);

                    exitVel = glm::vec3(
                        blended2D.x * speed * cosT,
                        speed * std::sin(theta),
                        blended2D.y * speed * cosT
                    );
                }
                else
                {
                    // Unassisted shot straight forward
                    glm::vec3 localDir(0.0f, 0.61566f, 0.78801f); // 38 deg
                    glm::vec3 worldDir = glm::normalize(chassisRot * localDir);
                    exitVel = worldDir * 15.0f;
                }

                glm::vec3 chassisLinVel(0.0f);
                auto* chassisRb = m_scene.GetComponent<RigidBodyComponent>(m_robot.chassisEntity);
                if (bi && chassisRb && !chassisRb->bodyID.IsInvalid())
                {
                    JPH::Vec3 lv = bi->GetLinearVelocity(chassisRb->bodyID);
                    chassisLinVel = glm::vec3(lv.GetX(), lv.GetY(), lv.GetZ());
                }

                exitVel += chassisLinVel * 0.25f;

                auto* tc = m_scene.GetComponent<TransformComponent>(e);
                auto* wt = m_scene.GetComponent<WorldTransformComponent>(e);
                auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                auto* piece = m_scene.GetComponent<GamePieceComponent>(e);

                if (tc) tc->position = launchPos;
                if (wt) wt->worldPosition = launchPos;

                if (bi && rb && !rb->bodyID.IsInvalid() && bi->IsAdded(rb->bodyID))
                {
                    bi->SetMotionType(rb->bodyID, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                    bi->SetPosition(rb->bodyID, JPH::RVec3(launchPos.x, launchPos.y, launchPos.z), JPH::EActivation::Activate);
                    bi->SetLinearVelocity(rb->bodyID, JPH::Vec3(exitVel.x, exitVel.y, exitVel.z));
                    bi->SetAngularVelocity(rb->bodyID, JPH::Vec3(-28.0f, 0.0f, 0.0f)); // backspin
                }

                if (piece)
                {
                    piece->state = GamePieceState::Launched;
                    piece->linearVelocity = exitVel;
                    piece->capturingRobot = invalidEntity;
                    piece->boundContactZone = invalidEntity;
                }

                m_activeShots[e] = 0.0f;
                m_ballsFired++;
                ballsToErase.push_back(e);
                continue;
            }

            // Sync smooth kinematic position to ECS and Jolt
            glm::vec3 worldPos = chassisPos + (chassisRot * info.currentLocalPos);
            auto* tc = m_scene.GetComponent<TransformComponent>(e);
            auto* wt = m_scene.GetComponent<WorldTransformComponent>(e);
            auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);

            if (tc) tc->position = worldPos;
            if (wt) wt->worldPosition = worldPos;

            if (bi && rb && !rb->bodyID.IsInvalid() && bi->IsAdded(rb->bodyID))
            {
                bi->SetPosition(rb->bodyID, JPH::RVec3(worldPos.x, worldPos.y, worldPos.z), JPH::EActivation::DontActivate);
            }
        }

        for (Entity e : ballsToErase)
        {
            m_heldBalls.erase(e);
        }

        // D. Reset Field (R key - strictly disallowed mid arcade challenge, permitted in Free Practice)
        if (io.WasKeyPressed(GLFW_KEY_R))
        {
            bool isMidChallenge = (m_mode == GameMode::ArcadeRush && (m_state == GameState::Playing || m_state == GameState::Countdown));
            if (!isMidChallenge)
            {
                ResetField();
            }
        }
    }

    void BananaGame::CheckGoalScoring(double dt)
    {
        // High Goal Trigger Bounding Cylinder around m_goalCenter (X=0, Y=2.2m, Z=5.5m)
        const float targetRadiusSq = m_goalRadius * m_goalRadius;
        JPH::BodyInterface* bi = m_joltContext.IsInitialized() ? &m_joltContext.GetBodyInterface() : nullptr;

        for (Entity e : m_ballEntities)
        {
            if (!m_scene.IsEntityValid(e)) continue;

            auto* tc = m_scene.GetComponent<TransformComponent>(e);
            auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
            if (!tc || !piece) continue;

            // Only score balls that are in flight or descending, and not currently on cooldown
            if ((piece->state == GamePieceState::Launched || piece->state == GamePieceState::Free) && m_ballScoreCooldown[e] <= 0.0f)
            {
                float dx = tc->position.x - m_goalCenter.x;
                float dz = tc->position.z - m_goalCenter.z;
                float distSq = dx * dx + dz * dz;

                // Inside rim diameter and passing downward through net level
                if (distSq < targetRadiusSq && tc->position.y >= 1.80f && tc->position.y <= 2.45f)
                {
                    auto* rb = m_scene.GetComponent<RigidBodyComponent>(e);
                    bool isFalling = (rb && rb->linearVelocity.y < 0.25f) || (piece->linearVelocity.y < 0.25f);

                    if (isFalling)
                    {
                        TriggerGoalScored(tc->position);
                        m_activeShots.erase(e);

                        // Prevent counting multiple scores for the same ball
                        m_ballScoreCooldown[e] = 2.5f;

                        // Recycle ball: move both ECS and Jolt physical body immediately below net onto floor
                        glm::vec3 recyclePos(0.0f, 0.40f, 4.8f);
                        tc->position = recyclePos;
                        auto* wt = m_scene.GetComponent<WorldTransformComponent>(e);
                        if (wt) wt->worldPosition = recyclePos;

                        if (bi && rb && !rb->bodyID.IsInvalid() && bi->IsAdded(rb->bodyID))
                        {
                            bi->SetPositionAndRotation(rb->bodyID,
                                JPH::RVec3(recyclePos.x, recyclePos.y, recyclePos.z),
                                JPH::Quat::sIdentity(),
                                JPH::EActivation::Activate);
                            bi->SetLinearAndAngularVelocity(rb->bodyID,
                                JPH::Vec3(0.0f, -0.5f, -1.0f),
                                JPH::Vec3::sZero());
                        }

                        piece->state = GamePieceState::Free;
                        piece->linearVelocity = glm::vec3(0.0f, -0.5f, -1.0f);
                    }
                }
            }
        }

        // Active shot flight tracking: detect misses and reset combo
        std::vector<Entity> missedShots;
        for (auto& [e, flightTime] : m_activeShots)
        {
            flightTime += static_cast<float>(dt);
            if (!m_scene.IsEntityValid(e))
            {
                missedShots.push_back(e);
                continue;
            }

            auto* tc = m_scene.GetComponent<TransformComponent>(e);
            auto* piece = m_scene.GetComponent<GamePieceComponent>(e);
            if (!tc || !piece)
            {
                missedShots.push_back(e);
                continue;
            }

            // Ball missed: either hit floor after flight (flightTime >= 0.35s and pos.y <= 0.15m) or timed out (> 3.5s)
            bool hitFloor = (flightTime >= 0.35f && tc->position.y <= 0.15f);
            bool timedOut = (flightTime > 3.5f);

            if (hitFloor || timedOut)
            {
                if (m_comboStreak > 0)
                {
                    m_comboStreak = 0;
                    ScorePopup popup;
                    popup.text = "COMBO LOST!";
                    popup.color = glm::vec3(1.0f, 0.2f, 0.2f);
                    popup.worldPos = tc->position + glm::vec3(0.0f, 0.35f, 0.0f);
                    popup.lifetime = 0.0f;
                    popup.maxLifetime = 1.2f;
                    m_scorePopups.push_back(popup);
                }
                piece->state = GamePieceState::Free;
                missedShots.push_back(e);
            }
        }
        for (Entity e : missedShots)
        {
            m_activeShots.erase(e);
        }
    }

    void BananaGame::TriggerGoalScored(const glm::vec3& ballPos)
    {
        m_goalsScored++;
        m_comboStreak++;
        if (m_comboStreak > m_maxCombo) m_maxCombo = m_comboStreak;

        int addedPoints = 100 * m_comboStreak;
        m_score += addedPoints;
        if (m_score > m_highScore)
        {
            m_highScore = m_score;
            SaveHighScore();
        }

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

    std::string BananaGame::GetHighScoreFilePath() const
    {
        std::string exeDir = GetExecutableDirectory();
        std::error_code ec;

        const std::vector<std::string> searchPrefixes = {
            exeDir.empty() ? "" : exeDir + "/Resources/Config/",
            exeDir.empty() ? "" : exeDir + "/",
            "FRCBananaSim/Resources/Config/",
            "Resources/Config/",
            "../../FRCBananaSim/Resources/Config/",
            "../Resources/Config/",
            "../", "../../", ""
        };

        // If file already exists anywhere in standard search paths, use that exact path
        for (const auto& prefix : searchPrefixes)
        {
            if (prefix.empty()) continue;
            std::string candidate = prefix + "highscore.json";
            if (std::filesystem::exists(candidate, ec))
            {
                return candidate;
            }
        }

        // If not found yet, prefer writing to Resources/Config/ if that directory exists
        for (const auto& prefix : searchPrefixes)
        {
            if (!prefix.empty() && std::filesystem::is_directory(prefix, ec))
            {
                return prefix + "highscore.json";
            }
        }

        if (!exeDir.empty())
        {
            return exeDir + "/highscore.json";
        }

        return "highscore.json";
    }

    void BananaGame::LoadHighScore()
    {
        std::string path = GetHighScoreFilePath();
        if (!std::filesystem::exists(path))
        {
            SIM_LOG_INFO("[BananaGame] No previous highscore.json found at: " << path);
            return;
        }

        std::ifstream file(path);
        if (!file.is_open())
        {
            SIM_LOG_WARN("[BananaGame] Failed to open high score file: " << path);
            return;
        }

        try
        {
            nlohmann::json j;
            file >> j;
            if (j.contains("highScore"))
            {
                m_highScore = j["highScore"].get<int>();
            }
            if (j.contains("maxCombo"))
            {
                m_maxCombo = j["maxCombo"].get<int>();
            }
            SIM_LOG_INFO("[BananaGame] Loaded persistent high score: " << m_highScore << " PTS (Best Combo: x" << m_maxCombo << ") from " << path);
        }
        catch (const std::exception& e)
        {
            SIM_LOG_WARN("[BananaGame] Could not parse high score JSON (" << path << "): " << e.what());
        }
    }

    void BananaGame::SaveHighScore()
    {
        if (m_highScore <= 0)
        {
            return;
        }

        std::string path = GetHighScoreFilePath();
        std::ofstream file(path);
        if (!file.is_open())
        {
            SIM_LOG_WARN("[BananaGame] Could not open " << path << " to write high score.");
            return;
        }

        try
        {
            nlohmann::json j;
            j["highScore"] = m_highScore;
            j["maxCombo"] = m_maxCombo;
            file << j.dump(4) << std::endl;
            SIM_LOG_INFO("[BananaGame] Saved persistent high score: " << m_highScore << " PTS to " << path);
        }
        catch (const std::exception& e)
        {
            SIM_LOG_WARN("[BananaGame] Failed to serialize high score: " << e.what());
        }
    }

    void BananaGame::Update(double dt)
    {
        // High-precision real wall-clock delta time (guarantees timer ticks exactly 1 sec per real sec)
        double now = glfwGetTime();
        if (m_lastRealTime <= 0.0)
        {
            m_lastRealTime = now;
        }
        float realDt = static_cast<float>(now - m_lastRealTime);
        m_lastRealTime = now;
        realDt = std::clamp(realDt, 0.001f, 0.100f);

        // Decay ball score cooldowns
        for (auto& [e, cd] : m_ballScoreCooldown)
        {
            if (cd > 0.0f) cd -= realDt;
        }

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

        // Reset Field (R key: available in Free Practice, menus, or paused; blocked mid Arcade Rush)
        if (io.WasKeyPressed(GLFW_KEY_R))
        {
            bool isMidChallenge = (m_mode == GameMode::ArcadeRush && (m_state == GameState::Playing || m_state == GameState::Countdown));
            if (!isMidChallenge)
            {
                ResetField();
            }
        }

        // Game State Machine Transitions
        if (m_state == GameState::Countdown)
        {
            m_countdownTimer -= realDt;
            if (m_countdownTimer <= 0.0f)
            {
                m_state = GameState::Playing;
            }
        }
        else if (m_state == GameState::Playing)
        {
            if (m_mode == GameMode::ArcadeRush)
            {
                m_sessionTimer -= realDt;
                if (m_sessionTimer <= 0.0f)
                {
                    m_sessionTimer = 0.0f;
                    m_state = GameState::GameOver;
                    SaveHighScore();
                }
            }
        }

        // Animate Floating Score Popups
        for (auto it = m_scorePopups.begin(); it != m_scorePopups.end();)
        {
            it->lifetime += realDt;
            it->worldPos.y += static_cast<float>(realDt * 0.6f); // Float upward
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
            m_goalFlashTimer -= realDt;
        }

        // Smooth Camera Follow
        UpdateCamera(realDt);
    }

    void BananaGame::UpdateCamera(double dt)
    {
        auto& camera = m_renderer.GetCamera();
        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (!chassisT) return;

        glm::vec3 robotPos = chassisT->position;

        // Dedicated field-aligned follow camera: smooth follow behind (-Z) and above (+Y)
        // Does NOT rotate with robot yaw, matching natural field-centric driver controls
        camera.SetMode(Renderer::Camera::Mode::FreeFly);
        const glm::vec3 worldOffset(0.0f, 2.50f, -4.50f);
        glm::vec3 targetCamPos = robotPos + worldOffset;
        glm::vec3 targetLookAt = robotPos + glm::vec3(0.0f, 0.40f, 0.80f);

        float blend = std::clamp(static_cast<float>(dt * 8.0), 0.01f, 1.0f);
        m_smoothCameraPos = glm::mix(m_smoothCameraPos, targetCamPos, blend);
        m_smoothCameraTarget = glm::mix(m_smoothCameraTarget, targetLookAt, blend);

        camera.position = m_smoothCameraPos;
        camera.orbitTarget = m_smoothCameraTarget;

        glm::vec3 dir = glm::normalize(m_smoothCameraTarget - m_smoothCameraPos);
        camera.pitchDegrees = glm::degrees(std::asin(std::clamp(dir.y, -0.999f, 0.999f)));
        camera.yawDegrees = glm::degrees(std::atan2(dir.z, dir.x));
    }

    void BananaGame::Render3D()
    {
        // 1. Update mechanism & wheel visual mesh transforms before drawing pass
        if (m_robot.flywheelSystem) m_robot.flywheelSystem->RenderUpdate();
        if (m_robot.swerveSystem) m_robot.swerveSystem->RenderUpdate();

        // 2. Render all solid 3D meshes (Field carpet, arena perimeter walls, goal post & backboard, balls, robot links)
        m_scene.RenderUpdateSystems();

        // 3. Draw Target Goal Rim Highlight & Flash FX
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

        // 4. Flush Line Buffer to GPU
        m_renderer.FlushDebugDraw();
    }

    namespace
    {
        void ApplyGameTheme()
        {
            ImGuiStyle& style = ImGui::GetStyle();
            style.WindowRounding    = 14.0f;
            style.ChildRounding     = 10.0f;
            style.FrameRounding     = 8.0f;
            style.PopupRounding     = 12.0f;
            style.ScrollbarRounding = 8.0f;
            style.GrabRounding      = 6.0f;
            style.TabRounding       = 8.0f;

            style.WindowBorderSize  = 1.5f;
            style.FrameBorderSize   = 1.0f;
            style.PopupBorderSize   = 1.5f;

            style.WindowPadding     = ImVec2(24.0f, 24.0f);
            style.FramePadding      = ImVec2(14.0f, 10.0f);
            style.ItemSpacing       = ImVec2(12.0f, 12.0f);
            style.ItemInnerSpacing  = ImVec2(8.0f, 6.0f);

            ImVec4* colors = style.Colors;
            colors[ImGuiCol_Text]                  = ImVec4(0.95f, 0.96f, 0.98f, 1.00f);
            colors[ImGuiCol_TextDisabled]          = ImVec4(0.50f, 0.55f, 0.65f, 1.00f);
            colors[ImGuiCol_WindowBg]              = ImVec4(0.06f, 0.09f, 0.14f, 0.96f);
            colors[ImGuiCol_ChildBg]               = ImVec4(0.08f, 0.12f, 0.18f, 0.60f);
            colors[ImGuiCol_PopupBg]               = ImVec4(0.06f, 0.09f, 0.14f, 0.96f);
            colors[ImGuiCol_Border]                = ImVec4(0.20f, 0.38f, 0.60f, 0.65f);
            colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.40f);
            colors[ImGuiCol_FrameBg]               = ImVec4(0.10f, 0.16f, 0.25f, 0.70f);
            colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.14f, 0.24f, 0.38f, 0.85f);
            colors[ImGuiCol_FrameBgActive]         = ImVec4(0.18f, 0.30f, 0.48f, 1.00f);
            colors[ImGuiCol_TitleBg]               = ImVec4(0.06f, 0.09f, 0.14f, 1.00f);
            colors[ImGuiCol_TitleBgActive]         = ImVec4(0.08f, 0.13f, 0.20f, 1.00f);
            colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(0.04f, 0.06f, 0.10f, 0.75f);
            colors[ImGuiCol_Button]                = ImVec4(0.12f, 0.26f, 0.44f, 0.85f);
            colors[ImGuiCol_ButtonHovered]         = ImVec4(0.18f, 0.38f, 0.64f, 1.00f);
            colors[ImGuiCol_ButtonActive]          = ImVec4(0.10f, 0.48f, 0.80f, 1.00f);
            colors[ImGuiCol_Header]                = ImVec4(0.12f, 0.26f, 0.44f, 0.70f);
            colors[ImGuiCol_HeaderHovered]         = ImVec4(0.18f, 0.38f, 0.64f, 0.85f);
            colors[ImGuiCol_HeaderActive]          = ImVec4(0.22f, 0.44f, 0.72f, 1.00f);
            colors[ImGuiCol_Separator]             = ImVec4(0.18f, 0.32f, 0.50f, 0.40f);
            colors[ImGuiCol_SeparatorHovered]      = ImVec4(0.24f, 0.44f, 0.68f, 0.80f);
            colors[ImGuiCol_SeparatorActive]       = ImVec4(0.30f, 0.55f, 0.85f, 1.00f);
            colors[ImGuiCol_CheckMark]             = ImVec4(0.20f, 0.90f, 0.50f, 1.00f);
        }

        void DrawKeycapBadge(ImDrawList* dl, ImVec2 pos, ImVec2 size, const char* label, bool active)
        {
            ImVec2 p0 = pos;
            ImVec2 p1 = ImVec2(pos.x + size.x, pos.y + size.y);

            // Drop shadow
            dl->AddRectFilled(ImVec2(p0.x + 1, p0.y + 2), ImVec2(p1.x + 1, p1.y + 2), IM_COL32(0, 0, 0, 110), 5.0f);

            // Keycap body
            ImU32 fillCol = active ? IM_COL32(0, 190, 100, 255) : IM_COL32(22, 32, 46, 255);
            dl->AddRectFilled(p0, p1, fillCol, 5.0f);

            // Bevel highlight
            dl->AddLine(ImVec2(p0.x + 2, p0.y + 1), ImVec2(p1.x - 2, p0.y + 1),
                        active ? IM_COL32(180, 255, 220, 220) : IM_COL32(65, 90, 125, 200), 1.0f);

            // Border
            ImU32 borderCol = active ? IM_COL32(0, 255, 140, 255) : IM_COL32(45, 68, 98, 200);
            dl->AddRect(p0, p1, borderCol, 5.0f, 0, 1.0f);

            // Text
            ImVec2 textSize = ImGui::CalcTextSize(label);
            ImVec2 textPos(p0.x + (size.x - textSize.x) * 0.5f, p0.y + (size.y - textSize.y) * 0.5f);
            ImU32 textCol = active ? IM_COL32(10, 24, 14, 255) : IM_COL32(220, 232, 248, 255);
            dl->AddText(textPos, textCol, label);
        }
    }

    void BananaGame::RenderUI()
    {
        // Apply modern dark glass game styling
        ApplyGameTheme();

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

        // Draw In-World Floating Score Badges with glowing pill backgrounds
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
                    if (fade <= 0.0f) continue;

                    ImVec2 textSize = ImGui::CalcTextSize(popup.text.c_str());
                    ImVec2 pillMin(screenX - textSize.x * 0.5f - 14.0f, screenY - textSize.y * 0.5f - 6.0f);
                    ImVec2 pillMax(screenX + textSize.x * 0.5f + 14.0f, screenY + textSize.y * 0.5f + 6.0f);

                    // Soft drop shadow
                    drawList->AddRectFilled(ImVec2(pillMin.x + 2, pillMin.y + 3), ImVec2(pillMax.x + 2, pillMax.y + 3),
                        IM_COL32(0, 0, 0, static_cast<int>(fade * 140.0f)), 8.0f);
                    // Dark glass pill background
                    drawList->AddRectFilled(pillMin, pillMax,
                        IM_COL32(10, 16, 26, static_cast<int>(fade * 230.0f)), 8.0f);
                    // Glowing border
                    ImU32 borderCol = ImColor(popup.color.r, popup.color.g, popup.color.b, fade * 0.9f);
                    drawList->AddRect(pillMin, pillMax, borderCol, 8.0f, 0, 1.5f);
                    // Badge Text
                    ImU32 textColor = ImColor(popup.color.r, popup.color.g, popup.color.b, fade);
                    drawList->AddText(ImVec2(screenX - textSize.x * 0.5f, screenY - textSize.y * 0.5f), textColor, popup.text.c_str());
                }
            }
        }
    }

    void BananaGame::DrawStartScreenUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        float modalW = 780.0f;
        float modalH = 590.0f;
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(modalW, modalH), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
        ImGui::Begin("##StartScreenModal", nullptr, flags);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 winPos = ImGui::GetWindowPos();

        // Top Glowing Cyan Accent Line
        dl->AddLine(ImVec2(winPos.x + 24, winPos.y + 1), ImVec2(winPos.x + modalW - 24, winPos.y + 1),
            IM_COL32(0, 220, 255, 230), 2.5f);

        // Header Title
        ImGui::SetCursorPos(ImVec2(24.0f, 16.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.86f, 0.18f, 1.0f));
        ImGui::SetWindowFontScale(2.0f);
        ImGui::Text("FRC BANANA SIMULATOR");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::SetCursorPos(ImVec2(24.0f, 48.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.30f, 0.85f, 1.0f, 1.0f));
        ImGui::SetWindowFontScale(1.1f);
        ImGui::Text("ROBOT COMP 2  *  COMPETITION ARCADE SHOOTOUT");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        float curY = 74.0f;
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        ImGui::Separator();
        curY += 10.0f;

        // High Score Trophy Banner (if established)
        if (m_highScore > 0)
        {
            ImVec2 b0(winPos.x + 24.0f, winPos.y + curY);
            ImVec2 b1(winPos.x + modalW - 24.0f, winPos.y + curY + 34.0f);
            dl->AddRectFilled(b0, b1, IM_COL32(36, 30, 10, 220), 8.0f);
            dl->AddRect(b0, b1, IM_COL32(255, 205, 40, 180), 8.0f, 0, 1.5f);
            dl->AddText(ImVec2(b0.x + 16, b0.y + 8), IM_COL32(255, 220, 50, 255), "ALL-TIME HIGH SCORE RECORD:");
            char hsStr[32]; snprintf(hsStr, sizeof(hsStr), "%d PTS", m_highScore);
            dl->AddText(ImVec2(b1.x - 110, b0.y + 8), IM_COL32(255, 240, 90, 255), hsStr);
            curY += 42.0f;
        }

        // Modern Controls Grid (2 Sleek Cards)
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        ImGui::TextColored(ImVec4(0.35f, 0.90f, 1.0f, 1.0f), "COMPETITION PILOT CONTROLS:");
        curY += 22.0f;

        float colW = (modalW - 48.0f - 14.0f) * 0.5f;
        float cardH = 168.0f;
        ImVec2 c0(winPos.x + 24.0f, winPos.y + curY);

        // Left Controls Card
        dl->AddRectFilled(c0, ImVec2(c0.x + colW, c0.y + cardH), IM_COL32(12, 18, 28, 220), 8.0f);
        dl->AddRect(c0, ImVec2(c0.x + colW, c0.y + cardH), IM_COL32(40, 70, 110, 160), 8.0f);

        DrawKeycapBadge(dl, ImVec2(c0.x + 12, c0.y + 10), ImVec2(96, 24), "W / A / S / D", false);
        dl->AddText(ImVec2(c0.x + 116, c0.y + 14), IM_COL32(220, 230, 245, 255), "Field-Centric Swerve");

        DrawKeycapBadge(dl, ImVec2(c0.x + 12, c0.y + 40), ImVec2(96, 24), "Q  /  E", false);
        dl->AddText(ImVec2(c0.x + 116, c0.y + 44), IM_COL32(220, 230, 245, 255), "Rotate Robot Heading");

        DrawKeycapBadge(dl, ImVec2(c0.x + 12, c0.y + 70), ImVec2(96, 24), "SPACE", false);
        dl->AddText(ImVec2(c0.x + 116, c0.y + 74), IM_COL32(220, 230, 245, 255), "Intake Rollers + Basket");

        DrawKeycapBadge(dl, ImVec2(c0.x + 12, c0.y + 100), ImVec2(96, 24), "F  /  RMB", false);
        dl->AddText(ImVec2(c0.x + 116, c0.y + 104), IM_COL32(220, 230, 245, 255), "Feeder Funnel to Shooter");

        DrawKeycapBadge(dl, ImVec2(c0.x + 12, c0.y + 130), ImVec2(96, 24), "LMB  /  C", false);
        dl->AddText(ImVec2(c0.x + 116, c0.y + 134), IM_COL32(220, 230, 245, 255), "Flywheel Shooter (Fire)");

        // Right Controls Card
        ImVec2 c1(winPos.x + 24.0f + colW + 14.0f, winPos.y + curY);
        dl->AddRectFilled(c1, ImVec2(c1.x + colW, c1.y + cardH), IM_COL32(12, 18, 28, 220), 8.0f);
        dl->AddRect(c1, ImVec2(c1.x + colW, c1.y + cardH), IM_COL32(40, 70, 110, 160), 8.0f);

        DrawKeycapBadge(dl, ImVec2(c1.x + 12, c1.y + 10), ImVec2(80, 24), "I", false);
        dl->AddText(ImVec2(c1.x + 100, c1.y + 14), IM_COL32(220, 230, 245, 255), "Deploy / Stow Intake Pivot");

        DrawKeycapBadge(dl, ImVec2(c1.x + 12, c1.y + 40), ImVec2(80, 24), "Z", false);
        dl->AddText(ImVec2(c1.x + 100, c1.y + 44), IM_COL32(220, 230, 245, 255), "Zero Gyroscope Heading");

        DrawKeycapBadge(dl, ImVec2(c1.x + 12, c1.y + 70), ImVec2(80, 24), "R", false);
        dl->AddText(ImVec2(c1.x + 100, c1.y + 74), IM_COL32(220, 230, 245, 255), "Reset Arena (Practice only)");

        DrawKeycapBadge(dl, ImVec2(c1.x + 12, c1.y + 100), ImVec2(80, 24), "ESC / P", false);
        dl->AddText(ImVec2(c1.x + 100, c1.y + 104), IM_COL32(220, 230, 245, 255), "Pause Match");

        DrawKeycapBadge(dl, ImVec2(c1.x + 12, c1.y + 130), ImVec2(80, 24), "INFO", false);
        dl->AddText(ImVec2(c1.x + 100, c1.y + 134), IM_COL32(160, 220, 180, 255), "Intake MUST be down to vacuum!");

        curY += cardH + 12.0f;

        // Pro-Tip Pill
        ImVec2 p0(winPos.x + 24.0f, winPos.y + curY);
        ImVec2 p1(winPos.x + modalW - 24.0f, winPos.y + curY + 34.0f);
        dl->AddRectFilled(p0, p1, IM_COL32(10, 32, 26, 220), 6.0f);
        dl->AddRect(p0, p1, IM_COL32(0, 200, 130, 180), 6.0f, 0, 1.2f);
        dl->AddText(ImVec2(p0.x + 14, p0.y + 8), IM_COL32(0, 255, 160, 255),
            "PRO-TIP: Press [SPACE] + [F] + [LMB] together to continuously vacuum, feed & stream balls!");
        curY += 44.0f;

        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        ImGui::Separator();
        curY += 10.0f;

        // Primary Game Mode Action Buttons
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.40f, 0.90f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.00f, 0.85f, 0.50f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.00f, 0.50f, 0.30f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        ImGui::SetWindowFontScale(1.25f);
        if (ImGui::Button("START 60s ARCADE CHALLENGE", ImVec2(modalW - 48.0f, 48)))
        {
            StartGame(GameMode::ArcadeRush);
        }
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor(3);

        curY += 56.0f;

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.32f, 0.58f, 0.85f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.44f, 0.74f, 1.00f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.08f, 0.24f, 0.46f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("FREE PRACTICE (Unlimited Time & Manual [R] Reset)", ImVec2(modalW - 48.0f, 36)))
        {
            StartGame(GameMode::FreePractice);
        }
        ImGui::PopStyleColor(3);

        ImGui::End();
    }

    void BananaGame::DrawHUDUI()
    {
        ImGuiIO& io = ImGui::GetIO();

        // Invisible Fullscreen HUD Container
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        ImGuiWindowFlags hudFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                                    ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
                                    ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("##GameHUDOverlay", nullptr, hudFlags);
        ImDrawList* dl = ImGui::GetWindowDrawList();

        // =========================================================================
        // 1. TOP CENTER SCOREBOARD RIBBON (Esports Stadium Banner)
        // =========================================================================
        float sbW = 580.0f;
        float sbH = 68.0f;
        float sbX = (io.DisplaySize.x - sbW) * 0.5f;
        float sbY = 16.0f;

        // Multi-layer drop shadow
        dl->AddRectFilled(ImVec2(sbX - 3, sbY + 4), ImVec2(sbX + sbW + 3, sbY + sbH + 6), IM_COL32(0, 0, 0, 140), 14.0f);

        // Dark glass pill backdrop
        dl->AddRectFilledMultiColor(ImVec2(sbX, sbY), ImVec2(sbX + sbW, sbY + sbH),
            IM_COL32(12, 18, 30, 235), IM_COL32(12, 18, 30, 235),
            IM_COL32(8, 12, 20, 245), IM_COL32(8, 12, 20, 245));

        // Glowing border
        dl->AddRect(ImVec2(sbX, sbY), ImVec2(sbX + sbW, sbY + sbH), IM_COL32(45, 95, 165, 180), 12.0f, 0, 1.5f);

        // Top neon cyan accent line
        dl->AddLine(ImVec2(sbX + 24, sbY + 1), ImVec2(sbX + sbW - 24, sbY + 1), IM_COL32(0, 220, 255, 230), 2.0f);

        // --- Left Column: Match Timer ---
        float col1Center = sbX + 87.5f;
        ImVec2 tTitleSz = ImGui::CalcTextSize("MATCH TIME");
        dl->AddText(ImVec2(col1Center - tTitleSz.x * 0.5f, sbY + 10), IM_COL32(110, 165, 225, 220), "MATCH TIME");
        if (m_mode == GameMode::ArcadeRush)
        {
            int sec = static_cast<int>(m_sessionTimer);
            char timeStr[16];
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d", sec / 60, sec % 60);

            ImU32 timeCol = (sec <= 10)
                ? ((static_cast<int>(m_sessionTimer * 4.0f) % 2 == 0) ? IM_COL32(255, 60, 60, 255) : IM_COL32(255, 180, 60, 255))
                : IM_COL32(255, 255, 255, 255);

            ImVec2 tValSz = ImGui::CalcTextSize(timeStr);
            float scaledTW = tValSz.x * 1.85f;
            ImGui::SetCursorPos(ImVec2(col1Center - scaledTW * 0.5f, sbY + 24));
            ImGui::SetWindowFontScale(1.85f);
            ImGui::TextColored(ImColor(timeCol), "%s", timeStr);
            ImGui::SetWindowFontScale(1.0f);
        }
        else
        {
            ImVec2 fpSz = ImGui::CalcTextSize("FREE PLAY");
            float scaledFPW = fpSz.x * 1.5f;
            ImGui::SetCursorPos(ImVec2(col1Center - scaledFPW * 0.5f, sbY + 24));
            ImGui::SetWindowFontScale(1.5f);
            ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "FREE PLAY");
            ImGui::SetWindowFontScale(1.0f);
        }

        // Vertical Divider 1
        dl->AddLine(ImVec2(sbX + 175, sbY + 12), ImVec2(sbX + 175, sbY + sbH - 12), IM_COL32(50, 75, 110, 140), 1.0f);

        // --- Center Column: Score ---
        float col2Center = sbX + 290.0f;
        ImVec2 sTitleSz = ImGui::CalcTextSize("MATCH SCORE");
        dl->AddText(ImVec2(col2Center - sTitleSz.x * 0.5f, sbY + 10), IM_COL32(255, 195, 70, 220), "MATCH SCORE");
        char scoreBuf[32];
        snprintf(scoreBuf, sizeof(scoreBuf), "%d PTS", m_score);
        ImVec2 sValSz = ImGui::CalcTextSize(scoreBuf);
        float scaledSW = sValSz.x * 1.9f;
        ImGui::SetCursorPos(ImVec2(col2Center - scaledSW * 0.5f, sbY + 24));
        ImGui::SetWindowFontScale(1.9f);
        ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.15f, 1.0f), "%s", scoreBuf);
        ImGui::SetWindowFontScale(1.0f);

        // Vertical Divider 2
        dl->AddLine(ImVec2(sbX + 405, sbY + 12), ImVec2(sbX + 405, sbY + sbH - 12), IM_COL32(50, 75, 110, 140), 1.0f);

        // --- Right Column: Combo Streak or Session Best ---
        float col3Center = sbX + 492.5f;
        if (m_comboStreak > 1)
        {
            // Blazing flame streak badge
            ImVec2 kb0(sbX + 417.0f, sbY + 14);
            ImVec2 kb1(sbX + sbW - 16.0f, sbY + sbH - 14);
            dl->AddRectFilled(kb0, kb1, IM_COL32(220, 60, 20, 225), 8.0f);
            dl->AddRect(kb0, kb1, IM_COL32(255, 205, 30, 255), 8.0f, 0, 1.5f);

            char comboStr[32];
            snprintf(comboStr, sizeof(comboStr), "x%d STREAK!", m_comboStreak);
            ImVec2 cSize = ImGui::CalcTextSize(comboStr);
            dl->AddText(ImVec2(kb0.x + (kb1.x - kb0.x - cSize.x) * 0.5f, kb0.y + 10), IM_COL32(255, 255, 255, 255), comboStr);
        }
        else
        {
            ImVec2 bTitleSz = ImGui::CalcTextSize("SESSION BEST");
            dl->AddText(ImVec2(col3Center - bTitleSz.x * 0.5f, sbY + 10), IM_COL32(140, 160, 190, 220), "SESSION BEST");
            char hsBuf[32];
            snprintf(hsBuf, sizeof(hsBuf), "%d PTS", m_highScore);
            ImVec2 bValSz = ImGui::CalcTextSize(hsBuf);
            float scaledBW = bValSz.x * 1.35f;
            ImGui::SetCursorPos(ImVec2(col3Center - scaledBW * 0.5f, sbY + 26));
            ImGui::SetWindowFontScale(1.35f);
            ImGui::TextColored(ImVec4(0.85f, 0.88f, 0.92f, 1.0f), "%s", hsBuf);
            ImGui::SetWindowFontScale(1.0f);
        }

        // =========================================================================
        // 2. TOP RIGHT: TARGET AIM ASSIST & DISTANCE HUD
        // =========================================================================
        bool isAimedAtGoal = false;
        float distToGoal = 0.0f;
        auto* chassisT = m_scene.GetComponent<TransformComponent>(m_robot.chassisEntity);
        if (chassisT)
        {
            glm::vec3 toGoal = m_goalCenter - chassisT->position;
            distToGoal = std::hypot(toGoal.x, toGoal.z);
            glm::vec3 robotFwd = chassisT->rotation * glm::vec3(0.0f, 0.0f, 1.0f);
            glm::vec2 fwd2D = glm::normalize(glm::vec2(robotFwd.x, robotFwd.z));
            glm::vec2 goal2D = glm::normalize(glm::vec2(toGoal.x, toGoal.z));
            float dot = glm::dot(fwd2D, goal2D);
            isAimedAtGoal = (dot > 0.94f); // Within ~20 degrees of target goal
        }

        float tw = 215.0f;
        float th = 60.0f;
        float tx = io.DisplaySize.x - tw - 20.0f;
        float ty = 16.0f;

        dl->AddRectFilled(ImVec2(tx, ty), ImVec2(tx + tw, ty + th), IM_COL32(10, 16, 26, 215), 10.0f);
        dl->AddRect(ImVec2(tx, ty), ImVec2(tx + tw, ty + th),
            isAimedAtGoal ? IM_COL32(0, 255, 140, 220) : IM_COL32(40, 80, 140, 150), 10.0f, 0, 1.2f);

        char distBuf[32];
        snprintf(distBuf, sizeof(distBuf), "TARGET GOAL: %.1f m", distToGoal);
        dl->AddText(ImVec2(tx + 12, ty + 10), IM_COL32(120, 190, 255, 230), distBuf);

        if (isAimedAtGoal)
        {
            dl->AddCircleFilled(ImVec2(tx + 18, ty + 38), 5.0f, IM_COL32(0, 255, 130, 255));
            dl->AddText(ImVec2(tx + 28, ty + 31), IM_COL32(0, 255, 130, 255), "LOCKED ON TARGET");
        }
        else
        {
            dl->AddCircle(ImVec2(tx + 18, ty + 38), 5.0f, IM_COL32(140, 160, 190, 200), 16, 1.2f);
            dl->AddText(ImVec2(tx + 28, ty + 31), IM_COL32(140, 160, 190, 220), "YAW TO AIM (Q / E)");
        }

        // =========================================================================
        // 3. BOTTOM COCKPIT ACTION DOCK (Ball Inventory & 3 Ability Cards)
        // =========================================================================
        float dkW = 790.0f;
        float dkH = 114.0f;
        float dkX = (io.DisplaySize.x - dkW) * 0.5f;
        float dkY = io.DisplaySize.y - dkH - 16.0f;

        // Drop shadow
        dl->AddRectFilled(ImVec2(dkX - 4, dkY + 4), ImVec2(dkX + dkW + 4, dkY + dkH + 6), IM_COL32(0, 0, 0, 150), 14.0f);

        // Frosted glass chassis dock
        dl->AddRectFilledMultiColor(ImVec2(dkX, dkY), ImVec2(dkX + dkW, dkY + dkH),
            IM_COL32(10, 16, 26, 240), IM_COL32(10, 16, 26, 240),
            IM_COL32(6, 10, 18, 250), IM_COL32(6, 10, 18, 250));

        // Border & top neon cyan accent
        dl->AddRect(ImVec2(dkX, dkY), ImVec2(dkX + dkW, dkY + dkH), IM_COL32(45, 95, 165, 170), 14.0f, 0, 1.5f);
        dl->AddLine(ImVec2(dkX + 30, dkY + 1), ImVec2(dkX + dkW - 30, dkY + 1), IM_COL32(0, 210, 255, 180), 2.0f);

        // Count held balls by slot
        int cntIntake = 0;
        int cntBasket = 0;
        int cntFunnel = 0;
        for (const auto& [e, info] : m_heldBalls)
        {
            if (!m_scene.IsEntityValid(e)) continue;
            switch (info.slot)
            {
            case BallSlot::IntakeTray:  cntIntake++; break;
            case BallSlot::Basket:      cntBasket++; break;
            case BallSlot::Funnel:
            case BallSlot::MuzzleExit:  cntFunnel++; break;
            default: break;
            }
        }
        int totalHeld = cntIntake + cntBasket + cntFunnel;

        // --- Zone A: Ball Magazine & Inventory (Left 225px) ---
        float zX = dkX + 16.0f;
        float zY = dkY + 10.0f;

        char magTitle[32];
        snprintf(magTitle, sizeof(magTitle), "BALL STORAGE:  %d / 22", totalHeld);
        dl->AddText(ImVec2(zX, zY), IM_COL32(255, 205, 50, 255), magTitle);

        // Magazine Capacity Progress Bar
        float barW = 195.0f;
        float barH = 7.0f;
        float fillFrac = std::clamp(static_cast<float>(totalHeld) / 22.0f, 0.0f, 1.0f);
        dl->AddRectFilled(ImVec2(zX, zY + 18), ImVec2(zX + barW, zY + 18 + barH), IM_COL32(20, 30, 45, 255), 3.5f);
        ImU32 barCol = (totalHeld >= 22)
            ? IM_COL32(255, 70, 40, 255)
            : ((totalHeld >= 16) ? IM_COL32(255, 180, 30, 255) : IM_COL32(0, 220, 120, 255));
        dl->AddRectFilled(ImVec2(zX, zY + 18), ImVec2(zX + barW * fillFrac, zY + 18 + barH), barCol, 3.5f);

        // Visual Slot Breakdown
        dl->AddText(ImVec2(zX, zY + 30), IM_COL32(140, 160, 190, 220), "INTAKE TRAY [3]:");
        for (int p = 0; p < 3; ++p)
        {
            float px = zX + 120.0f + p * 16.0f;
            float py = zY + 37.0f;
            if (p < cntIntake)
                dl->AddCircleFilled(ImVec2(px, py), 4.5f, IM_COL32(0, 255, 130, 255));
            else
                dl->AddCircle(ImVec2(px, py), 4.5f, IM_COL32(40, 65, 95, 200), 12, 1.2f);
        }

        char bskBuf[32];
        snprintf(bskBuf, sizeof(bskBuf), "BASKET HOPPER: %d / 18", cntBasket);
        dl->AddText(ImVec2(zX, zY + 48), IM_COL32(140, 160, 190, 220), bskBuf);

        // Intake Pivot Deployment Indicator
        bool isPivotDown = (m_currentPivotAngle <= -1.40f);
        ImVec2 pv0(zX, zY + 68);
        ImVec2 pv1(zX + barW, zY + 94);
        if (isPivotDown)
        {
            dl->AddRectFilled(pv0, pv1, IM_COL32(12, 38, 24, 220), 5.0f);
            dl->AddRect(pv0, pv1, IM_COL32(0, 220, 110, 220), 5.0f, 0, 1.2f);
            dl->AddText(ImVec2(pv0.x + 8, pv0.y + 5), IM_COL32(0, 255, 140, 255), "PIVOT: DEPLOYED (DOWN)");
        }
        else
        {
            dl->AddRectFilled(pv0, pv1, IM_COL32(24, 28, 38, 220), 5.0f);
            dl->AddRect(pv0, pv1, IM_COL32(60, 75, 95, 180), 5.0f, 0, 1.0f);
            dl->AddText(ImVec2(pv0.x + 8, pv0.y + 5), IM_COL32(160, 175, 195, 220), "PIVOT: STOWED (UP) [I]");
        }

        // Vertical Divider
        dl->AddLine(ImVec2(dkX + 235, dkY + 12), ImVec2(dkX + 235, dkY + dkH - 12), IM_COL32(40, 65, 95, 140), 1.0f);

        // --- Zone B: 3 Ability Action Cards (Center/Right) ---
        float tileY = dkY + 12.0f;
        float tileH = 90.0f;

        // Card 1: INTAKE & BASKET
        ImVec2 a0(dkX + 250, tileY);
        ImVec2 a1(a0.x + 160, tileY + tileH);
        dl->AddRectFilled(a0, a1, m_btnIntake ? IM_COL32(14, 38, 26, 230) : IM_COL32(14, 20, 32, 220), 8.0f);
        dl->AddRect(a0, a1, m_btnIntake ? IM_COL32(0, 230, 120, 240) : IM_COL32(40, 65, 95, 180), 8.0f, 0, 1.2f);
        DrawKeycapBadge(dl, ImVec2(a0.x + 10, a0.y + 8), ImVec2(75, 22), "SPACE", m_btnIntake);
        dl->AddText(ImVec2(a0.x + 10, a0.y + 36), IM_COL32(230, 240, 255, 255), "INTAKE ROLLERS");
        dl->AddText(ImVec2(a0.x + 10, a0.y + 52), IM_COL32(120, 150, 185, 220), "Feed into Basket");
        if (m_btnIntake)
        {
            dl->AddCircleFilled(ImVec2(a0.x + 16, a0.y + 75), 4.5f, IM_COL32(0, 255, 130, 255));
            dl->AddText(ImVec2(a0.x + 26, a0.y + 68), IM_COL32(0, 255, 130, 255), "RUNNING");
        }
        else
        {
            dl->AddCircle(ImVec2(a0.x + 16, a0.y + 75), 4.5f, IM_COL32(100, 120, 145, 200), 12, 1.0f);
            dl->AddText(ImVec2(a0.x + 26, a0.y + 68), IM_COL32(100, 120, 145, 200), "READY");
        }

        // Card 2: FEEDER FUNNEL
        ImVec2 b0(dkX + 424, tileY);
        ImVec2 b1(b0.x + 160, tileY + tileH);
        dl->AddRectFilled(b0, b1, m_btnFunnel ? IM_COL32(14, 38, 26, 230) : IM_COL32(14, 20, 32, 220), 8.0f);
        dl->AddRect(b0, b1, m_btnFunnel ? IM_COL32(0, 230, 120, 240) : IM_COL32(40, 65, 95, 180), 8.0f, 0, 1.2f);
        DrawKeycapBadge(dl, ImVec2(b0.x + 10, b0.y + 8), ImVec2(80, 22), "F / RMB", m_btnFunnel);
        dl->AddText(ImVec2(b0.x + 10, b0.y + 36), IM_COL32(230, 240, 255, 255), "FEEDER FUNNEL");
        dl->AddText(ImVec2(b0.x + 10, b0.y + 52), IM_COL32(120, 150, 185, 220), "Lift to Flywheel");
        if (m_btnFunnel)
        {
            dl->AddCircleFilled(ImVec2(b0.x + 16, b0.y + 75), 4.5f, IM_COL32(0, 255, 130, 255));
            dl->AddText(ImVec2(b0.x + 26, b0.y + 68), IM_COL32(0, 255, 130, 255), "FEEDING");
        }
        else
        {
            dl->AddCircle(ImVec2(b0.x + 16, b0.y + 75), 4.5f, IM_COL32(100, 120, 145, 200), 12, 1.0f);
            dl->AddText(ImVec2(b0.x + 26, b0.y + 68), IM_COL32(100, 120, 145, 200), "READY");
        }

        // Card 3: FLYWHEEL SHOOTER
        ImVec2 d0(dkX + 598, tileY);
        ImVec2 d1(d0.x + 178, tileY + tileH);
        float rpm = m_flywheel ? static_cast<float>(m_flywheel->angularVelocityRpm) : 0.0f;
        dl->AddRectFilled(d0, d1, m_btnFlywheel ? IM_COL32(38, 28, 12, 230) : IM_COL32(14, 20, 32, 220), 8.0f);
        dl->AddRect(d0, d1, m_btnFlywheel ? IM_COL32(255, 195, 30, 240) : IM_COL32(40, 65, 95, 180), 8.0f, 0, 1.2f);
        DrawKeycapBadge(dl, ImVec2(d0.x + 10, d0.y + 8), ImVec2(75, 22), "LMB / C", m_btnFlywheel);

        char rpmBuf[32];
        snprintf(rpmBuf, sizeof(rpmBuf), "%.0f RPM", rpm);
        dl->AddText(ImVec2(d0.x + 95, d0.y + 11), m_btnFlywheel ? IM_COL32(255, 215, 40, 255) : IM_COL32(140, 160, 190, 220), rpmBuf);

        dl->AddText(ImVec2(d0.x + 10, d0.y + 36), IM_COL32(230, 240, 255, 255), "SHOOTER FLYWHEEL");

        // RPM Tachometer Power Gauge
        float rpmFrac = std::clamp(rpm / 5500.0f, 0.0f, 1.0f);
        float gaugeW = 158.0f;
        dl->AddRectFilled(ImVec2(d0.x + 10, d0.y + 54), ImVec2(d0.x + 10 + gaugeW, d0.y + 60), IM_COL32(20, 30, 45, 255), 3.0f);
        ImU32 rpmCol = (rpmFrac > 0.8f) ? IM_COL32(255, 205, 30, 255) : IM_COL32(0, 220, 255, 255);
        dl->AddRectFilled(ImVec2(d0.x + 10, d0.y + 54), ImVec2(d0.x + 10 + gaugeW * rpmFrac, d0.y + 60), rpmCol, 3.0f);

        if (m_btnFlywheel)
        {
            dl->AddCircleFilled(ImVec2(d0.x + 16, d0.y + 75), 4.5f, IM_COL32(255, 200, 30, 255));
            dl->AddText(ImVec2(d0.x + 26, d0.y + 68), IM_COL32(255, 200, 30, 255), "FIRING");
        }
        else
        {
            dl->AddCircle(ImVec2(d0.x + 16, d0.y + 75), 4.5f, IM_COL32(100, 120, 145, 200), 12, 1.0f);
            dl->AddText(ImVec2(d0.x + 26, d0.y + 68), IM_COL32(100, 120, 145, 200), "READY");
        }

        // =========================================================================
        // 4. SCREEN-CENTERED COUNTDOWN (3.. 2.. 1.. GO!)
        // =========================================================================
        if (m_state == GameState::Countdown)
        {
            float cX = io.DisplaySize.x * 0.5f;
            float cY = io.DisplaySize.y * 0.42f;

            // Halo backing disc
            dl->AddCircleFilled(ImVec2(cX, cY), 85.0f, IM_COL32(8, 14, 24, 235), 48);
            dl->AddCircle(ImVec2(cX, cY), 85.0f, IM_COL32(0, 210, 255, 220), 48, 2.5f);

            // Pulsing shockwave ring
            float pulse = fmod(m_countdownTimer, 1.0f);
            dl->AddCircle(ImVec2(cX, cY), 85.0f + (1.0f - pulse) * 35.0f,
                IM_COL32(0, 220, 255, static_cast<int>(pulse * 180.0f)), 48, 2.0f);

            int count = static_cast<int>(std::ceil(m_countdownTimer));
            if (count > 0)
            {
                char cStr[8]; snprintf(cStr, sizeof(cStr), "%d", count);
                ImVec2 sz = ImGui::CalcTextSize(cStr);
                float scale = 4.5f;
                float scaledW = sz.x * scale;
                float scaledH = sz.y * scale;
                ImGui::SetCursorPos(ImVec2(cX - scaledW * 0.5f, cY - scaledH * 0.5f));
                ImGui::SetWindowFontScale(scale);
                ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.15f, 1.0f), "%s", cStr);
                ImGui::SetWindowFontScale(1.0f);
            }
            else
            {
                const char* goStr = "GO!";
                ImVec2 sz = ImGui::CalcTextSize(goStr);
                float scale = 4.5f;
                float scaledW = sz.x * scale;
                float scaledH = sz.y * scale;
                ImGui::SetCursorPos(ImVec2(cX - scaledW * 0.5f, cY - scaledH * 0.5f));
                ImGui::SetWindowFontScale(scale);
                ImGui::TextColored(ImVec4(0.1f, 1.0f, 0.4f, 1.0f), "%s", goStr);
                ImGui::SetWindowFontScale(1.0f);
            }

            ImVec2 subSize = ImGui::CalcTextSize("GET READY TO SHOOT!");
            dl->AddText(ImVec2(cX - subSize.x * 0.5f, cY + 98.0f), IM_COL32(0, 220, 255, 240), "GET READY TO SHOOT!");
        }

        ImGui::End();
    }

    void BananaGame::DrawPauseUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        float pW = 380.0f;
        float pH = 275.0f;
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(pW, pH), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
        ImGui::Begin("##PauseModal", nullptr, flags);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 winPos = ImGui::GetWindowPos();

        // Top Amber Stripe
        dl->AddLine(ImVec2(winPos.x + 20, winPos.y + 1), ImVec2(winPos.x + pW - 20, winPos.y + 1), IM_COL32(255, 185, 30, 240), 2.5f);

        ImGui::SetCursorPos(ImVec2(24.0f, 16.0f));
        ImGui::SetWindowFontScale(1.6f);
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "// MATCH PAUSED");
        ImGui::SetWindowFontScale(1.0f);

        ImGui::SetCursorPos(ImVec2(24.0f, 48.0f));
        ImGui::TextDisabled("Current Score: %d PTS", m_score);

        ImGui::SetCursorPos(ImVec2(24.0f, 72.0f));
        ImGui::Separator();

        float curY = 88.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.40f, 0.90f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.00f, 0.85f, 0.50f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("RESUME MATCH [ESC]", ImVec2(pW - 48.0f, 40)))
        {
            m_state = GameState::Playing;
        }
        ImGui::PopStyleColor(2);
        curY += 50.0f;

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.32f, 0.58f, 0.85f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.44f, 0.74f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("RESTART MATCH", ImVec2(pW - 48.0f, 38)))
        {
            StartGame(m_mode);
        }
        ImGui::PopStyleColor(2);
        curY += 48.0f;

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.22f, 0.36f, 0.85f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.30f, 0.48f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("RETURN TO TITLE MENU", ImVec2(pW - 48.0f, 34)))
        {
            m_state = GameState::StartScreen;
        }
        ImGui::PopStyleColor(2);

        ImGui::End();
    }

    void BananaGame::DrawGameOverUI()
    {
        ImGuiIO& io = ImGui::GetIO();
        float gW = 560.0f;
        float gH = 460.0f;
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(gW, gH), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
        ImGui::Begin("##GameOverModal", nullptr, flags);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 winPos = ImGui::GetWindowPos();

        // Top Gold Accent Line
        dl->AddLine(ImVec2(winPos.x + 20, winPos.y + 1), ImVec2(winPos.x + gW - 20, winPos.y + 1), IM_COL32(255, 215, 30, 240), 2.5f);

        ImGui::SetCursorPos(ImVec2(24.0f, 16.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.20f, 1.0f));
        ImGui::SetWindowFontScale(1.85f);
        ImGui::Text("MATCH COMPLETE!");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::SetCursorPos(ImVec2(24.0f, 48.0f));
        ImGui::TextDisabled("Competition round recap & statistics:");

        ImGui::SetCursorPos(ImVec2(24.0f, 72.0f));
        ImGui::Separator();

        // 4 Modern Stat Cards in a 2x2 Grid
        float colW = (gW - 48.0f - 12.0f) * 0.5f;
        float cardH = 68.0f;
        float row1Y = 86.0f;
        float row2Y = row1Y + cardH + 10.0f;
        float col1X = 24.0f;
        float col2X = 24.0f + colW + 12.0f;

        // Card 1: Score
        ImVec2 s0(winPos.x + col1X, winPos.y + row1Y);
        dl->AddRectFilled(s0, ImVec2(s0.x + colW, s0.y + cardH), IM_COL32(14, 22, 34, 220), 8.0f);
        dl->AddRect(s0, ImVec2(s0.x + colW, s0.y + cardH), IM_COL32(255, 205, 30, 180), 8.0f, 0, 1.2f);
        dl->AddText(ImVec2(s0.x + 14, s0.y + 10), IM_COL32(255, 205, 50, 255), "FINAL SCORE");
        char scBuf[32]; snprintf(scBuf, sizeof(scBuf), "%d PTS", m_score);
        ImGui::SetCursorPos(ImVec2(col1X + 14.0f, row1Y + 28.0f));
        ImGui::SetWindowFontScale(1.6f);
        ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.15f, 1.0f), "%s", scBuf);
        ImGui::SetWindowFontScale(1.0f);

        // Card 2: Goals Made
        ImVec2 s1(winPos.x + col2X, winPos.y + row1Y);
        dl->AddRectFilled(s1, ImVec2(s1.x + colW, s1.y + cardH), IM_COL32(14, 22, 34, 220), 8.0f);
        dl->AddRect(s1, ImVec2(s1.x + colW, s1.y + cardH), IM_COL32(40, 80, 140, 160), 8.0f);
        dl->AddText(ImVec2(s1.x + 14, s1.y + 10), IM_COL32(120, 180, 255, 240), "GOALS SCORED");
        char glBuf[32]; snprintf(glBuf, sizeof(glBuf), "%d / %d SHOTS", m_goalsScored, m_ballsFired);
        ImGui::SetCursorPos(ImVec2(col2X + 14.0f, row1Y + 30.0f));
        ImGui::SetWindowFontScale(1.35f);
        ImGui::TextColored(ImVec4(0.9f, 0.95f, 1.0f, 1.0f), "%s", glBuf);
        ImGui::SetWindowFontScale(1.0f);

        // Card 3: Accuracy
        ImVec2 s2(winPos.x + col1X, winPos.y + row2Y);
        float acc = (m_ballsFired > 0) ? (static_cast<float>(m_goalsScored) / m_ballsFired * 100.0f) : 0.0f;
        dl->AddRectFilled(s2, ImVec2(s2.x + colW, s2.y + cardH), IM_COL32(14, 22, 34, 220), 8.0f);
        dl->AddRect(s2, ImVec2(s2.x + colW, s2.y + cardH), IM_COL32(40, 80, 140, 160), 8.0f);
        dl->AddText(ImVec2(s2.x + 14, s2.y + 10), IM_COL32(120, 180, 255, 240), "ACCURACY");
        char accBuf[32]; snprintf(accBuf, sizeof(accBuf), "%.1f %%", acc);
        ImGui::SetCursorPos(ImVec2(col1X + 14.0f, row2Y + 30.0f));
        ImGui::SetWindowFontScale(1.35f);
        ImVec4 accCol = (acc >= 75.0f) ? ImVec4(0.2f, 1.0f, 0.4f, 1.0f) : ImVec4(1.0f, 0.85f, 0.2f, 1.0f);
        ImGui::TextColored(accCol, "%s", accBuf);
        ImGui::SetWindowFontScale(1.0f);

        // Card 4: Max Combo
        ImVec2 s3(winPos.x + col2X, winPos.y + row2Y);
        dl->AddRectFilled(s3, ImVec2(s3.x + colW, s3.y + cardH), IM_COL32(14, 22, 34, 220), 8.0f);
        dl->AddRect(s3, ImVec2(s3.x + colW, s3.y + cardH), IM_COL32(40, 80, 140, 160), 8.0f);
        dl->AddText(ImVec2(s3.x + 14, s3.y + 10), IM_COL32(120, 180, 255, 240), "BEST STREAK");
        char cbBuf[32]; snprintf(cbBuf, sizeof(cbBuf), "x%d COMBO", m_maxCombo);
        ImGui::SetCursorPos(ImVec2(col2X + 14.0f, row2Y + 30.0f));
        ImGui::SetWindowFontScale(1.35f);
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "%s", cbBuf);
        ImGui::SetWindowFontScale(1.0f);

        // High Score / Performance Banner (cleanly below Row 2 at row2Y + cardH + 16 = 248px)
        float curY = row2Y + cardH + 16.0f;
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (m_score >= m_highScore && m_score > 0)
        {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "*** NEW HIGH SCORE RECORD ACHIEVED! ***");
        }
        else
        {
            ImGui::TextDisabled("High score to beat: %d PTS", m_highScore);
        }
        curY += 26.0f;

        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        ImGui::Separator();
        curY += 14.0f;

        // Action Buttons
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.40f, 0.90f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.00f, 0.85f, 0.50f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("PLAY AGAIN", ImVec2(gW - 48.0f, 44)))
        {
            StartGame(m_mode);
        }
        ImGui::PopStyleColor(2);
        curY += 54.0f;

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.28f, 0.48f, 0.85f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.38f, 0.62f, 1.00f));
        ImGui::SetCursorPos(ImVec2(24.0f, curY));
        if (ImGui::Button("RETURN TO MAIN MENU", ImVec2(gW - 48.0f, 36)))
        {
            m_state = GameState::StartScreen;
        }
        ImGui::PopStyleColor(2);

        ImGui::End();
    }
}
