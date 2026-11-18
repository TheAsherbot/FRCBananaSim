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
#include <iomanip>
#include <sstream>

using namespace FRCBananaSim::Core::ECS::Core;
using namespace FRCBananaSim::Core::ECS::Components;
using namespace FRCBananaSim::Physics::Components;
using namespace FRCBananaSim::Renderer::Components;
using namespace FRCBananaSim::Control;
using namespace FRCBananaSim::Robot::Drivetrain::Components;
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

        // Destroy chassis entity if valid
        if (m_scene.IsEntityValid(m_robot.chassisEntity))
        {
            if (m_robot.physicsSystem)
            {
                m_robot.physicsSystem->DestroyBody(m_robot.chassisEntity);
            }
            m_scene.DestroyEntity(m_robot.chassisEntity);
            m_robot.chassisEntity = invalidEntity;
        }

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

        SIM_LOG_INFO("[BananaGame] Teardown complete.");
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
            std::string editCandidate = prefix + "Resources/Robots/Comp 2 Robot EDITED/manifest.json";
            std::string editPrefixCandidate = prefix + "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/manifest.json";
            if (std::filesystem::exists(editCandidate))
            {
                manifestPath = editCandidate;
                sidecarPath = prefix + "Resources/Robots/Comp 2 Robot EDITED/sidecar.json";
                materialsPath = prefix + "Resources/Config/materials.json";
                foundRobot = true;
                SIM_LOG_INFO("[BananaGame] Loaded edited robot: " << manifestPath);
                break;
            }
            if (std::filesystem::exists(editPrefixCandidate))
            {
                manifestPath = editPrefixCandidate;
                sidecarPath = prefix + "FRCBananaSim/Resources/Robots/Comp 2 Robot EDITED/sidecar.json";
                materialsPath = prefix + "FRCBananaSim/Resources/Config/materials.json";
                foundRobot = true;
                SIM_LOG_INFO("[BananaGame] Loaded edited robot: " << manifestPath);
                break;
            }
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
        // 1. Clear held balls tracking and intake deployment state
        m_heldBalls.clear();
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
            CheckGoalScoring();
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

    void BananaGame::CheckGoalScoring()
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

        ImGui::BulletText("[1] Spacebar / Shift");
        ImGui::NextColumn();
        ImGui::Text("Intake: Runs Rollers + Basket (Deploys Intake)");
        ImGui::NextColumn();

        ImGui::BulletText("[2] F / RMB / V");
        ImGui::NextColumn();
        ImGui::Text("Funnel: Feeds Basket into Funnel Throat");
        ImGui::NextColumn();

        ImGui::BulletText("[3] Left Mouse / C");
        ImGui::NextColumn();
        ImGui::Text("Flywheel: Spins Shooter Flywheel to Fire");
        ImGui::NextColumn();

        ImGui::BulletText("I");
        ImGui::NextColumn();
        ImGui::Text("Toggle Intake Deploy / Undeploy");
        ImGui::NextColumn();

        ImGui::BulletText("R");
        ImGui::NextColumn();
        ImGui::Text("Reset Field (Free Practice only)");
        ImGui::NextColumn();

        ImGui::BulletText("Z");
        ImGui::NextColumn();
        ImGui::Text("Zero Gyroscope Forward Heading");
        ImGui::NextColumn();

        ImGui::BulletText("ESC / P");
        ImGui::NextColumn();
        ImGui::Text("Pause Game");
        ImGui::NextColumn();

        ImGui::Columns(1);
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.4f, 1.0f),
            "* Pro-tip: Intake must be DOWN to pick up balls! Press all 3 buttons together to vacuum, feed, and stream balls continuously into the goal!");
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
        ImGui::SetNextWindowPos(ImVec2(18, io.DisplaySize.y - 190), ImGuiCond_Always, ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(410, 175), ImGuiCond_Always);
        ImGui::Begin("##RobotHUD", nullptr, hudFlags);

        ImVec2 r0 = ImGui::GetWindowPos();
        ImVec2 r1 = ImVec2(r0.x + 410, r0.y + 175);
        drawList->AddRectFilled(r0, r1, IM_COL32(18, 22, 28, 215), 8.0f);
        drawList->AddRect(r0, r1, IM_COL32(60, 100, 160, 160), 8.0f);

        ImGui::SetCursorPos(ImVec2(12, 8));
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "ROBOT COMP 2 (3-STAGE MECH)");

        // Intake Pivot Status
        bool isPivotDown = (m_currentPivotAngle <= -1.40f);
        float pivotDeg = glm::degrees(m_currentPivotAngle);
        ImGui::SetCursorPos(ImVec2(12, 26));
        if (isPivotDown)
        {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "PIVOT: [ DEPLOYED / DOWN ] (%.0f deg)", pivotDeg);
        }
        else if (m_currentPivotAngle >= -0.25f)
        {
            ImGui::TextColored(ImVec4(0.6f, 0.65f, 0.7f, 1.0f), "PIVOT: [ STOWED / UP ] (Press I / Hold Space)");
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "PIVOT: [ MOVING... ] (%.0f deg)", pivotDeg);
        }

        // Button #1: Intake (Spacebar / Shift)
        ImGui::SetCursorPos(ImVec2(12, 46));
        if (m_btnIntake)
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "[1] INTAKE:   RUNNING + BASKET (SPACE)");
        else
            ImGui::TextColored(ImVec4(0.55f, 0.6f, 0.65f, 1.0f), "[1] INTAKE:   OFF (HOLD SPACE)");

        // Button #2: Funnel (F / RMB)
        ImGui::SetCursorPos(ImVec2(12, 66));
        if (m_btnFunnel)
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "[2] FUNNEL:   FEEDING + BASKET (F/RMB)");
        else
            ImGui::TextColored(ImVec4(0.55f, 0.6f, 0.65f, 1.0f), "[2] FUNNEL:   OFF (HOLD F / RMB)");

        // Button #3: Flywheel (LMB / C)
        float rpm = m_flywheel ? static_cast<float>(m_flywheel->angularVelocityRpm) : 0.0f;
        ImGui::SetCursorPos(ImVec2(12, 86));
        if (m_btnFlywheel)
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "[3] FLYWHEEL: %.0f RPM [FIRING] (LMB)", rpm);
        else
            ImGui::TextColored(ImVec4(0.55f, 0.6f, 0.65f, 1.0f), "[3] FLYWHEEL: %.0f RPM [OFF] (HOLD LMB)", rpm);

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

        // Ball Capacity & Inventory Display (Basket: 3 wide x 3 tall = up to 18, Intake: 3 wide = 3, Funnel = 1)
        ImGui::SetCursorPos(ImVec2(12, 108));
        if (cntIntake >= 3 && cntBasket >= 18)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.15f, 1.0f), "BALLS: [ %d/22 FULL ] Basket 18/18 | Intake 3/3 | Funnel %d/1",
                totalHeld, cntFunnel);
        }
        else if (totalHeld > 0)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.90f, 0.20f, 1.0f), "BALLS: [ %d/22 HELD ] Basket %d/18 | Intake %d/3 | Funnel %d/1",
                totalHeld, cntBasket, cntIntake, cntFunnel);
        }
        else
        {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "BALLS: [ 0/22 EMPTY ] - DEPLOY INTAKE & DRIVE OVER BALLS");
        }

        // Intake Status / Guidance
        ImGui::SetCursorPos(ImVec2(12, 128));
        if (cntIntake >= 3 && cntBasket >= 18)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.50f, 0.20f, 1.0f), "INTAKE FULL: Feed (F) or Fire (LMB) to take more");
        }
        else if (!isPivotDown)
        {
            ImGui::TextColored(ImVec4(0.95f, 0.80f, 0.20f, 1.0f), "INTAKE IS UP: Press 'I' or Hold Space to deploy");
        }
        else
        {
            ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.40f, 1.0f), "INTAKE READY: Rollers vacuuming from carpet");
        }

        // Mode & Reset Indicator
        ImGui::SetCursorPos(ImVec2(12, 148));
        if (m_mode == GameMode::FreePractice)
        {
            ImGui::TextDisabled("MODE: FIELD-CENTRIC | RESET: [R] KEY READY");
        }
        else
        {
            ImGui::TextDisabled("MODE: FIELD-CENTRIC | RESET: LOCKED DURING MATCH");
        }

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
