/**
 * @file PlaygroundScene.cpp
 * @brief Interactive visual sandbox for 2-DOF differential mechanisms and Block 6 shooter squeeze dynamics.
 */

#include "Playground/PlaygroundScene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Renderer/Components/MeshComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Robot/Mechanisms/Components/DifferentialComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Interaction/IntakeSurfaceModel.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/DebugDraw.h"

#include <imgui/imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace FRCBananaSim::Playground
{
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Physics::Components;
    using namespace Robot::Mechanisms::Components;
    using namespace Robot::Mechanisms::Systems;
    using namespace Robot::Sensors::Components;
    using namespace Robot::Sensors::Systems;
    using namespace Interaction;
    using namespace Interaction::Components;

    namespace
    {
        void AttachTransforms(Scene& scene, Entity entity, const glm::vec3& pos, const glm::vec3& scale,
            const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        {
            TransformComponent transform;
            transform.position = pos;
            transform.rotation = rot;
            transform.scale = scale;
            scene.AddComponent(entity, transform);
        }
    }

    PlaygroundScene::PlaygroundScene()
        : activeScene(nullptr),
        activeJolt(nullptr),
        activeCamera(nullptr),
        groundEntity(invalidEntity),
        mountStandEntity(invalidEntity),
        pitchYokeEntity(invalidEntity),
        rollToolEntity(invalidEntity),
        pitchJointEntity(invalidEntity),
        rollJointEntity(invalidEntity),
        diffMechanismEntity(invalidEntity),
        shooterStandEntity(invalidEntity),
        topFlywheelEntity(invalidEntity),
        bottomFlywheelEntity(invalidEntity),
        contactZoneEntity(invalidEntity),
        beamBreakEntity(invalidEntity),
        intakeRollerEntity(invalidEntity),
        intakeZoneEntity(invalidEntity),
        intakeBeamBreakEntity(invalidEntity),
        intakeVoltage(10.0f),
        intakeRollerRadiusM(0.0381f),
        intakeGripNormalForceN(45.0f),
        intakeViscousDampingNsPerM(8.0f),
        intakeMaxGripForceN(120.0f),
        intakeEnabled(true),
        autoFeedToShooter(true),
        lastIntakeTorqueNm(0.0),
        currentIntakeRpm(0.0),
        storedPiecesCount(0),
        hoodLeftCheekEntity(invalidEntity),
        hoodRightCheekEntity(invalidEntity),
        controlMode(PlaygroundControlMode::ManualSliders),
        manualVoltageA(0.0f),
        manualVoltageB(0.0f),
        testVoltageMagnitude(6.0f),
        testFrequencyHz(0.75f),
        testTimeSeconds(0.0),
        brakeModeA(true),
        brakeModeB(true),
        gearRatioPitch(40.0f),
        gearRatioRoll(30.0f),
        transmissionEfficiency(0.95f),
        frictionPitchNm(0.35f),
        frictionRollNm(0.20f),
        commandedFlywheelVoltage(8.0f),
        targetFlywheelRPM(2800.0f), // Realistic FRC shooter default RPM
        useFlywheelClosedLoop(true),
        complianceK(25000.0f),
        restitutionE(0.25f),
        frictionMu(0.68f),
        hoodFrictionMu(0.05f),
        nominalGapM(0.065f),
        uncompressedSizeM(0.130f),
        contactLengthM(0.140f),
        pieceMassKg(0.235f),
        flywheelInertiaKgm2(0.0032f),
        rollerRadiusM(0.0508f),
        isDualFlywheel(false),
        rapidFireEnabled(false),
        rapidFireTimer(0.0),
        isOverhandShooter(false),
        hoodStartAngleDeg(-20.0f),
        hoodEndAngleDeg(50.0f),
        notePitchDeg(35.0f),
        enableSpinPhysics(true),
        ballSpinMultiplier(1.0f),
        groundBiteFriction(0.75f),
        currentBallSpinRpm(0.0),
        lastImpactSpinRpm(0.0),
        lastPreImpactSpeedMPerS(0.0),
        lastPostImpactSpeedMPerS(0.0),
        lastMagnusLiftN(0.0),
        pieceActive(false),
        initialShotRpm(0.0),
        lastExitSpeedMPerS(0.0),
        currentFlightSpeedMPerS(0.0),
        lastRpmSag(0.0),
        peakNormalForceN(0.0),
        peakTractionForceN(0.0),
        currentSqueezeDepthM(0.0),
        currentNormalForceN(0.0),
        currentTractionForceN(0.0),
        shotTransitTimeMs(0.0),
        hasShotFired(false),
        lastShotDistanceM(0.0f),
        lastLandingPos(0.0f, -10.0f, 0.0f),
        hasLandingRecorded(false)
    {
        rpmHistory.fill(0.0f);
        torqueHistory.fill(0.0f);
        forceHistory.fill(0.0f);
    }

    PlaygroundScene::~PlaygroundScene()
    {
        Shutdown();
    }

    void PlaygroundScene::DestroyTrackedEntity(Entity& entity)
    {
        if (entity != invalidEntity && activeScene)
        {
            if (physicsSystem)
            {
                physicsSystem->DestroyBody(entity);
            }
            activeScene->DestroyEntity(entity);
            entity = invalidEntity;
        }
    }

    bool PlaygroundScene::Initialize(Scene& scene, Physics::JoltContext& jolt)
    {
        SIM_PROFILE_SCOPE("PlaygroundScene::Initialize");

        activeScene = &scene;
        activeJolt = &jolt;

        // 1. Instantiate Core Subsystems
        physicsSystem = std::make_unique<Physics::Systems::PhysicsSystem>(scene, jolt);
        jointSystem = std::make_unique<Physics::Systems::JointSystem>(scene, jolt);
        diffSystem = std::make_unique<DifferentialMechanismSystem>(scene, &jolt);
        flywheelSystem = std::make_unique<FlywheelSystem>(scene, &jolt);
        sensorSystem = std::make_unique<SensorSystem>(scene);

        // 2. High-Contrast Slate Testing Floor (Y = 0.0m)
        groundEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, groundEntity, glm::vec3(0.0f, -0.05f, 10.0f), glm::vec3(20.0f, 0.10f, 40.0f));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Plane;
            mesh.color = glm::vec3(0.11f, 0.13f, 0.16f); // Deep slate floor
            scene.AddComponent(groundEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.70f;
            rb.restitution = 0.40f;
            scene.AddComponent(groundEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(10.0f, 0.05f, 20.0f);
            scene.AddComponent(groundEntity, col);
        }

        // Initialize with Launcher Scene Selector active
        sceneStarted = false;
        selectedScene = PlaygroundSceneType::BallShooterHood;

        // Initial camera pose
        cameraFocusRequested = true;
        cameraFocusTarget = glm::vec3(0.0f, 1.0f, 2.0f);
        cameraFocusDistance = 3.2f;

        return true;
    }

    void PlaygroundScene::LoadScene(PlaygroundSceneType type)
    {
        SIM_PROFILE_SCOPE("PlaygroundScene::LoadScene");

        UnloadScene();

        activeSceneType = type;
        sceneStarted = true;
        hasLandingRecorded = false;
        lastLandingPos = glm::vec3(0.0f, -10.0f, 0.0f);

        if (type == PlaygroundSceneType::BallShooterHood)
        {
            isDualFlywheel = false;
            targetFlywheelRPM = 3200.0f;
            uncompressedSizeM = 0.130f; // 130mm foam ball
            nominalGapM = 0.075f;       // 75mm compression gap (55mm squish)
            complianceK = 65000.0f;     // Ball foam stiffness
            pieceMassKg = 0.200f;
            isOverhandShooter = false;
            hoodStartAngleDeg = -20.0f;
            hoodEndAngleDeg = 50.0f;
            currentFlightSpeedMPerS = 0.0;
            lastExitSpeedMPerS = 0.0;
            SpawnBallShooterHoodRig();
            cameraFocusRequested = true;
            cameraFocusTarget = glm::vec3(0.0f, 1.05f, 0.2f);
            cameraFocusDistance = 2.4f;
        }
        else if (type == PlaygroundSceneType::NoteShooterDual)
        {
            isDualFlywheel = true;
            targetFlywheelRPM = 3200.0f;
            uncompressedSizeM = 0.0508f; // 2-inch thickness note
            nominalGapM = 0.0381f;       // 1.5-inch gap (0.5-inch squish)
            complianceK = 180000.0f;     // High-density closed-cell foam ring stiffness
            pieceMassKg = 0.235f;        // 2024 Note mass
            notePitchDeg = 35.0f;
            currentFlightSpeedMPerS = 0.0;
            lastExitSpeedMPerS = 0.0;
            SpawnNoteShooterDualRig();
            cameraFocusRequested = true;
            cameraFocusTarget = glm::vec3(0.0f, 1.05f, 0.2f);
            cameraFocusDistance = 2.4f;
        }
        else if (type == PlaygroundSceneType::DifferentialWrist)
        {
            SpawnDifferentialWristRig();
            cameraFocusRequested = true;
            cameraFocusTarget = glm::vec3(0.0f, 1.00f, 0.15f);
            cameraFocusDistance = 1.8f;
        }

        // Warm up physics constraints
        if (physicsSystem) { physicsSystem->FixedUpdate(0.005); }
        if (jointSystem) { jointSystem->FixedUpdate(0.005); }
        if (flywheelSystem) { flywheelSystem->FixedUpdate(0.005); }
    }

    void PlaygroundScene::UnloadScene()
    {
        if (!activeScene) return;

        // 1. Unload Differential Wrist Entities
        DestroyTrackedEntity(mountStandEntity);
        DestroyTrackedEntity(pitchYokeEntity);
        DestroyTrackedEntity(rollToolEntity);
        DestroyTrackedEntity(pitchJointEntity);
        DestroyTrackedEntity(rollJointEntity);
        DestroyTrackedEntity(diffMechanismEntity);

        // 2. Unload Shooter Entities
        DestroyTrackedEntity(shooterStandEntity);
        DestroyTrackedEntity(topFlywheelEntity);
        DestroyTrackedEntity(bottomFlywheelEntity);
        DestroyTrackedEntity(contactZoneEntity);
        DestroyTrackedEntity(beamBreakEntity);
        DestroyTrackedEntity(hoodLeftCheekEntity);
        DestroyTrackedEntity(hoodRightCheekEntity);

        // 3. Unload Intake Entities
        DestroyTrackedEntity(intakeRollerEntity);
        DestroyTrackedEntity(intakeZoneEntity);
        DestroyTrackedEntity(intakeBeamBreakEntity);
        storedPiecesCount = 0;
        lastIntakeTorqueNm = 0.0;
        currentIntakeRpm = 0.0;

        // 4. Unload Hood Segments
        for (auto& seg : hoodSegmentEntities)
        {
            DestroyTrackedEntity(seg);
        }
        hoodSegmentEntities.clear();

        // 4. Unload Projectile Pool
        for (auto& p : projectilePool)
        {
            DestroyTrackedEntity(p.entity);
            p.active = false;
            p.inSqueeze = false;
            p.progressM = 0.0;
            p.hasHitGround = false;
            p.trajectoryTrail.clear();
        }
        nextProjectileIdx = 0;
        pieceActive = false;
        hasShotFired = false;
        currentFlightSpeedMPerS = 0.0;
        lastExitSpeedMPerS = 0.0;
    }

    void PlaygroundScene::SpawnDifferentialWristRig()
    {
        if (!activeScene) return;
        Scene& scene = *activeScene;

        mountStandEntity = scene.CreateEntity();
        {
            glm::vec3 standPos(0.0f, 0.50f, 0.0f);
            glm::vec3 standDims(0.16f, 1.00f, 0.16f);
            AttachTransforms(scene, mountStandEntity, standPos, standDims);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.28f, 0.30f, 0.33f);
            scene.AddComponent(mountStandEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.isSensor = true;
            scene.AddComponent(mountStandEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = standDims * 0.5f;
            scene.AddComponent(mountStandEntity, col);
        }

        pitchYokeEntity = scene.CreateEntity();
        {
            glm::vec3 yokePos(0.0f, 1.00f, 0.05f);
            glm::vec3 yokeDims(0.22f, 0.08f, 0.12f);
            AttachTransforms(scene, pitchYokeEntity, yokePos, yokeDims);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.10f, 0.65f, 0.95f); // Electric Cyan Yoke
            scene.AddComponent(pitchYokeEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 1.20f;
            rb.gravityFactor = 0.0f;
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            rb.allowSleeping = false;
            rb.isSensor = true;
            scene.AddComponent(pitchYokeEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = yokeDims * 0.5f;
            scene.AddComponent(pitchYokeEntity, col);
        }

        pitchJointEntity = scene.CreateEntity();
        {
            JointComponent jc;
            jc.type = JointType::Hinge;
            jc.parentEntity = mountStandEntity;
            jc.childEntity = pitchYokeEntity;
            jc.SetWorldSpace(glm::vec3(0.0f, 1.00f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            jc.hasLimits = true;
            jc.minLimit = glm::radians(-85.0f);
            jc.maxLimit = glm::radians(+85.0f);
            jc.limitSpringFrequency = 25.0f;
            jc.limitSpringDamping = 1.0f;
            scene.AddComponent(pitchJointEntity, jc);
        }

        rollToolEntity = scene.CreateEntity();
        {
            glm::vec3 toolPos(0.0f, 1.00f, 0.20f);
            glm::vec3 toolDims(0.12f, 0.06f, 0.18f);
            AttachTransforms(scene, rollToolEntity, toolPos, toolDims);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(1.00f, 0.45f, 0.10f); // Bright Safety Orange Tool
            scene.AddComponent(rollToolEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 0.65f;
            rb.gravityFactor = 0.0f;
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            rb.allowSleeping = false;
            rb.isSensor = true;
            scene.AddComponent(rollToolEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = toolDims * 0.5f;
            scene.AddComponent(rollToolEntity, col);
        }

        rollJointEntity = scene.CreateEntity();
        {
            JointComponent jc;
            jc.type = JointType::Hinge;
            jc.parentEntity = pitchYokeEntity;
            jc.childEntity = rollToolEntity;
            jc.SetWorldSpace(glm::vec3(0.0f, 1.00f, 0.11f), glm::vec3(0.0f, 0.0f, 1.0f));
            jc.hasLimits = true;
            jc.minLimit = glm::radians(-175.0f);
            jc.maxLimit = glm::radians(+175.0f);
            jc.limitSpringFrequency = 25.0f;
            jc.limitSpringDamping = 1.0f;
            scene.AddComponent(rollJointEntity, jc);
        }

        diffMechanismEntity = scene.CreateEntity();
        {
            DifferentialComponent dc;
            dc.name = "playground_wrist_diff";
            dc.differentialType = DifferentialType::SymmetricBevel;
            dc.jointEntityA = pitchJointEntity;
            dc.jointEntityB = rollJointEntity;
            dc.linkEntityA = pitchYokeEntity;
            dc.linkEntityB = rollToolEntity;
            dc.t_mq[0][0] = gearRatioPitch;
            dc.t_mq[0][1] = gearRatioRoll;
            dc.t_mq[1][0] = gearRatioPitch;
            dc.t_mq[1][1] = -gearRatioRoll;
            dc.t_qm = glm::inverse(dc.t_mq);
            dc.efficiency = static_cast<double>(transmissionEfficiency);
            dc.coulombFrictionNm = { static_cast<double>(frictionPitchNm), static_cast<double>(frictionRollNm) };
            dc.motorModelA = Actuation::DCMotorModel::KrakenX60(1);
            dc.motorModelB = Actuation::DCMotorModel::KrakenX60(1);
            dc.commandedVoltageA = 0.0;
            dc.commandedVoltageB = 0.0;
            dc.brakeModeA = true;
            dc.brakeModeB = true;
            scene.AddComponent(diffMechanismEntity, dc);
        }
    }

    void PlaygroundScene::SpawnBallShooterHoodRig()
    {
        if (!activeScene) return;
        Scene& scene = *activeScene;

        // 1. Support Column (Positioned below the shooter)
        shooterStandEntity = scene.CreateEntity();
        {
            glm::vec3 standPos(0.0f, 0.45f, 0.0f);
            glm::vec3 standDims(0.18f, 0.90f, 0.18f);
            AttachTransforms(scene, shooterStandEntity, standPos, standDims);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.20f, 0.22f, 0.25f);
            scene.AddComponent(shooterStandEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            scene.AddComponent(shooterStandEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = standDims * 0.5f;
            scene.AddComponent(shooterStandEntity, col);
        }

        // 2. Twin Side Cheek Brackets
        constexpr float hubY = 1.05f;
        constexpr float hubZ = 0.0f;
        constexpr float hoodWidth = 0.28f;

        hoodLeftCheekEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, hoodLeftCheekEntity, glm::vec3(-0.145f, hubY - 0.02f, hubZ), glm::vec3(0.008f, 0.28f, 0.34f));
            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.22f, 0.24f, 0.28f); // Anodized Gunmetal Aluminum
            scene.AddComponent(hoodLeftCheekEntity, mesh);
        }

        hoodRightCheekEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, hoodRightCheekEntity, glm::vec3(+0.145f, hubY - 0.02f, hubZ), glm::vec3(0.008f, 0.28f, 0.34f));
            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.22f, 0.24f, 0.28f);
            scene.AddComponent(hoodRightCheekEntity, mesh);
        }

        // 3. Central Flywheel Roller (Top Rotor)
        topFlywheelEntity = scene.CreateEntity();
        {
            glm::vec3 fwPos(0.0f, hubY, hubZ);
            glm::vec3 fwScale(0.24f, 2.0f * rollerRadiusM, 2.0f * rollerRadiusM);
            AttachTransforms(scene, topFlywheelEntity, fwPos, fwScale);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::CylinderX;
            mesh.color = glm::vec3(0.96f, 0.72f, 0.12f); // High-Grip Urethane Gold
            scene.AddComponent(topFlywheelEntity, mesh);

            FlywheelComponent fc;
            fc.name = "playground_flywheel_top";
            fc.spinAxis = { 1.0f, 0.0f, 0.0f };
            fc.flywheelInertiaKgm2 = flywheelInertiaKgm2;
            fc.rollerRadiusM = rollerRadiusM;
            fc.gearRatio = 1.0;
            fc.efficiency = 0.98;
            fc.subSteps = 8;
            fc.motorCount = 2;
            fc.motorSlots = { 10, 11 };
            fc.motorModel = Actuation::DCMotorModel::KrakenX60(1.0);
            fc.commandedVoltage = commandedFlywheelVoltage;
            scene.AddComponent(topFlywheelEntity, fc);
        }

        // 4. Contact Zone Component
        contactZoneEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, contactZoneEntity, glm::vec3(0.0f, hubY - 0.5f * (rollerRadiusM + nominalGapM), hubZ),
                glm::vec3(0.24f, nominalGapM, 0.16f));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.12f, 0.85f, 0.45f); // Transparent Nip Green
            scene.AddComponent(contactZoneEntity, mesh);

            ContactZoneComponent cz;
            cz.name = "ball_shooter_zone";
            cz.role = "shooter";
            cz.parentLink = shooterStandEntity;
            cz.localOrigin = { 0.0f, hubY - 0.5f * (rollerRadiusM + nominalGapM), hubZ };
            cz.dimensions = { 0.24f, nominalGapM, 0.16f };
            cz.tangentAxis = { 0.0f, 0.0f, 1.0f };
            cz.compressionAxis = { 0.0f, 1.0f, 0.0f };
            cz.nominalGapM = nominalGapM;
            cz.isEngaged = false;
            scene.AddComponent(contactZoneEntity, cz);
        }

        // 5. Optical Beam-Break Sensor (DIO Channel 2)
        beamBreakEntity = scene.CreateEntity();
        {
            BeamBreakComponent bb;
            bb.name = "playground_beam_break";
            bb.sensorEntity = contactZoneEntity;
            bb.dioChannel = 2;
            bb.debounceTimeS = 0.008;
            bb.normallyClosed = false;
            scene.AddComponent(beamBreakEntity, bb);
        }

        // 6. Spawn Curved Polycarb Hood Segments
        hoodSegmentEntities.clear();
        hoodSegmentEntities.reserve(kHoodSegments);
        for (size_t i = 0; i < kHoodSegments; ++i)
        {
            Entity seg = scene.CreateEntity();
            AttachTransforms(scene, seg, glm::vec3(0.0f, hubY, hubZ), glm::vec3(hoodWidth, 0.008f, 0.02f));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.15f, 0.65f, 0.90f); // Tinted Polycarbonate Cyan
            scene.AddComponent(seg, mesh);

            hoodSegmentEntities.push_back(seg);
        }

        UpdateHoodGeometry();

        // 7. Initialize Dynamic Ball Projectile Pool (Fluorescent Orange Foam Balls)
        constexpr float ballRadius = 0.065f; // 130mm diameter
        for (size_t i = 0; i < kMaxProjectiles; ++i)
        {
            auto& p = projectilePool[i];
            p.entity = scene.CreateEntity();
            p.active = false;
            p.inSqueeze = false;
            p.progressM = 0.0;
            p.velocityMPerS = 2.5;
            p.lifeTimeS = 0.0;
            p.hasHitGround = false;

            AttachTransforms(scene, p.entity, glm::vec3(0.0f, -10.0f, 0.0f), glm::vec3(2.0f * ballRadius));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Sphere;
            mesh.color = glm::vec3(1.00f, 0.40f, 0.05f); // High-Visibility Fluorescent Orange
            scene.AddComponent(p.entity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = pieceMassKg;
            rb.gravityFactor = 1.0f;
            rb.linearDamping = 0.015f;
            rb.angularDamping = 0.035f;
            rb.friction = 0.55f;
            rb.restitution = 0.70f;
            rb.allowSleeping = true;
            scene.AddComponent(p.entity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Sphere;
            col.radius = ballRadius;
            scene.AddComponent(p.entity, col);

            GamePieceComponent gp;
            gp.state = GamePieceState::Free;
            gp.spec.massKg = pieceMassKg;
            gp.spec.complianceK = complianceK;
            gp.spec.restitutionE = restitutionE;
            gp.spec.surfaceFriction = frictionMu;
            scene.AddComponent(p.entity, gp);
        }

        // 8. Ground Intake Roller, Zone & Beam-Break Sensor
        intakeRollerEntity = scene.CreateEntity();
        {
            glm::vec3 rollerPos(0.0f, 0.12f, -0.55f);
            glm::vec3 rollerScale(0.32f, 2.0f * intakeRollerRadiusM, 2.0f * intakeRollerRadiusM);
            AttachTransforms(scene, intakeRollerEntity, rollerPos, rollerScale);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::CylinderX;
            mesh.color = glm::vec3(0.08f, 0.85f, 0.70f); // Bright Aqua Green Silicone
            scene.AddComponent(intakeRollerEntity, mesh);

            FlywheelComponent fc;
            fc.name = "ball_intake_roller";
            fc.spinAxis = { 1.0f, 0.0f, 0.0f }; // Pulls pieces forward (+Z) into throat
            fc.flywheelInertiaKgm2 = 0.0015f;
            fc.rollerRadiusM = intakeRollerRadiusM;
            fc.gearRatio = 1.0;
            fc.efficiency = 0.95;
            fc.subSteps = 8;
            fc.motorCount = 1;
            fc.motorSlots = { 17 };
            fc.motorModel = Actuation::DCMotorModel::KrakenX60(1);
            fc.commandedVoltage = intakeEnabled ? intakeVoltage : 0.0f;
            scene.AddComponent(intakeRollerEntity, fc);
        }

        intakeZoneEntity = scene.CreateEntity();
        {
            glm::vec3 zonePos(0.0f, 0.12f, -0.55f);
            glm::vec3 zoneDims(0.38f, 0.22f, 0.40f);
            AttachTransforms(scene, intakeZoneEntity, zonePos, zoneDims);

            ContactZoneComponent cz;
            cz.name = "ball_intake_zone";
            cz.role = "intake";
            cz.parentLink = shooterStandEntity;
            cz.localOrigin = zonePos;
            cz.dimensions = zoneDims;
            cz.tangentAxis = { 0.0f, 0.25f, 0.968f }; // Pulls inward and up toward shooter entrance
            cz.compressionAxis = { 0.0f, 0.968f, -0.25f };
            cz.nominalGapM = 0.075f;
            cz.isEngaged = false;
            scene.AddComponent(intakeZoneEntity, cz);
        }

        intakeBeamBreakEntity = scene.CreateEntity();
        {
            BeamBreakComponent bb;
            bb.name = "ball_intake_beam_break";
            bb.sensorEntity = intakeZoneEntity;
            bb.dioChannel = 3;
            bb.debounceTimeS = 0.008;
            bb.normallyClosed = false;
            scene.AddComponent(intakeBeamBreakEntity, bb);
        }
    }

    void PlaygroundScene::SpawnNoteShooterDualRig()
    {
        if (!activeScene) return;
        Scene& scene = *activeScene;

        // 1. Support Column
        shooterStandEntity = scene.CreateEntity();
        {
            glm::vec3 standPos(0.0f, 0.45f, 0.0f);
            glm::vec3 standDims(0.18f, 0.90f, 0.18f);
            AttachTransforms(scene, shooterStandEntity, standPos, standDims);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.20f, 0.22f, 0.25f);
            scene.AddComponent(shooterStandEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            scene.AddComponent(shooterStandEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = standDims * 0.5f;
            scene.AddComponent(shooterStandEntity, col);
        }

        // 2. Dual Counter-Rotating Flywheel Rollers
        constexpr float hubY = 1.05f;
        float halfGap = 0.5f * nominalGapM + rollerRadiusM;

        // Top Flywheel Roller
        topFlywheelEntity = scene.CreateEntity();
        {
            glm::vec3 fwPos(0.0f, hubY + halfGap, 0.0f);
            glm::vec3 fwScale(0.38f, 2.0f * rollerRadiusM, 2.0f * rollerRadiusM);
            AttachTransforms(scene, topFlywheelEntity, fwPos, fwScale);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::CylinderX;
            mesh.color = glm::vec3(0.96f, 0.72f, 0.12f); // Gold Urethane
            scene.AddComponent(topFlywheelEntity, mesh);

            FlywheelComponent fc;
            fc.name = "note_flywheel_top";
            fc.spinAxis = { 1.0f, 0.0f, 0.0f };
            fc.flywheelInertiaKgm2 = flywheelInertiaKgm2;
            fc.rollerRadiusM = rollerRadiusM;
            fc.gearRatio = 1.0;
            fc.efficiency = 0.98;
            fc.subSteps = 8;
            fc.motorCount = 1;
            fc.motorSlots = { 10 };
            fc.motorModel = Actuation::DCMotorModel::KrakenX60(1);
            fc.commandedVoltage = commandedFlywheelVoltage;
            scene.AddComponent(topFlywheelEntity, fc);
        }

        // Bottom Flywheel Roller (Counter-Rotating)
        bottomFlywheelEntity = scene.CreateEntity();
        {
            glm::vec3 fwPos(0.0f, hubY - halfGap, 0.0f);
            glm::vec3 fwScale(0.38f, 2.0f * rollerRadiusM, 2.0f * rollerRadiusM);
            AttachTransforms(scene, bottomFlywheelEntity, fwPos, fwScale);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::CylinderX;
            mesh.color = glm::vec3(0.96f, 0.72f, 0.12f);
            scene.AddComponent(bottomFlywheelEntity, mesh);

            FlywheelComponent fc;
            fc.name = "note_flywheel_bottom";
            fc.spinAxis = { -1.0f, 0.0f, 0.0f }; // Counter-rotating
            fc.flywheelInertiaKgm2 = flywheelInertiaKgm2;
            fc.rollerRadiusM = rollerRadiusM;
            fc.gearRatio = 1.0;
            fc.efficiency = 0.98;
            fc.subSteps = 8;
            fc.motorCount = 1;
            fc.motorSlots = { 11 };
            fc.motorModel = Actuation::DCMotorModel::KrakenX60(1);
            fc.commandedVoltage = commandedFlywheelVoltage;
            scene.AddComponent(bottomFlywheelEntity, fc);
        }

        // 3. Side Brackets
        hoodLeftCheekEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, hoodLeftCheekEntity, glm::vec3(-0.21f, hubY, 0.0f), glm::vec3(0.008f, 0.32f, 0.28f));
            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.22f, 0.24f, 0.28f);
            scene.AddComponent(hoodLeftCheekEntity, mesh);
        }

        hoodRightCheekEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, hoodRightCheekEntity, glm::vec3(+0.21f, hubY, 0.0f), glm::vec3(0.008f, 0.32f, 0.28f));
            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.22f, 0.24f, 0.28f);
            scene.AddComponent(hoodRightCheekEntity, mesh);
        }

        // 4. Contact Zone Component
        contactZoneEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, contactZoneEntity, glm::vec3(0.0f, hubY, 0.0f), glm::vec3(0.38f, nominalGapM, 0.18f));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.12f, 0.85f, 0.45f);
            scene.AddComponent(contactZoneEntity, mesh);

            ContactZoneComponent cz;
            cz.name = "note_shooter_zone";
            cz.role = "shooter";
            cz.parentLink = shooterStandEntity;
            cz.localOrigin = { 0.0f, hubY, 0.0f };
            cz.dimensions = { 0.38f, nominalGapM, 0.18f };
            cz.tangentAxis = { 0.0f, 0.0f, 1.0f };
            cz.compressionAxis = { 0.0f, 1.0f, 0.0f };
            cz.nominalGapM = nominalGapM;
            cz.isEngaged = false;
            scene.AddComponent(contactZoneEntity, cz);
        }

        // 5. Optical Beam-Break Sensor (DIO Channel 2)
        beamBreakEntity = scene.CreateEntity();
        {
            BeamBreakComponent bb;
            bb.name = "note_beam_break";
            bb.sensorEntity = contactZoneEntity;
            bb.dioChannel = 2;
            bb.debounceTimeS = 0.008;
            bb.normallyClosed = false;
            scene.AddComponent(beamBreakEntity, bb);
        }

        contactLengthM = 0.18f;

        // 6. Initialize 2024 Note Projectile Pool (14-inch OD, 2-inch thickness orange foam torus / slab)
        constexpr float noteOD = 0.3556f;
        constexpr float noteThick = 0.0508f;
        for (size_t i = 0; i < kMaxProjectiles; ++i)
        {
            auto& p = projectilePool[i];
            p.entity = scene.CreateEntity();
            p.active = false;
            p.inSqueeze = false;
            p.progressM = 0.0;
            p.velocityMPerS = 2.5;
            p.lifeTimeS = 0.0;
            p.hasHitGround = false;

            AttachTransforms(scene, p.entity, glm::vec3(0.0f, -10.0f, 0.0f), glm::vec3(noteOD, noteThick, noteOD));

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::Cylinder;
            mesh.color = glm::vec3(1.00f, 0.38f, 0.00f); // 2024 Crescendo Note Orange
            scene.AddComponent(p.entity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = pieceMassKg;
            rb.gravityFactor = 1.0f;
            rb.linearDamping = 0.015f;
            rb.angularDamping = 0.035f;
            rb.friction = 0.65f;
            rb.restitution = 0.45f;
            rb.allowSleeping = true;
            scene.AddComponent(p.entity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(noteOD * 0.5f, noteThick * 0.5f, noteOD * 0.5f);
            scene.AddComponent(p.entity, col);

            GamePieceComponent gp;
            gp.state = GamePieceState::Free;
            gp.spec.massKg = pieceMassKg;
            gp.spec.complianceK = complianceK;
            gp.spec.restitutionE = restitutionE;
            gp.spec.surfaceFriction = frictionMu;
            scene.AddComponent(p.entity, gp);
        }

        UpdateNoteShooterGeometry();

        // 7. Ground Intake Roller, Zone & Beam-Break Sensor
        intakeRollerEntity = scene.CreateEntity();
        {
            glm::vec3 rollerPos(0.0f, 0.08f, -0.55f);
            glm::vec3 rollerScale(0.42f, 2.0f * intakeRollerRadiusM, 2.0f * intakeRollerRadiusM);
            AttachTransforms(scene, intakeRollerEntity, rollerPos, rollerScale);

            Renderer::Components::MeshComponent mesh;
            mesh.shape = Renderer::Components::PrimitiveShape::CylinderX;
            mesh.color = glm::vec3(0.08f, 0.85f, 0.70f); // Bright Aqua Green Silicone
            scene.AddComponent(intakeRollerEntity, mesh);

            FlywheelComponent fc;
            fc.name = "note_intake_roller";
            fc.spinAxis = { 1.0f, 0.0f, 0.0f }; // Pulls piece inward (+Z) into throat
            fc.flywheelInertiaKgm2 = 0.0015f;
            fc.rollerRadiusM = intakeRollerRadiusM;
            fc.gearRatio = 1.0;
            fc.efficiency = 0.95;
            fc.subSteps = 8;
            fc.motorCount = 1;
            fc.motorSlots = { 17 };
            fc.motorModel = Actuation::DCMotorModel::KrakenX60(1);
            fc.commandedVoltage = intakeEnabled ? intakeVoltage : 0.0f;
            scene.AddComponent(intakeRollerEntity, fc);
        }

        intakeZoneEntity = scene.CreateEntity();
        {
            glm::vec3 zonePos(0.0f, 0.08f, -0.55f);
            glm::vec3 zoneDims(0.44f, 0.16f, 0.40f);
            AttachTransforms(scene, intakeZoneEntity, zonePos, zoneDims);

            ContactZoneComponent cz;
            cz.name = "note_intake_zone";
            cz.role = "intake";
            cz.parentLink = shooterStandEntity;
            cz.localOrigin = zonePos;
            cz.dimensions = zoneDims;
            cz.tangentAxis = { 0.0f, 0.25f, 0.968f };
            cz.compressionAxis = { 0.0f, 0.968f, -0.25f };
            cz.nominalGapM = 0.040f;
            cz.isEngaged = false;
            scene.AddComponent(intakeZoneEntity, cz);
        }

        intakeBeamBreakEntity = scene.CreateEntity();
        {
            BeamBreakComponent bb;
            bb.name = "note_intake_beam_break";
            bb.sensorEntity = intakeZoneEntity;
            bb.dioChannel = 3;
            bb.debounceTimeS = 0.008;
            bb.normallyClosed = false;
            scene.AddComponent(intakeBeamBreakEntity, bb);
        }
    }

    void PlaygroundScene::UpdateHoodGeometry()
    {
        if (hoodSegmentEntities.empty() || !activeScene) return;

        if (hoodEndAngleDeg <= hoodStartAngleDeg + 5.0f)
        {
            hoodEndAngleDeg = hoodStartAngleDeg + 5.0f;
        }

        constexpr float hubY = 1.05f;
        constexpr float hubZ = 0.0f;
        constexpr float hoodWidth = 0.28f;
        constexpr float hoodThickness = 0.008f;

        float rHood = rollerRadiusM + nominalGapM;
        float rPath = rollerRadiusM + 0.5f * nominalGapM;

        float startRad = glm::radians(hoodStartAngleDeg);
        float endRad = glm::radians(hoodEndAngleDeg);
        float totalArcSpan = endRad - startRad;

        contactLengthM = rPath * totalArcSpan;

        float segSpan = totalArcSpan / static_cast<float>(kHoodSegments);
        float segArcLen = (rHood * segSpan) * 1.08f;

        for (size_t i = 0; i < kHoodSegments; ++i)
        {
            float theta = startRad + (static_cast<float>(i) + 0.5f) * segSpan;

            float ySeg = isOverhandShooter
                ? (hubY + (rHood + 0.5f * hoodThickness) * std::cos(theta))
                : (hubY - (rHood + 0.5f * hoodThickness) * std::cos(theta));

            glm::vec3 segPos(
                0.0f,
                ySeg,
                hubZ + (rHood + 0.5f * hoodThickness) * std::sin(theta)
            );

            glm::quat segRot = isOverhandShooter
                ? glm::angleAxis(theta, glm::vec3(1.0f, 0.0f, 0.0f))
                : glm::angleAxis(-theta, glm::vec3(1.0f, 0.0f, 0.0f));

            auto* t = activeScene->GetComponent<TransformComponent>(hoodSegmentEntities[i]);
            if (t)
            {
                t->position = segPos;
                t->rotation = segRot;
                t->scale = glm::vec3(hoodWidth, hoodThickness, segArcLen);
            }
        }

        // Update contact zone position to match underhand vs overhand
        auto* cz = activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity);
        auto* tCz = activeScene->GetComponent<TransformComponent>(contactZoneEntity);
        float yCz = isOverhandShooter
            ? (hubY + 0.5f * (rollerRadiusM + nominalGapM))
            : (hubY - 0.5f * (rollerRadiusM + nominalGapM));
        if (tCz && cz)
        {
            tCz->position = glm::vec3(0.0f, yCz, hubZ);
            cz->localOrigin = { 0.0f, yCz, hubZ };
        }

        // Update side cheeks position
        auto* tLeft = activeScene->GetComponent<TransformComponent>(hoodLeftCheekEntity);
        auto* tRight = activeScene->GetComponent<TransformComponent>(hoodRightCheekEntity);
        float yCheek = isOverhandShooter ? (hubY + 0.02f) : (hubY - 0.02f);
        if (tLeft) { tLeft->position = glm::vec3(-0.145f, yCheek, hubZ); }
        if (tRight) { tRight->position = glm::vec3(+0.145f, yCheek, hubZ); }
    }

    void PlaygroundScene::UpdateNoteShooterGeometry()
    {
        if (!activeScene) return;

        constexpr float hubY = 1.05f;
        float halfGap = 0.5f * nominalGapM + rollerRadiusM;
        float pitchRad = glm::radians(notePitchDeg);

        glm::vec3 shootDir(0.0f, std::sin(pitchRad), std::cos(pitchRad));
        glm::vec3 normalDir(0.0f, std::cos(pitchRad), -std::sin(pitchRad));
        glm::quat pitchRot = glm::angleAxis(-pitchRad, glm::vec3(1.0f, 0.0f, 0.0f));

        // Top Flywheel Roller
        auto* tTop = activeScene->GetComponent<TransformComponent>(topFlywheelEntity);
        if (tTop)
        {
            tTop->position = glm::vec3(0.0f, hubY, 0.0f) + normalDir * halfGap;
            tTop->rotation = pitchRot;
        }

        // Bottom Flywheel Roller
        auto* tBot = activeScene->GetComponent<TransformComponent>(bottomFlywheelEntity);
        if (tBot)
        {
            tBot->position = glm::vec3(0.0f, hubY, 0.0f) - normalDir * halfGap;
            tBot->rotation = pitchRot;
        }

        // Left Cheek
        auto* tLeft = activeScene->GetComponent<TransformComponent>(hoodLeftCheekEntity);
        if (tLeft)
        {
            tLeft->position = glm::vec3(-0.21f, hubY, 0.0f);
            tLeft->rotation = pitchRot;
        }

        // Right Cheek
        auto* tRight = activeScene->GetComponent<TransformComponent>(hoodRightCheekEntity);
        if (tRight)
        {
            tRight->position = glm::vec3(+0.21f, hubY, 0.0f);
            tRight->rotation = pitchRot;
        }

        // Contact Zone
        auto* tCz = activeScene->GetComponent<TransformComponent>(contactZoneEntity);
        auto* cz = activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity);
        if (tCz && cz)
        {
            tCz->position = glm::vec3(0.0f, hubY, 0.0f);
            tCz->rotation = pitchRot;
            cz->localOrigin = { 0.0f, hubY, 0.0f };
            cz->tangentAxis = shootDir;
            cz->compressionAxis = normalDir;
        }
    }

    void PlaygroundScene::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("PlaygroundScene::FixedUpdate");

        if (!activeScene || !sceneStarted)
        {
            return;
        }

        testTimeSeconds += dt;

        // ---------------------------------------------------------------------
        // 1. Advance Differential Wrist Test Rig
        // ---------------------------------------------------------------------
        if (activeSceneType == PlaygroundSceneType::DifferentialWrist)
        {
            auto* diff = activeScene->GetComponent<DifferentialComponent>(diffMechanismEntity);
            if (diff)
            {
                switch (controlMode)
                {
                case PlaygroundControlMode::ManualSliders:
                    diff->commandedVoltageA = static_cast<double>(manualVoltageA);
                    diff->commandedVoltageB = static_cast<double>(manualVoltageB);
                    break;
                case PlaygroundControlMode::PitchTest:
                    diff->commandedVoltageA = static_cast<double>(testVoltageMagnitude);
                    diff->commandedVoltageB = static_cast<double>(testVoltageMagnitude);
                    break;
                case PlaygroundControlMode::RollTest:
                    diff->commandedVoltageA = static_cast<double>(testVoltageMagnitude);
                    diff->commandedVoltageB = -static_cast<double>(testVoltageMagnitude);
                    break;
                case PlaygroundControlMode::SineSweep:
                {
                    double sineWave = testVoltageMagnitude * std::sin(2.0 * 3.14159265 * testFrequencyHz * testTimeSeconds);
                    diff->commandedVoltageA = sineWave;
                    diff->commandedVoltageB = sineWave;
                    break;
                }
                case PlaygroundControlMode::BrakeHold:
                default:
                    diff->commandedVoltageA = 0.0;
                    diff->commandedVoltageB = 0.0;
                    break;
                }

                diff->brakeModeA = brakeModeA;
                diff->brakeModeB = brakeModeB;
                diff->efficiency = static_cast<double>(transmissionEfficiency);
                diff->coulombFrictionNm = { static_cast<double>(frictionPitchNm), static_cast<double>(frictionRollNm) };
            }

            if (diffSystem) { diffSystem->FixedUpdate(dt); }
        }

        // ---------------------------------------------------------------------
        // 2. Advance Shooter Rigs (Ball Shooter or Note Shooter)
        // ---------------------------------------------------------------------
        if (activeSceneType == PlaygroundSceneType::BallShooterHood || activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            auto* fwTop = activeScene->GetComponent<FlywheelComponent>(topFlywheelEntity);
            auto* fwBottom = activeScene->GetComponent<FlywheelComponent>(bottomFlywheelEntity);
            auto* cz = activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity);

            if (fwTop)
            {
                fwTop->flywheelInertiaKgm2 = flywheelInertiaKgm2;
                fwTop->rollerRadiusM = rollerRadiusM;

                if (useFlywheelClosedLoop)
                {
                    constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                    double currentRpm = fwTop->angularVelocityRadPerSec * radSToRpm;
                    double rpmError = targetFlywheelRPM - currentRpm;
                    double feedforward = (targetFlywheelRPM / 6000.0) * 12.0;
                    double pEffort = rpmError * 0.015;
                    commandedFlywheelVoltage = static_cast<float>(std::clamp(feedforward + pEffort, -12.0, 12.0));
                }

                fwTop->commandedVoltage = commandedFlywheelVoltage;
            }

            if (fwBottom)
            {
                fwBottom->flywheelInertiaKgm2 = flywheelInertiaKgm2;
                fwBottom->rollerRadiusM = rollerRadiusM;
                fwBottom->commandedVoltage = commandedFlywheelVoltage;
            }

            // -----------------------------------------------------------------
            // Active Intake Roller & Kinematic Capture Handoff
            // -----------------------------------------------------------------
            auto* fwIntake = activeScene->GetComponent<FlywheelComponent>(intakeRollerEntity);
            auto* czIntake = activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity);
            if (fwIntake)
            {
                fwIntake->rollerRadiusM = intakeRollerRadiusM;
                fwIntake->commandedVoltage = intakeEnabled ? intakeVoltage : 0.0f;
                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                currentIntakeRpm = fwIntake->angularVelocityRadPerSec * radSToRpm;
            }

            bool anyPieceInIntake = false;
            if (czIntake && fwIntake && activeJolt && activeJolt->IsInitialized())
            {
                JPH::BodyInterface& bi = activeJolt->GetBodyInterface();

                for (size_t i = 0; i < kMaxProjectiles; ++i)
                {
                    auto& p = projectilePool[i];
                    if (!p.active || p.inSqueeze) continue;

                    auto* gp = activeScene->GetComponent<GamePieceComponent>(p.entity);
                    auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
                    auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                    if (!gp || !rb || !t || rb->bodyID.IsInvalid()) continue;

                    if (gp->state == GamePieceState::Free || gp->state == GamePieceState::Intaking)
                    {
                        JPH::RVec3 jPos = bi.GetPosition(rb->bodyID);
                        glm::vec3 piecePos(static_cast<float>(jPos.GetX()), static_cast<float>(jPos.GetY()), static_cast<float>(jPos.GetZ()));

                        auto* tCz = activeScene->GetComponent<TransformComponent>(intakeZoneEntity);
                        glm::vec3 zonePos = tCz ? tCz->position : czIntake->localOrigin;
                        glm::quat zoneRot = tCz ? tCz->rotation : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                        double pieceRadius = (activeSceneType == PlaygroundSceneType::BallShooterHood) ? 0.065 : 0.1778;

                        bool inZone = IntakeSurfaceModel::TestZoneOverlap(piecePos, pieceRadius, zonePos, zoneRot, czIntake->dimensions);
                        if (inZone)
                        {
                            anyPieceInIntake = true;
                            if (gp->state == GamePieceState::Free)
                            {
                                gp->state = GamePieceState::Intaking;
                                p.progressM = 0.0;
                            }

                            // Calculate roller surface velocity
                            glm::vec3 vRoller = IntakeSurfaceModel::CalculateRollerSurfaceVelocity(
                                fwIntake->angularVelocityRadPerSec,
                                fwIntake->rollerRadiusM,
                                czIntake->tangentAxis,
                                glm::vec3(0.0f) // Stationary test stand
                            );

                            JPH::Vec3 jVel = bi.GetLinearVelocity(rb->bodyID);
                            glm::vec3 pieceVel(jVel.GetX(), jVel.GetY(), jVel.GetZ());

                            // Calculate traction force and reaction load torque
                            IntakeParameters params;
                            params.rollerRadiusM = intakeRollerRadiusM;
                            params.normalForceN = intakeGripNormalForceN;
                            params.viscousDampingNsPerM = intakeViscousDampingNsPerM;
                            params.frictionMu = gp->spec.surfaceFriction > 0.0 ? gp->spec.surfaceFriction : 0.68;
                            params.maxGripForceN = intakeMaxGripForceN;
                            params.centerlineCaptureThresholdM = 0.22;

                            auto traction = IntakeSurfaceModel::CalculateIntakeTraction(
                                params,
                                vRoller,
                                pieceVel,
                                czIntake->tangentAxis,
                                p.progressM
                            );

                            // Apply reaction torque to intake roller motor
                            fwIntake->externalLoadTorqueNm += traction.loadTorqueNm;
                            lastIntakeTorqueNm = traction.loadTorqueNm;

                            // Apply traction force to game piece body in Jolt
                            bi.AddForce(rb->bodyID, JPH::Vec3(
                                static_cast<float>(traction.intakeForceN.x),
                                static_cast<float>(traction.intakeForceN.y),
                                static_cast<float>(traction.intakeForceN.z)
                            ));

                            // Track progress along intake path
                            float vTangent = glm::dot(pieceVel, czIntake->tangentAxis);
                            p.progressM += dt * std::max(0.0f, vTangent);

                            // Visual draw of traction vector
                            Utils::DebugDraw::Line(piecePos, piecePos + glm::normalize(czIntake->tangentAxis) * 0.2f, glm::vec3(0.1f, 1.0f, 0.4f), false);

                            // Check capture threshold
                            if (p.progressM >= 0.22 || piecePos.z >= -0.42f)
                            {
                                // Kinematic Capture Handoff
                                gp->state = GamePieceState::Stored;
                                storedPiecesCount++;

                                // Convert body to Kinematic in staging position
                                rb->motionType = BodyMotionType::Kinematic;
                                bi.SetMotionType(rb->bodyID, JPH::EMotionType::Kinematic, JPH::EActivation::DontActivate);
                                bi.SetLinearAndAngularVelocity(rb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());

                                // Staging position at throat entrance
                                glm::vec3 stagingPos(0.0f, 0.25f, -0.38f);
                                bi.SetPositionAndRotation(rb->bodyID,
                                    JPH::RVec3(stagingPos.x, stagingPos.y, stagingPos.z),
                                    JPH::Quat::sIdentity(),
                                    JPH::EActivation::DontActivate);
                                t->position = stagingPos;

                                // Auto-feed directly to shooter if enabled
                                if (autoFeedToShooter)
                                {
                                    p.active = false;
                                    bi.SetPositionAndRotation(rb->bodyID,
                                        JPH::RVec3(0.0f, -10.0f, 0.0f),
                                        JPH::Quat::sIdentity(),
                                        JPH::EActivation::DontActivate);
                                    FeedStoredPiece();
                                }
                            }
                        }
                        else if (gp->state == GamePieceState::Intaking && piecePos.z < -1.1f)
                        {
                            // Ejected or rolled out backwards
                            gp->state = GamePieceState::Free;
                        }
                    }
                }

                czIntake->isEngaged = anyPieceInIntake || (storedPiecesCount > 0);
            }

            // Rapid Fire Auto-Burst Trigger
            if (rapidFireEnabled)
            {
                rapidFireTimer += dt;
                float interval = (activeSceneType == PlaygroundSceneType::NoteShooterDual) ? 0.35f : 0.28f;
                if (rapidFireTimer >= interval)
                {
                    rapidFireTimer = 0.0;
                    FireGamePiece();
                }
            }

            constexpr float hubY = 1.05f;
            constexpr float hubZ = 0.0f;
            bool anyBallInSqueeze = false;

            // -----------------------------------------------------------------
            // SCENE SPECIFIC SQUEEZE & LAUNCH
            // -----------------------------------------------------------------
            if (activeSceneType == PlaygroundSceneType::BallShooterHood)
            {
                float rPath = rollerRadiusM + 0.5f * nominalGapM;
                float startRad = glm::radians(hoodStartAngleDeg);
                float endRad = glm::radians(hoodEndAngleDeg);
                float totalArcSpan = endRad - startRad;
                constexpr float ballRadius = 0.065f;

                for (size_t i = 0; i < kMaxProjectiles; ++i)
                {
                    auto& p = projectilePool[i];
                    if (!p.active) continue;

                    p.lifeTimeS += dt;

                    if (p.inSqueeze && fwTop)
                    {
                        anyBallInSqueeze = true;
                        shotTransitTimeMs += dt * 1000.0;
                        currentFlightSpeedMPerS = p.velocityMPerS;

                        SqueezeParameters params;
                        params.stiffnessK = complianceK;
                        params.restitutionE = restitutionE;
                        params.frictionMu = frictionMu;
                        params.hoodFrictionMu = hoodFrictionMu;
                        params.rollerRadiusM = fwTop->rollerRadiusM;
                        params.nominalGapM = nominalGapM;
                        params.uncompressedSizeM = uncompressedSizeM;
                        params.contactLengthM = contactLengthM;
                        params.pieceMassKg = pieceMassKg;
                        params.isDualFlywheel = false;

                        double vRoller = fwTop->angularVelocityRadPerSec * params.rollerRadiusM;

                        SqueezeState state;
                        state.progressM = p.progressM;
                        state.velocityMPerS = p.velocityMPerS;

                        ShooterSqueezeModel::StepSqueeze(params, state, vRoller, dt, fwTop->subSteps);

                        p.progressM = state.progressM;
                        p.velocityMPerS = state.velocityMPerS;
                        currentSqueezeDepthM = state.compressionDepthM;
                        currentNormalForceN = state.normalForceN;
                        currentTractionForceN = state.tractionForceN;

                        peakNormalForceN = std::max(peakNormalForceN, state.normalForceN);
                        peakTractionForceN = std::max(peakTractionForceN, state.tractionForceN);

                        // Reaction torque on flywheel
                        fwTop->externalLoadTorqueNm += state.loadTorqueNm;

                        // Visual interpolation along hood curve
                        float alpha = std::clamp(static_cast<float>(p.progressM / std::max(1e-4, static_cast<double>(contactLengthM))), 0.0f, 1.0f);
                        float theta = startRad + alpha * totalArcSpan;

                        float yBall = isOverhandShooter ? (hubY + rPath * std::cos(theta)) : (hubY - rPath * std::cos(theta));
                        glm::vec3 ballPos(
                            0.0f,
                            yBall,
                            hubZ + rPath * std::sin(theta)
                        );

                        float squish = static_cast<float>(currentSqueezeDepthM);

                        auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                        if (t)
                        {
                            t->position = ballPos;
                            t->scale = glm::vec3(2.0f * ballRadius + 0.3f * squish,
                                std::max(0.04f, 2.0f * ballRadius - squish),
                                2.0f * ballRadius + 0.3f * squish);
                        }

                        if (p.trajectoryTrail.empty() || glm::distance(p.trajectoryTrail.back(), ballPos) > 0.035f)
                        {
                            p.trajectoryTrail.push_back(ballPos);
                            if (p.trajectoryTrail.size() > 120) { p.trajectoryTrail.erase(p.trajectoryTrail.begin()); }
                        }

                        // Muzzle clearance
                        if (p.progressM >= contactLengthM)
                        {
                            p.inSqueeze = false;

                            // In single-flywheel rolling contact, centerline speed is ~55% of surface speed
                            lastExitSpeedMPerS = p.velocityMPerS * 0.55;
                            currentFlightSpeedMPerS = lastExitSpeedMPerS;
                            lastActualEnergyTransferredJ = 0.5 * pieceMassKg * lastExitSpeedMPerS * lastExitSpeedMPerS;
                            lastRecoilImpulseNs = pieceMassKg * lastExitSpeedMPerS;

                            auto* gpPiece = activeScene->GetComponent<GamePieceComponent>(p.entity);
                            if (gpPiece) { gpPiece->state = GamePieceState::Launched; }

                            constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                            double currentRpm = fwTop->angularVelocityRadPerSec * radSToRpm;
                            lastRpmSag = initialShotRpm - currentRpm;

                            float exitPitchRad = endRad;
                            glm::vec3 launchDir = isOverhandShooter
                                ? glm::vec3(0.0f, -std::sin(exitPitchRad), std::cos(exitPitchRad))
                                : glm::vec3(0.0f, std::sin(exitPitchRad), std::cos(exitPitchRad));
                            float launchSpeed = static_cast<float>(lastExitSpeedMPerS);

                            float yExit = isOverhandShooter ? (hubY + rPath * std::cos(exitPitchRad)) : (hubY - rPath * std::cos(exitPitchRad));
                            glm::vec3 exitMuzzlePos = glm::vec3(
                                0.0f,
                                yExit,
                                hubZ + rPath * std::sin(exitPitchRad)
                            ) + launchDir * 0.05f;

                            if (activeJolt && activeJolt->IsInitialized())
                            {
                                auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
                                if (rb && !rb->bodyID.IsInvalid())
                                {
                                    JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                                    bi.SetMotionType(rb->bodyID, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                                    bi.SetPositionAndRotation(rb->bodyID,
                                        JPH::RVec3(exitMuzzlePos.x, exitMuzzlePos.y, exitMuzzlePos.z),
                                        JPH::Quat::sIdentity(), JPH::EActivation::Activate);

                                    bi.SetLinearVelocity(rb->bodyID,
                                        JPH::Vec3(launchDir.x * launchSpeed, launchDir.y * launchSpeed, launchDir.z * launchSpeed));

                                    // Single-flywheel rolling contact:
                                    // Underhand (bottom hood, top roller) produces backspin (negative omega_x)
                                    // Overhand (top hood, bottom roller) produces topspin (positive omega_x)
                                    float spinSign = isOverhandShooter ? +1.0f : -1.0f;
                                    float spinRadS = spinSign * static_cast<float>(launchSpeed / ballRadius) * ballSpinMultiplier;
                                    bi.SetAngularVelocity(rb->bodyID, JPH::Vec3(spinRadS, 0.0f, 0.0f));
                                    p.spinRadPerSec = std::abs(spinRadS);
                                    constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                                    currentBallSpinRpm = spinRadS * radSToRpm;

                                    bi.ActivateBody(rb->bodyID);
                                }
                            }

                            if (t) { t->scale = glm::vec3(2.0f * ballRadius); }
                        }
                    }
                    else
                    {
                        auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                        auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);

                        if (activeJolt && rb && !rb->bodyID.IsInvalid() && t)
                        {
                            JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                            JPH::Vec3 jVel = bi.GetLinearVelocity(rb->bodyID);
                            JPH::Vec3 jOmega = bi.GetAngularVelocity(rb->bodyID);
                            glm::vec3 linVel(jVel.GetX(), jVel.GetY(), jVel.GetZ());
                            glm::vec3 angVel(jOmega.GetX(), jOmega.GetY(), jOmega.GetZ());
                            float speed = glm::length(linVel);
                            float spinMag = glm::length(angVel);

                            if (!p.hasHitGround)
                            {
                                currentFlightSpeedMPerS = static_cast<double>(speed);
                            }

                            if (p.trajectoryTrail.empty() || glm::distance(p.trajectoryTrail.back(), t->position) > 0.04f)
                            {
                                p.trajectoryTrail.push_back(t->position);
                                if (p.trajectoryTrail.size() > 120) { p.trajectoryTrail.erase(p.trajectoryTrail.begin()); }
                            }

                            constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                            currentBallSpinRpm = angVel.x * radSToRpm;
                            p.spinRadPerSec = spinMag;

                            // 1. Aerodynamic Air Drag & Magnus Effect Lift during flight
                            if (enableSpinPhysics && speed > 0.1f && t->position.y > 0.075f)
                            {
                                constexpr float rho = 1.225f; // kg/m^3 air density
                                constexpr float area = 3.14159265f * ballRadius * ballRadius;

                                // Quadratic air resistance drag: F_drag = -0.5 * rho * C_d * A * v^2
                                glm::vec3 dragForce = -0.5f * rho * 0.40f * area * speed * linVel;

                                // Magnus lift force: F_magnus = 0.5 * rho * C_L * A * v^2 * (omega x v) / (|omega| * |v|)
                                glm::vec3 crossW = glm::cross(angVel, linVel);
                                float spinParam = (spinMag * ballRadius) / std::max(speed, 0.1f);
                                float cLift = std::min(0.50f, 1.25f * spinParam);
                                glm::vec3 magnusForce = (spinMag > 0.1f)
                                    ? (0.5f * rho * cLift * area * speed * (crossW / spinMag))
                                    : glm::vec3(0.0f);

                                lastMagnusLiftN = magnusForce.y;
                                glm::vec3 netAirForce = dragForce + magnusForce;
                                bi.AddForce(rb->bodyID, JPH::Vec3(netAirForce.x, netAirForce.y, netAirForce.z));

                                // Aerodynamic skin friction torque (slowly dampens spin in air)
                                bi.AddTorque(rb->bodyID, JPH::Vec3(-0.00015f * angVel.x, -0.00015f * angVel.y, -0.00015f * angVel.z));
                            }

                            // 2. Ground Contact Dynamics: Spin Friction "Bite" & Traction Deceleration
                            if (t->position.y <= 0.082f && t->position.z > 0.35f)
                            {
                                if (!p.hasHitGround)
                                {
                                    p.hasHitGround = true;
                                    p.landingPos = t->position;
                                    lastLandingPos = t->position;
                                    lastShotDistanceM = std::sqrt(t->position.x * t->position.x + t->position.z * t->position.z);
                                    hasLandingRecorded = true;
                                    p.preImpactSpeedMPerS = glm::length(glm::vec2(linVel.x, linVel.z));
                                    lastPreImpactSpeedMPerS = p.preImpactSpeedMPerS;
                                    lastImpactSpinRpm = angVel.x * radSToRpm;
                                }

                                // Bottom contact patch slip velocity: v_slip = v_lin + omega x (0, -R, 0)
                                glm::vec3 vSlip(linVel.x + angVel.z * ballRadius, 0.0f, linVel.z - angVel.x * ballRadius);
                                float slipSpeed = glm::length(vSlip);

                                if (enableSpinPhysics && slipSpeed > 0.02f)
                                {
                                    // Contact normal impulse during ground compression / bounce
                                    float normalImpulse = pieceMassKg * (std::max(0.0f, -linVel.y) * 1.5f + 9.81f * static_cast<float>(dt) * 2.5f);
                                    float maxFrictionImpulse = groundBiteFriction * normalImpulse;

                                    // Target friction impulse to stop contact slip: J_roll = (2/5) * m * vSlip
                                    float targetImpulse = std::min(maxFrictionImpulse, 0.40f * pieceMassKg * slipSpeed);
                                    glm::vec3 slipDir = vSlip / slipSpeed;
                                    glm::vec3 jFriction = -slipDir * targetImpulse;

                                    // Linear braking impulse (slashes forward speed on landing)
                                    bi.AddImpulse(rb->bodyID, JPH::Vec3(jFriction.x, 0.0f, jFriction.z));

                                    // Angular torque impulse: converts backspin to rolling contact
                                    glm::vec3 angImpulse(-ballRadius * jFriction.z, 0.0f, ballRadius * jFriction.x);
                                    bi.AddAngularImpulse(rb->bodyID, JPH::Vec3(angImpulse.x, angImpulse.y, angImpulse.z));

                                    float postSpeed = glm::length(glm::vec2(linVel.x + jFriction.x / pieceMassKg, linVel.z + jFriction.z / pieceMassKg));
                                    p.postImpactSpeedMPerS = postSpeed;
                                    lastPostImpactSpeedMPerS = postSpeed;
                                }
                            }

                            // 3. Visual Spin Indicator (3D rotating seam bands & spin axis arrow)
                            {
                                glm::quat q = t->rotation;
                                glm::vec3 pos = t->position;
                                constexpr int kRingSegments = 12;
                                constexpr float stepAngle = (2.0f * 3.14159265f) / static_cast<float>(kRingSegments);

                                // Equatorial seam band (rotates in 3D with the ball orientation)
                                for (int s = 0; s < kRingSegments; ++s)
                                {
                                    float a1 = s * stepAngle;
                                    float a2 = (s + 1) * stepAngle;
                                    glm::vec3 p1 = pos + q * glm::vec3(ballRadius * std::cos(a1), ballRadius * std::sin(a1), 0.0f);
                                    glm::vec3 p2 = pos + q * glm::vec3(ballRadius * std::cos(a2), ballRadius * std::sin(a2), 0.0f);
                                    Utils::DebugDraw::Line(p1, p2, glm::vec3(0.95f, 0.95f, 0.95f), true);
                                }

                                // Meridian seam band (perpendicular contrast band)
                                for (int s = 0; s < kRingSegments; ++s)
                                {
                                    float a1 = s * stepAngle;
                                    float a2 = (s + 1) * stepAngle;
                                    glm::vec3 p1 = pos + q * glm::vec3(0.0f, ballRadius * std::cos(a1), ballRadius * std::sin(a1));
                                    glm::vec3 p2 = pos + q * glm::vec3(0.0f, ballRadius * std::cos(a2), ballRadius * std::sin(a2));
                                    Utils::DebugDraw::Line(p1, p2, glm::vec3(0.15f, 0.15f, 0.20f), true);
                                }

                                // Magenta spin vector arrow
                                if (spinMag > 15.0f)
                                {
                                    glm::vec3 axisDir = angVel / spinMag;
                                    Utils::DebugDraw::Line(pos, pos + axisDir * 0.12f, glm::vec3(0.95f, 0.25f, 0.85f), false);
                                }
                            }
                        }

                        if (p.lifeTimeS > 15.0)
                        {
                            p.active = false;
                            if (t) { t->position = glm::vec3(0.0f, -10.0f, 0.0f); }
                        }
                    }
                }
            }
            else if (activeSceneType == PlaygroundSceneType::NoteShooterDual)
            {
                float pitchRad = glm::radians(notePitchDeg);
                glm::vec3 launchDir(0.0f, std::sin(pitchRad), std::cos(pitchRad));

                for (size_t i = 0; i < kMaxProjectiles; ++i)
                {
                    auto& p = projectilePool[i];
                    if (!p.active) continue;

                    p.lifeTimeS += dt;

                    if (p.inSqueeze && fwTop)
                    {
                        anyBallInSqueeze = true;
                        shotTransitTimeMs += dt * 1000.0;
                        currentFlightSpeedMPerS = p.velocityMPerS;

                        SqueezeParameters params;
                        params.stiffnessK = complianceK;
                        params.restitutionE = restitutionE;
                        params.frictionMu = frictionMu;
                        params.hoodFrictionMu = 0.0;
                        params.rollerRadiusM = fwTop->rollerRadiusM;
                        params.nominalGapM = nominalGapM;
                        params.uncompressedSizeM = uncompressedSizeM;
                        params.contactLengthM = contactLengthM;
                        params.pieceMassKg = pieceMassKg;
                        params.isDualFlywheel = true;

                        double vRoller = fwTop->angularVelocityRadPerSec * params.rollerRadiusM;

                        SqueezeState state;
                        state.progressM = p.progressM;
                        state.velocityMPerS = p.velocityMPerS;

                        ShooterSqueezeModel::StepSqueeze(params, state, vRoller, dt, fwTop->subSteps);

                        p.progressM = state.progressM;
                        p.velocityMPerS = state.velocityMPerS;
                        currentSqueezeDepthM = state.compressionDepthM;
                        currentNormalForceN = state.normalForceN;
                        currentTractionForceN = state.tractionForceN;

                        peakNormalForceN = std::max(peakNormalForceN, state.normalForceN);
                        peakTractionForceN = std::max(peakTractionForceN, state.tractionForceN);

                        // Dual reaction torque on top and bottom flywheels
                        fwTop->externalLoadTorqueNm += 0.5 * state.loadTorqueNm;
                        if (fwBottom) { fwBottom->externalLoadTorqueNm += 0.5 * state.loadTorqueNm; }

                        float zProgress = static_cast<float>(-0.5 * contactLengthM + p.progressM);
                        glm::vec3 notePos(0.0f, hubY + zProgress * std::sin(pitchRad), zProgress * std::cos(pitchRad));

                        auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                        if (t)
                        {
                            t->position = notePos;
                            t->rotation = glm::angleAxis(-pitchRad, glm::vec3(1.0f, 0.0f, 0.0f));
                            float squish = static_cast<float>(currentSqueezeDepthM);
                            t->scale = glm::vec3(0.3556f + 0.3f * squish, std::max(0.02f, 0.0508f - squish), 0.3556f + 0.3f * squish);
                        }

                        if (p.trajectoryTrail.empty() || glm::distance(p.trajectoryTrail.back(), notePos) > 0.035f)
                        {
                            p.trajectoryTrail.push_back(notePos);
                            if (p.trajectoryTrail.size() > 120) { p.trajectoryTrail.erase(p.trajectoryTrail.begin()); }
                        }

                        if (p.progressM >= contactLengthM)
                        {
                            p.inSqueeze = false;
                            // Dual counter-rotating traction launches Note at direct speed
                            lastExitSpeedMPerS = p.velocityMPerS;
                            currentFlightSpeedMPerS = lastExitSpeedMPerS;
                            lastActualEnergyTransferredJ = 0.5 * pieceMassKg * lastExitSpeedMPerS * lastExitSpeedMPerS;
                            lastRecoilImpulseNs = pieceMassKg * lastExitSpeedMPerS;

                            auto* gpPiece = activeScene->GetComponent<GamePieceComponent>(p.entity);
                            if (gpPiece) { gpPiece->state = GamePieceState::Launched; }

                            constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                            double currentRpm = fwTop->angularVelocityRadPerSec * radSToRpm;
                            lastRpmSag = initialShotRpm - currentRpm;

                            float launchSpeed = static_cast<float>(lastExitSpeedMPerS);
                            glm::vec3 exitMuzzlePos = glm::vec3(0.0f, hubY, 0.0f) + launchDir * (0.5f * static_cast<float>(contactLengthM) + 0.06f);

                            if (activeJolt && activeJolt->IsInitialized())
                            {
                                auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
                                if (rb && !rb->bodyID.IsInvalid())
                                {
                                    JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                                    bi.SetMotionType(rb->bodyID, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                                    bi.SetPositionAndRotation(rb->bodyID,
                                        JPH::RVec3(exitMuzzlePos.x, exitMuzzlePos.y, exitMuzzlePos.z),
                                        JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -pitchRad), JPH::EActivation::Activate);

                                    bi.SetLinearVelocity(rb->bodyID,
                                        JPH::Vec3(launchDir.x * launchSpeed, launchDir.y * launchSpeed, launchDir.z * launchSpeed));

                                    // Dual counter-rotating rollers produce net zero pitch torque: flat flight along pitch plane
                                    bi.SetAngularVelocity(rb->bodyID, JPH::Vec3(0.0f, 0.0f, 0.0f));

                                    bi.ActivateBody(rb->bodyID);
                                }
                            }

                            if (t) { t->scale = glm::vec3(0.3556f, 0.0508f, 0.3556f); }
                        }
                    }
                    else
                    {
                        auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                        auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
                        if (activeJolt && rb && !rb->bodyID.IsInvalid() && t)
                        {
                            JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                            JPH::Vec3 jVel = bi.GetLinearVelocity(rb->bodyID);
                            float speed = glm::length(glm::vec3(jVel.GetX(), jVel.GetY(), jVel.GetZ()));

                            if (!p.hasHitGround)
                            {
                                currentFlightSpeedMPerS = static_cast<double>(speed);
                            }

                            if (p.trajectoryTrail.empty() || glm::distance(p.trajectoryTrail.back(), t->position) > 0.04f)
                            {
                                p.trajectoryTrail.push_back(t->position);
                                if (p.trajectoryTrail.size() > 120) { p.trajectoryTrail.erase(p.trajectoryTrail.begin()); }
                            }

                            if (!p.hasHitGround && t->position.y <= 0.06f && t->position.z > 0.4f)
                            {
                                p.hasHitGround = true;
                                p.landingPos = t->position;
                                lastLandingPos = t->position;
                                lastShotDistanceM = std::sqrt(t->position.x * t->position.x + t->position.z * t->position.z);
                                hasLandingRecorded = true;
                                currentFlightSpeedMPerS = static_cast<double>(speed);

                                auto* gpPiece = activeScene->GetComponent<GamePieceComponent>(p.entity);
                                if (gpPiece) { gpPiece->state = GamePieceState::Free; }
                            }

                            if (p.lifeTimeS > 15.0)
                            {
                                p.active = false;
                                if (t) { t->position = glm::vec3(0.0f, -10.0f, 0.0f); }
                            }
                        }
                    }
                }
            }

            if (cz) { cz->isEngaged = anyBallInSqueeze; }
            pieceActive = anyBallInSqueeze;

            // Trajectory Aiming Laser & Ballistic Prediction Arc
            glm::vec3 launchDir;
            glm::vec3 muzzlePos;
            if (activeSceneType == PlaygroundSceneType::BallShooterHood)
            {
                float endRad = glm::radians(hoodEndAngleDeg);
                float rPath = rollerRadiusM + 0.5f * nominalGapM;
                if (isOverhandShooter)
                {
                    launchDir = glm::vec3(0.0f, -std::sin(endRad), std::cos(endRad));
                    muzzlePos = glm::vec3(0.0f, hubY + rPath * std::cos(endRad), hubZ + rPath * std::sin(endRad));
                }
                else
                {
                    launchDir = glm::vec3(0.0f, std::sin(endRad), std::cos(endRad));
                    muzzlePos = glm::vec3(0.0f, hubY - rPath * std::cos(endRad), hubZ + rPath * std::sin(endRad));
                }
            }
            else
            {
                float pitchRad = glm::radians(notePitchDeg);
                launchDir = glm::vec3(0.0f, std::sin(pitchRad), std::cos(pitchRad));
                muzzlePos = glm::vec3(0.0f, hubY, 0.0f) + launchDir * (0.5f * static_cast<float>(contactLengthM) + 0.06f);
            }

            // Muzzle pointer laser line
            Utils::DebugDraw::Line(muzzlePos, muzzlePos + launchDir * 1.5f, glm::vec3(0.1f, 1.0f, 0.3f), false);

            // Ballistic Arc (28 segments)
            double currentRollerSpeed = fwTop ? (fwTop->angularVelocityRadPerSec * fwTop->rollerRadiusM) : 15.0;
            float defaultSpeed = static_cast<float>(currentRollerSpeed * (activeSceneType == PlaygroundSceneType::NoteShooterDual ? 0.95 : 0.55));
            float estSpeed = (lastExitSpeedMPerS > 2.0) ? static_cast<float>(lastExitSpeedMPerS) : defaultSpeed;
            glm::vec3 prevPoint = muzzlePos;
            constexpr float dtSim = 0.045f;
            for (int step = 1; step <= 28; ++step)
            {
                float tSim = step * dtSim;
                glm::vec3 nextPoint = muzzlePos + launchDir * (estSpeed * tSim) + glm::vec3(0.0f, -0.5f * 9.81f * tSim * tSim, 0.0f);
                if (nextPoint.y < 0.02f)
                {
                    Utils::DebugDraw::Line(prevPoint, glm::vec3(nextPoint.x, 0.02f, nextPoint.z), glm::vec3(0.9f, 0.8f, 0.2f), true);
                    break;
                }
                Utils::DebugDraw::Line(prevPoint, nextPoint, glm::vec3(0.1f, 0.9f, 0.4f), true);
                prevPoint = nextPoint;
            }

            // Record Telemetry Ring Buffers
            if (fwTop)
            {
                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                rpmHistory[historyIndex] = static_cast<float>(fwTop->angularVelocityRadPerSec * radSToRpm);
                torqueHistory[historyIndex] = static_cast<float>(fwTop->recoilTorqueNm);
                forceHistory[historyIndex] = static_cast<float>(currentNormalForceN);
                historyIndex = (historyIndex + 1) % kHistorySize;
            }

            // -----------------------------------------------------------------
            // Block 6 3D Visual Overlays: Trails, Lasers, Contact Zones & Vectors
            // -----------------------------------------------------------------
            if (showTrajectoryTrails)
            {
                glm::vec3 trailColor = (activeSceneType == PlaygroundSceneType::BallShooterHood)
                    ? glm::vec3(0.15f, 0.85f, 1.0f)   // Luminous Cyan
                    : glm::vec3(1.0f, 0.55f, 0.15f);  // Luminous Orange/Gold

                for (size_t i = 0; i < kMaxProjectiles; ++i)
                {
                    const auto& p = projectilePool[i];
                    if (!p.active || p.trajectoryTrail.size() < 2) continue;
                    for (size_t pt = 0; pt + 1 < p.trajectoryTrail.size(); ++pt)
                    {
                        Utils::DebugDraw::Line(p.trajectoryTrail[pt], p.trajectoryTrail[pt + 1], trailColor, false);
                    }
                }
            }

            if (showBeamBreakLasers)
            {
                // 1. Shooter Throat Beam Break (DIO 2)
                auto* czShooter = activeScene ? activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity) : nullptr;
                auto* tCzShooter = activeScene ? activeScene->GetComponent<TransformComponent>(contactZoneEntity) : nullptr;
                if (czShooter && tCzShooter)
                {
                    bool tripped = czShooter->isEngaged;
                    glm::vec3 laserCol = tripped ? glm::vec3(1.0f, 0.15f, 0.15f) : glm::vec3(0.10f, 1.0f, 0.35f);
                    float halfWidth = std::max(0.14f, czShooter->dimensions.x * 0.5f);
                    glm::vec3 pEmitter = tCzShooter->position + tCzShooter->rotation * glm::vec3(-halfWidth, 0.0f, 0.0f);
                    glm::vec3 pReceiver = tCzShooter->position + tCzShooter->rotation * glm::vec3(halfWidth, 0.0f, 0.0f);
                    Utils::DebugDraw::Line(pEmitter, pReceiver, laserCol, false);
                    Utils::DebugDraw::Sphere(pEmitter, 0.012f, laserCol, 8, false);
                    Utils::DebugDraw::Sphere(pReceiver, 0.012f, laserCol, 8, false);
                }

                // 2. Intake Feeder Beam Break (DIO 3)
                auto* czIn = activeScene ? activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity) : nullptr;
                auto* tCzIn = activeScene ? activeScene->GetComponent<TransformComponent>(intakeZoneEntity) : nullptr;
                if (czIn && tCzIn)
                {
                    bool trippedIn = czIn->isEngaged;
                    glm::vec3 laserColIn = trippedIn ? glm::vec3(1.0f, 0.15f, 0.15f) : glm::vec3(0.10f, 1.0f, 0.35f);
                    float halfWidthIn = std::max(0.18f, czIn->dimensions.x * 0.5f);
                    glm::vec3 pEmitterIn = tCzIn->position + tCzIn->rotation * glm::vec3(-halfWidthIn, 0.0f, 0.0f);
                    glm::vec3 pReceiverIn = tCzIn->position + tCzIn->rotation * glm::vec3(halfWidthIn, 0.0f, 0.0f);
                    Utils::DebugDraw::Line(pEmitterIn, pReceiverIn, laserColIn, false);
                    Utils::DebugDraw::Sphere(pEmitterIn, 0.012f, laserColIn, 8, false);
                    Utils::DebugDraw::Sphere(pReceiverIn, 0.012f, laserColIn, 8, false);
                }
            }

            if (showContactZones)
            {
                auto* czShooter = activeScene ? activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity) : nullptr;
                auto* tCzShooter = activeScene ? activeScene->GetComponent<TransformComponent>(contactZoneEntity) : nullptr;
                if (czShooter && tCzShooter)
                {
                    glm::vec3 czCol = czShooter->isEngaged ? glm::vec3(1.0f, 0.35f, 0.10f) : glm::vec3(0.90f, 0.65f, 0.15f);
                    DrawOrientedBox(tCzShooter->position, tCzShooter->rotation, czShooter->dimensions * 0.5f, czCol);
                }

                auto* czIn = activeScene ? activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity) : nullptr;
                auto* tCzIn = activeScene ? activeScene->GetComponent<TransformComponent>(intakeZoneEntity) : nullptr;
                if (czIn && tCzIn)
                {
                    glm::vec3 czInCol = czIn->isEngaged ? glm::vec3(1.0f, 0.90f, 0.20f) : glm::vec3(0.15f, 0.85f, 0.85f);
                    DrawOrientedBox(tCzIn->position, tCzIn->rotation, czIn->dimensions * 0.5f, czInCol);
                }
            }

            if (showAeroForceVectors)
            {
                for (size_t i = 0; i < kMaxProjectiles; ++i)
                {
                    const auto& p = projectilePool[i];
                    if (!p.active || p.inSqueeze || p.hasHitGround) continue;
                    auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                    auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
                    if (t && rb && !rb->bodyID.IsInvalid() && activeJolt)
                    {
                        JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                        JPH::Vec3 jv = bi.GetLinearVelocity(rb->bodyID);
                        glm::vec3 vel(jv.GetX(), jv.GetY(), jv.GetZ());
                        Utils::DebugDraw::Line(t->position, t->position + vel * 0.08f, glm::vec3(0.2f, 1.0f, 0.3f), false);
                        if (std::abs(lastMagnusLiftN) > 0.05)
                        {
                            Utils::DebugDraw::Line(t->position, t->position + glm::vec3(0.0f, static_cast<float>(lastMagnusLiftN) * 0.25f, 0.0f), glm::vec3(1.0f, 0.85f, 0.15f), false);
                        }
                    }
                }
            }
        }

        // ---------------------------------------------------------------------
        // 3. Render Ground Meter Distance Lines
        // ---------------------------------------------------------------------
        if (activeSceneType == PlaygroundSceneType::BallShooterHood || activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            // Sidelines along shooting alley
            Utils::DebugDraw::Line(glm::vec3(-4.0f, 0.005f, 0.0f), glm::vec3(-4.0f, 0.005f, 22.0f), glm::vec3(0.4f, 0.5f, 0.6f), true);
            Utils::DebugDraw::Line(glm::vec3(+4.0f, 0.005f, 0.0f), glm::vec3(+4.0f, 0.005f, 22.0f), glm::vec3(0.4f, 0.5f, 0.6f), true);
            Utils::DebugDraw::Line(glm::vec3(0.0f, 0.005f, 0.0f), glm::vec3(0.0f, 0.005f, 22.0f), glm::vec3(0.2f, 0.3f, 0.4f), true);

            // Transverse Meter Lines
            for (int m = 1; m <= 20; ++m)
            {
                float z = static_cast<float>(m);
                bool isMajor = (m % 5 == 0);
                glm::vec3 lineColor = isMajor ? glm::vec3(0.95f, 0.82f, 0.15f) : glm::vec3(0.28f, 0.38f, 0.48f);
                float halfWidth = isMajor ? 4.5f : 3.8f;

                Utils::DebugDraw::Line(glm::vec3(-halfWidth, 0.005f, z), glm::vec3(halfWidth, 0.005f, z), lineColor, true);

                // Tick marks on major lines
                if (isMajor)
                {
                    Utils::DebugDraw::Line(glm::vec3(-halfWidth, 0.005f, z - 0.15f), glm::vec3(-halfWidth, 0.005f, z + 0.15f), lineColor, true);
                    Utils::DebugDraw::Line(glm::vec3(+halfWidth, 0.005f, z - 0.15f), glm::vec3(+halfWidth, 0.005f, z + 0.15f), lineColor, true);
                    Utils::DebugDraw::Line(glm::vec3(0.0f, 0.005f, z - 0.15f), glm::vec3(0.0f, 0.005f, z + 0.15f), lineColor, true);
                }
            }

            // Draw landing marker if available
            if (hasLandingRecorded)
            {
                glm::vec3 lp = lastLandingPos;
                lp.y = 0.015f;
                // Crosshair at impact point
                Utils::DebugDraw::Line(lp + glm::vec3(-0.35f, 0.0f, -0.35f), lp + glm::vec3(0.35f, 0.0f, 0.35f), glm::vec3(1.0f, 0.2f, 0.2f), false);
                Utils::DebugDraw::Line(lp + glm::vec3(-0.35f, 0.0f, 0.35f), lp + glm::vec3(0.35f, 0.0f, -0.35f), glm::vec3(1.0f, 0.2f, 0.2f), false);
            }
        }

        // Step Shared Subsystems
        if (flywheelSystem) { flywheelSystem->FixedUpdate(dt); }
        if (sensorSystem) { sensorSystem->FixedUpdate(dt); }
        if (jointSystem) { jointSystem->FixedUpdate(dt); }
        if (physicsSystem) { physicsSystem->FixedUpdate(dt); }
    }

    void PlaygroundScene::FireGamePiece()
    {
        auto* fwTop = activeScene ? activeScene->GetComponent<FlywheelComponent>(topFlywheelEntity) : nullptr;
        constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
        initialShotRpm = fwTop ? (fwTop->angularVelocityRadPerSec * radSToRpm) : 0.0;

        size_t idx = nextProjectileIdx;
        nextProjectileIdx = (nextProjectileIdx + 1) % kMaxProjectiles;

        auto& p = projectilePool[idx];
        p.active = true;
        p.inSqueeze = true;
        p.progressM = 0.0;
        p.velocityMPerS = 2.5; // Enters throat at 2.5 m/s
        p.lifeTimeS = 0.0;
        p.hasHitGround = false;
        p.trajectoryTrail.clear();

        // Calculate theoretical 1D squeeze predictions & energy balance oracle
        double vRoller = fwTop ? (fwTop->angularVelocityRadPerSec * fwTop->rollerRadiusM) : 0.0;
        if (activeSceneType == PlaygroundSceneType::BallShooterHood)
        {
            lastTheoreticalExitSpeedMPerS = 0.55 * vRoller;
        }
        else
        {
            lastTheoreticalExitSpeedMPerS = 0.95 * vRoller; // Dual flywheel with ~5% slip
        }

        double ekLin = 0.5 * pieceMassKg * lastTheoreticalExitSpeedMPerS * lastTheoreticalExitSpeedMPerS;
        double ekRot = (activeSceneType == PlaygroundSceneType::BallShooterHood)
            ? (0.5 * 0.4 * pieceMassKg * 0.065 * 0.065 * std::pow(lastTheoreticalExitSpeedMPerS / 0.065, 2))
            : 0.0;
        double totalEk = ekLin + ekRot;
        double slipLoss = 0.25 * totalEk;
        double deltaEfw = totalEk + slipLoss;
        double J_total = (activeSceneType == PlaygroundSceneType::NoteShooterDual)
            ? (2.0 * flywheelInertiaKgm2)
            : flywheelInertiaKgm2;
        double w0 = fwTop ? fwTop->angularVelocityRadPerSec : 0.0;
        double w1 = std::sqrt(std::max(0.0, w0 * w0 - (2.0 * deltaEfw / std::max(1e-6, J_total))));
        lastTheoreticalRpmSag = (w0 - w1) * radSToRpm;
        lastTheoreticalEnergyTransferredJ = totalEk;
        lastRecoilImpulseNs = pieceMassKg * lastTheoreticalExitSpeedMPerS;

        auto* gpPiece = activeScene ? activeScene->GetComponent<GamePieceComponent>(p.entity) : nullptr;
        if (gpPiece) { gpPiece->state = GamePieceState::Engaged; }

        pieceActive = true;
        hasShotFired = true;
        shotTransitTimeMs = 0.0;
        peakNormalForceN = 0.0;
        peakTractionForceN = 0.0;

        constexpr float hubY = 1.05f;
        constexpr float hubZ = 0.0f;

        if (activeSceneType == PlaygroundSceneType::BallShooterHood)
        {
            float rPath = rollerRadiusM + 0.5f * nominalGapM;
            float startRad = glm::radians(hoodStartAngleDeg);
            float yEntry = isOverhandShooter ? (hubY + rPath * std::cos(startRad)) : (hubY - rPath * std::cos(startRad));

            glm::vec3 entryPos(0.0f, yEntry, hubZ + rPath * std::sin(startRad));
            auto* t = activeScene ? activeScene->GetComponent<TransformComponent>(p.entity) : nullptr;
            if (t)
            {
                t->position = entryPos;
                t->scale = glm::vec3(2.0f * 0.065f);
            }
        }
        else if (activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            float pitchRad = glm::radians(notePitchDeg);
            glm::vec3 entryPos = glm::vec3(0.0f, hubY, 0.0f) - glm::vec3(0.0f, std::sin(pitchRad), std::cos(pitchRad)) * (0.5f * static_cast<float>(contactLengthM) + 0.08f);

            auto* t = activeScene ? activeScene->GetComponent<TransformComponent>(p.entity) : nullptr;
            if (t)
            {
                t->position = entryPos;
                t->rotation = glm::angleAxis(-pitchRad, glm::vec3(1.0f, 0.0f, 0.0f));
                t->scale = glm::vec3(0.3556f, 0.0508f, 0.3556f);
            }
        }

        if (activeJolt && activeJolt->IsInitialized())
        {
            auto* rb = activeScene ? activeScene->GetComponent<RigidBodyComponent>(p.entity) : nullptr;
            if (rb && !rb->bodyID.IsInvalid())
            {
                JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                bi.SetMotionType(rb->bodyID, JPH::EMotionType::Kinematic, JPH::EActivation::DontActivate);
                bi.SetLinearAndAngularVelocity(rb->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            }
        }
    }

    void PlaygroundScene::SpawnGroundGamePiece()
    {
        if (!activeScene) return;

        size_t idx = nextProjectileIdx;
        nextProjectileIdx = (nextProjectileIdx + 1) % kMaxProjectiles;

        auto& p = projectilePool[idx];
        p.active = true;
        p.inSqueeze = false;
        p.progressM = 0.0;
        p.velocityMPerS = 0.0;
        p.lifeTimeS = 0.0;
        p.hasHitGround = false;
        p.trajectoryTrail.clear();

        float spawnY = 0.065f;
        if (activeSceneType == PlaygroundSceneType::BallShooterHood)
        {
            constexpr float ballRadius = 0.065f;
            spawnY = ballRadius + 0.005f;
        }
        else if (activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            constexpr float noteThick = 0.0508f;
            spawnY = noteThick * 0.5f + 0.005f;
        }

        // Place on the floor in front of the intake roller
        glm::vec3 groundPos(0.0f, spawnY, -0.85f);
        p.trajectoryTrail.push_back(groundPos);

        auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
        if (t)
        {
            t->position = groundPos;
            t->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            if (activeSceneType == PlaygroundSceneType::BallShooterHood)
            {
                t->scale = glm::vec3(2.0f * 0.065f);
            }
            else
            {
                t->scale = glm::vec3(0.3556f, 0.0508f, 0.3556f);
            }
        }

        auto* gp = activeScene->GetComponent<GamePieceComponent>(p.entity);
        if (gp)
        {
            gp->state = GamePieceState::Free;
            gp->transitProgressM = 0.0;
            gp->transitVelocityMPerS = 0.0;
        }

        auto* rb = activeScene->GetComponent<RigidBodyComponent>(p.entity);
        if (rb)
        {
            rb->motionType = BodyMotionType::Dynamic;
            if (activeJolt && activeJolt->IsInitialized() && !rb->bodyID.IsInvalid())
            {
                JPH::BodyInterface& bi = activeJolt->GetBodyInterface();
                bi.SetMotionType(rb->bodyID, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                bi.SetPositionAndRotation(rb->bodyID,
                    JPH::RVec3(groundPos.x, groundPos.y, groundPos.z),
                    JPH::Quat::sIdentity(),
                    JPH::EActivation::Activate);
                // Roll gently forward into the intake zone
                bi.SetLinearVelocity(rb->bodyID, JPH::Vec3(0.0f, 0.0f, 0.45f));
                bi.SetAngularVelocity(rb->bodyID, JPH::Vec3(0.0f, 0.0f, 0.0f));
                bi.ActivateBody(rb->bodyID);
            }
        }
    }

    void PlaygroundScene::FeedStoredPiece()
    {
        if (storedPiecesCount == 0 && !autoFeedToShooter) return;

        if (storedPiecesCount > 0)
        {
            storedPiecesCount--;
        }

        FireGamePiece();
    }

    void PlaygroundScene::ResetWristPose()
    {
        manualVoltageA = 0.0f;
        manualVoltageB = 0.0f;
        controlMode = PlaygroundControlMode::BrakeHold;
        testTimeSeconds = 0.0;

        if (!activeScene || !activeJolt) return;

        JPH::BodyInterface& bi = activeJolt->GetBodyInterface();

        auto* rbYoke = activeScene->GetComponent<RigidBodyComponent>(pitchYokeEntity);
        if (rbYoke && !rbYoke->bodyID.IsInvalid())
        {
            bi.SetPositionAndRotation(rbYoke->bodyID,
                JPH::RVec3(0.0f, 1.00f, 0.05f),
                JPH::Quat::sIdentity(),
                JPH::EActivation::Activate);
            bi.SetLinearAndAngularVelocity(rbYoke->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        }

        auto* rbTool = activeScene->GetComponent<RigidBodyComponent>(rollToolEntity);
        if (rbTool && !rbTool->bodyID.IsInvalid())
        {
            bi.SetPositionAndRotation(rbTool->bodyID,
                JPH::RVec3(0.0f, 1.00f, 0.20f),
                JPH::Quat::sIdentity(),
                JPH::EActivation::Activate);
            bi.SetLinearAndAngularVelocity(rbTool->bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        }
    }

    void PlaygroundScene::DrawOrientedBox(const glm::vec3& center, const glm::quat& rot, const glm::vec3& halfExtents, const glm::vec3& color)
    {
        glm::vec3 corners[8];
        for (int i = 0; i < 8; ++i)
        {
            glm::vec3 localOffset(
                (i & 1 ? 1.0f : -1.0f) * halfExtents.x,
                (i & 2 ? 1.0f : -1.0f) * halfExtents.y,
                (i & 4 ? 1.0f : -1.0f) * halfExtents.z
            );
            corners[i] = center + rot * localOffset;
        }

        // 12 edges
        Utils::DebugDraw::Line(corners[0], corners[1], color, true);
        Utils::DebugDraw::Line(corners[2], corners[3], color, true);
        Utils::DebugDraw::Line(corners[4], corners[5], color, true);
        Utils::DebugDraw::Line(corners[6], corners[7], color, true);

        Utils::DebugDraw::Line(corners[0], corners[2], color, true);
        Utils::DebugDraw::Line(corners[1], corners[3], color, true);
        Utils::DebugDraw::Line(corners[4], corners[6], color, true);
        Utils::DebugDraw::Line(corners[5], corners[7], color, true);

        Utils::DebugDraw::Line(corners[0], corners[4], color, true);
        Utils::DebugDraw::Line(corners[1], corners[5], color, true);
        Utils::DebugDraw::Line(corners[2], corners[6], color, true);
        Utils::DebugDraw::Line(corners[3], corners[7], color, true);
    }

    void PlaygroundScene::RenderBallisticOracleHUD(bool isBallShooter)
    {
        if (ImGui::CollapsingHeader("Block 6 Ballistic Shot Oracle & Energy Balance", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // 1. Muzzle Exit Speed Oracle
            ImGui::TextColored(ImVec4(0.15f, 0.85f, 0.95f, 1.0f), "Kinematic Exit Velocity Oracle:");
            ImGui::BulletText("Expected (1D Squeeze Model): %.2f m/s (%.1f ft/s)",
                lastTheoreticalExitSpeedMPerS, lastTheoreticalExitSpeedMPerS * 3.28084);
            ImGui::BulletText("Actual Measured Exit Speed:  %.2f m/s (%.1f ft/s)",
                lastExitSpeedMPerS, lastExitSpeedMPerS * 3.28084);

            if (lastTheoreticalExitSpeedMPerS > 1.0 && lastExitSpeedMPerS > 1.0)
            {
                double speedDelta = std::abs(lastExitSpeedMPerS - lastTheoreticalExitSpeedMPerS);
                double pctErr = (speedDelta / lastTheoreticalExitSpeedMPerS) * 100.0;
                ImGui::Text("  Speed Accuracy Error: ");
                ImGui::SameLine();
                if (pctErr <= 5.0)
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "%.1f%%  [ PASS: <= 5%% Error ]", pctErr);
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.15f, 1.0f), "%.1f%%  [ SLIP COMPLIANCE CALIBRATION ]", pctErr);
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // 2. Flywheel RPM Sag & Energy Balance
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.10f, 1.0f), "Flywheel RPM Sag & Energy Balance:");
            ImGui::BulletText("Theoretical Energy Sag: %.1f RPM", lastTheoreticalRpmSag);
            ImGui::BulletText("Actual Measured RPM Sag: %.1f RPM", lastRpmSag);

            if (lastTheoreticalRpmSag > 5.0 && lastRpmSag > 5.0)
            {
                double sagDelta = std::abs(lastRpmSag - lastTheoreticalRpmSag);
                double sagPctErr = (sagDelta / lastTheoreticalRpmSag) * 100.0;
                ImGui::Text("  RPM Sag Accuracy Error: ");
                ImGui::SameLine();
                if (sagPctErr <= 10.0)
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "%.1f%%  [ PASS: <= 10%% Energy Balance ]", sagPctErr);
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.15f, 1.0f), "%.1f%%  [ DISSIPATIVE HYSTERESIS ]", sagPctErr);
                }
            }

            ImGui::BulletText("Projectile Kinetic Energy: %.2f J", lastActualEnergyTransferredJ);
            ImGui::BulletText("Theoretical Transfer Work: %.2f J", lastTheoreticalEnergyTransferredJ);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // 3. Chassis Recoil & Aerodynamics
            ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.90f, 1.0f), "Chassis Recoil & Flight Dynamics:");
            ImGui::BulletText("Linear Recoil Impulse: %.3f N*s (J = -m * v_exit)", lastRecoilImpulseNs);
            if (isBallShooter)
            {
                ImGui::BulletText("Peak Magnus Rotational Lift: %.2f N", lastMagnusLiftN);
                ImGui::BulletText("Ball Spin Aerodynamics: Quadratic Air Drag & Non-Singular Magnus");
            }
            else
            {
                ImGui::BulletText("Flat Flight Plane: Net-Zero Differential Pitch Torque");
            }
            ImGui::BulletText("CCD Muzzle Masking: 3 Ticks (15 ms) Collision Immunity");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // 4. Interactive 3D Overlays & HUD Toggles
            ImGui::TextColored(ImVec4(0.15f, 0.85f, 0.95f, 1.0f), "3D Viewport Diagnostics & Overlays:");
            ImGui::Checkbox("3D Trajectory Trails", &showTrajectoryTrails);
            ImGui::SameLine();
            ImGui::Checkbox("Beam-Break Optical Lasers", &showBeamBreakLasers);
            ImGui::Checkbox("Contact Zones 3D Wireframes", &showContactZones);
            ImGui::SameLine();
            ImGui::Checkbox("Piece State Badges", &showGamePieceStateBadges);
            ImGui::Checkbox("Aerodynamic Force Vectors", &showAeroForceVectors);
        }
    }

    void PlaygroundScene::RenderDistanceFieldHUD()
    {
        if (!activeCamera || (activeSceneType != PlaygroundSceneType::BallShooterHood && activeSceneType != PlaygroundSceneType::NoteShooterDual))
        {
            return;
        }

        ImVec2 displaySize = ImGui::GetIO().DisplaySize;
        if (displaySize.x <= 10.0f || displaySize.y <= 10.0f) return;

        float aspect = displaySize.x / displaySize.y;
        glm::mat4 matView = activeCamera->GetViewMatrix();
        glm::mat4 matProj = activeCamera->GetProjectionMatrix(aspect);
        glm::mat4 matVP = matProj * matView;

        auto ProjectPoint = [&](const glm::vec3& pWorld, ImVec2& outScreen) -> bool
        {
            glm::vec4 clip = matVP * glm::vec4(pWorld, 1.0f);
            if (clip.w <= 0.1f) return false;
            float ndcX = clip.x / clip.w;
            float ndcY = clip.y / clip.w;
            if (ndcX < -1.15f || ndcX > 1.15f || ndcY < -1.15f || ndcY > 1.15f) return false;
            outScreen.x = (ndcX * 0.5f + 0.5f) * displaySize.x;
            outScreen.y = (1.0f - (ndcY * 0.5f + 0.5f)) * displaySize.y;
            return true;
        };

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();

        // 1. Draw In-World Meter Distance Badges along Sideline
        for (int m = 1; m <= 20; ++m)
        {
            float z = static_cast<float>(m);
            bool isMajor = (m % 5 == 0);

            ImVec2 screenPos;
            if (ProjectPoint(glm::vec3(4.1f, 0.05f, z), screenPos))
            {
                char label[32];
                if (isMajor)
                {
                    std::snprintf(label, sizeof(label), " %d METERS ", m);
                    ImVec2 txtSize = ImGui::CalcTextSize(label);
                    drawList->AddRectFilled(ImVec2(screenPos.x - 2, screenPos.y - 2),
                        ImVec2(screenPos.x + txtSize.x + 2, screenPos.y + txtSize.y + 2),
                        IM_COL32(20, 24, 32, 220), 4.0f);
                    drawList->AddRect(ImVec2(screenPos.x - 2, screenPos.y - 2),
                        ImVec2(screenPos.x + txtSize.x + 2, screenPos.y + txtSize.y + 2),
                        IM_COL32(245, 210, 40, 255), 4.0f);
                    drawList->AddText(screenPos, IM_COL32(255, 235, 80, 255), label);
                }
                else
                {
                    std::snprintf(label, sizeof(label), "%dm", m);
                    ImVec2 txtSize = ImGui::CalcTextSize(label);
                    drawList->AddRectFilled(ImVec2(screenPos.x - 2, screenPos.y - 1),
                        ImVec2(screenPos.x + txtSize.x + 2, screenPos.y + txtSize.y + 1),
                        IM_COL32(15, 20, 26, 180), 3.0f);
                    drawList->AddText(screenPos, IM_COL32(140, 180, 210, 220), label);
                }
            }
        }

        // 2. Draw Last Impact Landing Marker Badge
        if (hasLandingRecorded)
        {
            ImVec2 screenImpact;
            if (ProjectPoint(lastLandingPos + glm::vec3(0.0f, 0.10f, 0.0f), screenImpact))
            {
                char impactText[64];
                std::snprintf(impactText, sizeof(impactText), " IMPACT: %.2f m (%.1f ft) ",
                    lastShotDistanceM, lastShotDistanceM * 3.28084f);

                ImVec2 txtSize = ImGui::CalcTextSize(impactText);
                ImVec2 boxMin(screenImpact.x - txtSize.x * 0.5f - 6.0f, screenImpact.y - txtSize.y - 14.0f);
                ImVec2 boxMax(screenImpact.x + txtSize.x * 0.5f + 6.0f, screenImpact.y - 4.0f);

                drawList->AddRectFilled(boxMin, boxMax, IM_COL32(180, 30, 30, 240), 5.0f);
                drawList->AddRect(boxMin, boxMax, IM_COL32(255, 220, 220, 255), 5.0f, 0, 1.5f);
                drawList->AddText(ImVec2(boxMin.x + 6.0f, boxMin.y + 2.0f), IM_COL32(255, 255, 255, 255), impactText);

                // Small arrow down to impact point
                drawList->AddTriangleFilled(
                    ImVec2(screenImpact.x - 5.0f, boxMax.y),
                    ImVec2(screenImpact.x + 5.0f, boxMax.y),
                    ImVec2(screenImpact.x, screenImpact.y),
                    IM_COL32(220, 30, 30, 255));
            }
        }

        // 3. Draw Optical Beam-Break Sensor Screen Badges
        if (showBeamBreakLasers)
        {
            // Shooter Beam Break DIO 2
            auto* czShooter = activeScene ? activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity) : nullptr;
            auto* tCzShooter = activeScene ? activeScene->GetComponent<TransformComponent>(contactZoneEntity) : nullptr;
            if (czShooter && tCzShooter)
            {
                ImVec2 scr;
                if (ProjectPoint(tCzShooter->position + glm::vec3(0.0f, 0.14f, 0.0f), scr))
                {
                    bool tripped = czShooter->isEngaged;
                    const char* tag = tripped ? " DIO 2: [TRIPPED] " : " DIO 2: [CLEAR] ";
                    ImVec2 txtSz = ImGui::CalcTextSize(tag);
                    ImVec2 bMin(scr.x - txtSz.x * 0.5f - 4.0f, scr.y - txtSz.y - 4.0f);
                    ImVec2 bMax(scr.x + txtSz.x * 0.5f + 4.0f, scr.y);
                    drawList->AddRectFilled(bMin, bMax, tripped ? IM_COL32(180, 25, 25, 230) : IM_COL32(20, 120, 50, 210), 3.0f);
                    drawList->AddRect(bMin, bMax, tripped ? IM_COL32(255, 100, 100, 255) : IM_COL32(80, 255, 120, 255), 3.0f);
                    drawList->AddText(ImVec2(bMin.x + 4.0f, bMin.y + 2.0f), IM_COL32(255, 255, 255, 255), tag);
                }
            }

            // Intake Feeder Beam Break DIO 3
            auto* czIntake = activeScene ? activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity) : nullptr;
            auto* tCzIntake = activeScene ? activeScene->GetComponent<TransformComponent>(intakeZoneEntity) : nullptr;
            if (czIntake && tCzIntake)
            {
                ImVec2 scr;
                if (ProjectPoint(tCzIntake->position + glm::vec3(0.0f, 0.14f, 0.0f), scr))
                {
                    bool tripped = czIntake->isEngaged;
                    const char* tag = tripped ? " DIO 3: [INTAKE TRIPPED] " : " DIO 3: [INTAKE READY] ";
                    ImVec2 txtSz = ImGui::CalcTextSize(tag);
                    ImVec2 bMin(scr.x - txtSz.x * 0.5f - 4.0f, scr.y - txtSz.y - 4.0f);
                    ImVec2 bMax(scr.x + txtSz.x * 0.5f + 4.0f, scr.y);
                    drawList->AddRectFilled(bMin, bMax, tripped ? IM_COL32(180, 25, 25, 230) : IM_COL32(20, 120, 50, 210), 3.0f);
                    drawList->AddRect(bMin, bMax, tripped ? IM_COL32(255, 100, 100, 255) : IM_COL32(80, 255, 120, 255), 3.0f);
                    drawList->AddText(ImVec2(bMin.x + 4.0f, bMin.y + 2.0f), IM_COL32(255, 255, 255, 255), tag);
                }
            }
        }

        // 4. Draw In-World Game Piece Lifecycle State Badges
        if (showGamePieceStateBadges)
        {
            for (size_t i = 0; i < kMaxProjectiles; ++i)
            {
                const auto& p = projectilePool[i];
                if (!p.active) continue;

                auto* t = activeScene->GetComponent<TransformComponent>(p.entity);
                auto* gp = activeScene->GetComponent<GamePieceComponent>(p.entity);
                if (!t) continue;

                ImVec2 scr;
                if (ProjectPoint(t->position + glm::vec3(0.0f, 0.16f, 0.0f), scr))
                {
                    char badgeText[64];
                    ImU32 bgCol = IM_COL32(30, 40, 50, 220);
                    ImU32 borderCol = IM_COL32(120, 160, 200, 255);

                    if (p.inSqueeze)
                    {
                        std::snprintf(badgeText, sizeof(badgeText), " ENGAGED: %.1f mm | %.0f N ", currentSqueezeDepthM * 1000.0, currentNormalForceN);
                        bgCol = IM_COL32(190, 80, 10, 230);
                        borderCol = IM_COL32(255, 180, 50, 255);
                    }
                    else if (gp && gp->state == GamePieceState::Intaking)
                    {
                        std::snprintf(badgeText, sizeof(badgeText), " INTAKING (Traction Grip) ");
                        bgCol = IM_COL32(10, 140, 120, 230);
                        borderCol = IM_COL32(50, 240, 200, 255);
                    }
                    else if (gp && gp->state == GamePieceState::Stored)
                    {
                        std::snprintf(badgeText, sizeof(badgeText), " STORED (Hopper Staging) ");
                        bgCol = IM_COL32(20, 90, 160, 230);
                        borderCol = IM_COL32(80, 180, 255, 255);
                    }
                    else if (!p.hasHitGround)
                    {
                        std::snprintf(badgeText, sizeof(badgeText), " LAUNCHED: %.1f m/s ", currentFlightSpeedMPerS);
                        bgCol = IM_COL32(140, 20, 130, 230);
                        borderCol = IM_COL32(240, 80, 230, 255);
                    }
                    else
                    {
                        std::snprintf(badgeText, sizeof(badgeText), " REST (FREE): %.2f m ", lastShotDistanceM);
                        bgCol = IM_COL32(30, 80, 30, 220);
                        borderCol = IM_COL32(90, 200, 90, 255);
                    }

                    ImVec2 txtSz = ImGui::CalcTextSize(badgeText);
                    ImVec2 bMin(scr.x - txtSz.x * 0.5f - 4.0f, scr.y - txtSz.y - 4.0f);
                    ImVec2 bMax(scr.x + txtSz.x * 0.5f + 4.0f, scr.y);
                    drawList->AddRectFilled(bMin, bMax, bgCol, 4.0f);
                    drawList->AddRect(bMin, bMax, borderCol, 4.0f);
                    drawList->AddText(ImVec2(bMin.x + 4.0f, bMin.y + 2.0f), IM_COL32(255, 255, 255, 255), badgeText);
                }
            }
        }
    }

    void PlaygroundScene::RenderUI()
    {
        // ---------------------------------------------------------------------
        // MODE A: Startup Scene Selection Dialog
        // ---------------------------------------------------------------------
        if (!sceneStarted)
        {
            ImGui::SetNextWindowSize(ImVec2(560.0f, 440.0f), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowPos(ImVec2(100.0f, 80.0f), ImGuiCond_FirstUseEver);

            ImGui::Begin("FRC Banana Simulator — Simulation Playground Launcher", nullptr, ImGuiWindowFlags_NoCollapse);

            ImGui::TextColored(ImVec4(0.15f, 0.85f, 0.95f, 1.0f), "WELCOME TO THE FRC BANANA PLAYGROUND");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::TextWrapped("Select a dedicated simulation scene to load. Each scene loads only its specific mechanism, physics models, and range tools.");
            ImGui::Spacing();

            ImGui::Text("Select Playground Scene:");
            const char* sceneNames[] = {
                "1. Ball Shooter & Curved Polycarbonate Hood",
                "2. 2024 Note Shooter (Dual Counter-Rotating Flywheels)",
                "3. 2-DOF Differential Wrist Mechanism"
            };

            int currentSceneIdx = static_cast<int>(selectedScene);
            if (ImGui::Combo("##SceneSelectCombo", &currentSceneIdx, sceneNames, IM_ARRAYSIZE(sceneNames)))
            {
                selectedScene = static_cast<PlaygroundSceneType>(currentSceneIdx);
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Scene Feature Card
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.10f, 1.0f), "Scene Description & Capabilities:");
            if (selectedScene == PlaygroundSceneType::BallShooterHood)
            {
                ImGui::BulletText("Active Ground Intake Roller with viscous surface drag & kinematic capture handoff.");
                ImGui::BulletText("Single top flywheel roller with adjustable curved polycarbonate hood backing.");
                ImGui::BulletText("Sliders for Hood Start Angle (-60 to +15 deg) & End Angle (+15 to +85 deg).");
                ImGui::BulletText("Realistic single-flywheel rolling contact kinematics & foam ball flight.");
                ImGui::BulletText("Ground distance meter lines with floating range badges (1m to 20m).");
            }
            else if (selectedScene == PlaygroundSceneType::NoteShooterDual)
            {
                ImGui::BulletText("Active Ground Intake Roller with viscous surface drag & kinematic capture handoff.");
                ImGui::BulletText("Dual counter-rotating Kraken X60 flywheels (top and bottom high-grip rollers).");
                ImGui::BulletText("2024 FRC Note Game Piece (14-inch OD, 2-inch thickness foam ring).");
                ImGui::BulletText("Direct dual-drive traction acceleration and coupled flywheel RPM sag.");
                ImGui::BulletText("Ground distance meter lines with live landing carry measurement.");
            }
            else
            {
                ImGui::BulletText("Coupled 2-DOF Bevel-Gear Differential Wrist (Pitch & Roll axes).");
                ImGui::BulletText("Dual Kraken X60 Motor models with back-EMF velocity feedback & braking.");
                ImGui::BulletText("Symmetric Pitch Drive, Anti-Symmetric Roll Drive, and Sinusoidal Frequency Sweeps.");
                ImGui::BulletText("Independent Coulomb Joint Friction and Transmission Efficiency tuning.");
            }

            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Start Button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.25f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.78f, 0.32f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.08f, 0.50f, 0.18f, 1.0f));

            if (ImGui::Button("START SCENE", ImVec2(240.0f, 42.0f)))
            {
                LoadScene(selectedScene);
            }
            ImGui::PopStyleColor(3);

            ImGui::SameLine();
            ImGui::TextDisabled("(Loads this scene only)");

            ImGui::End();
            return;
        }

        // Render Ground Distance Labels in 3D Viewport
        RenderDistanceFieldHUD();

        // ---------------------------------------------------------------------
        // MODE B: Active Scene HUD & Interactive Control Dashboard
        // ---------------------------------------------------------------------
        ImGui::SetNextWindowSize(ImVec2(520.0f, 780.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(25.0f, 25.0f), ImGuiCond_FirstUseEver);

        const char* windowTitle = (activeSceneType == PlaygroundSceneType::BallShooterHood)
            ? "FRC Banana Sim — Ball Shooter & Curved Hood Sandbox"
            : ((activeSceneType == PlaygroundSceneType::NoteShooterDual)
                ? "FRC Banana Sim — 2024 Note Shooter Sandbox"
                : "FRC Banana Sim — 2-DOF Differential Wrist Sandbox");

        ImGui::Begin(windowTitle, nullptr, ImGuiWindowFlags_None);

        // Top Navigation Bar
        if (ImGui::Button("❮ Switch Scene"))
        {
            sceneStarted = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("⟲ Reload Scene"))
        {
            LoadScene(activeSceneType);
        }
        ImGui::SameLine();
        if (activeSceneType == PlaygroundSceneType::BallShooterHood || activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            if (ImGui::Button("Focus Shooter"))
            {
                cameraFocusRequested = true;
                cameraFocusTarget = glm::vec3(0.0f, 1.05f, 0.2f);
                cameraFocusDistance = 2.4f;
            }
            ImGui::SameLine();
            if (ImGui::Button("Focus Range (10m)"))
            {
                cameraFocusRequested = true;
                cameraFocusTarget = glm::vec3(0.0f, 0.5f, 8.0f);
                cameraFocusDistance = 10.0f;
            }
        }
        else
        {
            if (ImGui::Button("Focus Wrist"))
            {
                cameraFocusRequested = true;
                cameraFocusTarget = glm::vec3(0.0f, 1.00f, 0.15f);
                cameraFocusDistance = 1.8f;
            }
        }

        ImGui::Separator();
        ImGui::Spacing();

        // ---------------------------------------------------------------------
        // SCENE 1: BALL SHOOTER & CURVED HOOD
        // ---------------------------------------------------------------------
        if (activeSceneType == PlaygroundSceneType::BallShooterHood)
        {
            if (ImGui::CollapsingHeader("Curved Hood & Geometry Tuning", ImGuiTreeNodeFlags_DefaultOpen))
            {
                bool hoodChanged = false;

                const char* styles[] = { "Underhand (Bottom Hood, Backspin)", "Overhand (Top Hood, Topspin)" };
                int currentStyle = isOverhandShooter ? 1 : 0;
                if (ImGui::Combo("Shooter Style", &currentStyle, styles, IM_ARRAYSIZE(styles)))
                {
                    isOverhandShooter = (currentStyle == 1);
                    if (isOverhandShooter)
                    {
                        hoodStartAngleDeg = -70.0f;
                        hoodEndAngleDeg = -25.0f;
                    }
                    else
                    {
                        hoodStartAngleDeg = -20.0f;
                        hoodEndAngleDeg = 50.0f;
                    }
                    hoodChanged = true;
                }

                if (isOverhandShooter)
                {
                    if (ImGui::SliderFloat("Hood Start Angle (deg)", &hoodStartAngleDeg, -85.0f, -30.0f, "%.1f deg"))
                    {
                        hoodChanged = true;
                    }
                    if (ImGui::SliderFloat("Hood End Angle (deg)", &hoodEndAngleDeg, -70.0f, 25.0f, "%.1f deg"))
                    {
                        hoodChanged = true;
                    }
                    float exitAngle = -hoodEndAngleDeg;
                    ImGui::Text("Effective Exit Pitch Angle: %.1f deg", exitAngle);
                }
                else
                {
                    if (ImGui::SliderFloat("Hood Start Angle (deg)", &hoodStartAngleDeg, -60.0f, 15.0f, "%.1f deg"))
                    {
                        hoodChanged = true;
                    }
                    if (ImGui::SliderFloat("Hood End Angle / Pitch (deg)", &hoodEndAngleDeg, 15.0f, 85.0f, "%.1f deg"))
                    {
                        hoodChanged = true;
                    }
                }

                if (ImGui::SliderFloat("Compression Gap (m)", &nominalGapM, 0.040f, 0.095f, "%.4f m"))
                {
                    hoodChanged = true;
                }
                if (ImGui::SliderFloat("Ball Diameter (m)", &uncompressedSizeM, 0.090f, 0.180f, "%.3f m"))
                {
                    hoodChanged = true;
                }

                if (hoodChanged)
                {
                    UpdateHoodGeometry();
                }

                ImGui::Spacing();
                float totalSpanDeg = hoodEndAngleDeg - hoodStartAngleDeg;
                ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.95f, 1.0f), "Contact Arc Length: %.1f mm (%.1f deg span)",
                    contactLengthM * 1000.0f, totalSpanDeg);
                float launchPitch = isOverhandShooter ? -hoodEndAngleDeg : hoodEndAngleDeg;
                ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.95f, 1.0f), "Launch Pitch Angle: %.1f deg", launchPitch);
                ImGui::TextDisabled(isOverhandShooter
                    ? "Rendered in 3D: Top Polycarb Hood (Overhand) + Downward Magnus Lift (Topspin)"
                    : "Rendered in 3D: Bottom Polycarb Hood (Underhand) + Upward Magnus Lift (Backspin)");
            }

            if (ImGui::CollapsingHeader("Flywheel Drive & Launch Control", ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto* fwTop = activeScene ? activeScene->GetComponent<FlywheelComponent>(topFlywheelEntity) : nullptr;
                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                double currentRpm = fwTop ? (fwTop->angularVelocityRadPerSec * radSToRpm) : 0.0;
                double rollerSpeed = fwTop ? (fwTop->angularVelocityRadPerSec * fwTop->rollerRadiusM) : 0.0;

                ImGui::Checkbox("Closed-Loop Velocity Control", &useFlywheelClosedLoop);
                if (useFlywheelClosedLoop)
                {
                    ImGui::SliderFloat("Target Flywheel RPM", &targetFlywheelRPM, 1000.0f, 6000.0f, "%.0f RPM");
                }
                else
                {
                    ImGui::SliderFloat("Manual Voltage", &commandedFlywheelVoltage, 0.0f, 12.0f, "%.2f V");
                }

                ImGui::Text("Flywheel Speed: ");
                ImGui::SameLine();
                ImVec4 rpmColor = (std::abs(currentRpm - targetFlywheelRPM) < 100.0)
                    ? ImVec4(0.15f, 0.90f, 0.35f, 1.0f)
                    : ImVec4(0.95f, 0.65f, 0.15f, 1.0f);
                ImGui::TextColored(rpmColor, "%.1f RPM (Roller: %.1f m/s / %.1f ft/s)",
                    currentRpm, rollerSpeed, rollerSpeed * 3.28084);

                ImGui::Spacing();

                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.95f, 0.40f, 0.05f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.50f, 0.15f, 1.0f));
                if (ImGui::Button("FIRE FOAM BALL", ImVec2(200.0f, 36.0f)))
                {
                    FireGamePiece();
                }
                ImGui::PopStyleColor(2);

                ImGui::SameLine();
                ImGui::Checkbox("Rapid Fire Stream", &rapidFireEnabled);

                // Optical Beam Break Sensor
                auto* cz = activeScene ? activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity) : nullptr;
                bool beamTripped = cz && cz->isEngaged;
                ImGui::Text("Beam Break (DIO 2): ");
                ImGui::SameLine();
                if (beamTripped)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f), "[ TRIPPED ] (Ball Squeezing)");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.3f, 1.0f), "[ CLEAR ] (Ready)");
                }
            }

            if (ImGui::CollapsingHeader("Active Ground Intake & Hopper Feeder", ImGuiTreeNodeFlags_DefaultOpen))
            {
            if (activeSceneType == PlaygroundSceneType::BallShooterHood)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.08f, 0.65f, 0.55f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.12f, 0.80f, 0.68f, 1.0f));
                if (ImGui::Button("SPAWN GROUND FOAM BALL", ImVec2(220.0f, 32.0f)))
                {
                    SpawnGroundGamePiece();
                }
                ImGui::PopStyleColor(2);
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.08f, 0.65f, 0.55f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.12f, 0.80f, 0.68f, 1.0f));
                if (ImGui::Button("SPAWN GROUND NOTE", ImVec2(220.0f, 32.0f)))
                {
                    SpawnGroundGamePiece();
                }
                ImGui::PopStyleColor(2);
            }

                ImGui::SameLine();
                ImGui::Checkbox("Auto-Feed to Shooter", &autoFeedToShooter);

                ImGui::Spacing();
                ImGui::Checkbox("Intake Active", &intakeEnabled);
                ImGui::SameLine();
                ImGui::SliderFloat("Intake Voltage", &intakeVoltage, -12.0f, 12.0f, "%.1f V");

                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                double surfaceSpeed = (currentIntakeRpm / radSToRpm) * intakeRollerRadiusM;
                ImGui::Text("Intake Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(intakeEnabled ? ImVec4(0.15f, 0.90f, 0.35f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "%.1f RPM (Roller: %.2f m/s)", currentIntakeRpm, surfaceSpeed);

                ImGui::Text("Intake Motor Load Torque: ");
                ImGui::SameLine();
                ImGui::TextColored(std::abs(lastIntakeTorqueNm) > 0.05 ? ImVec4(1.0f, 0.45f, 0.15f, 1.0f) : ImVec4(0.5f, 0.8f, 0.9f, 1.0f),
                    "%.3f N*m", lastIntakeTorqueNm);

                // Optical Beam Break Sensor (DIO 3)
                auto* czIn = activeScene ? activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity) : nullptr;
                bool intakeTripped = czIn && czIn->isEngaged;
                ImGui::Text("Intake Beam Break (DIO 3): ");
                ImGui::SameLine();
                if (intakeTripped)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f), "[ TRIPPED ] (Piece in Intake/Feeder)");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.3f, 1.0f), "[ CLEAR ] (Ready)");
                }

                ImGui::Text("Stored Pieces in Hopper: %zu", storedPiecesCount);
                ImGui::SameLine();
                bool canFeed = (storedPiecesCount > 0);
                if (!canFeed) { ImGui::BeginDisabled(); }
                if (ImGui::Button("FEED STORED PIECE", ImVec2(160.0f, 24.0f)))
                {
                    FeedStoredPiece();
                }
                if (!canFeed) { ImGui::EndDisabled(); }

                if (ImGui::TreeNode("Intake Surface Friction & Grip Parameters"))
                {
                    ImGui::SliderFloat("Grip Normal Force (N)", &intakeGripNormalForceN, 10.0f, 150.0f, "%.1f N");
                    ImGui::SliderFloat("Viscous Drag (N*s/m)", &intakeViscousDampingNsPerM, 0.0f, 30.0f, "%.1f N*s/m");
                    ImGui::SliderFloat("Max Grip Force (N)", &intakeMaxGripForceN, 20.0f, 300.0f, "%.1f N");
                    ImGui::TreePop();
                }
            }

            if (ImGui::CollapsingHeader("Live Ballistics & Distance Tracking", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (hasLandingRecorded)
                {
                    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.10f, 1.0f), "LAST SHOT CARRY DISTANCE: %.2f m (%.1f ft)",
                        lastShotDistanceM, lastShotDistanceM * 3.28084f);
                    ImGui::Text("Impact Coordinates: X = %.2f m, Z = %.2f m", lastLandingPos.x, lastLandingPos.z);

                    if (lastPreImpactSpeedMPerS > 0.05)
                    {
                        float dropPct = static_cast<float>((1.0 - lastPostImpactSpeedMPerS / lastPreImpactSpeedMPerS) * 100.0);
                        ImGui::Text("Ground Impact Bounce Bite: ");
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.15f, 1.0f), "%.2f m/s -> %.2f m/s (-%.1f%% bite)",
                            lastPreImpactSpeedMPerS, lastPostImpactSpeedMPerS, std::max(0.0f, dropPct));

                        ImGui::Text("Impact Spin Quench: ");
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(0.9f, 0.75f, 0.2f, 1.0f), "%.0f RPM Backspin -> Rolling Transition", lastImpactSpinRpm);
                    }
                }
                else
                {
                    ImGui::TextDisabled("Fire a shot to measure landing distance on the meter grid.");
                }

                ImGui::Spacing();
                ImGui::Text("Current Live Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.95f, 1.0f), "%.2f m/s (%.1f ft/s)",
                    currentFlightSpeedMPerS, currentFlightSpeedMPerS * 3.28084);

                ImGui::Text("Muzzle Exit Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "%.2f m/s (%.1f ft/s)",
                    lastExitSpeedMPerS, lastExitSpeedMPerS * 3.28084);

                ImGui::Text("Flywheel Transient RPM Sag: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "%.1f RPM", lastRpmSag);

                if (currentBallSpinRpm < -10.0)
                {
                    ImGui::Text("Ball Spin Rate: ");
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.90f, 1.0f), "%.0f RPM Backspin", std::abs(currentBallSpinRpm));
                }
                else if (currentBallSpinRpm > 10.0)
                {
                    ImGui::Text("Ball Spin Rate: ");
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.00f, 1.0f), "%.0f RPM Topspin", std::abs(currentBallSpinRpm));
                }
                else
                {
                    ImGui::Text("Ball Spin Rate: ");
                    ImGui::SameLine();
                    ImGui::TextDisabled("0 RPM");
                }

                if (std::abs(lastMagnusLiftN) > 1e-4)
                {
                    float gravOffset = static_cast<float>((lastMagnusLiftN / (pieceMassKg * 9.81f)) * 100.0);
                    ImGui::Text("Aerodynamic Magnus Lift: ");
                    ImGui::SameLine();
                    ImVec4 liftCol = (lastMagnusLiftN >= 0.0) ? ImVec4(0.25f, 0.85f, 1.0f, 1.0f) : ImVec4(1.0f, 0.4f, 0.2f, 1.0f);
                    ImGui::TextColored(liftCol, "%+.2f N (%+.1f%% of gravity)", lastMagnusLiftN, gravOffset);
                }
            }

            if (ImGui::CollapsingHeader("Ball Spin Aerodynamics & Ground Bite Tuning", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Enable Aerodynamic Magnus Lift & Ground Bite", &enableSpinPhysics);
                ImGui::SliderFloat("Backspin / Topspin Ratio Scale", &ballSpinMultiplier, 0.0f, 1.5f, "%.2fx");
                ImGui::SliderFloat("Ground Bite Friction", &groundBiteFriction, 0.2f, 1.5f, "%.2f");
                ImGui::TextDisabled("Underhand generates Backspin (upward lift). Overhand generates Topspin (downward curve).");
                ImGui::TextDisabled("Impact: High slip velocity creates strong friction braking (backspin) or forward kick (topspin).");
            }

            if (ImGui::CollapsingHeader("Live Telemetry Scopes (RPM Sag & Squeeze Force)"))
            {
                ImGui::Text("Flywheel RPM Scope (Sag on transit):");
                ImGui::PlotLines("##RPMPlot", rpmHistory.data(), static_cast<int>(kHistorySize),
                    static_cast<int>(historyIndex), "RPM", 0.0f, 5000.0f, ImVec2(460.0f, 85.0f));

                ImGui::Text("Normal Squeeze Force (N):");
                ImGui::PlotLines("##ForcePlot", forceHistory.data(), static_cast<int>(kHistorySize),
                    static_cast<int>(historyIndex), "Normal Force (N)", 0.0f, 350.0f, ImVec2(460.0f, 70.0f));
            }

            RenderBallisticOracleHUD(true);
        }

        // ---------------------------------------------------------------------
        // SCENE 2: 2024 NOTE SHOOTER (DUAL FLYWHEELS)
        // ---------------------------------------------------------------------
        else if (activeSceneType == PlaygroundSceneType::NoteShooterDual)
        {
            if (ImGui::CollapsingHeader("2024 Note Shooter Geometry", ImGuiTreeNodeFlags_DefaultOpen))
            {
                bool noteRigChanged = false;
                if (ImGui::SliderFloat("Shooter Pitch Angle (deg)", &notePitchDeg, 0.0f, 75.0f, "%.1f deg"))
                {
                    noteRigChanged = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Reset Angle"))
                {
                    notePitchDeg = 35.0f;
                    noteRigChanged = true;
                }

                if (ImGui::SliderFloat("Dual Flywheel Gap (m)", &nominalGapM, 0.025f, 0.050f, "%.4f m"))
                {
                    noteRigChanged = true;
                }
                if (ImGui::SliderFloat("Note Foam Thickness (m)", &uncompressedSizeM, 0.040f, 0.060f, "%.4f m"))
                {
                    noteRigChanged = true;
                }

                if (noteRigChanged)
                {
                    UpdateNoteShooterGeometry();
                }

                ImGui::Spacing();
                float squishMm = (uncompressedSizeM - nominalGapM) * 1000.0f;
                ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.95f, 1.0f), "Compression Squish: %.1f mm (%.2f in)",
                    squishMm, squishMm / 25.4f);
                ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.95f, 1.0f), "Dual Traction: Both top & bottom rollers drive the Note at %.1f deg", notePitchDeg);
                ImGui::TextDisabled("Rendered in 3D: Tilted dual Kraken X60 rollers + Cheek brackets dynamically rotate with pitch slider");
            }

            if (ImGui::CollapsingHeader("Dual Flywheel Drive & Launch Control", ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto* fwTop = activeScene ? activeScene->GetComponent<FlywheelComponent>(topFlywheelEntity) : nullptr;
                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                double currentRpm = fwTop ? (fwTop->angularVelocityRadPerSec * radSToRpm) : 0.0;
                double rollerSpeed = fwTop ? (fwTop->angularVelocityRadPerSec * fwTop->rollerRadiusM) : 0.0;

                ImGui::Checkbox("Closed-Loop Velocity Control", &useFlywheelClosedLoop);
                if (useFlywheelClosedLoop)
                {
                    ImGui::SliderFloat("Target Flywheel RPM", &targetFlywheelRPM, 1500.0f, 5500.0f, "%.0f RPM");
                }
                else
                {
                    ImGui::SliderFloat("Manual Voltage", &commandedFlywheelVoltage, 0.0f, 12.0f, "%.2f V");
                }

                ImGui::Text("Dual Rollers Speed: ");
                ImGui::SameLine();
                ImVec4 rpmColor = (std::abs(currentRpm - targetFlywheelRPM) < 100.0)
                    ? ImVec4(0.15f, 0.90f, 0.35f, 1.0f)
                    : ImVec4(0.95f, 0.65f, 0.15f, 1.0f);
                ImGui::TextColored(rpmColor, "%.1f RPM (Surface: %.1f m/s / %.1f ft/s)",
                    currentRpm, rollerSpeed, rollerSpeed * 3.28084);

                ImGui::Spacing();

                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.00f, 0.38f, 0.00f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.48f, 0.10f, 1.0f));
                if (ImGui::Button("FIRE 2024 NOTE", ImVec2(200.0f, 36.0f)))
                {
                    FireGamePiece();
                }
                ImGui::PopStyleColor(2);

                ImGui::SameLine();
                ImGui::Checkbox("Rapid Fire Stream", &rapidFireEnabled);

                // Optical Beam Break Sensor
                auto* cz = activeScene ? activeScene->GetComponent<ContactZoneComponent>(contactZoneEntity) : nullptr;
                bool beamTripped = cz && cz->isEngaged;
                ImGui::Text("Note Beam Break (DIO 2): ");
                ImGui::SameLine();
                if (beamTripped)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f), "[ TRIPPED ] (Note Squeezing)");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.3f, 1.0f), "[ CLEAR ] (Ready)");
                }
            }

            if (ImGui::CollapsingHeader("Active Ground Intake & Hopper Feeder", ImGuiTreeNodeFlags_DefaultOpen))
            {
            if (activeSceneType == PlaygroundSceneType::BallShooterHood)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.08f, 0.65f, 0.55f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.12f, 0.80f, 0.68f, 1.0f));
                if (ImGui::Button("SPAWN GROUND FOAM BALL", ImVec2(220.0f, 32.0f)))
                {
                    SpawnGroundGamePiece();
                }
                ImGui::PopStyleColor(2);
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.08f, 0.65f, 0.55f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.12f, 0.80f, 0.68f, 1.0f));
                if (ImGui::Button("SPAWN GROUND NOTE", ImVec2(220.0f, 32.0f)))
                {
                    SpawnGroundGamePiece();
                }
                ImGui::PopStyleColor(2);
            }

                ImGui::SameLine();
                ImGui::Checkbox("Auto-Feed to Shooter", &autoFeedToShooter);

                ImGui::Spacing();
                ImGui::Checkbox("Intake Active", &intakeEnabled);
                ImGui::SameLine();
                ImGui::SliderFloat("Intake Voltage", &intakeVoltage, -12.0f, 12.0f, "%.1f V");

                constexpr double radSToRpm = 60.0 / (2.0 * 3.14159265358979323846);
                double surfaceSpeed = (currentIntakeRpm / radSToRpm) * intakeRollerRadiusM;
                ImGui::Text("Intake Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(intakeEnabled ? ImVec4(0.15f, 0.90f, 0.35f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "%.1f RPM (Roller: %.2f m/s)", currentIntakeRpm, surfaceSpeed);

                ImGui::Text("Intake Motor Load Torque: ");
                ImGui::SameLine();
                ImGui::TextColored(std::abs(lastIntakeTorqueNm) > 0.05 ? ImVec4(1.0f, 0.45f, 0.15f, 1.0f) : ImVec4(0.5f, 0.8f, 0.9f, 1.0f),
                    "%.3f N*m", lastIntakeTorqueNm);

                // Optical Beam Break Sensor (DIO 3)
                auto* czIn = activeScene ? activeScene->GetComponent<ContactZoneComponent>(intakeZoneEntity) : nullptr;
                bool intakeTripped = czIn && czIn->isEngaged;
                ImGui::Text("Intake Beam Break (DIO 3): ");
                ImGui::SameLine();
                if (intakeTripped)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f), "[ TRIPPED ] (Piece in Intake/Feeder)");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.3f, 1.0f), "[ CLEAR ] (Ready)");
                }

                ImGui::Text("Stored Pieces in Hopper: %zu", storedPiecesCount);
                ImGui::SameLine();
                bool canFeed = (storedPiecesCount > 0);
                if (!canFeed) { ImGui::BeginDisabled(); }
                if (ImGui::Button("FEED STORED PIECE", ImVec2(160.0f, 24.0f)))
                {
                    FeedStoredPiece();
                }
                if (!canFeed) { ImGui::EndDisabled(); }

                if (ImGui::TreeNode("Intake Surface Friction & Grip Parameters"))
                {
                    ImGui::SliderFloat("Grip Normal Force (N)", &intakeGripNormalForceN, 10.0f, 150.0f, "%.1f N");
                    ImGui::SliderFloat("Viscous Drag (N*s/m)", &intakeViscousDampingNsPerM, 0.0f, 30.0f, "%.1f N*s/m");
                    ImGui::SliderFloat("Max Grip Force (N)", &intakeMaxGripForceN, 20.0f, 300.0f, "%.1f N");
                    ImGui::TreePop();
                }
            }

            if (ImGui::CollapsingHeader("Live Ballistics & Distance Tracking", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (hasLandingRecorded)
                {
                    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.10f, 1.0f), "LAST NOTE CARRY DISTANCE: %.2f m (%.1f ft)",
                        lastShotDistanceM, lastShotDistanceM * 3.28084f);
                    ImGui::Text("Landing Coordinates: X = %.2f m, Z = %.2f m", lastLandingPos.x, lastLandingPos.z);
                }
                else
                {
                    ImGui::TextDisabled("Fire a Note to measure landing distance on the meter grid.");
                }

                ImGui::Spacing();
                ImGui::Text("Current Live Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.95f, 1.0f), "%.2f m/s (%.1f ft/s)",
                    currentFlightSpeedMPerS, currentFlightSpeedMPerS * 3.28084);

                ImGui::Text("Note Exit Speed: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "%.2f m/s (%.1f ft/s)",
                    lastExitSpeedMPerS, lastExitSpeedMPerS * 3.28084);

                ImGui::Text("Flywheel RPM Sag: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "%.1f RPM", lastRpmSag);
            }

            if (ImGui::CollapsingHeader("Live Telemetry Scopes (RPM Sag & Squeeze Force)"))
            {
                ImGui::Text("Flywheel RPM Scope:");
                ImGui::PlotLines("##RPMPlot", rpmHistory.data(), static_cast<int>(kHistorySize),
                    static_cast<int>(historyIndex), "RPM", 0.0f, 5500.0f, ImVec2(460.0f, 85.0f));

                ImGui::Text("Normal Squeeze Force (N):");
                ImGui::PlotLines("##ForcePlot", forceHistory.data(), static_cast<int>(kHistorySize),
                    static_cast<int>(historyIndex), "Normal Force (N)", 0.0f, 400.0f, ImVec2(460.0f, 70.0f));
            }

            RenderBallisticOracleHUD(false);
        }

        // ---------------------------------------------------------------------
        // SCENE 3: DIFFERENTIAL WRIST
        // ---------------------------------------------------------------------
        else if (activeSceneType == PlaygroundSceneType::DifferentialWrist)
        {
            if (ImGui::CollapsingHeader("Differential Wrist Controls", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const char* modes[] = { "Manual Sliders", "Pitch Test (+V, +V)", "Roll Test (+V, -V)", "Sine Sweep", "Brake Hold" };
                int currentMode = static_cast<int>(controlMode);
                if (ImGui::Combo("Control Mode", &currentMode, modes, IM_ARRAYSIZE(modes)))
                {
                    controlMode = static_cast<PlaygroundControlMode>(currentMode);
                }

                if (controlMode == PlaygroundControlMode::ManualSliders)
                {
                    ImGui::SliderFloat("Motor A Voltage (V)", &manualVoltageA, -12.0f, 12.0f, "%.2f V");
                    ImGui::SliderFloat("Motor B Voltage (V)", &manualVoltageB, -12.0f, 12.0f, "%.2f V");
                }
                else if (controlMode == PlaygroundControlMode::SineSweep)
                {
                    ImGui::SliderFloat("Sweep Amplitude (V)", &testVoltageMagnitude, 0.0f, 12.0f, "%.1f V");
                    ImGui::SliderFloat("Sweep Frequency (Hz)", &testFrequencyHz, 0.1f, 3.0f, "%.2f Hz");
                }

                ImGui::Spacing();
                if (ImGui::Button("Reset Rest Pose", ImVec2(160.0f, 28.0f)))
                {
                    ResetWristPose();
                }

                ImGui::Separator();
                auto* diff = activeScene ? activeScene->GetComponent<DifferentialComponent>(diffMechanismEntity) : nullptr;
                if (diff)
                {
                    constexpr double radToDeg = 180.0 / 3.14159265358979323846;
                    ImGui::Text("Pitch Joint Angle: %.1f deg (%.1f deg/s)",
                        diff->jointPositionsRad[0] * radToDeg, diff->jointVelocitiesRadS[0] * radToDeg);
                    ImGui::Text("Roll Joint Angle:  %.1f deg (%.1f deg/s)",
                        diff->jointPositionsRad[1] * radToDeg, diff->jointVelocitiesRadS[1] * radToDeg);
                }
            }
        }

        ImGui::End();
    }

    void PlaygroundScene::Shutdown()
    {
        UnloadScene();

        DestroyTrackedEntity(groundEntity);

        diffSystem.reset();
        flywheelSystem.reset();
        sensorSystem.reset();
        jointSystem.reset();
        physicsSystem.reset();

        activeScene = nullptr;
        activeJolt = nullptr;
        activeCamera = nullptr;
    }
}
