/**
 * @file PlaygroundScene.cpp
 * @brief Implementation of the interactive differential mechanism playground.
 */

#include "Playground/PlaygroundScene.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Renderer/Components/MeshComponent.h"
#include "Robot/Mechanisms/Components/DifferentialComponent.h"
#include "Actuation/DCMotorModel.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <imgui/imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Playground
{
    using namespace Core::ECS::Core;
    using namespace Core::ECS::Components;
    using namespace Physics::Components;
    using namespace Renderer::Components;
    using namespace Robot::Mechanisms::Components;
    using namespace Robot::Mechanisms::Systems;

    PlaygroundScene::PlaygroundScene()
        : activeScene(nullptr),
        activeJolt(nullptr),
        groundEntity(invalidEntity),
        mountStandEntity(invalidEntity),
        pitchYokeEntity(invalidEntity),
        rollToolEntity(invalidEntity),
        pitchJointEntity(invalidEntity),
        rollJointEntity(invalidEntity),
        diffMechanismEntity(invalidEntity),
        physicsSystem(nullptr),
        jointSystem(nullptr),
        diffSystem(nullptr),
        controlMode(PlaygroundControlMode::ManualSliders),
        manualVoltageA(0.0f),
        manualVoltageB(0.0f),
        testVoltageMagnitude(6.0f),
        testFrequencyHz(0.5f),
        testTimeSeconds(0.0),
        brakeModeA(true),
        brakeModeB(true),
        gearRatioPitch(45.0f),
        gearRatioRoll(30.0f),
        transmissionEfficiency(0.95f),
        frictionPitchNm(0.35f),
        frictionRollNm(0.20f)
    {}

    PlaygroundScene::~PlaygroundScene()
    {}

    static void AttachTransforms(Scene& scene, Entity entity, const glm::vec3& pos,
        const glm::vec3& scale, const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
    {
        TransformComponent transform;
        transform.position = pos;
        transform.scale = scale;
        transform.rotation = rot;
        scene.AddComponent(entity, transform);

        WorldTransformComponent worldTransform;
        worldTransform.worldPosition = pos;
        worldTransform.worldScale = scale;
        worldTransform.worldRotation = rot;
        scene.AddComponent(entity, worldTransform);
    }

    bool PlaygroundScene::Initialize(Scene& scene, Physics::JoltContext& jolt)
    {
        SIM_PROFILE_SCOPE("PlaygroundScene::Initialize");

        activeScene = &scene;
        activeJolt = &jolt;

        // 1. Instantiate Core Systems
        physicsSystem = std::make_unique<Physics::Systems::PhysicsSystem>(scene, jolt);
        jointSystem = std::make_unique<Physics::Systems::JointSystem>(scene, jolt);
        diffSystem = std::make_unique<DifferentialMechanismSystem>(scene, &jolt);

        // 2. Ground Grid Plane (Y = 0.0m)
        groundEntity = scene.CreateEntity();
        {
            AttachTransforms(scene, groundEntity, glm::vec3(0.0f, -0.05f, 0.0f), glm::vec3(10.0f, 0.10f, 10.0f));

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Plane;
            mesh.color = glm::vec3(0.15f, 0.17f, 0.20f); // Slate dark testing pad
            scene.AddComponent(groundEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            rb.friction = 0.8f;
            scene.AddComponent(groundEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = glm::vec3(5.0f, 0.05f, 5.0f);
            scene.AddComponent(groundEntity, col);
        }

        // 3. Mount Pedestal Stand (Base root at origin, top at Y = 1.0m)
        mountStandEntity = scene.CreateEntity();
        {
            glm::vec3 standPos(0.0f, 0.50f, 0.0f);
            glm::vec3 standDims(0.18f, 1.00f, 0.18f);
            AttachTransforms(scene, mountStandEntity, standPos, standDims);

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.30f, 0.32f, 0.35f); // Gunmetal Pillar
            scene.AddComponent(mountStandEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Static;
            scene.AddComponent(mountStandEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = standDims * 0.5f;
            scene.AddComponent(mountStandEntity, col);
        }

        // 4. Pitch Yoke Link (Pitches around horizontal lateral +X axis at Y = 1.0m)
        pitchYokeEntity = scene.CreateEntity();
        {
            glm::vec3 yokePos(0.0f, 1.00f, 0.05f);
            glm::vec3 yokeDims(0.24f, 0.08f, 0.12f);
            AttachTransforms(scene, pitchYokeEntity, yokePos, yokeDims);

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(0.10f, 0.65f, 0.95f); // Vibrant Electric Cyan Yoke
            scene.AddComponent(pitchYokeEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 1.20f;
            rb.gravityFactor = 0.0f; // Controlled by analytic gravity solver
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            scene.AddComponent(pitchYokeEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = yokeDims * 0.5f;
            scene.AddComponent(pitchYokeEntity, col);
        }

        // 5. Pitch Joint Hinge (Connects Mount Stand -> Pitch Yoke)
        pitchJointEntity = scene.CreateEntity();
        {
            JointComponent jc;
            jc.type = JointType::Hinge;
            jc.parentEntity = mountStandEntity;
            jc.childEntity = pitchYokeEntity;
            jc.SetWorldSpace(glm::vec3(0.0f, 1.00f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // Hinge around +X

            jc.hasLimits = true;
            jc.minLimit = glm::radians(-85.0f);
            jc.maxLimit = glm::radians(+85.0f);
            jc.limitSpringFrequency = 40.0f;
            jc.limitSpringDamping = 1.0f;
            scene.AddComponent(pitchJointEntity, jc);
        }

        // 6. Roll End-Effector Tool (Rolls around distal +Z axis extending from yoke)
        rollToolEntity = scene.CreateEntity();
        {
            glm::vec3 toolPos(0.0f, 1.00f, 0.20f);
            glm::vec3 toolDims(0.14f, 0.06f, 0.18f);
            AttachTransforms(scene, rollToolEntity, toolPos, toolDims);

            MeshComponent mesh;
            mesh.shape = PrimitiveShape::Cube;
            mesh.color = glm::vec3(1.00f, 0.45f, 0.10f); // Bright Safety Orange Tool
            scene.AddComponent(rollToolEntity, mesh);

            RigidBodyComponent rb;
            rb.motionType = BodyMotionType::Dynamic;
            rb.mass = 0.65f;
            rb.gravityFactor = 0.0f;
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            scene.AddComponent(rollToolEntity, rb);

            ColliderComponent col;
            col.shape = ColliderShape::Box;
            col.boxHalfExtents = toolDims * 0.5f;
            scene.AddComponent(rollToolEntity, col);
        }

        // 7. Roll Joint Hinge (Connects Pitch Yoke -> Roll Tool)
        rollJointEntity = scene.CreateEntity();
        {
            JointComponent jc;
            jc.type = JointType::Hinge;
            jc.parentEntity = pitchYokeEntity;
            jc.childEntity = rollToolEntity;
            jc.SetWorldSpace(glm::vec3(0.0f, 1.00f, 0.11f), glm::vec3(0.0f, 0.0f, 1.0f)); // Hinge around +Z

            jc.hasLimits = true;
            jc.minLimit = glm::radians(-175.0f);
            jc.maxLimit = glm::radians(+175.0f);
            jc.limitSpringFrequency = 40.0f;
            jc.limitSpringDamping = 1.0f;
            scene.AddComponent(rollJointEntity, jc);
        }

        // 8. 2-DOF Coupled Differential Mechanism Component
        diffMechanismEntity = scene.CreateEntity();
        {
            DifferentialComponent diff;
            diff.name = "playground_diffy_wrist";
            diff.jointEntityA = pitchJointEntity;
            diff.jointEntityB = rollJointEntity;
            diff.linkEntityA = pitchYokeEntity;
            diff.linkEntityB = rollToolEntity;

            diff.motorModelA = Actuation::DCMotorModel::KrakenX44();
            diff.motorModelB = Actuation::DCMotorModel::KrakenX44();
            diff.motorCountA = 1;
            diff.motorCountB = 1;

            diff.efficiency = transmissionEfficiency;
            diff.coulombFrictionNm = { frictionPitchNm, frictionRollNm };
            diff.viscousDampingNms = { 0.04, 0.02 };

            DifferentialMechanismSystem::ConfigureSymmetricBevel(diff, gearRatioPitch, gearRatioRoll, false);

            scene.AddComponent(diffMechanismEntity, diff);
        }

        // Warm up and instantiate Jolt physical constraint handles
        physicsSystem->FixedUpdate(0.005);
        jointSystem->FixedUpdate(0.005);

        return true;
    }

    void PlaygroundScene::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("PlaygroundScene::FixedUpdate");

        if (!activeScene)
        {
            return;
        }

        testTimeSeconds += dt;

        auto* diff = activeScene->GetComponent<DifferentialComponent>(diffMechanismEntity);
        if (diff)
        {
            // Evaluate selected test pattern
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

            // Runtime transmission tuning updates
            diff->efficiency = static_cast<double>(transmissionEfficiency);
            diff->coulombFrictionNm = { static_cast<double>(frictionPitchNm), static_cast<double>(frictionRollNm) };
        }

        // 1. Solve coupled semi-implicit differential motor dynamics
        if (diffSystem)
        {
            diffSystem->FixedUpdate(dt);
        }

        // 2. Drive physical Jolt constraints and motors
        if (jointSystem)
        {
            jointSystem->FixedUpdate(dt);
        }

        // 3. Advance Jolt rigid body positions and contacts
        if (physicsSystem)
        {
            physicsSystem->FixedUpdate(dt);
        }
    }

    void PlaygroundScene::ResetPose()
    {
        manualVoltageA = 0.0f;
        manualVoltageB = 0.0f;
        controlMode = PlaygroundControlMode::BrakeHold;
        testTimeSeconds = 0.0;

        if (!activeScene || !activeJolt)
        {
            return;
        }

        // Reset Joint target velocities and applied torques
        if (auto* pj = activeScene->GetComponent<JointComponent>(pitchJointEntity))
        {
            pj->appliedTorque = 0.0f;
            pj->currentAngle = 0.0f;
            pj->currentVelocity = 0.0f;
        }

        if (auto* rj = activeScene->GetComponent<JointComponent>(rollJointEntity))
        {
            rj->appliedTorque = 0.0f;
            rj->currentAngle = 0.0f;
            rj->currentVelocity = 0.0f;
        }

        // Reset physical Jolt body transforms
        if (auto* tYoke = activeScene->GetComponent<TransformComponent>(pitchYokeEntity))
        {
            tYoke->position = glm::vec3(0.0f, 1.00f, 0.05f);
            tYoke->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        }

        if (auto* tTool = activeScene->GetComponent<TransformComponent>(rollToolEntity))
        {
            tTool->position = glm::vec3(0.0f, 1.00f, 0.20f);
            tTool->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    void PlaygroundScene::RenderUI()
    {
        if (!activeScene)
        {
            return;
        }

        auto* diff = activeScene->GetComponent<DifferentialComponent>(diffMechanismEntity);
        if (!diff)
        {
            return;
        }

        ImGui::SetNextWindowSize(ImVec2(440, 620), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("2-DOF Differential Mechanism Playground", nullptr, ImGuiWindowFlags_NoCollapse))
        {
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Kinematic Coupling: Symmetric Bevel Gear Cluster");
            ImGui::Separator();

            // 1. Control Modes
            ImGui::Text("Test Modes:");
            if (ImGui::Button("Manual Sliders")) { controlMode = PlaygroundControlMode::ManualSliders; }
            ImGui::SameLine();
            if (ImGui::Button("Pure Pitch (+V, +V)")) { controlMode = PlaygroundControlMode::PitchTest; }
            ImGui::SameLine();
            if (ImGui::Button("Pure Roll (+V, -V)")) { controlMode = PlaygroundControlMode::RollTest; }

            if (ImGui::Button("Sine Sweep")) { controlMode = PlaygroundControlMode::SineSweep; }
            ImGui::SameLine();
            if (ImGui::Button("Brake Hold")) { controlMode = PlaygroundControlMode::BrakeHold; manualVoltageA = 0.0f; manualVoltageB = 0.0f; }
            ImGui::SameLine();
            if (ImGui::Button("Reset Rest Pose")) { ResetPose(); }

            ImGui::Separator();

            // 2. Commanded Voltages
            if (controlMode == PlaygroundControlMode::ManualSliders)
            {
                ImGui::SliderFloat("Motor A (Left) [V]", &manualVoltageA, -12.0f, 12.0f, "%.1f V");
                ImGui::SliderFloat("Motor B (Right) [V]", &manualVoltageB, -12.0f, 12.0f, "%.1f V");
            }
            else
            {
                ImGui::SliderFloat("Test Amplitude [V]", &testVoltageMagnitude, 0.0f, 12.0f, "%.1f V");
                if (controlMode == PlaygroundControlMode::SineSweep)
                {
                    ImGui::SliderFloat("Frequency [Hz]", &testFrequencyHz, 0.1f, 3.0f, "%.2f Hz");
                }
            }

            ImGui::Checkbox("Brake Mode Motor A", &brakeModeA);
            ImGui::SameLine();
            ImGui::Checkbox("Brake Mode Motor B", &brakeModeB);

            ImGui::Separator();

            // 3. Joint Kinematics Telemetry Readout
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Joint Coordinates (Physical Wrist):");
            double pitchDeg = glm::degrees(diff->jointPositionsRad.x);
            double rollDeg = glm::degrees(diff->jointPositionsRad.y);
            double pitchVelDegS = glm::degrees(diff->jointVelocitiesRadS.x);
            double rollVelDegS = glm::degrees(diff->jointVelocitiesRadS.y);

            ImGui::Text("Pitch Angle:    %6.2f deg  |  Vel: %6.1f deg/s", pitchDeg, pitchVelDegS);
            ImGui::Text("Roll Angle:     %6.2f deg  |  Vel: %6.1f deg/s", rollDeg, rollVelDegS);
            ImGui::Text("Delivered Tau:  [Pitch: %5.2f Nm, Roll: %5.2f Nm]",
                diff->deliveredJointTorquesNm.x, diff->deliveredJointTorquesNm.y);

            ImGui::Separator();

            // 4. Motor Rotor Telemetry Readout
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Actuator Telemetry (Ungeared Rotors):");
            double rpmA = diff->motorRotorSpeedsRadS.x * (60.0 / (2.0 * 3.14159265));
            double rpmB = diff->motorRotorSpeedsRadS.y * (60.0 / (2.0 * 3.14159265));

            ImGui::Text("Motor A (CAN 15): %6.0f RPM  |  %5.1f A  |  %4.1f V",
                rpmA, diff->motorStatorCurrentsA.x, diff->commandedVoltageA);
            ImGui::Text("Motor B (CAN 16): %6.0f RPM  |  %5.1f A  |  %4.1f V",
                rpmB, diff->motorStatorCurrentsA.y, diff->commandedVoltageB);

            ImGui::Separator();

            // 5. Live Gearing & Friction Calibration Tuning
            if (ImGui::CollapsingHeader("Transmission & Friction Tuning", ImGuiTreeNodeFlags_DefaultOpen))
            {
                bool matrixDirty = false;
                if (ImGui::SliderFloat("Pitch Ratio (Gp)", &gearRatioPitch, 5.0f, 100.0f, "%.1f:1")) { matrixDirty = true; }
                if (ImGui::SliderFloat("Roll Ratio (Gr)", &gearRatioRoll, 5.0f, 100.0f, "%.1f:1")) { matrixDirty = true; }

                if (matrixDirty)
                {
                    DifferentialMechanismSystem::ConfigureSymmetricBevel(*diff, gearRatioPitch, gearRatioRoll, false);
                }

                ImGui::SliderFloat("Efficiency (eta)", &transmissionEfficiency, 0.50f, 1.00f, "%.2f");
                ImGui::SliderFloat("Coulomb Pitch [Nm]", &frictionPitchNm, 0.0f, 2.0f, "%.2f Nm");
                ImGui::SliderFloat("Coulomb Roll [Nm]", &frictionRollNm, 0.0f, 2.0f, "%.2f Nm");
            }
        }
        ImGui::End();
    }
}