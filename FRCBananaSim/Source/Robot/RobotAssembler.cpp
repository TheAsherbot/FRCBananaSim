/**
 * @file RobotAssembler.cpp
 * @brief Implementation of Schema v1 robot programmatic assembly and 8-stage pipeline stepping.
 */

#include "Robot/RobotAssembler.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Physics/Components/ColliderComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Robot/Drivetrain/Components/SwerveChassisComponent.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FourBarComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Robot/Mechanisms/Components/CouplingComponent.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"
#include "Robot/Sensors/Components/IMUComponent.h"
#include "Robot/Sensors/Components/EncoderComponent.h"
#include "Robot/Sensors/Components/LimitSwitchComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Components/AnalogInputComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Actuation/DCMotorModel.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/Log.h"

#include <iostream>
#include <algorithm>
#include <cmath>

namespace
{
    /**
     * @brief Helper to instantiate a DCMotorModel from a Schema motorPreset string.
     */
    FRCBananaSim::Actuation::DCMotorModel CreateMotorFromPreset(const std::string& preset,
        double gearRatio,
        double efficiency)
    {
        using namespace FRCBananaSim::Actuation;
        if (preset == "KrakenX60FOC") return DCMotorModel::KrakenX60FOC(gearRatio, efficiency);
        if (preset == "KrakenX44")    return DCMotorModel::KrakenX44(gearRatio, efficiency);
        if (preset == "KrakenX44FOC") return DCMotorModel::KrakenX44FOC(gearRatio, efficiency);
        if (preset == "Falcon500")   return DCMotorModel::Falcon500(gearRatio, efficiency);
        if (preset == "NEO1" || preset == "NEO") return DCMotorModel::NEO1(gearRatio, efficiency);
        if (preset == "NEO2")        return DCMotorModel::NEO2(gearRatio, efficiency);
        if (preset == "NEO550")      return DCMotorModel::NEO550(gearRatio, efficiency);
        if (preset == "NEOVortex")   return DCMotorModel::NEOVortex(gearRatio, efficiency);
        if (preset == "Minion")      return DCMotorModel::Minion(gearRatio, efficiency);
        return DCMotorModel::KrakenX60(gearRatio, efficiency);
    }
}

namespace FRCBananaSim::Robot
{
    RobotAssembler::RobotAssembler(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : scene(scene),
        joltContext(jolt)
    {}

    RobotAssembler::~RobotAssembler()
    {}

    AssembledRobot RobotAssembler::Assemble(const Schema::RobotManifest& manifest,
        const Schema::RobotSidecar& sidecar,
        const Schema::MaterialLibrary& materials,
        const glm::vec3& worldSpawnPosition,
        const glm::quat& worldSpawnOrientation)
    {
        SIM_PROFILE_SCOPE("RobotAssembler::Assemble");

        AssembledRobot result;
        result.robotName = manifest.robotName;

        // 1. Spawn Battery Component Entity
        result.batteryEntity = scene.CreateEntity();
        Actuation::Components::BatteryComponent batteryComp;
        batteryComp.batteryModel = Actuation::BatteryModel(
            sidecar.electrical.openCircuitVoltageV,
            sidecar.electrical.internalResistanceOhm,
            sidecar.electrical.quiescentCurrentA,
            60.0,
            40.0,
            sidecar.electrical.brownoutWarningV,
            sidecar.electrical.brownoutShutoffV,
            sidecar.electrical.brownoutRecoveryV
        );
        batteryComp.terminalVoltage = sidecar.electrical.openCircuitVoltageV;
        batteryComp.totalCurrent = sidecar.electrical.quiescentCurrentA;
        scene.AddComponent(result.batteryEntity, batteryComp);

        // 2. Compute Initial World Poses for All Links from Kinematic Joint Tree
        struct LinkPose
        {
            glm::vec3 worldPos{ 0.0f };
            glm::quat worldRot{ 1.0f, 0.0f, 0.0f, 0.0f };
        };
        std::unordered_map<std::string, LinkPose> linkPoses;

        std::string rootLinkName = "base_link";
        for (const auto& link : manifest.links)
        {
            if (link.isRoot)
            {
                rootLinkName = link.name;
                break;
            }
        }
        linkPoses[rootLinkName] = { worldSpawnPosition, worldSpawnOrientation };

        bool expanded = true;
        while (expanded)
        {
            expanded = false;
            for (const auto& joint : manifest.joints)
            {
                auto pIt = linkPoses.find(joint.parentLink);
                if (pIt != linkPoses.end() && linkPoses.find(joint.childLink) == linkPoses.end())
                {
                    glm::vec3 originEngine = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(joint.originM));
                    glm::vec3 childWorldPos = pIt->second.worldPos + (pIt->second.worldRot * originEngine);
                    glm::quat childWorldRot = pIt->second.worldRot;
                    linkPoses[joint.childLink] = { childWorldPos, childWorldRot };
                    expanded = true;
                }
            }
        }

        // Attach visual links of flywheels/intakes to their parent links
        for (const auto& mech : sidecar.mechanisms)
        {
            if ((mech.role == "flywheel" || mech.role == "intake") && !mech.visualLink.empty() && !mech.parentLink.empty())
            {
                auto pIt = linkPoses.find(mech.parentLink);
                if (pIt != linkPoses.end() && linkPoses.find(mech.visualLink) == linkPoses.end())
                {
                    glm::vec3 mountOffset(0.0f, 0.06f, 0.06f);
                    linkPoses[mech.visualLink] = {
                        pIt->second.worldPos + (pIt->second.worldRot * mountOffset),
                        pIt->second.worldRot
                    };
                }
            }
        }

        // Fallback for unlinked meshes
        for (const auto& link : manifest.links)
        {
            if (linkPoses.find(link.name) == linkPoses.end())
            {
                linkPoses[link.name] = { worldSpawnPosition, worldSpawnOrientation };
            }
        }

        // 3. Spawn Links (RigidBodies, Transforms, Colliders)
        uint32_t linkSubGroupIndex = 1;
        for (const auto& link : manifest.links)
        {
            Core::ECS::Core::Entity linkEntity = scene.CreateEntity();
            result.linkEntities[link.name] = linkEntity;

            if (link.isRoot)
            {
                result.rootLinkEntity = linkEntity;
                result.chassisEntity = linkEntity;
            }

            const auto& pose = linkPoses[link.name];
            glm::vec3 worldPos = pose.worldPos;
            glm::quat worldRot = pose.worldRot;

            Core::ECS::Components::TransformComponent tc;
            tc.position = worldPos;
            tc.rotation = worldRot;
            tc.scale = glm::vec3(1.0f);
            scene.AddComponent(linkEntity, tc);

            Core::ECS::Components::WorldTransformComponent wt;
            wt.worldPosition = worldPos;
            wt.worldRotation = worldRot;
            wt.worldScale = glm::vec3(1.0f);
            scene.AddComponent(linkEntity, wt);

            // Add Physical RigidBodyComponent
            Physics::Components::RigidBodyComponent rb;
            rb.motionType = Physics::Components::BodyMotionType::Dynamic;
            rb.mass = static_cast<float>(link.massKg);
            rb.collisionGroupID = 1; // Robot group: mutual self-collisions disabled via GroupFilterTable
            rb.collisionSubGroupID = linkSubGroupIndex++;

            rb.allowSleeping = false;
            rb.gravityFactor = 0.0f; // Analytic drivetrain and mechanism solvers own gravity
            if (!link.isRoot)
            {
                rb.isSensor = true; // Prevent child mechanism links from colliding with ground/chassis
            }

            if (link.inertiaTensor.IsPositiveDefinite())
            {
                glm::dmat3 inertiaNWU = link.inertiaTensor.ToMatrix();
                glm::dmat3 inertiaEngine = Schema::RobotSchemaLoader::ConvertNWUInertiaToEngine(inertiaNWU);
                rb.hasCustomInertia = true;
                rb.inertiaTensor = glm::mat3(inertiaEngine);
            }

            const auto* mat = materials.GetMaterial(link.material);
            if (mat)
            {
                rb.friction = static_cast<float>(mat->friction);
                rb.restitution = static_cast<float>(mat->restitution);
            }

            scene.AddComponent(linkEntity, rb);

            // Add Primary Collider if defined
            if (!link.colliders.empty())
            {
                const auto& colDef = link.colliders[0];
                Physics::Components::ColliderComponent col;

                if (colDef.type == "sphere")
                {
                    col.shape = Physics::Components::ColliderShape::Sphere;
                    col.radius = static_cast<float>(colDef.dimensionsM.x);
                }
                else if (colDef.type == "cylinder")
                {
                    col.shape = Physics::Components::ColliderShape::Cylinder;
                    col.radius = static_cast<float>(colDef.dimensionsM.x);
                    col.halfHeight = static_cast<float>(colDef.dimensionsM.y * 0.5);
                }
                else if (colDef.type == "capsule")
                {
                    col.shape = Physics::Components::ColliderShape::Capsule;
                    col.radius = static_cast<float>(colDef.dimensionsM.x);
                    col.halfHeight = static_cast<float>(colDef.dimensionsM.y * 0.5);
                }
                else
                {
                    col.shape = Physics::Components::ColliderShape::Box;
                    glm::vec3 dimsEngine = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(colDef.dimensionsM));
                    col.boxHalfExtents = glm::abs(dimsEngine) * 0.5f;
                }

                col.localOffset = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(colDef.originM));
                col.localRotation = glm::quat(Schema::RobotSchemaLoader::ConvertNWUQuatToEngine(glm::dquat(colDef.rotationQuat)));

                scene.AddComponent(linkEntity, col);
            }
        }

        // 4. Attach CompositeMassComponent to Chassis for Dynamic Parallel Axis Tracking
        if (result.chassisEntity != Core::ECS::Core::invalidEntity)
        {
            Mechanisms::Components::CompositeMassComponent compMass;
            compMass.chassisEntity = result.chassisEntity;
            for (const auto& [name, entity] : result.linkEntities)
            {
                if (entity != result.chassisEntity)
                {
                    compMass.mechanismLinks.push_back(entity);
                }
            }
            scene.AddComponent(result.chassisEntity, compMass);
        }

        // 5. Spawn Joints (Hinges, Sliders, Fixed)
        for (const auto& joint : manifest.joints)
        {
            Core::ECS::Core::Entity jointEntity = scene.CreateEntity();
            result.jointEntities[joint.name] = jointEntity;

            auto pIt = result.linkEntities.find(joint.parentLink);
            auto cIt = result.linkEntities.find(joint.childLink);

            if (pIt != result.linkEntities.end() && cIt != result.linkEntities.end())
            {
                Physics::Components::JointComponent jc;
                jc.parentEntity = pIt->second;
                jc.childEntity = cIt->second;

                const auto& parentPose = linkPoses[joint.parentLink];
                glm::vec3 axisEngine = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(joint.axis));
                if (joint.type == "revolute" || joint.type == "continuous")
                {
                    // In Schema v1 NWU, +Y [0,1,0] revolute joint angles are defined positive pitching upward (+X toward +Z),
                    // which corresponds to right-handed rotation about -X (-1,0,0) in Engine coordinates (+Z toward +Y).
                    axisEngine.x = -axisEngine.x;
                }
                glm::vec3 originEngine = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(joint.originM));
                glm::vec3 worldAnchor = parentPose.worldPos + (parentPose.worldRot * originEngine);
                glm::vec3 worldAxis = glm::normalize(parentPose.worldRot * axisEngine);

                if (joint.type == "revolute" || joint.type == "continuous")
                {
                    jc.type = Physics::Components::JointType::Hinge;
                    jc.SetWorldSpace(worldAnchor, worldAxis);
                    jc.hasLimits = joint.limits.hasLimits;
                    if (joint.limits.hasLimits)
                    {
                        jc.minLimit = static_cast<float>(joint.limits.lower);
                        jc.maxLimit = static_cast<float>(joint.limits.upper);
                        jc.hasLimitSpring = true;
                        jc.limitSpringFrequency = static_cast<float>(joint.limits.springFrequencyHz > 0.0 ? joint.limits.springFrequencyHz : 40.0);
                        jc.limitSpringDamping = static_cast<float>(joint.limits.springDampingRatio > 0.0 ? joint.limits.springDampingRatio : 1.0);
                    }
                }
                else if (joint.type == "prismatic")
                {
                    jc.type = Physics::Components::JointType::Slider;
                    float minL = static_cast<float>(joint.limits.lower);
                    float maxL = static_cast<float>(joint.limits.upper);
                    jc.SetWorldSpaceSlider(worldAnchor, worldAxis, minL, maxL);
                    jc.hasLimits = joint.limits.hasLimits;
                    jc.motorMode = Physics::Components::JointMotorMode::Position;
                    jc.targetPosition = 0.0f;
                    jc.springFrequency = 100.0f;
                    jc.springDamping = 1.0f;
                    jc.maxMotorForce = 50000.0f;
                }
                else if (joint.type == "fixed")
                {
                    jc.type = Physics::Components::JointType::Fixed;
                    jc.SetWorldSpace(worldAnchor, worldAxis);
                    jc.hasLimits = true;
                    jc.minLimit = 0.0f;
                    jc.maxLimit = 0.0f;
                }

                scene.AddComponent(jointEntity, jc);
            }
        }

        // 6. Spawn Mechanisms (Elevators, Arms, Wrists, Flywheels, Intakes)
        uint32_t nextMechanismSlot = 8;
        for (const auto& mech : sidecar.mechanisms)
        {
            Core::ECS::Core::Entity mechEntity = scene.CreateEntity();
            result.mechanismEntities[mech.name] = mechEntity;

            uint32_t mechSlot = nextMechanismSlot;
            if (nextMechanismSlot < Control::MAX_MOTOR_SLOTS)
            {
                ++nextMechanismSlot;
            }

            if (mech.role == "elevator")
            {
                Mechanisms::Components::ElevatorComponent elev;
                elev.name = mech.name;
                elev.gearRatio = mech.gearRatio;
                elev.drumRadiusM = (mech.drumRadiusM > 1e-6) ? mech.drumRadiusM : 0.01905;
                elev.motorCount = static_cast<uint32_t>(mech.controllers.empty() ? 1 : mech.controllers.size());
                elev.coulombFrictionN = mech.coulombFriction;
                elev.viscousDampingNspm = mech.viscousDamping;
                elev.springPreloadForceN = mech.springAssist.forceN;
                elev.statorCurrentLimitA = mech.statorCurrentLimitA;
                elev.efficiency = mech.efficiency;
                elev.motorSlots = { mechSlot };
                //elev.motorModel = CreateMotorFromPreset(mech.motorPreset, elev.gearRatio, elev.efficiency).WithMotorCount(elev.motorCount);
                elev.motorModel = CreateMotorFromPreset(mech.motorPreset, elev.gearRatio, elev.efficiency);

                auto jIt = result.jointEntities.find(mech.targetJoint);
                if (jIt != result.jointEntities.end())
                {
                    Mechanisms::Components::ElevatorStageInfo stage;
                    stage.jointEntity = jIt->second;

                    const auto* jDef = manifest.FindJoint(mech.targetJoint);
                    if (jDef)
                    {
                        auto lIt = result.linkEntities.find(jDef->childLink);
                        if (lIt != result.linkEntities.end())
                        {
                            stage.stageEntity = lIt->second;
                        }
                        const auto* lDef = manifest.FindLink(jDef->childLink);
                        if (lDef)
                        {
                            stage.stageMassKg = lDef->massKg;
                            // Accumulate directly coupled child slider stage mass (e.g., carriage on stage 1)
                            for (const auto& childJ : manifest.joints)
                            {
                                if (childJ.parentLink == jDef->childLink && childJ.type == "prismatic")
                                {
                                    const auto* childLDef = manifest.FindLink(childJ.childLink);
                                    if (childLDef)
                                    {
                                        stage.stageMassKg += childLDef->massKg;
                                    }
                                }
                            }
                        }
                        if (jDef->limits.hasLimits)
                        {
                            stage.maxTravelM = jDef->limits.upper - jDef->limits.lower;
                        }
                    }
                    elev.stages.push_back(stage);
                }

                scene.AddComponent(mechEntity, elev);
            }
            else if (mech.role == "arm" || mech.role == "wrist" || mech.role == "four_bar")
            {
                Mechanisms::Components::ArmComponent arm;
                arm.name = mech.name;
                arm.gearRatio = mech.gearRatio;
                arm.motorCount = static_cast<uint32_t>(mech.controllers.empty() ? 1 : mech.controllers.size());
                arm.coulombFrictionNm = mech.coulombFriction;
                arm.viscousDampingNms = mech.viscousDamping;
                arm.statorCurrentLimitA = mech.statorCurrentLimitA;
                arm.efficiency = mech.efficiency;
                arm.motorSlots = { mechSlot };
                //arm.motorModel = CreateMotorFromPreset(mech.motorPreset, arm.gearRatio, arm.efficiency).WithMotorCount(arm.motorCount);
                arm.motorModel = CreateMotorFromPreset(mech.motorPreset, arm.gearRatio, arm.efficiency);

                auto jIt = result.jointEntities.find(mech.targetJoint);
                if (jIt != result.jointEntities.end())
                {
                    arm.jointEntity = jIt->second;
                }

                const auto* jDef = manifest.FindJoint(mech.targetJoint);
                if (jDef)
                {
                    auto lIt = result.linkEntities.find(jDef->childLink);
                    if (lIt != result.linkEntities.end())
                    {
                        arm.linkEntity = lIt->second;
                    }
                    const auto* lDef = manifest.FindLink(jDef->childLink);
                    if (lDef)
                    {
                        arm.linkMassKg = lDef->massKg;
                        arm.centerOfMassDistanceM = glm::length(lDef->centerOfMassM);
                        arm.springType = Mechanisms::Components::ArmSpringType::GasStrut;
                        arm.springPreloadTorqueNm = arm.linkMassKg * 9.80665 * arm.centerOfMassDistanceM;
                        arm.springRestAngleRad = 0.0;

                        if (arm.linkEntity != Core::ECS::Core::invalidEntity)
                        {
                            auto* linkRb = scene.GetComponent<Physics::Components::RigidBodyComponent>(arm.linkEntity);
                            if (linkRb)
                            {
                                //float reflI = static_cast<float>(arm.motorModel.GetRotorInertia() * arm.gearRatio * arm.gearRatio);
                                float reflI = static_cast<float>(arm.motorModel.GetRotorInertia() * arm.motorCount * arm.gearRatio * arm.gearRatio);
                                linkRb->hasCustomInertia = true;
                                linkRb->inertiaTensor[0][0] += reflI;
                                linkRb->inertiaTensor[1][1] += reflI;
                                linkRb->inertiaTensor[2][2] += reflI;
                            }
                        }
                    }
                    if (jDef->limits.hasLimits)
                    {
                        arm.minAngleRad = jDef->limits.lower;
                        arm.maxAngleRad = jDef->limits.upper;
                    }
                }

                scene.AddComponent(mechEntity, arm);
            }
            else if (mech.role == "flywheel" || mech.role == "intake")
            {
                Mechanisms::Components::FlywheelComponent fw;
                fw.name = mech.name;
                fw.gearRatio = mech.gearRatio;
                fw.motorCount = static_cast<uint32_t>(mech.controllers.empty() ? 1 : mech.controllers.size());
                fw.flywheelInertiaKgm2 = (mech.flywheelInertiaKgm2 > 1e-9) ? mech.flywheelInertiaKgm2 : 0.0035;
                fw.efficiency = mech.efficiency;
                fw.coulombFrictionNm = (mech.coulombFriction > 1e-6) ? mech.coulombFriction : 0.02;
                fw.viscousDampingNms = (mech.viscousDamping > 1e-9) ? mech.viscousDamping : 0.0001;
                fw.statorCurrentLimitA = mech.statorCurrentLimitA;
                fw.motorSlots = { mechSlot };
                //fw.motorModel = CreateMotorFromPreset(mech.motorPreset, fw.gearRatio, fw.efficiency).WithMotorCount(fw.motorCount);
                fw.motorModel = CreateMotorFromPreset(mech.motorPreset, fw.gearRatio, fw.efficiency);

                auto pIt = result.linkEntities.find(mech.parentLink);
                if (pIt != result.linkEntities.end())
                {
                    fw.parentEntity = pIt->second;
                }
                else
                {
                    fw.parentEntity = result.chassisEntity;
                }

                auto vIt = result.linkEntities.find(mech.visualLink);
                if (vIt != result.linkEntities.end())
                {
                    fw.visualEntity = vIt->second;
                }

                glm::vec3 spinAxisEng = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(mech.spinAxis));
                if (glm::length(spinAxisEng) > 1e-4f)
                {
                    fw.spinAxis = glm::normalize(spinAxisEng);
                }
                fw.visualLocalOffset = glm::vec3(0.0f, 0.06f, 0.06f);

                scene.AddComponent(mechEntity, fw);
            }
        }

        // 7. Spawn Couplings (PowerCoupling, FourBarLoop)
        for (const auto& coup : sidecar.couplings)
        {
            Core::ECS::Core::Entity coupEntity = scene.CreateEntity();

            if (coup.type == "power_coupling")
            {
                Mechanisms::Components::CouplingComponent comp;
                comp.name = coup.name;
                comp.ratio = coup.powerCoupling.ratio;
                comp.inverted = coup.powerCoupling.inverted;
                comp.efficiency = coup.powerCoupling.efficiency;
                comp.backlashRad = coup.powerCoupling.backlashRad;

                auto mIt = result.jointEntities.find(coup.powerCoupling.masterJoint);
                if (mIt != result.jointEntities.end())
                {
                    comp.masterJoint = mIt->second;
                }

                auto sIt = result.jointEntities.find(coup.powerCoupling.slaveJoint);
                if (sIt != result.jointEntities.end())
                {
                    comp.slaveJoint = sIt->second;
                }

                if (coup.powerCoupling.transmissionType == "belt")
                {
                    comp.transmissionType = Mechanisms::Components::TransmissionType::TimingBelt;
                }
                else if (coup.powerCoupling.transmissionType == "chain")
                {
                    comp.transmissionType = Mechanisms::Components::TransmissionType::RollerChain;
                }
                else if (coup.powerCoupling.transmissionType == "cable")
                {
                    comp.transmissionType = Mechanisms::Components::TransmissionType::Cable;
                }
                else if (coup.powerCoupling.transmissionType == "virtual_fourbar")
                {
                    comp.transmissionType = Mechanisms::Components::TransmissionType::VirtualFourBar;
                }
                else
                {
                    comp.transmissionType = Mechanisms::Components::TransmissionType::Gear;
                }

                comp.solverMode = (coup.solverMode == "jolt_constraint")
                    ? Mechanisms::Components::CouplingSolverMode::JoltConstraint
                    : Mechanisms::Components::CouplingSolverMode::AnalyticCoupled;

                scene.AddComponent(coupEntity, comp);
            }
            else if (coup.type == "four_bar_loop")
            {
                Mechanisms::Components::FourBarComponent fb;
                fb.name = coup.name;

                auto cjIt = result.jointEntities.find(coup.fourBar.crankJoint);
                if (cjIt != result.jointEntities.end()) fb.crankJoint = cjIt->second;

                auto cpjIt = result.jointEntities.find(coup.fourBar.couplerJoint);
                if (cpjIt != result.jointEntities.end()) fb.couplerJoint = cpjIt->second;

                auto fjIt = result.jointEntities.find(coup.fourBar.followerJoint);
                if (fjIt != result.jointEntities.end()) fb.followerJoint = fjIt->second;

                auto plIt = result.linkEntities.find(coup.fourBar.closureParentLink);
                if (plIt != result.linkEntities.end()) fb.followerEntity = plIt->second;

                auto clIt = result.linkEntities.find(coup.fourBar.closureChildLink);
                if (clIt != result.linkEntities.end()) fb.couplerEntity = clIt->second;

                fb.closureParentAnchor = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(coup.fourBar.closureParentAnchorM));
                fb.closureChildAnchor = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(coup.fourBar.closureChildAnchorM));
                glm::vec3 closureAxisEngine = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(coup.fourBar.closureAxis));
                closureAxisEngine.x = -closureAxisEngine.x;
                fb.hingeAxis = closureAxisEngine;

                if (fb.followerEntity != Core::ECS::Core::invalidEntity && fb.couplerEntity != Core::ECS::Core::invalidEntity)
                {
                    Core::ECS::Core::Entity closureJointEntity = scene.CreateEntity();
                    Physics::Components::JointComponent closureJc;
                    closureJc.type = Physics::Components::JointType::Hinge;
                    closureJc.parentEntity = fb.followerEntity;
                    closureJc.childEntity = fb.couplerEntity;

                    const auto& followerPose = linkPoses[coup.fourBar.closureParentLink];
                    const auto& couplerPose = linkPoses[coup.fourBar.closureChildLink];

                    // Place closure pivot to form an exact parallelogram with crank -> coupler
                    glm::vec3 followerTipLocal = glm::vec3(0.0f, 0.0f, 0.24f);
                    const auto* couplerJointDef = manifest.FindJoint(coup.fourBar.couplerJoint);
                    if (couplerJointDef)
                    {
                        followerTipLocal = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(couplerJointDef->originM));
                    }

                    glm::vec3 worldClosurePivot = followerPose.worldPos + (followerPose.worldRot * followerTipLocal);
                    glm::vec3 worldClosureAxis = glm::normalize(followerPose.worldRot * closureAxisEngine);

                    closureJc.SetWorldSpace(worldClosurePivot, worldClosureAxis);
                    closureJc.hasLimits = false;
                    scene.AddComponent(closureJointEntity, closureJc);

                    fb.closureJoint = closureJointEntity;

                    auto* folRb = scene.GetComponent<Physics::Components::RigidBodyComponent>(fb.followerEntity);
                    if (folRb)
                    {
                        folRb->hasCustomInertia = true;
                        folRb->inertiaTensor[0][0] = std::max(folRb->inertiaTensor[0][0], 0.05f);
                        folRb->inertiaTensor[1][1] = std::max(folRb->inertiaTensor[1][1], 0.05f);
                        folRb->inertiaTensor[2][2] = std::max(folRb->inertiaTensor[2][2], 0.05f);
                    }

                    // Compute exact local anchors relative to each body's Jolt COM
                    glm::vec3 com1Pos = followerPose.worldPos;
                    auto* col1 = scene.GetComponent<Physics::Components::ColliderComponent>(fb.followerEntity);
                    if (col1)
                    {
                        com1Pos += followerPose.worldRot * col1->localOffset;
                    }
                    fb.closureParentAnchor = glm::inverse(followerPose.worldRot) * (worldClosurePivot - com1Pos);

                    glm::vec3 com2Pos = couplerPose.worldPos;
                    auto* col2 = scene.GetComponent<Physics::Components::ColliderComponent>(fb.couplerEntity);
                    if (col2)
                    {
                        com2Pos += couplerPose.worldRot * col2->localOffset;
                    }
                    fb.closureChildAnchor = glm::inverse(couplerPose.worldRot) * (worldClosurePivot - com2Pos);
                }

                scene.AddComponent(coupEntity, fb);
            }
        }

        // 8. Spawn Sensors (CANcoders, IMUs, Limit Switches, Beam Breaks, Analog Inputs)
        for (const auto& s : sidecar.sensors)
        {
            Core::ECS::Core::Entity se = scene.CreateEntity();
            result.sensorEntities[s.name] = se;

            if (s.type == "cancoder")
            {
                Sensors::Components::EncoderComponent enc;
                enc.name = s.name;
                enc.bindingType = Sensors::Components::EncoderBindingType::CANcoder;
                enc.slot = s.canId;
                auto jIt = result.jointEntities.find(s.targetJoint);
                if (jIt != result.jointEntities.end())
                {
                    enc.targetJoint = jIt->second;
                }
                enc.model = Sensors::Models::EncoderSensorModel(s.cpr, s.sensorToMechanismRatio, s.inverted, s.offsetRad);
                scene.AddComponent(se, enc);
            }
            else if (s.type == "imu_pigeon2" || s.type == "imu_navx")
            {
                Sensors::Components::IMUComponent imu;
                imu.name = s.name;
                imu.parentChassis = result.chassisEntity;
                Sensors::Models::IMUConfig cfg;
                cfg.mountOffsetM = s.mountOffsetM;
                cfg.mountRotationQuat = s.mountRotationQuat;
                cfg.gyroNoiseStdDevRadS = s.gyroNoiseStdDevRadS;
                cfg.gyroBiasRandomWalkRadS2 = s.gyroBiasRandomWalkRadS2;
                cfg.accelNoiseStdDevMS2 = s.accelNoiseStdDevMS2;
                imu.model = Sensors::Models::IMUSensorModel(cfg);
                scene.AddComponent(se, imu);
            }
            else if (s.type == "limit_switch")
            {
                Sensors::Components::LimitSwitchComponent lim;
                lim.name = s.name;
                lim.dioChannel = s.dioChannel;
                lim.triggerMin = s.triggerMin;
                lim.triggerMax = s.triggerMax;
                lim.normallyClosed = s.normallyClosed;
                lim.direction = (s.limitDirection == "reverse")
                    ? Sensors::Components::LimitSwitchDirection::Reverse
                    : Sensors::Components::LimitSwitchDirection::Forward;
                auto jIt = result.jointEntities.find(s.targetJoint);
                if (jIt != result.jointEntities.end())
                {
                    lim.targetJoint = jIt->second;
                }
                scene.AddComponent(se, lim);
            }
            else if (s.type == "beam_break")
            {
                Sensors::Components::BeamBreakComponent bb;
                bb.name = s.name;
                bb.dioChannel = s.dioChannel;
                bb.normallyClosed = s.normallyClosed;
                bb.debounceTimeS = s.debounceTimeS;
                bb.sensorEntity = se;
                scene.AddComponent(se, bb);
            }
            else if (s.type == "analog_input")
            {
                Sensors::Components::AnalogInputComponent aic;
                aic.name = s.name;
                aic.analogChannel = s.analogChannel;
                aic.mode = Sensors::Components::AnalogInputMode::JointPosition;
                aic.minInputValue = s.triggerMin;
                aic.maxInputValue = (s.triggerMax > s.triggerMin) ? s.triggerMax : (s.triggerMin + 1.0);
                aic.minVoltage = 0.0;
                aic.maxVoltage = 5.0;
                auto jIt = result.jointEntities.find(s.targetJoint);
                if (jIt != result.jointEntities.end())
                {
                    aic.targetJoint = jIt->second;
                }
                scene.AddComponent(se, aic);
            }
        }

        // 9. Spawn Contact Zones
        for (const auto& cz : manifest.contactZones)
        {
            Core::ECS::Core::Entity cze = scene.CreateEntity();
            result.contactZoneEntities[cz.name] = cze;

            Sensors::Components::ContactZoneComponent comp;
            comp.name = cz.name;
            comp.role = cz.role;
            auto lIt = result.linkEntities.find(cz.parentLink);
            if (lIt != result.linkEntities.end())
            {
                comp.parentLink = lIt->second;
            }
            comp.dimensions = Schema::RobotSchemaLoader::ConvertNWUVectorToEngine(glm::vec3(cz.dimensionsM));
            scene.AddComponent(cze, comp);
        }

        // 10. Configure Swerve Drivetrain Subsystem & Attach SwerveChassisComponent
        if (sidecar.swerve.modules.size() == 4 && result.chassisEntity != Core::ECS::Core::invalidEntity)
        {
            result.swerveSystem = std::make_shared<Drivetrain::System::SwerveDriveSystem>(scene, joltContext);

            Drivetrain::Components::SwerveChassisComponent chassisComp;
            chassisComp.batteryEntity = result.batteryEntity;
            chassisComp.chassisMass = manifest.massSummary.totalMassKg > 0.0 ? manifest.massSummary.totalMassKg : 54.0;
            scene.AddComponent(result.chassisEntity, chassisComp);
        }

        // 11. Configure SimDeviceDispatcher
        result.dispatcher = std::make_shared<Bridge::SimDeviceDispatcher>();
        result.dispatcher->ConfigureFromSidecar(sidecar);

        // 12. Instantiate Official 8-Stage Execution Pipeline Systems
        result.electricalSystem = std::make_shared<Actuation::Systems::ElectricalSystem>(scene, result.swerveSystem.get());
        result.elevatorSystem = std::make_shared<Mechanisms::Systems::ElevatorSystem>(scene, joltContext);
        result.mechanismSystem = std::make_shared<Mechanisms::Systems::MechanismSystem>(scene, joltContext);
        result.flywheelSystem = std::make_shared<Mechanisms::Systems::FlywheelSystem>(scene, joltContext);
        result.compositeMassSystem = std::make_shared<Mechanisms::Systems::CompositeMassSystem>(
            scene, result.swerveSystem ? result.swerveSystem->GetModel() : nullptr, joltContext);

        if (joltContext && joltContext->IsInitialized())
        {
            result.physicsSystem = std::make_shared<Physics::Systems::PhysicsSystem>(scene, *joltContext);
        }

        result.sensorSystem = std::make_shared<Sensors::Systems::SensorSystem>(scene, joltContext);

        result.isSuccess = true;
        return result;
    }

    AssembledRobot RobotAssembler::AssembleFromFiles(const std::string& manifestPath,
        const std::string& sidecarPath,
        const std::string& materialsPath,
        const glm::vec3& worldSpawnPosition,
        const glm::quat& worldSpawnOrientation)
    {
        Schema::RobotSchemaLoader loader;
        if (!loader.LoadRobot(manifestPath, sidecarPath, materialsPath))
        {
            AssembledRobot failure;
            failure.isSuccess = false;
            failure.errorMessage = "Failed to load and validate Schema v1 files";
            return failure;
        }

        return Assemble(loader.GetManifest(), loader.GetSidecar(), loader.GetMaterials(),
            worldSpawnPosition, worldSpawnOrientation);
    }

    void RobotAssembler::StepPipeline(AssembledRobot& robot, double dt)
    {
        SIM_PROFILE_SCOPE("RobotAssembler::StepPipeline");

        // Stage 1: ElectricalSystem (Current aggregation, terminal voltage sag, brownout latch)
        if (robot.electricalSystem)
        {
            robot.electricalSystem->FixedUpdate(dt);
        }

        // Stage 2: ElevatorSystem (Linear motor forces, Coulomb friction stick, cascade/continuous rigging)
        if (robot.elevatorSystem)
        {
            robot.elevatorSystem->FixedUpdate(dt);
        }

        // Stage 3: MechanismSystem (Arm hinge torques, Coulomb stick latching, four-bar loop closure, couplings)
        if (robot.mechanismSystem)
        {
            robot.mechanismSystem->FixedUpdate(dt);
        }

        // Stage 4: FlywheelSystem (Sub-stepped 1D rotational ODEs and body reaction recoil)
        if (robot.flywheelSystem)
        {
            robot.flywheelSystem->FixedUpdate(dt);
        }

        // Stage 5: CompositeMassSystem (Dynamic CoM and Parallel Axis Theorem handoff to swerve model)
        if (robot.compositeMassSystem)
        {
            robot.compositeMassSystem->FixedUpdate(dt);
        }

        // Stage 6: SwerveDriveSystem (6-DOF analytic swerve solver and midpoint velocity handoff)
        if (robot.swerveSystem)
        {
            robot.swerveSystem->FixedUpdate(dt);
        }

        // Stage 7: PhysicsSystem (Jolt rigid body step and ECS transform sync)
        if (robot.physicsSystem)
        {
            robot.physicsSystem->FixedUpdate(dt);
        }

        // Stage 8: SensorSystem (Joint telemetry, noisy IMU, debounce beam breaks -> SimOutputs)
        if (robot.sensorSystem)
        {
            robot.sensorSystem->FixedUpdate(dt);
        }
    }
}