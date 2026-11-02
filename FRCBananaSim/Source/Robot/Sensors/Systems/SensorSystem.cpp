/**
 * @file SensorSystem.cpp
 * @brief Implementation of sensor pipeline populating SimOutputs at 200 Hz.
 */

#include "Robot/Sensors/Systems/SensorSystem.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Physics/Components/RigidBodyComponent.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Robot::Sensors::Systems
{
    SensorSystem::SensorSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt)
        : ISystem(scene),
        joltContext(jolt),
        externalOutputs(nullptr),
        currentOutputs()
    {}

    SensorSystem::~SensorSystem()
    {}

    void SensorSystem::SetOutputs(Control::SimOutputs* outputs)
    {
        externalOutputs = outputs;
    }

    const Control::SimOutputs& SensorSystem::GetOutputs() const
    {
        return (externalOutputs != nullptr) ? *externalOutputs : currentOutputs;
    }

    Control::SimOutputs& SensorSystem::GetOutputs()
    {
        return (externalOutputs != nullptr) ? *externalOutputs : currentOutputs;
    }

    void SensorSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("SensorSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        Control::SimOutputs& out = (externalOutputs != nullptr) ? *externalOutputs : currentOutputs;

        // 1. Process Elevators -> Motor Slots Feedback
        auto elevatorEntities = scene.Query<Mechanisms::Components::ElevatorComponent>();
        for (auto entity : elevatorEntities)
        {
            auto* elevator = scene.GetComponent<Mechanisms::Components::ElevatorComponent>(entity);
            if (!elevator)
            {
                continue;
            }

            double drumCircumference = 2.0 * std::numbers::pi * elevator->drumRadiusM;
            double drumRevs = (drumCircumference > 1e-6) ? (elevator->totalCarriageHeightM / drumCircumference) : 0.0;
            double drumRps = (drumCircumference > 1e-6) ? (elevator->totalCarriageVelocityMps / drumCircumference) : 0.0;
            double perMotorCurrent = elevator->statorCurrentA / static_cast<double>(std::max(1u, elevator->motorCount));

            for (uint32_t slot : elevator->motorSlots)
            {
                if (slot < Control::MAX_MOTOR_SLOTS)
                {
                    out.encoderPositions[slot] = elevator->totalCarriageHeightM;
                    out.encoderVelocities[slot] = elevator->totalCarriageVelocityMps;
                    out.rotorPositionsRot[slot] = drumRevs * elevator->gearRatio;
                    out.rotorVelocitiesRps[slot] = drumRps * elevator->gearRatio;
                    out.motorCurrents[slot] = perMotorCurrent;
                }
            }
        }

        // 2. Process Arms -> Motor Slots Feedback
        auto armEntities = scene.Query<Mechanisms::Components::ArmComponent>();
        for (auto entity : armEntities)
        {
            auto* arm = scene.GetComponent<Mechanisms::Components::ArmComponent>(entity);
            if (!arm)
            {
                continue;
            }

            constexpr double invTwoPi = 1.0 / (2.0 * std::numbers::pi);
            double rotorRot = arm->currentAngleRad * invTwoPi * arm->gearRatio;
            double rotorRps = arm->currentVelocityRadPerSec * invTwoPi * arm->gearRatio;
            double perMotorCurrent = arm->statorCurrentA / static_cast<double>(std::max(1u, arm->motorCount));

            for (uint32_t slot : arm->motorSlots)
            {
                if (slot < Control::MAX_MOTOR_SLOTS)
                {
                    out.encoderPositions[slot] = arm->currentAngleRad;
                    out.encoderVelocities[slot] = arm->currentVelocityRadPerSec;
                    out.rotorPositionsRot[slot] = rotorRot;
                    out.rotorVelocitiesRps[slot] = rotorRps;
                    out.motorCurrents[slot] = perMotorCurrent;
                }
            }
        }

        // 3. Process Flywheels -> Motor Slots Feedback
        auto flywheelEntities = scene.Query<Mechanisms::Components::FlywheelComponent>();
        for (auto entity : flywheelEntities)
        {
            auto* flywheel = scene.GetComponent<Mechanisms::Components::FlywheelComponent>(entity);
            if (!flywheel)
            {
                continue;
            }

            constexpr double invTwoPi = 1.0 / (2.0 * std::numbers::pi);
            double rotorRot = flywheel->angularPositionRad * invTwoPi * flywheel->gearRatio;
            double rotorRps = flywheel->angularVelocityRadPerSec * invTwoPi * flywheel->gearRatio;
            double perMotorCurrent = flywheel->statorCurrentA / static_cast<double>(std::max(1u, flywheel->motorCount));

            for (uint32_t slot : flywheel->motorSlots)
            {
                if (slot < Control::MAX_MOTOR_SLOTS)
                {
                    out.encoderPositions[slot] = flywheel->angularPositionRad;
                    out.encoderVelocities[slot] = flywheel->angularVelocityRadPerSec;
                    out.rotorPositionsRot[slot] = rotorRot;
                    out.rotorVelocitiesRps[slot] = rotorRps;
                    out.motorCurrents[slot] = perMotorCurrent;
                }
            }
        }

        // 4. Process Standalone Encoders & CANcoders
        auto encoderEntities = scene.Query<Components::EncoderComponent>();
        for (auto entity : encoderEntities)
        {
            auto* enc = scene.GetComponent<Components::EncoderComponent>(entity);
            if (!enc)
            {
                continue;
            }

            double jointPos = 0.0;
            double jointVel = 0.0;
            if (enc->targetJoint != Core::ECS::Core::invalidEntity)
            {
                auto* joint = scene.GetComponent<Physics::Components::JointComponent>(enc->targetJoint);
                if (joint)
                {
                    jointPos = joint->currentPosition;
                    jointVel = joint->currentVelocity;
                }
            }

            enc->model.Update(jointPos, jointVel, dt);

            if (enc->bindingType == Components::EncoderBindingType::CANcoder)
            {
                if (enc->slot < Control::MAX_CANCODER_SLOTS)
                {
                    out.cancoderPositionsRot[enc->slot] = enc->model.GetPositionRotations();
                    out.cancoderVelocitiesRps[enc->slot] = enc->model.GetVelocityRps();
                }
            }
            else if (enc->bindingType == Components::EncoderBindingType::MotorRotor)
            {
                if (enc->slot < Control::MAX_MOTOR_SLOTS)
                {
                    out.rotorPositionsRot[enc->slot] = enc->model.GetPositionRotations();
                    out.rotorVelocitiesRps[enc->slot] = enc->model.GetVelocityRps();
                }
            }
        }

        // 5. Process 6-Axis IMUs (Pigeon 2 / navX)
        auto imuEntities = scene.Query<Components::IMUComponent>();
        for (auto entity : imuEntities)
        {
            auto* imu = scene.GetComponent<Components::IMUComponent>(entity);
            if (!imu)
            {
                continue;
            }

            if (imu->useKinematicOverride)
            {
                imu->model.Update(imu->overrideAngularVelocityNWU,
                    imu->overrideLinearAccelNWU,
                    imu->overrideOrientationNWU,
                    dt);
            }
            else
            {
                glm::dvec3 angularVelNWU(0.0);
                glm::dvec3 linearAccelNWU(0.0);
                glm::dquat orientationNWU(1.0, 0.0, 0.0, 0.0);

                if (imu->parentChassis != Core::ECS::Core::invalidEntity && joltContext && joltContext->IsInitialized())
                {
                    auto* rb = scene.GetComponent<Physics::Components::RigidBodyComponent>(imu->parentChassis);
                    if (rb && !rb->bodyID.IsInvalid())
                    {
                        JPH::BodyInterface& bi = joltContext->GetBodyInterface();
                        JPH::Vec3 joltAngVel = bi.GetAngularVelocity(rb->bodyID);
                        JPH::Vec3 joltLinVel = bi.GetLinearVelocity(rb->bodyID);
                        JPH::Quat joltRot = bi.GetRotation(rb->bodyID);

                        glm::vec3 engineAngVel(joltAngVel.GetX(), joltAngVel.GetY(), joltAngVel.GetZ());
                        glm::vec3 engineLinVel(joltLinVel.GetX(), joltLinVel.GetY(), joltLinVel.GetZ());
                        glm::quat engineRot(joltRot.GetW(), joltRot.GetX(), joltRot.GetY(), joltRot.GetZ());

                        glm::vec3 engineLinAccel(0.0f);
                        if (hasPreviousChassisVelocity)
                        {
                            engineLinAccel = (engineLinVel - previousChassisVelocity) / static_cast<float>(dt);
                        }
                        previousChassisVelocity = engineLinVel;
                        hasPreviousChassisVelocity = true;

                        angularVelNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(engineAngVel);
                        linearAccelNWU = Schema::RobotSchemaLoader::ConvertEngineVectorToNWU(engineLinAccel);
                        orientationNWU = Schema::RobotSchemaLoader::ConvertEngineQuatToNWU(engineRot);
                    }
                }

                imu->model.Update(angularVelNWU, linearAccelNWU, orientationNWU, dt);
            }

            out.gyroYaw = imu->model.GetYawRad();
            out.gyroPitch = imu->model.GetPitchRad();
            out.gyroRoll = imu->model.GetRollRad();
            out.gyroYawRate = imu->model.GetYawRateRadPerSec();
            out.gyroAngularVelocityRadPerSec = imu->model.GetAngularVelocityRadPerSec();
            out.accelSpecificForceMetersPerSec2 = imu->model.GetSpecificForceMetersPerSec2();
            out.imuOrientationQuat = imu->model.GetOrientationQuat();
            break; // Primary IMU writes to SimOutputs singleton slots
        }

        // 6. Process Limit Switches
        auto limitEntities = scene.Query<Components::LimitSwitchComponent>();
        for (auto entity : limitEntities)
        {
            auto* limit = scene.GetComponent<Components::LimitSwitchComponent>(entity);
            if (!limit)
            {
                continue;
            }

            double jointPos = 0.0;
            if (limit->targetJoint != Core::ECS::Core::invalidEntity)
            {
                auto* joint = scene.GetComponent<Physics::Components::JointComponent>(limit->targetJoint);
                if (joint)
                {
                    jointPos = joint->currentPosition;
                }
            }

            // Hysteresis trip band evaluation
            if (limit->isTriggered)
            {
                if (jointPos < (limit->triggerMin - limit->hysteresis) || jointPos >(limit->triggerMax + limit->hysteresis))
                {
                    limit->isTriggered = false;
                }
            }
            else
            {
                if (jointPos >= limit->triggerMin && jointPos <= limit->triggerMax)
                {
                    limit->isTriggered = true;
                }
            }

            limit->dioState = limit->normallyClosed ? !limit->isTriggered : limit->isTriggered;

            if (limit->dioChannel < 32)
            {
                if (limit->dioState)
                {
                    out.digitalInputs |= (1u << limit->dioChannel);
                }
                else
                {
                    out.digitalInputs &= ~(1u << limit->dioChannel);
                }
            }

            if (limit->attachToMotorSlot < Control::MAX_MOTOR_SLOTS)
            {
                if (limit->direction == Components::LimitSwitchDirection::Forward)
                {
                    out.forwardLimitTriggered[limit->attachToMotorSlot] = limit->isTriggered;
                }
                else if (limit->direction == Components::LimitSwitchDirection::Reverse)
                {
                    out.reverseLimitTriggered[limit->attachToMotorSlot] = limit->isTriggered;
                }
            }
        }

        // 7. Process Beam Breaks
        auto beamEntities = scene.Query<Components::BeamBreakComponent>();
        for (auto entity : beamEntities)
        {
            auto* beam = scene.GetComponent<Components::BeamBreakComponent>(entity);
            if (!beam)
            {
                continue;
            }

            if (beam->sensorEntity != Core::ECS::Core::invalidEntity)
            {
                auto* cz = scene.GetComponent<Components::ContactZoneComponent>(beam->sensorEntity);
                if (cz)
                {
                    beam->isBroken = cz->isEngaged;
                }
            }

            bool physicalActive = beam->normallyClosed ? !beam->isBroken : beam->isBroken;

            if (physicalActive != beam->dioState)
            {
                beam->debounceTimerS += dt;
                if (beam->debounceTimerS >= beam->debounceTimeS)
                {
                    beam->dioState = physicalActive;
                    beam->debounceTimerS = 0.0;
                }
            }
            else
            {
                beam->debounceTimerS = 0.0;
            }

            if (beam->dioChannel < 32)
            {
                if (beam->dioState)
                {
                    out.digitalInputs |= (1u << beam->dioChannel);
                }
                else
                {
                    out.digitalInputs &= ~(1u << beam->dioChannel);
                }
            }
        }

        // 8. Process Analog Inputs
        auto analogEntities = scene.Query<Components::AnalogInputComponent>();
        for (auto entity : analogEntities)
        {
            auto* analog = scene.GetComponent<Components::AnalogInputComponent>(entity);
            if (!analog)
            {
                continue;
            }

            if (analog->mode == Components::AnalogInputMode::JointPosition)
            {
                double jointPos = 0.0;
                if (analog->targetJoint != Core::ECS::Core::invalidEntity)
                {
                    auto* joint = scene.GetComponent<Physics::Components::JointComponent>(analog->targetJoint);
                    if (joint)
                    {
                        jointPos = joint->currentPosition;
                    }
                }

                double span = analog->maxInputValue - analog->minInputValue;
                double t = (std::abs(span) > 1e-9) ? std::clamp((jointPos - analog->minInputValue) / span, 0.0, 1.0) : 0.0;
                analog->voltage = std::clamp(analog->minVoltage + t * (analog->maxVoltage - analog->minVoltage), 0.0, 5.0);
            }
            else if (analog->mode == Components::AnalogInputMode::CustomVoltage)
            {
                analog->voltage = std::clamp(analog->voltage, 0.0, 5.0);
            }

            if (analog->analogChannel < Control::MAX_ANALOG_INPUTS)
            {
                out.analogInputs[analog->analogChannel] = analog->voltage;
            }
        }
    }
}