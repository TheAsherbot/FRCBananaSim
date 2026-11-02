/**
 * @file SensorSystem.h
 * @brief FixedUpdate system evaluating sensor models, joint telemetry, limit switches, and DIO/Analog states.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Core/ECS/Core/Scene.h"
#include "Control/SimControlBoundary.h"
#include "Robot/Sensors/Components/IMUComponent.h"
#include "Robot/Sensors/Components/EncoderComponent.h"
#include "Robot/Sensors/Components/LimitSwitchComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Components/AnalogInputComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Physics/JoltContext.h"

namespace FRCBananaSim::Robot::Sensors::Systems
{
    /**
     * @class SensorSystem
     * @brief Stage 8 fixed-timestep system populating SimOutputs from physical joints, IMUs, and contact sensors.
     */
    class SensorSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the sensor pipeline system.
         * @param scene Reference to governing ECS scene.
         * @param jolt Optional pointer to JoltContext for rigid-body chassis telemetry.
         */
        SensorSystem(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~SensorSystem() override;

        /**
         * @brief Reads post-step joint telemetry and sensor volumes, writes directly to SimOutputs.
         * @param dt Timestep duration in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Sets external SimOutputs destination. If nullptr, internal buffer is used.
         * @param outputs Pointer to target SimOutputs struct.
         */
        void SetOutputs(Control::SimOutputs* outputs);

        /**
         * @brief Gets current simulation feedback outputs.
         * @return Const reference to SimOutputs.
         */
        const Control::SimOutputs& GetOutputs() const;

        /**
         * @brief Gets mutable reference to current simulation feedback outputs.
         * @return Mutable reference to SimOutputs.
         */
        Control::SimOutputs& GetOutputs();

    private:
        Physics::JoltContext* joltContext = nullptr;
        Control::SimOutputs* externalOutputs = nullptr;
        Control::SimOutputs currentOutputs;

        glm::vec3 previousChassisVelocity = { 0.0f, 0.0f, 0.0f };
        bool hasPreviousChassisVelocity = false;
    };
}