/**
 * @file SwerveDriveSystem.h
 * @brief ECS system adapting the pure mathematical 6-DOF swerve drivetrain model to Jolt physics and visual scene trees.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Robot/Drivetrain/Systems/SwerveTerrainProbe.h"
#include "Control/SimControlBoundary.h"
#include "Physics/JoltContext.h"
#include <memory>

namespace FRCBananaSim::Robot::Drivetrain::System
{
    /**
     * @class SwerveDriveSystem
     * @brief Bridges control inputs, Jolt body state, closed-form swerve dynamics, and visual transforms.
     */
    class SwerveDriveSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the swerve drive system adapter.
         * @param scene ECS scene containing the robot entities.
         * @param joltContext Optional pointer to Jolt physics context for terrain queries and body sync.
         */
        explicit SwerveDriveSystem(Core::ECS::Core::Scene& scene,
            Physics::JoltContext* joltContext = nullptr);

        /**
         * @brief Sets incoming motor commands and enabled state.
         * @param inputs Incoming SimInputs.
         */
        void SetInputs(const Control::SimInputs& inputs);

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

        /**
         * @brief Executes 200 Hz fixed timestep drivetrain integration and Jolt midpoint velocity handoff.
         * @param dt Timestep duration in seconds (0.005 s for 200 Hz).
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Updates visual transforms of steer housings and wheels for rendering.
         */
        void RenderUpdate() override;

        /**
         * @brief Overrides terrain provider (used for headless unit testing with analytic planes or ramps).
         * @param provider Shared pointer to ITerrainProvider.
         */
        void SetCustomTerrainProvider(std::shared_ptr<Model::ITerrainProvider> provider);

        /**
         * @brief Gets access to the underlying pure simulation model.
         * @return Pointer to SwerveDrivetrainModel, or nullptr if not initialized.
         */
        Model::SwerveDrivetrainModel* GetModel();

    private:
        Physics::JoltContext* joltContext = nullptr;
        std::unique_ptr<Systems::SwerveTerrainProbe> defaultProbe;
        std::shared_ptr<Model::ITerrainProvider> activeTerrainProvider;

        Control::SimInputs currentInputs;
        Control::SimOutputs currentOutputs;

        // Midpoint velocity error correction vector between Jolt and model (NWU)
        glm::vec3 velocityDeltaLin{ 0.0f };
        glm::vec3 velocityDeltaAng{ 0.0f };
        bool isInitialized = false;

        void InitializeModel();
    };
}