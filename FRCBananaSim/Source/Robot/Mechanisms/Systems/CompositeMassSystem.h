/**
 * @file CompositeMassSystem.h
 * @brief System aggregating link kinematics and computing composite mass, CG, and Parallel Axis inertia.
 */

#pragma once

#include "Core/ECS/Core/System.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Physics/JoltContext.h"
#include "Robot/Mechanisms/Components/CompositeMassComponent.h"

namespace FRCBananaSim::Robot::Mechanisms::Systems
{
    /**
     * @class CompositeMassSystem
     * @brief Computes 3D composite mass properties across articulating mechanism links and updates the swerve model.
     */
    class CompositeMassSystem : public Core::ECS::Core::ISystem
    {
    public:
        /**
         * @brief Constructs the composite mass evaluation system.
         * @param scene Reference to the governing ECS scene.
         * @param swerveModel Optional pointer to SwerveDrivetrainModel to update with computed mass properties.
         * @param jolt Optional pointer to JoltContext for reading physical body transforms.
         */
        CompositeMassSystem(Core::ECS::Core::Scene& scene,
            Robot::Drivetrain::Model::SwerveDrivetrainModel* swerveModel = nullptr,
            Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Virtual destructor.
         */
        virtual ~CompositeMassSystem() override;

        /**
         * @brief Evaluates link kinematics and updates composite mass properties across all CompositeMassComponents.
         * @param dt Timestep duration in seconds.
         */
        virtual void FixedUpdate(double dt) override;

        /**
         * @brief Sets or updates the target swerve drivetrain model receiving mass property updates.
         * @param model Pointer to SwerveDrivetrainModel.
         */
        void SetSwerveModel(Robot::Drivetrain::Model::SwerveDrivetrainModel* model);

    private:
        /**
         * @brief Calculates composite mass, CG, and Parallel Axis inertia for a single composite mass assembly.
         * @param comp Mutable reference to CompositeMassComponent.
         */
        void UpdateAssembly(Components::CompositeMassComponent& comp);

        Robot::Drivetrain::Model::SwerveDrivetrainModel* swerveModel;
        Physics::JoltContext* joltContext;
    };
}