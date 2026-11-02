/**
 * @file RobotAssembler.h
 * @brief Master programmatic robot assembly constructing ECS hierarchies, physical bodies, and the 8-stage execution pipeline from Schema v1 data.
 */

#pragma once

#include "Core/ECS/Core/Scene.h"
#include "Core/ECS/Core/Entity.h"
#include "Physics/JoltContext.h"
#include "Robot/Schema/RobotManifest.h"
#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/MaterialLibrary.h"
#include "Robot/Schema/RobotSchemaLoader.h"
#include "Bridge/SimDeviceDispatcher.h"
#include "Actuation/Systems/ElectricalSystem.h"
#include "Robot/Mechanisms/Systems/ElevatorSystem.h"
#include "Robot/Mechanisms/Systems/MechanismSystem.h"
#include "Robot/Mechanisms/Systems/FlywheelSystem.h"
#include "Robot/Mechanisms/Systems/CompositeMassSystem.h"
#include "Robot/Drivetrain/Systems/SwerveDriveSystem.h"
#include "Physics/Systems/PhysicsSystem.h"
#include "Robot/Sensors/Systems/SensorSystem.h"

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <unordered_map>

namespace FRCBananaSim::Robot
{
    /**
     * @struct AssembledRobot
     * @brief Collection of ECS entity handles, subsystems, and metadata for a Schema v1 assembled robot.
     */
    struct AssembledRobot
    {
        std::string robotName;
        Core::ECS::Core::Entity rootLinkEntity = Core::ECS::Core::invalidEntity;
        Core::ECS::Core::Entity chassisEntity = Core::ECS::Core::invalidEntity;
        Core::ECS::Core::Entity batteryEntity = Core::ECS::Core::invalidEntity;

        // Named lookup maps for links, joints, mechanisms, and sensors
        std::unordered_map<std::string, Core::ECS::Core::Entity> linkEntities;
        std::unordered_map<std::string, Core::ECS::Core::Entity> jointEntities;
        std::unordered_map<std::string, Core::ECS::Core::Entity> mechanismEntities;
        std::unordered_map<std::string, Core::ECS::Core::Entity> sensorEntities;
        std::unordered_map<std::string, Core::ECS::Core::Entity> contactZoneEntities;

        // Swerve Drivetrain Subsystem
        std::shared_ptr<Drivetrain::System::SwerveDriveSystem> swerveSystem;

        // SimDevice Dispatcher configured with CAN IDs from sidecar
        std::shared_ptr<Bridge::SimDeviceDispatcher> dispatcher;

        // System Pointers in 8-Stage Order
        std::shared_ptr<Actuation::Systems::ElectricalSystem> electricalSystem;
        std::shared_ptr<Mechanisms::Systems::ElevatorSystem> elevatorSystem;
        std::shared_ptr<Mechanisms::Systems::MechanismSystem> mechanismSystem;
        std::shared_ptr<Mechanisms::Systems::FlywheelSystem> flywheelSystem;
        std::shared_ptr<Mechanisms::Systems::CompositeMassSystem> compositeMassSystem;
        std::shared_ptr<Physics::Systems::PhysicsSystem> physicsSystem;
        std::shared_ptr<Sensors::Systems::SensorSystem> sensorSystem;

        bool isSuccess = false;
        std::string errorMessage;
    };

    /**
     * @class RobotAssembler
     * @brief Programmatically constructs the complete robot entity hierarchy and 8-stage pipeline from Schema v1 data.
     */
    class RobotAssembler
    {
    public:
        /**
         * @brief Constructs an assembler instance.
         * @param scene Reference to the governing ECS scene.
         * @param jolt Optional pointer to JoltContext for physical bodies and constraints.
         */
        RobotAssembler(Core::ECS::Core::Scene& scene, Physics::JoltContext* jolt = nullptr);

        /**
         * @brief Destructor.
         */
        ~RobotAssembler();

        /**
         * @brief Assembles a complete robot from pre-loaded Schema v1 structures.
         * @param manifest Loaded manifest.json specification.
         * @param sidecar Loaded sidecar.json electromechanical specification.
         * @param materials Loaded materials.json library.
         * @param worldSpawnPosition Spawn point in Engine world space (+Y Up, +Z Forward).
         * @param worldSpawnOrientation Spawn rotation quaternion in Engine world space.
         * @return AssembledRobot containing entity handles and system references.
         */
        AssembledRobot Assemble(const Schema::RobotManifest& manifest,
            const Schema::RobotSidecar& sidecar,
            const Schema::MaterialLibrary& materials,
            const glm::vec3& worldSpawnPosition = glm::vec3(0.0f, 0.20f, 0.0f),
            const glm::quat& worldSpawnOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

        /**
         * @brief Convenience overload assembling directly from filepaths.
         * @param manifestPath Path to manifest.json.
         * @param sidecarPath Path to sidecar.json (or empty to infer from manifest directory).
         * @param materialsPath Path to materials.json (or empty to infer from Resources/Config/materials.json).
         * @param worldSpawnPosition Spawn point.
         * @param worldSpawnOrientation Spawn rotation.
         * @return AssembledRobot.
         */
        AssembledRobot AssembleFromFiles(const std::string& manifestPath,
            const std::string& sidecarPath = "",
            const std::string& materialsPath = "",
            const glm::vec3& worldSpawnPosition = glm::vec3(0.0f, 0.20f, 0.0f),
            const glm::quat& worldSpawnOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

        /**
         * @brief Advances all 8 pipeline systems for a step of duration dt.
         * @param robot Assembled robot structure holding system pointers.
         * @param dt Timestep in seconds (0.005 s for 200 Hz).
         */
        static void StepPipeline(AssembledRobot& robot, double dt);

    private:
        Core::ECS::Core::Scene& scene;
        Physics::JoltContext* joltContext = nullptr;
    };
}