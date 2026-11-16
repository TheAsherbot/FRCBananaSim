/**
 * @file ConfiguratorState.h
 * @brief Central document model, CAD ingestion state, auto-generator, and linter for SidecarConfigurator.
 */

#pragma once

#include "Robot/Schema/RobotManifest.h"
#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/MaterialLibrary.h"

#include <string>
#include <vector>
#include <set>

namespace FRCBananaSim::Tools::Configurator
{
    /**
     * @class ConfiguratorState
     * @brief Manages the open CAD manifest, active sidecar configuration, diagnostics, and linter passes.
     */
    class ConfiguratorState
    {
    public:
        /**
         * @brief Default constructor initializing blank document state.
         */
        ConfiguratorState();

        /**
         * @brief Opens and ingests an exported CAD folder containing manifest.json (and optional sidecar/materials).
         * @param folderPath Directory path containing CAD exports.
         * @return True if manifest.json was loaded and validated successfully.
         */
        bool LoadCADFolder(const std::string& folderPath);

        /**
         * @brief Directly loads a manifest.json file.
         * @param filePath Full path to manifest.json.
         * @return True if parsing and topological verification succeeded.
         */
        bool LoadManifest(const std::string& filePath);

        /**
         * @brief Directly loads an existing sidecar.json file.
         * @param filePath Full path to sidecar.json.
         * @return True if parsed successfully.
         */
        bool LoadSidecar(const std::string& filePath);

        /**
         * @brief Serializes and writes the current RobotSidecar model to a JSON file.
         * @param filePath Target output path (defaults to sidecarPath or folder/sidecar.json).
         * @return True if serialized and written cleanly.
         */
        bool SaveSidecar(const std::string& filePath = "");

        /**
         * @brief One-click bootstrap feature: inspects manifest.json to auto-seed a complete starter sidecar.
         *
         * Generates:
         * 1. 4-module Swerve Drivetrain configuration from CAD module instances.
         * 2. 1-DOF Arm/Wrist mechanisms for revolute joints.
         * 3. 1-DOF Elevator mechanisms for prismatic joints.
         * 4. IMU sensor bound to root chassis link.
         */
        void AutoGenerateStarterSidecar();

        /**
         * @brief Runs live diagnostic validation across CAN topologies, PWM channels, orphan joints, and differentials.
         */
        void RunLinter();

        /**
         * @brief Resets all document state to defaults.
         */
        void Clear();

        /**
         * @brief Normalizes and checks if two folder paths refer to the same directory on disk.
         * @param a First path string.
         * @param b Second path string.
         * @return True if equivalent or identical.
         */
        static bool ArePathsEquivalent(const std::string& a, const std::string& b);

        /**
         * @brief Resolves the path to configurator_settings.json across project directory hierarchies.
         * @param customPath Optional explicit path.
         * @return Resolved settings file path.
         */
        static std::string ResolveSettingsPath(const std::string& customPath = "");

        /**
         * @brief Loads the list of up to 3 most recently opened robot folders from settings.
         * @param settingsFilePath Optional custom path to configurator_settings.json.
         * @return Vector of up to 3 recent robot directory paths (MRU order).
         */
        static std::vector<std::string> GetRecentRobots(const std::string& settingsFilePath = "");

        /**
         * @brief Adds a robot folder to the MRU list (max 3), updates lastRobotPath, and saves to settings JSON.
         * @param folderPath The directory path of the robot.
         * @param settingsFilePath Optional custom path to configurator_settings.json.
         */
        static void AddRecentRobot(const std::string& folderPath, const std::string& settingsFilePath = "");

        // Document paths
        std::string cadFolderPath;
        std::string manifestPath;
        std::string sidecarPath;
        std::string materialsPath;

        // Data models
        Robot::Schema::RobotManifest manifest;
        Robot::Schema::RobotSidecar sidecar;
        Robot::Schema::MaterialLibrary materials;

        // Status flags
        bool isCadLoaded{ false };
        bool isSidecarLoaded{ false };
        uint32_t sidecarVersion{ 1 };

        // Selection state
        enum class ActiveInspectorTab
        {
            CADSelection = 0,
            SwerveDrivetrain,
            Mechanisms,
            Couplings,
            Sensors,
            MassReconciliation,
            Bumpers
        };
        ActiveInspectorTab activeInspectorTab{ ActiveInspectorTab::CADSelection };

        std::string selectedLink;
        std::string selectedJoint;
        std::string selectedMechanism;
        std::string selectedSensor;

        // Diagnostics
        std::string statusMessage{ "Ready. Open a CAD folder or load Reference2026 to begin." };
        std::vector<std::string> linterErrors;
        std::vector<std::string> linterWarnings;
    };
}
