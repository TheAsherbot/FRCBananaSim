/**
 * @file SwervePanel.h
 * @brief Subsystem inspector panel for 4-module Swerve Drivetrains.
 */

#pragma once

#include <cstdint>
#include <string>
#include "Robot/Schema/RobotSidecar.h"

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class SwervePanel
     * @brief Dedicated editor for swerve drivetrain parameters, chassis link, actuator presets, and 4-module layout.
     */
    class SwervePanel
    {
    public:
        SwervePanel() = default;

        /**
         * @brief Renders the complete Swerve Drivetrain configuration panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

        /**
         * @brief Reads swerve drive parameters to match COTS presets and updates the dropdown indices.
         * @param swerve Swerve configuration to inspect.
         * @return True if a matching preset was found and selected.
         */
        bool SyncPresetFromSidecar(const Robot::Schema::SwerveConfig& swerve);

        /**
         * @brief Gets the currently selected COTS family and ratio indices.
         */
        int GetSelectedFamilyIndex() const { return selectedFamilyIdx; }
        int GetSelectedRatioIndex() const { return selectedRatioIdx; }

    private:
        // Caliper wear calculator scratch state
        float caliperDiameterInches{ 4.0f };
        bool useMetricCaliper{ false };
        float caliperDiameterMm{ 101.6f };

        // Symmetrical layout helper scratch state
        float wheelbaseM{ 0.56f };
        float trackwidthM{ 0.56f };

        // Two-tier hardware preset scratch state
        int selectedFamilyIdx{ 2 }; // Default SDS MK4n
        int selectedRatioIdx{ 1 };  // Default L2
        uint32_t lastSidecarVersion{ 0 };
        std::string lastSidecarPath;

        // Persistent Custom toggles
        bool isDriveMotorCustom{ false };
        bool isSteerMotorCustom{ false };
        bool isDriveServoCustom{ false };
        bool isSteerServoCustom{ false };
    };
}
