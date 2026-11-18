/**
 * @file SwervePanel.cpp
 * @brief Implementation of the Swerve Drivetrain configuration panel.
 */

#include "Tools/Configurator/Panels/SwervePanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>
#include "ConfiguratorWidgets.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

namespace FRCBananaSim::Tools::Configurator
{

    struct ModuleRatioOption
    {
        const char* ratioName;
        double driveRatio;
        double steerRatio;
        double wheelRadiusM;
        double driveEff;
        double steerEff;
    };

    struct SwerveFamilyDefinition
    {
        const char* familyName;
        std::vector<ModuleRatioOption> ratios;
    };

    static const SwerveFamilyDefinition g_SwerveFamilies[] = {
        {
            "SDS MK4 (Standard)",
            {
                { "L1 (8.14 : 1)", 8.14, 12.800, 0.0508, 0.96, 0.92 },
                { "L2 (6.75 : 1)", 6.75, 12.800, 0.0508, 0.96, 0.92 },
                { "L3 (6.12 : 1)", 6.12, 12.800, 0.0508, 0.96, 0.92 },
                { "L4 (5.14 : 1)", 5.14, 12.800, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "SDS MK4i (Inverted)",
            {
                { "L1 (8.14 : 1)", 8.14, 21.428, 0.0508, 0.96, 0.92 },
                { "L2 (6.75 : 1)", 6.75, 21.428, 0.0508, 0.96, 0.92 },
                { "L3 (6.12 : 1)", 6.12, 21.428, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "SDS MK4n (Narrow)",
            {
                { "L1 (7.13 : 1)", 7.13, 18.750, 0.0508, 0.96, 0.92 },
                { "L2 (5.90 : 1)", 5.90, 18.750, 0.0508, 0.96, 0.92 },
                { "L3 (5.36 : 1)", 5.36, 18.750, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "SDS MK5i (Next-Gen Inverted)",
            {
                { "R1 (7.03 : 1)", 7.03, 26.000, 0.0508, 0.96, 0.92 },
                { "R2 (6.03 : 1)", 6.03, 26.000, 0.0508, 0.96, 0.92 },
                { "R3 (5.27 : 1)", 5.27, 26.000, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "SDS MK5n (Next-Gen Narrow)",
            {
                { "R1 (7.03 : 1)", 7.03, 26.091, 0.0508, 0.96, 0.92 },
                { "R2 (6.03 : 1)", 6.03, 26.091, 0.0508, 0.96, 0.92 },
                { "R3 (5.27 : 1)", 5.27, 26.091, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "WCP SwerveX (Standard)",
            {
                { "Standard (6.55 : 1)", 6.55, 10.290, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "WCP SwerveX (Flipped)",
            {
                { "Flipped (6.55 : 1)", 6.55, 13.370, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "WCP SwerveX2",
            {
                { "L1 (6.55 : 1)", 6.55, 12.000, 0.0508, 0.96, 0.92 },
                { "L2 (5.83 : 1)", 5.83, 12.000, 0.0508, 0.96, 0.92 },
                { "L3 (5.25 : 1)", 5.25, 12.000, 0.0508, 0.96, 0.92 }
            }
        },
        {
            "REV MAXSwerve",
            {
                { "Standard (5.50 : 1)",   5.50, 28.750, 0.0381, 0.94, 0.90 },
                { "High Speed (4.75 : 1)", 4.75, 28.750, 0.0381, 0.94, 0.90 },
                { "Low Speed (6.75 : 1)",  6.75, 28.750, 0.0381, 0.94, 0.90 }
            }
        }
    };

    bool SwervePanel::SyncPresetFromSidecar(const Robot::Schema::SwerveConfig& swerve)
    {
        int bestFam = -1;
        int bestRatio = -1;
        double minDiff = 1e9;

        for (int f = 0; f < static_cast<int>(IM_ARRAYSIZE(g_SwerveFamilies)); ++f)
        {
            const auto& fam = g_SwerveFamilies[f];
            for (int r = 0; r < static_cast<int>(fam.ratios.size()); ++r)
            {
                const auto& opt = fam.ratios[r];
                double dDrive = std::abs(swerve.driveGearRatio - opt.driveRatio);
                double dSteer = std::abs(swerve.steerGearRatio - opt.steerRatio);
                double dWheel = std::abs(swerve.wheelRadiusM - opt.wheelRadiusM);

                // Tolerance checks:
                // Drive ratio within 0.05
                // Steer ratio within 0.06 (handles 21.43 vs 21.428, 26.0909 vs 26.091)
                // Wheel radius within 15mm (allows worn tread from 4" down to 3.5")
                if (dDrive < 0.05 && dSteer < 0.06 && dWheel < 0.015)
                {
                    double totalDiff = dDrive + dSteer + dWheel * 10.0;
                    if (totalDiff < minDiff)
                    {
                        minDiff = totalDiff;
                        bestFam = f;
                        bestRatio = r;
                    }
                }
            }
        }

        if (bestFam != -1 && bestRatio != -1)
        {
            selectedFamilyIdx = bestFam;
            selectedRatioIdx = bestRatio;
            return true;
        }

        return false;
    }

    void SwervePanel::Render(ConfiguratorState& state)
    {
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "SWERVE DRIVETRAIN SUBSYSTEM");
        ImGui::Separator();

        if (!state.isSidecarLoaded && !state.isCadLoaded)
        {
            ImGui::TextWrapped("No robot loaded. Open a CAD folder or load a reference robot to configure swerve.");
            return;
        }

        auto& swerve = state.sidecar.swerve;

        // Auto-detect matching COTS swerve preset whenever a sidecar is loaded or reloaded
        if (lastSidecarVersion != state.sidecarVersion || lastSidecarPath != state.sidecarPath)
        {
            lastSidecarVersion = state.sidecarVersion;
            lastSidecarPath = state.sidecarPath;
            SyncPresetFromSidecar(swerve);
        }

        auto applyRatioOption = [&](const ModuleRatioOption& opt)
        {
            swerve.driveGearRatio = opt.driveRatio;
            swerve.steerGearRatio = opt.steerRatio;
            swerve.wheelRadiusM = opt.wheelRadiusM;
            swerve.driveEfficiency = opt.driveEff;
            swerve.steerEfficiency = opt.steerEff;
            state.RunLinter();
        };

        // -------------------------------------------------------------
        // 1. Chassis Root Link & Two-Tier Hardware Presets
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Chassis Link & Hardware Presets", ImGuiTreeNodeFlags_DefaultOpen))
        {
            std::vector<std::string> linkNames;
            int selectedLinkIdx = -1;
            for (size_t i = 0; i < state.manifest.links.size(); ++i)
            {
                linkNames.push_back(state.manifest.links[i].name);
                if (state.manifest.links[i].name == swerve.chassisLink)
                {
                    selectedLinkIdx = static_cast<int>(i);
                }
            }

            if (selectedLinkIdx == -1 && !linkNames.empty())
            {
                swerve.chassisLink = linkNames[0];
                selectedLinkIdx = 0;
            }

            std::vector<const char*> linkNamePtrs;
            for (const auto& name : linkNames)
            {
                linkNamePtrs.push_back(name.c_str());
            }

            ImGui::PushItemWidth(180.0f);
            if (!linkNamePtrs.empty())
            {
                if (ImGui::Combo("Chassis Link", &selectedLinkIdx, linkNamePtrs.data(), static_cast<int>(linkNamePtrs.size())))
                {
                    swerve.chassisLink = linkNames[selectedLinkIdx];
                    state.RunLinter();
                }
            }
            else
            {
                char chassisBuf[64];
                std::snprintf(chassisBuf, sizeof(chassisBuf), "%s", swerve.chassisLink.c_str());
                if (ImGui::InputText("Chassis Link", chassisBuf, sizeof(chassisBuf)))
                {
                    swerve.chassisLink = chassisBuf;
                    state.RunLinter();
                }
            }
            ImGui::PopItemWidth();

            ImGui::Spacing();
            ImGui::TextDisabled("Select COTS Swerve Module Hardware:");

            // Check if current sidecar swerve ratios match the selected COTS preset
            bool isMatchingCurrent = false;
            if (selectedFamilyIdx >= 0 && selectedFamilyIdx < static_cast<int>(IM_ARRAYSIZE(g_SwerveFamilies)))
            {
                const auto& fam = g_SwerveFamilies[selectedFamilyIdx];
                if (selectedRatioIdx >= 0 && selectedRatioIdx < static_cast<int>(fam.ratios.size()))
                {
                    const auto& opt = fam.ratios[selectedRatioIdx];
                    double dDrive = std::abs(swerve.driveGearRatio - opt.driveRatio);
                    double dSteer = std::abs(swerve.steerGearRatio - opt.steerRatio);
                    double dWheel = std::abs(swerve.wheelRadiusM - opt.wheelRadiusM);
                    if (dDrive < 0.05 && dSteer < 0.06 && dWheel < 0.015)
                    {
                        isMatchingCurrent = true;
                    }
                }
            }

            ImGui::SameLine();
            if (isMatchingCurrent)
            {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[COTS Preset Active]");
            }
            else
            {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[Custom / Modified]");
            }

            // Tier 1: Swerve Family Dropdown
            std::vector<const char*> familyNames;
            for (const auto& fam : g_SwerveFamilies)
            {
                familyNames.push_back(fam.familyName);
            }

            ImGui::SetNextItemWidth(210.0f);
            if (ImGui::Combo("Module Family", &selectedFamilyIdx, familyNames.data(), static_cast<int>(familyNames.size())))
            {
                selectedRatioIdx = 0; // Reset ratio index when family changes
            }

            // Tier 2: Gearset Ratio Sub-Dropdown (ONLY appears for families that have ratio choices)
            if (selectedFamilyIdx >= 0 && selectedFamilyIdx < static_cast<int>(IM_ARRAYSIZE(g_SwerveFamilies)))
            {
                const auto& activeFamily = g_SwerveFamilies[selectedFamilyIdx];
                if (activeFamily.ratios.size() > 1)
                {
                    std::vector<const char*> ratioNames;
                    for (const auto& r : activeFamily.ratios)
                    {
                        ratioNames.push_back(r.ratioName);
                    }

                    if (selectedRatioIdx >= static_cast<int>(activeFamily.ratios.size()))
                    {
                        selectedRatioIdx = 0;
                    }

                    ImGui::SetNextItemWidth(170.0f);
                    ImGui::Combo("Gearset Ratio", &selectedRatioIdx, ratioNames.data(), static_cast<int>(ratioNames.size()));
                    ImGui::SameLine(0.0f, 8.0f);
                }

                if (ImGui::Button("Apply Drivetrain Hardware"))
                {
                    if (selectedRatioIdx < static_cast<int>(activeFamily.ratios.size()))
                    {
                        applyRatioOption(activeFamily.ratios[selectedRatioIdx]);
                    }
                }
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Quick Presets:");
            float btnWidth = (ImGui::GetContentRegionAvail().x - 24.0f) * 0.20f;
            if (btnWidth < 65.0f) btnWidth = 65.0f;

            if (ImGui::Button("MK4i L2", ImVec2(btnWidth, 22.0f)))
            {
                selectedFamilyIdx = 1; selectedRatioIdx = 1;
                applyRatioOption(g_SwerveFamilies[1].ratios[1]);
            }
            ImGui::SameLine(0.0f, 6.0f);
            if (ImGui::Button("MK4n L2", ImVec2(btnWidth, 22.0f)))
            {
                selectedFamilyIdx = 2; selectedRatioIdx = 1;
                applyRatioOption(g_SwerveFamilies[2].ratios[1]);
            }
            ImGui::SameLine(0.0f, 6.0f);
            if (ImGui::Button("MK5i R2", ImVec2(btnWidth, 22.0f)))
            {
                selectedFamilyIdx = 3; selectedRatioIdx = 1;
                applyRatioOption(g_SwerveFamilies[3].ratios[1]);
            }
            ImGui::SameLine(0.0f, 6.0f);
            if (ImGui::Button("MK5n R2", ImVec2(btnWidth, 22.0f)))
            {
                selectedFamilyIdx = 4; selectedRatioIdx = 1;
                applyRatioOption(g_SwerveFamilies[4].ratios[1]);
            }
            ImGui::SameLine(0.0f, 6.0f);
            if (ImGui::Button("MAXSwerve", ImVec2(btnWidth, 22.0f)))
            {
                selectedFamilyIdx = 8; selectedRatioIdx = 0;
                applyRatioOption(g_SwerveFamilies[8].ratios[0]);
            }
        }

        // -------------------------------------------------------------
        // 2. Actuators, Motors & Servos
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Actuators & Motor / Servo Presets", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextDisabled("Motor Inversion Standard:");
            ImGui::SameLine(0.0f, 6.0f);
            ShowMotorInvertHelpMarker();
            ImGui::Spacing();

            // Verified motor models matching DCMotorModel.cpp
            const char* motorPresets[] = {
                "KrakenX60", "KrakenX60FOC", "KrakenX44", "KrakenX44FOC",
                "Falcon500", "Minion", "NEO", "NEO2", "NEO550", "NEO_Vortex",
                "GoBilda5202", "Custom"
            };

            // Verified datasheet specs matching ServoModel.cpp exactly
            const char* servoPresets[] = {
                "GoBildaSuperSpeed (0.08s/60°, 125 RPM)",
                "GoBildaSpeed (0.09s/60°, 111 RPM)",
                "GoBildaTorque (0.20s/60°, 50 RPM)",
                "SwyftSpeed (0.062s/60°, 161 RPM)",
                "SwyftBalanced (0.092s/60°, 109 RPM)",
                "SwyftTorque (0.112s/60°, 89 RPM)",
                "SwyftSlim (0.075s/60°, 133 RPM)",
                "SwyftNano (0.110s/60°, 91 RPM)",
                "Custom"
            };

            const char* servoIds[] = {
                "GoBildaSuperSpeed", "GoBildaSpeed", "GoBildaTorque",
                "SwyftSpeed", "SwyftBalanced", "SwyftTorque",
                "SwyftSlim", "SwyftNano", "Custom"
            };

            // A. Drive Actuator Mode
            int driveMode = (swerve.driveActuatorType == "servo") ? 1 : 0;
            ImGui::Text("Drive Actuator:");
            ImGui::SameLine();
            ImGui::RadioButton("CAN Motor##Drive", &driveMode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("PWM Servo##Drive", &driveMode, 1);
            swerve.driveActuatorType = (driveMode == 1) ? "servo" : "motor";

            if (driveMode == 0)
            {
                // Find drive motor preset index
                int drivePresetIdx = -1;
                for (int i = 0; i < IM_ARRAYSIZE(motorPresets) - 1; ++i)
                {
                    if (swerve.driveMotorPreset == motorPresets[i])
                    {
                        drivePresetIdx = i;
                        break;
                    }
                }
                if (drivePresetIdx == -1 || swerve.driveMotorPreset == "Custom" || isDriveMotorCustom)
                {
                    drivePresetIdx = IM_ARRAYSIZE(motorPresets) - 1; // "Custom"
                    isDriveMotorCustom = true;
                }
                else
                {
                    isDriveMotorCustom = false;
                }

                ImGui::SetNextItemWidth(180.0f);
                if (ImGui::Combo("Drive Motor Preset", &drivePresetIdx, motorPresets, IM_ARRAYSIZE(motorPresets)))
                {
                    if (drivePresetIdx < IM_ARRAYSIZE(motorPresets) - 1)
                    {
                        isDriveMotorCustom = false;
                        swerve.driveMotorPreset = motorPresets[drivePresetIdx];
                    }
                    else
                    {
                        isDriveMotorCustom = true;
                        swerve.driveMotorPreset = "Custom";
                    }
                    state.RunLinter();
                }

                if (isDriveMotorCustom)
                {
                    ImGui::Spacing();
                    ImGui::Indent(12.0f);
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Drive Motor Specs:");

                    float vNom = static_cast<float>(swerve.customDriveMotor.nominalVoltageV);
                    if (SliderWithDirectInput("Nominal Voltage (V)##CustomDriveMotor", &vNom, 4.0f, 24.0f, "%.1f"))
                    {
                        swerve.customDriveMotor.nominalVoltageV = static_cast<double>(vNom);
                    }

                    float stallTorque = static_cast<float>(swerve.customDriveMotor.stallTorqueNm);
                    if (SliderWithDirectInput("Stall Torque (N*m)##CustomDriveMotor", &stallTorque, 0.10f, 15.0f, "%.3f"))
                    {
                        swerve.customDriveMotor.stallTorqueNm = static_cast<double>(stallTorque);
                    }

                    float freeSpeed = static_cast<float>(swerve.customDriveMotor.freeSpeedRpm);
                    if (SliderWithDirectInput("Free Speed (RPM)##CustomDriveMotor", &freeSpeed, 500.0f, 25000.0f, "%.0f"))
                    {
                        swerve.customDriveMotor.freeSpeedRpm = static_cast<double>(freeSpeed);
                    }

                    float stallCurr = static_cast<float>(swerve.customDriveMotor.stallCurrentA);
                    if (SliderWithDirectInput("Stall Current (A)##CustomDriveMotor", &stallCurr, 5.0f, 500.0f, "%.1f"))
                    {
                        swerve.customDriveMotor.stallCurrentA = static_cast<double>(stallCurr);
                    }

                    float freeCurr = static_cast<float>(swerve.customDriveMotor.freeCurrentA);
                    if (SliderWithDirectInput("Free Current (A)##CustomDriveMotor", &freeCurr, 0.05f, 10.0f, "%.2f"))
                    {
                        swerve.customDriveMotor.freeCurrentA = static_cast<double>(freeCurr);
                    }

                    ImGui::Unindent(12.0f);
                }
            }
            else
            {
                // Find drive servo preset index
                int driveServoIdx = -1;
                for (int i = 0; i < IM_ARRAYSIZE(servoIds) - 1; ++i)
                {
                    if (swerve.driveServoPreset == servoIds[i])
                    {
                        driveServoIdx = i;
                        break;
                    }
                }
                if (driveServoIdx == -1 || swerve.driveServoPreset == "Custom" || isDriveServoCustom)
                {
                    driveServoIdx = IM_ARRAYSIZE(servoIds) - 1; // "Custom"
                    isDriveServoCustom = true;
                }
                else
                {
                    isDriveServoCustom = false;
                }

                ImGui::SetNextItemWidth(250.0f);
                if (ImGui::Combo("Drive Servo Preset", &driveServoIdx, servoPresets, IM_ARRAYSIZE(servoPresets)))
                {
                    if (driveServoIdx < IM_ARRAYSIZE(servoIds) - 1)
                    {
                        isDriveServoCustom = false;
                        swerve.driveServoPreset = servoIds[driveServoIdx];
                    }
                    else
                    {
                        isDriveServoCustom = true;
                        swerve.driveServoPreset = "Custom";
                    }
                    state.RunLinter();
                }

                if (isDriveServoCustom)
                {
                    ImGui::Spacing();
                    ImGui::Indent(12.0f);
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Drive Servo Specs:");

                    float vNom = static_cast<float>(swerve.customDriveServo.nominalVoltageV);
                    if (SliderWithDirectInput("Operating Voltage (V)##CustomDriveServo", &vNom, 4.8f, 8.4f, "%.1f"))
                    {
                        swerve.customDriveServo.nominalVoltageV = static_cast<double>(vNom);
                    }

                    float stallTorque = static_cast<float>(swerve.customDriveServo.stallTorqueKgCm);
                    if (SliderWithDirectInput("Stall Torque (kg*cm)##CustomDriveServo", &stallTorque, 1.0f, 80.0f, "%.1f"))
                    {
                        swerve.customDriveServo.stallTorqueKgCm = static_cast<double>(stallTorque);
                    }

                    float speedSec = static_cast<float>(swerve.customDriveServo.speedSecPer60Deg);
                    if (SliderWithDirectInput("Speed (s/60 deg)##CustomDriveServo", &speedSec, 0.030f, 0.400f, "%.3f"))
                    {
                        swerve.customDriveServo.speedSecPer60Deg = static_cast<double>(speedSec);
                    }
                    double equivRpm = (swerve.customDriveServo.speedSecPer60Deg > 0.001) ? (10.0 / swerve.customDriveServo.speedSecPer60Deg) : 0.0;
                    ImGui::TextDisabled("  Equivalent Speed: %.0f RPM", equivRpm);

                    float stallCurr = static_cast<float>(swerve.customDriveServo.stallCurrentA);
                    if (SliderWithDirectInput("Stall Current (A)##CustomDriveServo", &stallCurr, 0.5f, 10.0f, "%.2f"))
                    {
                        swerve.customDriveServo.stallCurrentA = static_cast<double>(stallCurr);
                    }

                    float freeCurr = static_cast<float>(swerve.customDriveServo.freeCurrentA);
                    if (SliderWithDirectInput("Free Current (A)##CustomDriveServo", &freeCurr, 0.02f, 1.0f, "%.3f"))
                    {
                        swerve.customDriveServo.freeCurrentA = static_cast<double>(freeCurr);
                    }

                    float travelRange = static_cast<float>(swerve.customDriveServo.travelRangeDeg);
                    if (SliderWithDirectInput("Travel Range (deg)##CustomDriveServo", &travelRange, 90.0f, 360.0f, "%.0f"))
                    {
                        swerve.customDriveServo.travelRangeDeg = static_cast<double>(travelRange);
                    }

                    ImGui::Unindent(12.0f);
                }
            }

            ImGui::Spacing();

            // B. Steer Actuator Mode
            int steerMode = (swerve.steerActuatorType == "servo") ? 1 : 0;
            ImGui::Text("Steer Actuator:");
            ImGui::SameLine();
            ImGui::RadioButton("CAN Motor##Steer", &steerMode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("PWM Servo##Steer", &steerMode, 1);
            swerve.steerActuatorType = (steerMode == 1) ? "servo" : "motor";

            if (steerMode == 0)
            {
                int steerPresetIdx = -1;
                for (int i = 0; i < IM_ARRAYSIZE(motorPresets) - 1; ++i)
                {
                    if (swerve.steerMotorPreset == motorPresets[i])
                    {
                        steerPresetIdx = i;
                        break;
                    }
                }
                if (steerPresetIdx == -1 || swerve.steerMotorPreset == "Custom" || isSteerMotorCustom)
                {
                    steerPresetIdx = IM_ARRAYSIZE(motorPresets) - 1;
                    isSteerMotorCustom = true;
                }
                else
                {
                    isSteerMotorCustom = false;
                }

                ImGui::SetNextItemWidth(180.0f);
                if (ImGui::Combo("Steer Motor Preset", &steerPresetIdx, motorPresets, IM_ARRAYSIZE(motorPresets)))
                {
                    if (steerPresetIdx < IM_ARRAYSIZE(motorPresets) - 1)
                    {
                        isSteerMotorCustom = false;
                        swerve.steerMotorPreset = motorPresets[steerPresetIdx];
                    }
                    else
                    {
                        isSteerMotorCustom = true;
                        swerve.steerMotorPreset = "Custom";
                    }
                    state.RunLinter();
                }

                if (isSteerMotorCustom)
                {
                    ImGui::Spacing();
                    ImGui::Indent(12.0f);
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Steer Motor Specs:");

                    float vNom = static_cast<float>(swerve.customSteerMotor.nominalVoltageV);
                    if (SliderWithDirectInput("Nominal Voltage (V)##CustomSteerMotor", &vNom, 4.0f, 24.0f, "%.1f"))
                    {
                        swerve.customSteerMotor.nominalVoltageV = static_cast<double>(vNom);
                    }

                    float stallTorque = static_cast<float>(swerve.customSteerMotor.stallTorqueNm);
                    if (SliderWithDirectInput("Stall Torque (N*m)##CustomSteerMotor", &stallTorque, 0.10f, 15.0f, "%.3f"))
                    {
                        swerve.customSteerMotor.stallTorqueNm = static_cast<double>(stallTorque);
                    }

                    float freeSpeed = static_cast<float>(swerve.customSteerMotor.freeSpeedRpm);
                    if (SliderWithDirectInput("Free Speed (RPM)##CustomSteerMotor", &freeSpeed, 500.0f, 25000.0f, "%.0f"))
                    {
                        swerve.customSteerMotor.freeSpeedRpm = static_cast<double>(freeSpeed);
                    }

                    float stallCurr = static_cast<float>(swerve.customSteerMotor.stallCurrentA);
                    if (SliderWithDirectInput("Stall Current (A)##CustomSteerMotor", &stallCurr, 5.0f, 500.0f, "%.1f"))
                    {
                        swerve.customSteerMotor.stallCurrentA = static_cast<double>(stallCurr);
                    }

                    float freeCurr = static_cast<float>(swerve.customSteerMotor.freeCurrentA);
                    if (SliderWithDirectInput("Free Current (A)##CustomSteerMotor", &freeCurr, 0.05f, 10.0f, "%.2f"))
                    {
                        swerve.customSteerMotor.freeCurrentA = static_cast<double>(freeCurr);
                    }

                    ImGui::Unindent(12.0f);
                }
            }
            else
            {
                int steerServoIdx = -1;
                for (int i = 0; i < IM_ARRAYSIZE(servoIds) - 1; ++i)
                {
                    if (swerve.steerServoPreset == servoIds[i])
                    {
                        steerServoIdx = i;
                        break;
                    }
                }
                if (steerServoIdx == -1 || swerve.steerServoPreset == "Custom" || isSteerServoCustom)
                {
                    steerServoIdx = IM_ARRAYSIZE(servoIds) - 1;
                    isSteerServoCustom = true;
                }
                else
                {
                    isSteerServoCustom = false;
                }

                ImGui::SetNextItemWidth(250.0f);
                if (ImGui::Combo("Steer Servo Preset", &steerServoIdx, servoPresets, IM_ARRAYSIZE(servoPresets)))
                {
                    if (steerServoIdx < IM_ARRAYSIZE(servoIds) - 1)
                    {
                        isSteerServoCustom = false;
                        swerve.steerServoPreset = servoIds[steerServoIdx];
                    }
                    else
                    {
                        isSteerServoCustom = true;
                        swerve.steerServoPreset = "Custom";
                    }
                    state.RunLinter();
                }

                if (isSteerServoCustom)
                {
                    ImGui::Spacing();
                    ImGui::Indent(12.0f);
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Steer Servo Specs:");

                    float vNom = static_cast<float>(swerve.customSteerServo.nominalVoltageV);
                    if (SliderWithDirectInput("Operating Voltage (V)##CustomSteerServo", &vNom, 4.8f, 8.4f, "%.1f"))
                    {
                        swerve.customSteerServo.nominalVoltageV = static_cast<double>(vNom);
                    }

                    float stallTorque = static_cast<float>(swerve.customSteerServo.stallTorqueKgCm);
                    if (SliderWithDirectInput("Stall Torque (kg*cm)##CustomSteerServo", &stallTorque, 1.0f, 80.0f, "%.1f"))
                    {
                        swerve.customSteerServo.stallTorqueKgCm = static_cast<double>(stallTorque);
                    }

                    float speedSec = static_cast<float>(swerve.customSteerServo.speedSecPer60Deg);
                    if (SliderWithDirectInput("Speed (s/60 deg)##CustomSteerServo", &speedSec, 0.030f, 0.400f, "%.3f"))
                    {
                        swerve.customSteerServo.speedSecPer60Deg = static_cast<double>(speedSec);
                    }
                    double equivRpm = (swerve.customSteerServo.speedSecPer60Deg > 0.001) ? (10.0 / swerve.customSteerServo.speedSecPer60Deg) : 0.0;
                    ImGui::TextDisabled("  Equivalent Speed: %.0f RPM", equivRpm);

                    float stallCurr = static_cast<float>(swerve.customSteerServo.stallCurrentA);
                    if (SliderWithDirectInput("Stall Current (A)##CustomSteerServo", &stallCurr, 0.5f, 10.0f, "%.2f"))
                    {
                        swerve.customSteerServo.stallCurrentA = static_cast<double>(stallCurr);
                    }

                    float freeCurr = static_cast<float>(swerve.customSteerServo.freeCurrentA);
                    if (SliderWithDirectInput("Free Current (A)##CustomSteerServo", &freeCurr, 0.02f, 1.0f, "%.3f"))
                    {
                        swerve.customSteerServo.freeCurrentA = static_cast<double>(freeCurr);
                    }

                    float travelRange = static_cast<float>(swerve.customSteerServo.travelRangeDeg);
                    if (SliderWithDirectInput("Travel Range (deg)##CustomSteerServo", &travelRange, 90.0f, 360.0f, "%.0f"))
                    {
                        swerve.customSteerServo.travelRangeDeg = static_cast<double>(travelRange);
                    }

                    ImGui::Unindent(12.0f);
                }
            }
        }

        // -------------------------------------------------------------
        // 3. Gearbox Ratios, Efficiencies & Current Limits (Sliders + Direct Input)
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Gear Ratios & Current Limits", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float driveRatio = static_cast<float>(swerve.driveGearRatio);
            if (SliderWithDirectInput("Drive Gear Reduction (G:1)", &driveRatio, 1.0f, 40.0f, "%.3f"))
            {
                swerve.driveGearRatio = static_cast<double>(driveRatio);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Drive gear reduction G:1 (e.g. 6.75:1 = enter 6.75. Multiplies torque by G and divides wheel speed by G. Do NOT enter 1/G).");

            float steerRatio = static_cast<float>(swerve.steerGearRatio);
            if (SliderWithDirectInput("Steer Gear Reduction (G:1)", &steerRatio, 1.0f, 50.0f, "%.3f"))
            {
                swerve.steerGearRatio = static_cast<double>(steerRatio);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Steer gear reduction G:1 (e.g. 21.43:1 = enter 21.43. Multiplies torque by G. Do NOT enter 1/G).");

            float driveEff = static_cast<float>(swerve.driveEfficiency);
            if (SliderWithDirectInput("Drive Efficiency", &driveEff, 0.50f, 1.0f, "%.2f"))
            {
                swerve.driveEfficiency = static_cast<double>(driveEff);
            }

            float steerEff = static_cast<float>(swerve.steerEfficiency);
            if (SliderWithDirectInput("Steer Efficiency", &steerEff, 0.50f, 1.0f, "%.2f"))
            {
                swerve.steerEfficiency = static_cast<double>(steerEff);
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Electrical Current Limits:");

            float dStator = static_cast<float>(swerve.driveStatorLimitA);
            if (SliderWithDirectInput("Drive Stator Limit (A)", &dStator, 10.0f, 140.0f, "%.0f"))
            {
                swerve.driveStatorLimitA = static_cast<double>(dStator);
            }

            float dSupply = static_cast<float>(swerve.driveSupplyLimitA);
            if (SliderWithDirectInput("Drive Supply Limit (A)", &dSupply, 10.0f, 90.0f, "%.0f"))
            {
                swerve.driveSupplyLimitA = static_cast<double>(dSupply);
            }

            float sStator = static_cast<float>(swerve.steerStatorLimitA);
            if (SliderWithDirectInput("Steer Stator Limit (A)", &sStator, 5.0f, 90.0f, "%.0f"))
            {
                swerve.steerStatorLimitA = static_cast<double>(sStator);
            }

            float sSupply = static_cast<float>(swerve.steerSupplyLimitA);
            if (SliderWithDirectInput("Steer Supply Limit (A)", &sSupply, 5.0f, 70.0f, "%.0f"))
            {
                swerve.steerSupplyLimitA = static_cast<double>(sSupply);
            }
        }

        // -------------------------------------------------------------
        // 4. Wheel Dimensions & Caliper Wear Calculator
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Wheel Radius & Caliper Wear Calculator", ImGuiTreeNodeFlags_DefaultOpen))
        {
            double wheelDiameterInches = (swerve.wheelRadiusM * 2.0) / 0.0254;
            ImGui::Text("Active Radius:   %.4f m (%.2f mm)",
                swerve.wheelRadiusM, swerve.wheelRadiusM * 1000.0);
            ImGui::Text("Effective Diam:  %.3f in (%.1f mm)",
                wheelDiameterInches, wheelDiameterInches * 25.4);

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Caliper Wear Input:");
            ImGui::Checkbox("Use Metric (Millimeters)", &useMetricCaliper);

            if (!useMetricCaliper)
            {
                SliderWithDirectInput("Caliper Diam (in)", &caliperDiameterInches, 1.0f, 8.0f, "%.3f");
                if (ImGui::Button("Apply Caliper Measurement##Inches"))
                {
                    swerve.wheelRadiusM = (static_cast<double>(caliperDiameterInches) * 0.0254) / 2.0;
                }
            }
            else
            {
                SliderWithDirectInput("Caliper Diam (mm)", &caliperDiameterMm, 25.0f, 200.0f, "%.1f");
                if (ImGui::Button("Apply Caliper Measurement##Mm"))
                {
                    swerve.wheelRadiusM = (static_cast<double>(caliperDiameterMm) * 0.001) / 2.0;
                }
            }
        }

        // -------------------------------------------------------------
        // 5. 4-Module Matrix & CAN / PWM Configuration
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("4-Module Layout & Channel Matrix", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // Ensure 4 modules exist
            if (swerve.modules.size() != 4)
            {
                swerve.modules.clear();
                swerve.modules.push_back({ "FrontLeft",  { -0.28, 0.0,  0.28 }, 1, 2, 21, "rio", 0, 1 });
                swerve.modules.push_back({ "FrontRight", {  0.28, 0.0,  0.28 }, 3, 4, 22, "rio", 2, 3 });
                swerve.modules.push_back({ "BackLeft",   { -0.28, 0.0, -0.28 }, 5, 6, 23, "rio", 4, 5 });
                swerve.modules.push_back({ "BackRight",  {  0.28, 0.0, -0.28 }, 7, 8, 24, "rio", 6, 7 });
            }

            // Symmetrical Layout Helper with Sliders + Direct Input
            ImGui::TextDisabled("Symmetrical Axle Geometry Helper (+X=Right, +Y=Up, +Z=Front):");
            SliderWithDirectInput("Trackwidth (X / Left-Right) (m)", &trackwidthM, 0.20f, 1.20f, "%.3f");
            SliderWithDirectInput("Wheelbase (Z / Front-Back) (m)", &wheelbaseM, 0.20f, 1.20f, "%.3f");

            if (ImGui::Button("Apply Symmetrical Positions"))
            {
                double hX = trackwidthM * 0.5;
                double hZ = wheelbaseM * 0.5;
                swerve.modules[0].posM = { -hX, 0.0,  hZ }; // FL (-X Left,  +Z Front)
                swerve.modules[1].posM = {  hX, 0.0,  hZ }; // FR (+X Right, +Z Front)
                swerve.modules[2].posM = { -hX, 0.0, -hZ }; // BL (-X Left,  -Z Back)
                swerve.modules[3].posM = {  hX, 0.0, -hZ }; // BR (+X Right, -Z Back)
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Module Cards with compact, wrap-safe layout
            for (size_t i = 0; i < swerve.modules.size(); ++i)
            {
                auto& mod = swerve.modules[i];
                ImGui::PushID(static_cast<int>(i));

                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "[Module %zu: %s]", i, mod.name.c_str());

                // Position Sliders + Direct Input (+X=Right, +Y=Up, +Z=Front)
                float posX = static_cast<float>(mod.posM.x);
                float posY = static_cast<float>(mod.posM.y);
                float posZ = static_cast<float>(mod.posM.z);
                if (SliderWithDirectInput("Pos X (Left [-] / Right [+]) (m)", &posX, -1.0f, 1.0f, "%.3f", 85.0f, 55.0f))
                {
                    mod.posM.x = posX;
                }
                if (SliderWithDirectInput("Pos Y (Elevation / Up [+]) (m)", &posY, -1.0f, 1.0f, "%.3f", 85.0f, 55.0f))
                {
                    mod.posM.y = posY;
                }
                if (SliderWithDirectInput("Pos Z (Back [-] / Front [+]) (m)", &posZ, -1.0f, 1.0f, "%.3f", 85.0f, 55.0f))
                {
                    mod.posM.z = posZ;
                }

                // CAN Bus Selector
                const char* canBusLabels[] = { "roboRIO (rio)", "CANivore (canivore)" };
                int busIdx = (mod.canBus == "canivore") ? 1 : 0;
                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::Combo("CAN Bus", &busIdx, canBusLabels, IM_ARRAYSIZE(canBusLabels)))
                {
                    mod.canBus = (busIdx == 1) ? "canivore" : "rio";
                    state.RunLinter();
                }

                // CAN IDs (using step=0 for compact direct numeric entry)
                int dCan = static_cast<int>(mod.driveCanId);
                int sCan = static_cast<int>(mod.steerCanId);
                int cCan = static_cast<int>(mod.cancoderId);

                ImGui::SetNextItemWidth(45.0f);
                if (ImGui::InputInt("Drive ID", &dCan, 0))
                {
                    mod.driveCanId = static_cast<uint32_t>(std::max(0, dCan));
                    state.RunLinter();
                }
                ImGui::SameLine(0.0f, 10.0f);
                ImGui::SetNextItemWidth(45.0f);
                if (ImGui::InputInt("Steer ID", &sCan, 0))
                {
                    mod.steerCanId = static_cast<uint32_t>(std::max(0, sCan));
                    state.RunLinter();
                }
                ImGui::SameLine(0.0f, 10.0f);
                ImGui::SetNextItemWidth(45.0f);
                if (ImGui::InputInt("CANcoder ID", &cCan, 0))
                {
                    mod.cancoderId = static_cast<uint32_t>(std::max(0, cCan));
                    state.RunLinter();
                }

                // PWM channels if servo swerve
                if (swerve.driveActuatorType == "servo" || swerve.steerActuatorType == "servo")
                {
                    int dPwm = static_cast<int>(mod.drivePwmChannel);
                    int sPwm = static_cast<int>(mod.steerPwmChannel);
                    ImGui::SetNextItemWidth(45.0f);
                    if (ImGui::InputInt("Drive PWM", &dPwm, 0))
                    {
                        mod.drivePwmChannel = static_cast<uint32_t>(std::clamp(dPwm, 0, 15));
                        state.RunLinter();
                    }
                    ImGui::SameLine(0.0f, 12.0f);
                    ImGui::SetNextItemWidth(45.0f);
                    if (ImGui::InputInt("Steer PWM", &sPwm, 0))
                    {
                        mod.steerPwmChannel = static_cast<uint32_t>(std::clamp(sPwm, 0, 15));
                        state.RunLinter();
                    }
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::PopID();
            }
        }
    }
}
