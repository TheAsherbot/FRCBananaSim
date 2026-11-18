/**
 * @file SensorPanel.cpp
 * @brief Implementation of the Sensor Suite configuration and calibration inspector panel.
 */

#include "Tools/Configurator/Panels/SensorPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>
#include "ConfiguratorWidgets.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Tools::Configurator
{

    static const char* kSensorTypes[] = {
        "imu_pigeon2", "imu_navx", "cancoder", "beam_break", "limit_switch", "analog_input", "laser_can", "color_sensor", "rev_through_bore", "andymark_lidar3", "custom"
    };

    static const char* kSensorTypeLabels[] = {
        "Pigeon 2.0 (CTRE 6-Axis IMU)",
        "navX2 (Kauai Labs 6-Axis IMU)",
        "CANcoder (CTRE Magnetic Encoder)",
        "Beam Break (Optical Digital Sensor)",
        "Limit Switch (Mechanical / Magnetic)",
        "Analog Input (Potentiometer / Pressure)",
        "LaserCAN (Time-of-Flight Distance)",
        "Color Sensor V3 (REV I2C Color)",
        "REV Through Bore Encoder (REV-11-1271)",
        "AndyMark LIDAR-Lite v3 / CAN Lidar (am-3829 / am-5684)",
        "Custom Transducer"
    };

    static const char* kCanBuses[] = {
        "rio", "canivore"
    };

    static const char* kCanBusLabels[] = {
        "roboRIO (rio)", "CANivore (canivore)"
    };

    static const char* kNavxBuses[] = {
        "spi", "i2c_onboard", "i2c_mxp"
    };

    static const char* kNavxBusLabels[] = {
        "SPI (Onboard)", "I2C (Onboard)", "I2C (MXP Extension)"
    };

    static const char* kLimitDirections[] = {
        "forward", "reverse"
    };

    static const char* kLimitDirectionLabels[] = {
        "Forward Limit", "Reverse Limit"
    };

    void SensorPanel::Render(ConfiguratorState& state)
    {
        ImGui::PushTextWrapPos(0.0f);

        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "SENSOR SUITE & INSTRUMENTATION");
        ImGui::Separator();

        if (!state.isSidecarLoaded && !state.isCadLoaded)
        {
            ImGui::TextWrapped("No robot loaded. Open a CAD folder or load a reference robot to configure sensors.");
            return;
        }

        auto& sensors = state.sidecar.sensors;

        // -------------------------------------------------------------
        // 1. Add New Sensor Section
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Add New Sensor", ImGuiTreeNodeFlags_None))
        {
            ImGui::TextDisabled("Sensor Name:");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputText("##NewSensorName", newSensorNameBuf, sizeof(newSensorNameBuf));

            ImGui::TextDisabled("Sensor Type:");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##NewSensorType", &newSensorTypeIdx, kSensorTypeLabels, IM_ARRAYSIZE(kSensorTypeLabels));

            ImGui::Spacing();
            if (ImGui::Button("+ Create Sensor", ImVec2(-1.0f, 0.0f)))
            {
                Robot::Schema::SensorConfig newSens;
                newSens.name = newSensorNameBuf;
                newSens.type = kSensorTypes[newSensorTypeIdx];

                if (newSens.type == "imu_pigeon2")
                {
                    newSens.bus = "rio";
                    newSens.canId = 25;
                    newSens.parentLink = (!state.manifest.links.empty()) ? state.manifest.links[0].name : "base_link";
                }
                else if (newSens.type == "imu_navx")
                {
                    newSens.bus = "spi";
                    newSens.parentLink = (!state.manifest.links.empty()) ? state.manifest.links[0].name : "base_link";
                }
                else if (newSens.type == "cancoder")
                {
                    newSens.bus = "rio";
                    newSens.canId = 21;
                    newSens.sensorToMechanismRatio = 1.0;
                    newSens.cpr = 4096;
                    // Find first revolute joint
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newSens.targetJoint = j.name;
                            break;
                        }
                    }
                }
                else if (newSens.type == "beam_break")
                {
                    newSens.bus = "rio";
                    newSens.dioChannel = 0;
                    newSens.normallyClosed = false;
                    newSens.debounceTimeS = 0.010;
                    auto effZones = state.GetEffectiveContactZones();
                    if (!effZones.empty())
                    {
                        newSens.contactZone = effZones[0].name;
                    }
                }
                else if (newSens.type == "limit_switch")
                {
                    newSens.bus = "rio";
                    newSens.dioChannel = 1;
                    newSens.limitDirection = "forward";
                    newSens.normallyClosed = false;
                    newSens.debounceTimeS = 0.010;
                    if (!state.manifest.joints.empty())
                    {
                        newSens.targetJoint = state.manifest.joints[0].name;
                    }
                }
                else if (newSens.type == "analog_input")
                {
                    newSens.bus = "analog";
                    newSens.analogChannel = 0;
                    newSens.triggerMin = 0.0;
                    newSens.triggerMax = 1.0;
                    if (!state.manifest.joints.empty())
                    {
                        newSens.targetJoint = state.manifest.joints[0].name;
                    }
                }
                else if (newSens.type == "laser_can")
                {
                    newSens.bus = "rio";
                    newSens.canId = 30;
                    newSens.parentLink = (!state.manifest.links.empty()) ? state.manifest.links[0].name : "base_link";
                }
                else if (newSens.type == "color_sensor")
                {
                    newSens.bus = "i2c_onboard";
                    newSens.i2cAddress = 0x52;
                }
                else if (newSens.type == "rev_through_bore")
                {
                    newSens.bus = "rio";
                    newSens.dioChannel = 2;
                    newSens.secondaryDioChannel = 3;
                    newSens.cpr = 1024;
                    newSens.sensorToMechanismRatio = 1.0;
                    newSens.customStringParams["encoderMode"] = "duty_cycle";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newSens.targetJoint = j.name;
                            break;
                        }
                    }
                }
                else if (newSens.type == "andymark_lidar3")
                {
                    newSens.bus = "rio";
                    newSens.canId = 35;
                    newSens.i2cAddress = 0x62;
                    newSens.triggerMin = 0.05;
                    newSens.triggerMax = 40.0;
                    newSens.customNumericParams["noiseStdDevM"] = 0.01;
                    newSens.parentLink = (!state.manifest.links.empty()) ? state.manifest.links[0].name : "base_link";
                }

                sensors.push_back(newSens);
                state.RunLinter();

                // Generate new default name
                std::snprintf(newSensorNameBuf, sizeof(newSensorNameBuf), "Sensor_%zu", sensors.size() + 1);
            }
            ImGui::Separator();
        }

        // -------------------------------------------------------------
        // 2. Sensor List & Summary
        // -------------------------------------------------------------
        ImGui::Text("Configured Sensors: %zu", sensors.size());
        ImGui::SameLine(0.0f, 15.0f);
        ImGui::SetNextItemWidth(180.0f);
        ImGui::InputTextWithHint("##FilterSensors", "Search sensors...", searchFilterBuf, sizeof(searchFilterBuf));

        std::string searchFilter = searchFilterBuf;
        std::transform(searchFilter.begin(), searchFilter.end(), searchFilter.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (sensors.empty())
        {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No sensors configured yet. Use 'Add New Sensor' above or auto-generate starter sidecar.");
            return;
        }

        int sensorToDelete = -1;
        int sensorToDuplicate = -1;

        for (size_t sIdx = 0; sIdx < sensors.size(); ++sIdx)
        {
            auto& sens = sensors[sIdx];

            std::string nameLower = sens.name;
            std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::string typeLower = sens.type;
            std::transform(typeLower.begin(), typeLower.end(), typeLower.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (!searchFilter.empty() && nameLower.find(searchFilter) == std::string::npos && typeLower.find(searchFilter) == std::string::npos)
            {
                continue;
            }

            ImGui::PushID(static_cast<int>(sIdx));

            char headerBuf[128];
            std::snprintf(headerBuf, sizeof(headerBuf), "[%s] %s##SensorHeader_%zu",
                sens.type.c_str(), sens.name.c_str(), sIdx);

            bool isCardOpen = ImGui::CollapsingHeader(headerBuf, ImGuiTreeNodeFlags_DefaultOpen);
            bool isFocused = ImGui::IsItemActive() || ImGui::IsItemClicked() || ImGui::IsItemHovered();
            if (isCardOpen && isFocused)
            {
                if (!sens.parentLink.empty()) state.selectedLink = sens.parentLink;
                if (!sens.targetJoint.empty()) state.selectedJoint = sens.targetJoint;
            }

            if (isCardOpen)
            {
                // Top Action Row
                float availW = ImGui::GetContentRegionAvail().x;
                float btnW = 75.0f;

                ImGui::TextDisabled("Sensor Nickname:");
                char nameEditBuf[64];
                strncpy_s(nameEditBuf, sizeof(nameEditBuf), sens.name.c_str(), _TRUNCATE);
                if (availW < 270.0f)
                {
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputText("##SensorName", nameEditBuf, sizeof(nameEditBuf)))
                    {
                        sens.name = nameEditBuf;
                        state.RunLinter();
                    }
                    if (ImGui::Button("Duplicate", ImVec2(btnW, 0.0f)))
                    {
                        sensorToDuplicate = static_cast<int>(sIdx);
                    }
                    ImGui::SameLine(0.0f, 8.0f);
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
                    if (ImGui::Button("Delete", ImVec2(btnW, 0.0f)))
                    {
                        sensorToDelete = static_cast<int>(sIdx);
                    }
                    ImGui::PopStyleColor(2);
                }
                else
                {
                    ImGui::SetNextItemWidth(std::max(80.0f, availW - (2.0f * btnW + 20.0f)));
                    if (ImGui::InputText("##SensorName", nameEditBuf, sizeof(nameEditBuf)))
                    {
                        sens.name = nameEditBuf;
                        state.RunLinter();
                    }

                    ImGui::SameLine(0.0f, 8.0f);
                    if (ImGui::Button("Duplicate", ImVec2(btnW, 0.0f)))
                    {
                        sensorToDuplicate = static_cast<int>(sIdx);
                    }

                    ImGui::SameLine(0.0f, 8.0f);
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
                    if (ImGui::Button("Delete", ImVec2(btnW, 0.0f)))
                    {
                        sensorToDelete = static_cast<int>(sIdx);
                    }
                    ImGui::PopStyleColor(2);
                }

                // Type selector
                ImGui::TextDisabled("Sensor Type:");
                int currentTypeIdx = 10; // custom
                for (int t = 0; t < IM_ARRAYSIZE(kSensorTypes); ++t)
                {
                    if (sens.type == kSensorTypes[t])
                    {
                        currentTypeIdx = t;
                        break;
                    }
                }
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##SensorType", &currentTypeIdx, kSensorTypeLabels, IM_ARRAYSIZE(kSensorTypeLabels)))
                {
                    sens.type = kSensorTypes[currentTypeIdx];
                    state.RunLinter();
                }

                ImGui::Separator();

                // -----------------------------------------------------
                // Sensor-Specific Parameters
                // -----------------------------------------------------

                // CASE A: Pigeon 2.0 (CTRE 6-Axis IMU)
                if (sens.type == "imu_pigeon2")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CTRE Pigeon 2.0 IMU Configuration:");

                    // CAN ID
                    int canIdInt = static_cast<int>(sens.canId);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("CAN ID##Pigeon", &canIdInt, 1, 5))
                    {
                        sens.canId = static_cast<uint32_t>(std::max(0, canIdInt));
                        state.RunLinter();
                    }

                    // CAN Bus
                    ImGui::SameLine(0.0f, 12.0f);
                    int busIdx = (sens.bus == "canivore") ? 1 : 0;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::Combo("CAN Bus##Pigeon", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                    {
                        sens.bus = kCanBuses[busIdx];
                        state.RunLinter();
                    }

                    // Mount Parent Link
                    std::vector<const char*> linkNames;
                    int selectedLinkIdx = -1;
                    for (size_t l = 0; l < state.manifest.links.size(); ++l)
                    {
                        linkNames.push_back(state.manifest.links[l].name.c_str());
                        if (sens.parentLink == state.manifest.links[l].name)
                        {
                            selectedLinkIdx = static_cast<int>(l);
                        }
                    }
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::Combo("Mount Link##Pigeon", &selectedLinkIdx, linkNames.data(), static_cast<int>(linkNames.size())))
                    {
                        if (selectedLinkIdx >= 0 && selectedLinkIdx < static_cast<int>(linkNames.size()))
                        {
                            sens.parentLink = linkNames[selectedLinkIdx];
                            state.RunLinter();
                        }
                    }

                    // Mount Offset (m)
                    float offX = static_cast<float>(sens.mountOffsetM.x);
                    float offY = static_cast<float>(sens.mountOffsetM.y);
                    float offZ = static_cast<float>(sens.mountOffsetM.z);
                    ImGui::Text("Mount Offset relative to Link Origin (+X=Right, +Y=Up, +Z=Front) (m):");
                    bool offChanged = false;
                    offChanged |= SliderWithDirectInput("X (Right [+]) (m)##PigeonOff", &offX, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Y (Up [+]) (m)##PigeonOff", &offY, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Z (Front [+]) (m)##PigeonOff", &offZ, -1.0f, 1.0f, "%.3f");
                    if (offChanged)
                    {
                        sens.mountOffsetM = glm::dvec3(offX, offY, offZ);
                    }

                    // Advanced Noise Calibration
                    if (ImGui::TreeNode("Noise & Drift Calibration##Pigeon"))
                    {
                        float gyroNoise = static_cast<float>(sens.gyroNoiseStdDevRadS);
                        float gyroBias = static_cast<float>(sens.gyroBiasRandomWalkRadS2);
                        float accelNoise = static_cast<float>(sens.accelNoiseStdDevMS2);

                        if (SliderWithDirectInput("Gyro Noise StdDev (rad/s)##Pigeon", &gyroNoise, 0.0f, 0.05f, "%.5f", 100.0f, 65.0f))
                        {
                            sens.gyroNoiseStdDevRadS = gyroNoise;
                        }
                        if (SliderWithDirectInput("Gyro Bias Walk (rad/s^2)##Pigeon", &gyroBias, 0.0f, 0.001f, "%.6f", 100.0f, 65.0f))
                        {
                            sens.gyroBiasRandomWalkRadS2 = gyroBias;
                        }
                        if (SliderWithDirectInput("Accel Noise StdDev (m/s^2)##Pigeon", &accelNoise, 0.0f, 0.2f, "%.4f", 100.0f, 65.0f))
                        {
                            sens.accelNoiseStdDevMS2 = accelNoise;
                        }
                        ImGui::TreePop();
                    }
                }
                // CASE B: navX2 (Kauai Labs 6-Axis IMU)
                else if (sens.type == "imu_navx")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Kauai Labs navX2 IMU Configuration:");

                    // Bus Interface
                    int busIdx = 0;
                    if (sens.bus == "i2c_onboard") { busIdx = 1; }
                    else if (sens.bus == "i2c_mxp") { busIdx = 2; }
                    ImGui::SetNextItemWidth(160.0f);
                    if (ImGui::Combo("Bus Interface##NavX", &busIdx, kNavxBusLabels, IM_ARRAYSIZE(kNavxBusLabels)))
                    {
                        sens.bus = kNavxBuses[busIdx];
                        state.RunLinter();
                    }

                    // Mount Parent Link
                    std::vector<const char*> linkNames;
                    int selectedLinkIdx = -1;
                    for (size_t l = 0; l < state.manifest.links.size(); ++l)
                    {
                        linkNames.push_back(state.manifest.links[l].name.c_str());
                        if (sens.parentLink == state.manifest.links[l].name)
                        {
                            selectedLinkIdx = static_cast<int>(l);
                        }
                    }
                    ImGui::SameLine(0.0f, 12.0f);
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Mount Link##NavX", &selectedLinkIdx, linkNames.data(), static_cast<int>(linkNames.size())))
                    {
                        if (selectedLinkIdx >= 0 && selectedLinkIdx < static_cast<int>(linkNames.size()))
                        {
                            sens.parentLink = linkNames[selectedLinkIdx];
                            state.RunLinter();
                        }
                    }

                    // Mount Offset (m)
                    float offX = static_cast<float>(sens.mountOffsetM.x);
                    float offY = static_cast<float>(sens.mountOffsetM.y);
                    float offZ = static_cast<float>(sens.mountOffsetM.z);
                    ImGui::Text("Mount Offset relative to Link Origin (+X=Right, +Y=Up, +Z=Front) (m):");
                    bool offChanged = false;
                    offChanged |= SliderWithDirectInput("X (Right [+]) (m)##NavXOff", &offX, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Y (Up [+]) (m)##NavXOff", &offY, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Z (Front [+]) (m)##NavXOff", &offZ, -1.0f, 1.0f, "%.3f");
                    if (offChanged)
                    {
                        sens.mountOffsetM = glm::dvec3(offX, offY, offZ);
                    }

                    // Advanced Noise Calibration
                    if (ImGui::TreeNode("Noise & Drift Calibration##NavX"))
                    {
                        float gyroNoise = static_cast<float>(sens.gyroNoiseStdDevRadS);
                        float gyroBias = static_cast<float>(sens.gyroBiasRandomWalkRadS2);
                        float accelNoise = static_cast<float>(sens.accelNoiseStdDevMS2);

                        if (SliderWithDirectInput("Gyro Noise StdDev (rad/s)##NavX", &gyroNoise, 0.0f, 0.05f, "%.5f", 100.0f, 65.0f))
                        {
                            sens.gyroNoiseStdDevRadS = gyroNoise;
                        }
                        if (SliderWithDirectInput("Gyro Bias Walk (rad/s^2)##NavX", &gyroBias, 0.0f, 0.001f, "%.6f", 100.0f, 65.0f))
                        {
                            sens.gyroBiasRandomWalkRadS2 = gyroBias;
                        }
                        if (SliderWithDirectInput("Accel Noise StdDev (m/s^2)##NavX", &accelNoise, 0.0f, 0.2f, "%.4f", 100.0f, 65.0f))
                        {
                            sens.accelNoiseStdDevMS2 = accelNoise;
                        }
                        ImGui::TreePop();
                    }
                }
                // CASE C: CANcoder (CTRE Magnetic Encoder)
                else if (sens.type == "cancoder")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CTRE CANcoder Absolute Encoder Configuration:");

                    // CAN ID & Bus
                    int canIdInt = static_cast<int>(sens.canId);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("CAN ID##CANcoder", &canIdInt, 1, 5))
                    {
                        sens.canId = static_cast<uint32_t>(std::max(0, canIdInt));
                        state.RunLinter();
                    }

                    ImGui::SameLine(0.0f, 12.0f);
                    int busIdx = (sens.bus == "canivore") ? 1 : 0;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::Combo("CAN Bus##CANcoder", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                    {
                        sens.bus = kCanBuses[busIdx];
                        state.RunLinter();
                    }

                    // Target Joint (Filtered to revolute / continuous)
                    std::vector<const char*> jointNames;
                    int selectedJointIdx = -1;
                    for (size_t j = 0; j < state.manifest.joints.size(); ++j)
                    {
                        const auto& joint = state.manifest.joints[j];
                        if (joint.type == "revolute" || joint.type == "continuous")
                        {
                            if (sens.targetJoint == joint.name)
                            {
                                selectedJointIdx = static_cast<int>(jointNames.size());
                            }
                            jointNames.push_back(joint.name.c_str());
                        }
                    }
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::Combo("Target Joint##CANcoder", &selectedJointIdx, jointNames.data(), static_cast<int>(jointNames.size())))
                    {
                        if (selectedJointIdx >= 0 && selectedJointIdx < static_cast<int>(jointNames.size()))
                        {
                            sens.targetJoint = jointNames[selectedJointIdx];
                            state.RunLinter();
                        }
                    }

                    // Ratio & CPR
                    float ratio = static_cast<float>(sens.sensorToMechanismRatio);
                    if (SliderWithDirectInput("Sensor-to-Mechanism Ratio##CANcoder", &ratio, 0.1f, 50.0f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.sensorToMechanismRatio = std::max(0.001, static_cast<double>(ratio));
                    }

                    // Offset in Radians and Degrees
                    float offsetRad = static_cast<float>(sens.offsetRad);
                    float offsetDeg = offsetRad * (180.0f / static_cast<float>(std::numbers::pi));

                    if (SliderWithDirectInput("Magnet Offset (rad)##CANcoder", &offsetRad, -3.14159f, 3.14159f, "%.4f", 100.0f, 65.0f))
                    {
                        sens.offsetRad = offsetRad;
                    }
                    ImGui::SameLine(0.0f, 10.0f);
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(%.1f deg)", offsetDeg);

                    // Inverted & CPR
                    bool inv = sens.inverted;
                    if (ImGui::Checkbox("Invert Sensor Direction##CANcoder", &inv))
                    {
                        sens.inverted = inv;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Inverts angular direction (+rad vs -rad). Use if positive joint motion decreases CANcoder angle");

                    int cprInt = static_cast<int>(sens.cpr);
                    ImGui::SetNextItemWidth(90.0f);
                    if (ImGui::InputInt("CPR (Counts Per Rev)##CANcoder", &cprInt, 1024, 4096))
                    {
                        sens.cpr = static_cast<uint32_t>(std::max(1, cprInt));
                    }
                }
                // CASE D: Beam Break (Optical Digital Sensor)
                else if (sens.type == "beam_break")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Optical Beam Break Sensor Configuration:");

                    // DIO Channel
                    int dioChan = static_cast<int>(sens.dioChannel);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("roboRIO DIO Channel [0..31]##BeamBreak", &dioChan, 1, 2))
                    {
                        sens.dioChannel = static_cast<uint32_t>(std::clamp(dioChan, 0, 31));
                        state.RunLinter();
                    }

                    // Contact Zone Picker
                    auto effZones = state.GetEffectiveContactZones();
                    std::vector<const char*> zoneNames;
                    int selectedZoneIdx = -1;
                    for (size_t z = 0; z < effZones.size(); ++z)
                    {
                        zoneNames.push_back(effZones[z].name.c_str());
                        if (sens.contactZone == effZones[z].name)
                        {
                            selectedZoneIdx = static_cast<int>(z);
                        }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Bound Contact Zone##BeamBreak", &selectedZoneIdx, zoneNames.data(), static_cast<int>(zoneNames.size())))
                    {
                        if (selectedZoneIdx >= 0 && selectedZoneIdx < static_cast<int>(zoneNames.size()))
                        {
                            sens.contactZone = zoneNames[selectedZoneIdx];
                            state.RunLinter();
                        }
                    }
                    if (effZones.empty())
                    {
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.4f, 1.0f), "Note: No contact zones defined. Add one in the Contact Zones panel.");
                    }

                    // Normally Closed & Debounce
                    bool nc = sens.normallyClosed;
                    if (ImGui::Checkbox("Normally Closed (NC)##BeamBreak", &nc))
                    {
                        sens.normallyClosed = nc;
                    }

                    ImGui::SameLine(0.0f, 16.0f);
                    bool inv = sens.inverted;
                    if (ImGui::Checkbox("Invert Logic##BeamBreak", &inv))
                    {
                        sens.inverted = inv;
                    }

                    float debounceMs = static_cast<float>(sens.debounceTimeS * 1000.0);
                    if (SliderWithDirectInput("Debounce Filter (ms)##BeamBreak", &debounceMs, 0.0f, 100.0f, "%.1f ms", 100.0f, 65.0f))
                    {
                        sens.debounceTimeS = std::max(0.0, static_cast<double>(debounceMs) / 1000.0);
                    }
                }
                // CASE E: Limit Switch (Mechanical / Magnetic)
                else if (sens.type == "limit_switch")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Limit Switch Sensor Configuration:");

                    // Signal Routing (roboRIO DIO vs Hard-Wired to Motor Controller)
                    int routeMode = (sens.attachToControllerCanId > 0) ? 1 : 0;
                    BeginOptionRow("Hardware Connection:", 170.0f);
                    if (ImGui::RadioButton("roboRIO DIO Channel##Limit", routeMode == 0))
                    {
                        sens.attachToControllerCanId = 0;
                        state.RunLinter();
                    }
                    AutoWrapSameLine(260.0f);
                    if (ImGui::RadioButton("Hard-Wired to Motor Controller##Limit", routeMode == 1))
                    {
                        sens.attachToControllerCanId = 1;
                        state.RunLinter();
                    }

                    if (sens.attachToControllerCanId == 0)
                    {
                        int dioChan = static_cast<int>(sens.dioChannel);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("DIO Channel [0..31]##Limit", &dioChan, 1, 2))
                        {
                            sens.dioChannel = static_cast<uint32_t>(std::clamp(dioChan, 0, 31));
                            state.RunLinter();
                        }
                    }
                    else
                    {
                        int ctrlCanId = static_cast<int>(sens.attachToControllerCanId);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("Attached Controller CAN ID##Limit", &ctrlCanId, 1, 5))
                        {
                            sens.attachToControllerCanId = static_cast<uint32_t>(std::max(1, ctrlCanId));
                            state.RunLinter();
                        }
                    }

                    // Limit Direction
                    int dirIdx = (sens.limitDirection == "reverse") ? 1 : 0;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::Combo("Limit Direction##Limit", &dirIdx, kLimitDirectionLabels, IM_ARRAYSIZE(kLimitDirectionLabels)))
                    {
                        sens.limitDirection = kLimitDirections[dirIdx];
                    }

                    // Target Joint
                    std::vector<const char*> jointNames;
                    int selectedJointIdx = -1;
                    for (size_t j = 0; j < state.manifest.joints.size(); ++j)
                    {
                        jointNames.push_back(state.manifest.joints[j].name.c_str());
                        if (sens.targetJoint == state.manifest.joints[j].name)
                        {
                            selectedJointIdx = static_cast<int>(j);
                        }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Monitored Joint##Limit", &selectedJointIdx, jointNames.data(), static_cast<int>(jointNames.size())))
                    {
                        if (selectedJointIdx >= 0 && selectedJointIdx < static_cast<int>(jointNames.size()))
                        {
                            sens.targetJoint = jointNames[selectedJointIdx];
                            state.RunLinter();
                        }
                    }

                    // Trip Range Band
                    float trigMin = static_cast<float>(sens.triggerMin);
                    float trigMax = static_cast<float>(sens.triggerMax);
                    if (SliderWithDirectInput("Trigger Min (rad / m)##Limit", &trigMin, -3.14f, 3.14f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.triggerMin = trigMin;
                    }
                    if (SliderWithDirectInput("Trigger Max (rad / m)##Limit", &trigMax, -3.14f, 3.14f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.triggerMax = trigMax;
                    }

                    // Normally Closed & Debounce
                    bool nc = sens.normallyClosed;
                    if (ImGui::Checkbox("Normally Closed (NC)##Limit", &nc))
                    {
                        sens.normallyClosed = nc;
                    }

                    float debounceMs = static_cast<float>(sens.debounceTimeS * 1000.0);
                    if (SliderWithDirectInput("Debounce Filter (ms)##Limit", &debounceMs, 0.0f, 100.0f, "%.1f ms", 100.0f, 65.0f))
                    {
                        sens.debounceTimeS = std::max(0.0, static_cast<double>(debounceMs) / 1000.0);
                    }
                }
                // CASE F: Analog Input (Potentiometer / Pressure Transducer)
                else if (sens.type == "analog_input")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Analog Input Transducer Configuration:");

                    // Analog Channel [0..7]
                    int anChan = static_cast<int>(sens.analogChannel);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("roboRIO Analog In [0..7]##Analog", &anChan, 1, 2))
                    {
                        sens.analogChannel = static_cast<uint32_t>(std::clamp(anChan, 0, 7));
                        state.RunLinter();
                    }

                    // Target Joint
                    std::vector<const char*> jointNames;
                    int selectedJointIdx = -1;
                    for (size_t j = 0; j < state.manifest.joints.size(); ++j)
                    {
                        jointNames.push_back(state.manifest.joints[j].name.c_str());
                        if (sens.targetJoint == state.manifest.joints[j].name)
                        {
                            selectedJointIdx = static_cast<int>(j);
                        }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Monitored Joint##Analog", &selectedJointIdx, jointNames.data(), static_cast<int>(jointNames.size())))
                    {
                        if (selectedJointIdx >= 0 && selectedJointIdx < static_cast<int>(jointNames.size()))
                        {
                            sens.targetJoint = jointNames[selectedJointIdx];
                            state.RunLinter();
                        }
                    }

                    // Range Calibration (0V to 5V)
                    float spanMin = static_cast<float>(sens.triggerMin);
                    float spanMax = static_cast<float>(sens.triggerMax);
                    if (SliderWithDirectInput("Min Value @ 0V (rad / m)##Analog", &spanMin, -5.0f, 5.0f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.triggerMin = spanMin;
                    }
                    if (SliderWithDirectInput("Max Value @ 5V (rad / m)##Analog", &spanMax, -5.0f, 5.0f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.triggerMax = spanMax;
                    }
                }
                // CASE G: LaserCAN (Time-of-Flight Distance)
                else if (sens.type == "laser_can")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Grapple LaserCAN TOF Distance Sensor:");

                    int canIdInt = static_cast<int>(sens.canId);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("CAN ID##LaserCAN", &canIdInt, 1, 5))
                    {
                        sens.canId = static_cast<uint32_t>(std::max(0, canIdInt));
                        state.RunLinter();
                    }

                    ImGui::SameLine(0.0f, 12.0f);
                    int busIdx = (sens.bus == "canivore") ? 1 : 0;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::Combo("CAN Bus##LaserCAN", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                    {
                        sens.bus = kCanBuses[busIdx];
                        state.RunLinter();
                    }

                    // Mount Parent Link
                    std::vector<const char*> linkNames;
                    int selectedLinkIdx = -1;
                    for (size_t l = 0; l < state.manifest.links.size(); ++l)
                    {
                        linkNames.push_back(state.manifest.links[l].name.c_str());
                        if (sens.parentLink == state.manifest.links[l].name)
                        {
                            selectedLinkIdx = static_cast<int>(l);
                        }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Mount Link##LaserCAN", &selectedLinkIdx, linkNames.data(), static_cast<int>(linkNames.size())))
                    {
                        if (selectedLinkIdx >= 0 && selectedLinkIdx < static_cast<int>(linkNames.size()))
                        {
                            sens.parentLink = linkNames[selectedLinkIdx];
                            state.RunLinter();
                        }
                    }

                    // Mount Offset (m)
                    float offX = static_cast<float>(sens.mountOffsetM.x);
                    float offY = static_cast<float>(sens.mountOffsetM.y);
                    float offZ = static_cast<float>(sens.mountOffsetM.z);
                    ImGui::TextDisabled("Mount Offset (+X=Right, +Y=Up, +Z=Front) (m):");
                    bool offChanged = false;
                    offChanged |= SliderWithDirectInput("Offset X (Right [+]) (m)##LaserCAN", &offX, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Offset Y (Up [+]) (m)##LaserCAN", &offY, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Offset Z (Front [+]) (m)##LaserCAN", &offZ, -1.0f, 1.0f, "%.3f");
                    if (offChanged)
                    {
                        sens.mountOffsetM = glm::dvec3(offX, offY, offZ);
                    }
                }
                // CASE H: Color Sensor V3 (REV I2C Color)
                else if (sens.type == "color_sensor")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "REV Color Sensor V3 Configuration:");

                    int busIdx = (sens.bus == "i2c_mxp") ? 1 : 0;
                    const char* i2cBuses[] = { "I2C (Onboard)", "I2C (MXP Extension)" };
                    ImGui::SetNextItemWidth(160.0f);
                    if (ImGui::Combo("I2C Bus##Color", &busIdx, i2cBuses, 2))
                    {
                        sens.bus = (busIdx == 1) ? "i2c_mxp" : "i2c_onboard";
                        state.RunLinter();
                    }

                    int addrHex = static_cast<int>(sens.i2cAddress);
                    ImGui::SetNextItemWidth(100.0f);
                    if (ImGui::InputInt("I2C Address (Hex)##Color", &addrHex, 1, 16))
                    {
                        sens.i2cAddress = static_cast<uint32_t>(std::clamp(addrHex, 0, 127));
                    }
                }
                // CASE I: REV Through Bore Encoder (REV-11-1271)
                else if (sens.type == "rev_through_bore")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "REV Through Bore Encoder (REV-11-1271):");

                    // 1. Output Signal Mode (Absolute DutyCycle vs Relative Quadrature)
                    std::string& modeStr = sens.customStringParams["encoderMode"];
                    if (modeStr.empty())
                    {
                        modeStr = (sens.secondaryDioChannel > 0) ? "quadrature" : "duty_cycle";
                    }
                    bool isQuad = (modeStr == "quadrature");

                    BeginOptionRow("Encoder Mode:", 220.0f);
                    if (ImGui::RadioButton("Absolute (PWM Duty Cycle)##REVMode", !isQuad))
                    {
                        modeStr = "duty_cycle";
                        sens.cpr = 1024;
                        state.RunLinter();
                    }
                    AutoWrapSameLine(210.0f);
                    if (ImGui::RadioButton("Relative Quadrature (A/B DIO)##REVMode", isQuad))
                    {
                        modeStr = "quadrature";
                        sens.cpr = 8192;
                        if (sens.secondaryDioChannel == 0 || sens.secondaryDioChannel == sens.dioChannel)
                        {
                            sens.secondaryDioChannel = (sens.dioChannel < 31) ? (sens.dioChannel + 1) : 0;
                        }
                        state.RunLinter();
                    }

                    if (!isQuad)
                    {
                        // Hardware Route for Absolute Mode:
                        // 0: roboRIO DIO Pin (PWM DutyCycle)
                        // 1: SPARK MAX/Flex Data Port
                        // 2: roboRIO I2C Bus (Onboard / MXP)
                        int routeMode = 0;
                        if (sens.bus == "i2c_onboard" || sens.bus == "i2c_mxp")
                        {
                            routeMode = 2;
                        }
                        else if (sens.attachToControllerCanId > 0)
                        {
                            routeMode = 1;
                        }

                        BeginOptionRow("Hardware Connection:", 150.0f);
                        if (ImGui::RadioButton("roboRIO DIO Pin (PWM)##REVAbs", routeMode == 0))
                        {
                            routeMode = 0;
                            sens.bus = "rio";
                            sens.attachToControllerCanId = 0;
                            state.RunLinter();
                        }
                        AutoWrapSameLine(210.0f);
                        if (ImGui::RadioButton("SPARK MAX/Flex Data Port##REVAbs", routeMode == 1))
                        {
                            routeMode = 1;
                            sens.bus = "rio";
                            sens.attachToControllerCanId = 1;
                            state.RunLinter();
                        }
                        AutoWrapSameLine(180.0f);
                        if (ImGui::RadioButton("roboRIO I2C Bus##REVAbs", routeMode == 2))
                        {
                            routeMode = 2;
                            if (sens.bus != "i2c_onboard" && sens.bus != "i2c_mxp")
                            {
                                sens.bus = "i2c_onboard";
                            }
                            if (sens.i2cAddress == 0) sens.i2cAddress = 0x36; // Common magnetic encoder I2C address
                            sens.attachToControllerCanId = 0;
                            state.RunLinter();
                        }

                        if (routeMode == 0)
                        {
                            int dioChan = static_cast<int>(sens.dioChannel);
                            ImGui::SetNextItemWidth(70.0f);
                            if (ImGui::InputInt("DutyCycle DIO Channel [0..31]##REV", &dioChan, 1, 2))
                            {
                                sens.dioChannel = static_cast<uint32_t>(std::clamp(dioChan, 0, 31));
                                state.RunLinter();
                            }
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Connects to the yellow Absolute PWM signal wire on the Through Bore cable");
                        }
                        else if (routeMode == 1)
                        {
                            int ctrlCanId = static_cast<int>(sens.attachToControllerCanId);
                            ImGui::SetNextItemWidth(70.0f);
                            if (ImGui::InputInt("SPARK Controller CAN ID##REV", &ctrlCanId, 1, 5))
                            {
                                sens.attachToControllerCanId = static_cast<uint32_t>(std::max(1, ctrlCanId));
                                state.RunLinter();
                            }
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Connected directly via 10-pin ribbon cable to SPARK MAX / Flex data port");
                        }
                        else if (routeMode == 2)
                        {
                            int busIdx = (sens.bus == "i2c_mxp") ? 1 : 0;
                            const char* i2cBuses[] = { "I2C (Onboard)", "I2C (MXP Extension)" };
                            ImGui::SetNextItemWidth(160.0f);
                            if (ImGui::Combo("I2C Port##REVI2C", &busIdx, i2cBuses, 2))
                            {
                                sens.bus = (busIdx == 1) ? "i2c_mxp" : "i2c_onboard";
                                state.RunLinter();
                            }

                            AutoWrapSameLine(140.0f, 12.0f);
                            int addrHex = static_cast<int>(sens.i2cAddress);
                            ImGui::SetNextItemWidth(90.0f);
                            if (ImGui::InputInt("I2C Address (Hex)##REVI2C", &addrHex, 1, 16))
                            {
                                sens.i2cAddress = static_cast<uint32_t>(std::clamp(addrHex, 1, 127));
                                state.RunLinter();
                            }
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("7-bit I2C device address (default 0x36 for magnetic absolute encoder chips, e.g. AS5600)");
                        }
                    }
                    else
                    {
                        // Quadrature: Channel A and Channel B DIO
                        int dioA = static_cast<int>(sens.dioChannel);
                        int dioB = static_cast<int>(sens.secondaryDioChannel);

                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("Channel A DIO [0..31]##REV", &dioA, 1, 2))
                        {
                            sens.dioChannel = static_cast<uint32_t>(std::clamp(dioA, 0, 31));
                            state.RunLinter();
                        }

                        ImGui::SameLine(0.0f, 15.0f);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("Channel B DIO [0..31]##REV", &dioB, 1, 2))
                        {
                            sens.secondaryDioChannel = static_cast<uint32_t>(std::clamp(dioB, 0, 31));
                            state.RunLinter();
                        }
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Optical quadrature signals: Channel A and Channel B (2048 CPR = 8192 counts/rev)");
                    }

                    // Target Joint (Filtered to revolute / continuous)
                    std::vector<const char*> jointNames;
                    int selectedJointIdx = -1;
                    for (size_t j = 0; j < state.manifest.joints.size(); ++j)
                    {
                        const auto& joint = state.manifest.joints[j];
                        if (joint.type == "revolute" || joint.type == "continuous")
                        {
                            if (sens.targetJoint == joint.name)
                            {
                                selectedJointIdx = static_cast<int>(jointNames.size());
                            }
                            jointNames.push_back(joint.name.c_str());
                        }
                    }
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::Combo("Target Joint##REV", &selectedJointIdx, jointNames.data(), static_cast<int>(jointNames.size())))
                    {
                        if (selectedJointIdx >= 0 && selectedJointIdx < static_cast<int>(jointNames.size()))
                        {
                            sens.targetJoint = jointNames[selectedJointIdx];
                            state.RunLinter();
                        }
                    }

                    // Ratio & CPR
                    float ratio = static_cast<float>(sens.sensorToMechanismRatio);
                    if (SliderWithDirectInput("Sensor-to-Mechanism Ratio##REV", &ratio, 0.1f, 50.0f, "%.3f", 100.0f, 65.0f))
                    {
                        sens.sensorToMechanismRatio = std::max(0.001, static_cast<double>(ratio));
                    }

                    int cprInt = static_cast<int>(sens.cpr);
                    ImGui::SetNextItemWidth(90.0f);
                    if (ImGui::InputInt("CPR (Counts Per Rev)##REV", &cprInt, 1024, 4096))
                    {
                        sens.cpr = static_cast<uint32_t>(std::max(1, cprInt));
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Quadrature optical is typically 8192 (4x decoding). Duty cycle absolute is typically 1024 or 4096.");

                    // Zero Offset in Radians and Degrees
                    float offsetRad = static_cast<float>(sens.offsetRad);
                    float offsetDeg = offsetRad * (180.0f / static_cast<float>(std::numbers::pi));

                    if (SliderWithDirectInput("Zero Offset (rad)##REV", &offsetRad, -3.14159f, 3.14159f, "%.4f", 100.0f, 65.0f))
                    {
                        sens.offsetRad = offsetRad;
                    }
                    ImGui::SameLine(0.0f, 10.0f);
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(%.1f deg)", offsetDeg);

                    // Direction Inversion
                    bool inv = sens.inverted;
                    if (ImGui::Checkbox("Invert Sensor Direction##REV", &inv))
                    {
                        sens.inverted = inv;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Inverts counting direction (+rad vs -rad).");
                }
                // CASE J: AndyMark LIDAR-Lite v3 / CAN Lidar (am-3829 / am-5684)
                else if (sens.type == "andymark_lidar3")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "AndyMark LIDAR-Lite v3 / CAN Lidar (am-3829 / am-5684):");

                    // 1. Hardware Interface Mode
                    int busMode = 0; // 0: CAN, 1: I2C, 2: DIO PWM
                    if (sens.bus == "i2c_onboard" || sens.bus == "i2c_mxp")
                    {
                        busMode = 1;
                    }
                    else if (sens.bus == "dio_pwm")
                    {
                        busMode = 2;
                    }

                    BeginOptionRow("Hardware Interface:", 170.0f);
                    if (ImGui::RadioButton("CAN Bus (am-5684)##LidarMode", busMode == 0))
                    {
                        sens.bus = "rio";
                        if (sens.canId == 0) sens.canId = 35;
                        state.RunLinter();
                    }
                    AutoWrapSameLine(170.0f);
                    if (ImGui::RadioButton("I2C Bus (am-3829)##LidarMode", busMode == 1))
                    {
                        sens.bus = "i2c_onboard";
                        if (sens.i2cAddress == 0) sens.i2cAddress = 0x62;
                        state.RunLinter();
                    }
                    AutoWrapSameLine(200.0f);
                    if (ImGui::RadioButton("DIO PWM Mode (am-3829)##LidarMode", busMode == 2))
                    {
                        sens.bus = "dio_pwm";
                        state.RunLinter();
                    }

                    if (busMode == 0)
                    {
                        int canIdInt = static_cast<int>(sens.canId);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("CAN ID##LidarCAN", &canIdInt, 1, 5))
                        {
                            sens.canId = static_cast<uint32_t>(std::max(0, canIdInt));
                            state.RunLinter();
                        }

                        ImGui::SameLine(0.0f, 12.0f);
                        int busIdx = (sens.bus == "canivore") ? 1 : 0;
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::Combo("CAN Bus##LidarCAN", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                        {
                            sens.bus = kCanBuses[busIdx];
                            state.RunLinter();
                        }
                    }
                    else if (busMode == 1)
                    {
                        int busIdx = (sens.bus == "i2c_mxp") ? 1 : 0;
                        const char* i2cBuses[] = { "I2C (Onboard)", "I2C (MXP Extension)" };
                        ImGui::SetNextItemWidth(160.0f);
                        if (ImGui::Combo("I2C Port##LidarI2C", &busIdx, i2cBuses, 2))
                        {
                            sens.bus = (busIdx == 1) ? "i2c_mxp" : "i2c_onboard";
                            state.RunLinter();
                        }

                        ImGui::SameLine(0.0f, 12.0f);
                        int addrHex = static_cast<int>(sens.i2cAddress);
                        ImGui::SetNextItemWidth(90.0f);
                        if (ImGui::InputInt("I2C Address (Hex 0x62=98)##LidarI2C", &addrHex, 1, 16))
                        {
                            sens.i2cAddress = static_cast<uint32_t>(std::clamp(addrHex, 0, 127));
                        }
                    }
                    else
                    {
                        int dioChan = static_cast<int>(sens.dioChannel);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputInt("roboRIO DIO Channel [0..31]##LidarDIO", &dioChan, 1, 2))
                        {
                            sens.dioChannel = static_cast<uint32_t>(std::clamp(dioChan, 0, 31));
                            state.RunLinter();
                        }
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("PWM output mode: 10 us pulse width per cm measured (am-3829 Mode pin)");
                    }

                    // Mount Parent Link
                    std::vector<const char*> linkNames;
                    int selectedLinkIdx = -1;
                    for (size_t l = 0; l < state.manifest.links.size(); ++l)
                    {
                        linkNames.push_back(state.manifest.links[l].name.c_str());
                        if (sens.parentLink == state.manifest.links[l].name)
                        {
                            selectedLinkIdx = static_cast<int>(l);
                        }
                    }
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::Combo("Mount Link##LidarLink", &selectedLinkIdx, linkNames.data(), static_cast<int>(linkNames.size())))
                    {
                        if (selectedLinkIdx >= 0 && selectedLinkIdx < static_cast<int>(linkNames.size()))
                        {
                            sens.parentLink = linkNames[selectedLinkIdx];
                            state.RunLinter();
                        }
                    }

                    // Mount Offset (m) in Robot Frame (+X=Right, +Y=Up, +Z=Front)
                    float offX = static_cast<float>(sens.mountOffsetM.x);
                    float offY = static_cast<float>(sens.mountOffsetM.y);
                    float offZ = static_cast<float>(sens.mountOffsetM.z);
                    ImGui::Text("Mount Offset relative to Link Origin (+X=Right, +Y=Up, +Z=Front) (m):");
                    bool offChanged = false;
                    offChanged |= SliderWithDirectInput("X (Right [+]) (m)##LidarOff", &offX, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Y (Up [+]) (m)##LidarOff", &offY, -1.0f, 1.0f, "%.3f");
                    offChanged |= SliderWithDirectInput("Z (Front [+]) (m)##LidarOff", &offZ, -1.0f, 1.0f, "%.3f");
                    if (offChanged)
                    {
                        sens.mountOffsetM = glm::dvec3(offX, offY, offZ);
                    }

                    // Pointing Direction
                    ImGui::Text("Laser Pointing Direction:");
                    if (ImGui::Button("Front (+Z)##LidarDir"))
                    {
                        sens.mountRotationQuat = glm::dquat(1.0, 0.0, 0.0, 0.0);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Rear (-Z)##LidarDir"))
                    {
                        sens.mountRotationQuat = glm::dquat(0.0, 0.0, 1.0, 0.0); // 180 deg around Y
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Left (-X)##LidarDir"))
                    {
                        sens.mountRotationQuat = glm::dquat(0.7071068, 0.0, -0.7071068, 0.0); // -90 deg around Y
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Right (+X)##LidarDir"))
                    {
                        sens.mountRotationQuat = glm::dquat(0.7071068, 0.0, 0.7071068, 0.0); // +90 deg around Y
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Down (-Y)##LidarDir"))
                    {
                        sens.mountRotationQuat = glm::dquat(0.7071068, 0.7071068, 0.0, 0.0); // 90 deg pitch down
                    }

                    // Range Bounds & Noise Calibration
                    float minR = static_cast<float>(sens.triggerMin);
                    float maxR = static_cast<float>(sens.triggerMax > 0.0 ? sens.triggerMax : 40.0);
                    bool rangeChanged = false;
                    rangeChanged |= SliderWithDirectInput("Min Range (m)##LidarRange", &minR, 0.01f, 2.0f, "%.3f m", 100.0f, 65.0f);
                    rangeChanged |= SliderWithDirectInput("Max Range (m)##LidarRange", &maxR, 0.5f, 50.0f, "%.2f m", 100.0f, 65.0f);
                    if (rangeChanged)
                    {
                        sens.triggerMin = static_cast<double>(minR);
                        sens.triggerMax = static_cast<double>(maxR);
                        state.RunLinter();
                    }

                    // Presets
                    ImGui::TextDisabled("Quick Presets:");
                    ImGui::SameLine();
                    if (ImGui::SmallButton("am-3829 LIDAR-Lite v3 (40m)"))
                    {
                        sens.triggerMin = 0.05;
                        sens.triggerMax = 40.0;
                        sens.bus = "i2c_onboard";
                        sens.i2cAddress = 0x62;
                        sens.customNumericParams["noiseStdDevM"] = 0.01;
                        state.RunLinter();
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("am-5684 CAN Lidar (4m)"))
                    {
                        sens.triggerMin = 0.03;
                        sens.triggerMax = 4.0;
                        sens.bus = "rio";
                        if (sens.canId == 0) sens.canId = 35;
                        sens.customNumericParams["noiseStdDevM"] = 0.005;
                        state.RunLinter();
                    }

                    float noiseM = static_cast<float>(sens.customNumericParams.count("noiseStdDevM") ? sens.customNumericParams["noiseStdDevM"] : 0.01);
                    if (SliderWithDirectInput("Measurement Noise StdDev (m)##Lidar", &noiseM, 0.0f, 0.05f, "%.4f m", 100.0f, 65.0f))
                    {
                        sens.customNumericParams["noiseStdDevM"] = std::max(0.0, static_cast<double>(noiseM));
                    }
                }
                // CASE K: Custom Transducer
                else
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Sensor Configuration:");

                    char customTypeBuf[64];
                    strncpy_s(customTypeBuf, sizeof(customTypeBuf), sens.type.c_str(), _TRUNCATE);
                    ImGui::SetNextItemWidth(160.0f);
                    if (ImGui::InputText("Custom Type ID##CustomSens", customTypeBuf, sizeof(customTypeBuf)))
                    {
                        sens.type = customTypeBuf;
                        state.RunLinter();
                    }

                    char busBuf[32];
                    strncpy_s(busBuf, sizeof(busBuf), sens.bus.c_str(), _TRUNCATE);
                    ImGui::SetNextItemWidth(120.0f);
                    if (ImGui::InputText("Bus Protocol##CustomSens", busBuf, sizeof(busBuf)))
                    {
                        sens.bus = busBuf;
                        state.RunLinter();
                    }

                    int canIdInt = static_cast<int>(sens.canId);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("CAN ID##CustomSens", &canIdInt, 1, 5))
                    {
                        sens.canId = static_cast<uint32_t>(std::max(0, canIdInt));
                        state.RunLinter();
                    }

                    int dioChan = static_cast<int>(sens.dioChannel);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("DIO Channel##CustomSens", &dioChan, 1, 2))
                    {
                        sens.dioChannel = static_cast<uint32_t>(std::clamp(dioChan, 0, 31));
                        state.RunLinter();
                    }

                    int anChan = static_cast<int>(sens.analogChannel);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("Analog Channel##CustomSens", &anChan, 1, 2))
                    {
                        sens.analogChannel = static_cast<uint32_t>(std::clamp(anChan, 0, 7));
                        state.RunLinter();
                    }
                }
            }

            ImGui::PopID();
            ImGui::Spacing();
        }

        // Process deletion if requested
        if (sensorToDelete >= 0 && sensorToDelete < static_cast<int>(sensors.size()))
        {
            sensors.erase(sensors.begin() + sensorToDelete);
            state.RunLinter();
        }

        // Process duplication if requested
        if (sensorToDuplicate >= 0 && sensorToDuplicate < static_cast<int>(sensors.size()))
        {
            auto dup = sensors[sensorToDuplicate];
            std::string baseName = dup.name;
            size_t copyIdx = 1;
            std::string newName = baseName + "_copy";
            auto nameExists = [&](const std::string& n) {
                for (const auto& s : sensors) {
                    if (s.name == n) return true;
                }
                return false;
            };
            while (nameExists(newName)) {
                copyIdx++;
                newName = baseName + "_copy" + std::to_string(copyIdx);
            }
            dup.name = newName;

            // Increment IDs to prevent immediate collisions
            if (dup.canId > 0 && dup.canId < 62)
            {
                dup.canId++;
            }
            if (dup.dioChannel >= 0 && dup.dioChannel < 31)
            {
                dup.dioChannel++;
            }
            if (dup.analogChannel >= 0 && dup.analogChannel < 7)
            {
                dup.analogChannel++;
            }

            sensors.push_back(dup);
            state.RunLinter();
        }

        ImGui::PopTextWrapPos();
    }
}
