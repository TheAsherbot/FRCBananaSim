/**
 * @file MechanismPanel.cpp
 * @brief Implementation of the 1-DOF robot mechanism configuration panel.
 */

#include "Tools/Configurator/Panels/MechanismPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>
#include "ConfiguratorWidgets.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace FRCBananaSim::Tools::Configurator
{

    static const char* kMotorPresets[] = {
        "KrakenX60", "KrakenX60FOC", "KrakenX44", "KrakenX44FOC",
        "Falcon500", "Minion", "NEO", "NEO2", "NEO550", "NEO_Vortex",
        "GoBilda5202", "Custom"
    };

    static const char* kServoPresets[] = {
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

    static const char* kServoIds[] = {
        "GoBildaSuperSpeed", "GoBildaSpeed", "GoBildaTorque",
        "SwyftSpeed", "SwyftBalanced", "SwyftTorque",
        "SwyftSlim", "SwyftNano", "Custom"
    };

    static const char* kMechanismRoles[] = {
        "elevator", "arm", "wrist", "turret", "flywheel", "intake", "differential"
    };

    static const char* kDifferentialTypes[] = {
        "symmetric_bevel", "custom_matrix", "core_xy", "coupled_arm"
    };

    static const char* kControllerVendors[] = {
        "CTRE_TalonFX", "REV_SparkMax", "CTRE_TalonSRX"
    };

    static const char* kCanBuses[] = {
        "rio", "canivore"
    };

    static const char* kCanBusLabels[] = {
        "roboRIO (rio)", "CANivore (canivore)"
    };

    static const char* kControllerRoles[] = {
        "leader", "follower"
    };

    void MechanismPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "ROBOT MECHANISMS & ACTUATORS");
        ImGui::Separator();

        if (!state.isSidecarLoaded && !state.isCadLoaded)
        {
            ImGui::TextWrapped("No robot loaded. Open a CAD folder or load a reference robot to configure mechanisms.");
            return;
        }

        auto& mechanisms = state.sidecar.mechanisms;

        // -------------------------------------------------------------
        // 1. Add New Mechanism Bar
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Add New Mechanism", ImGuiTreeNodeFlags_None))
        {
            ImGui::SetNextItemWidth(140.0f);
            ImGui::InputText("Name##NewMech", newMechNameBuf, sizeof(newMechNameBuf));
            ImGui::SameLine(0.0f, 8.0f);

            ImGui::SetNextItemWidth(110.0f);
            ImGui::Combo("Role##NewMech", &newMechRoleIdx, kMechanismRoles, IM_ARRAYSIZE(kMechanismRoles));
            ImGui::SameLine(0.0f, 8.0f);

            if (ImGui::Button("+ Create Mechanism"))
            {
                Robot::Schema::MechanismConfig newMech;
                newMech.name = (newMechNameBuf[0] != '\0') ? newMechNameBuf : "Mechanism";
                newMech.role = kMechanismRoles[newMechRoleIdx];

                // Assign default kinematic joint & parameters matching role
                if (newMech.role == "elevator")
                {
                    newMech.gearRatio = 9.0;
                    newMech.drumRadiusM = 0.01905;
                    newMech.springAssist.type = "constant_force";
                    newMech.springAssist.forceN = 45.0;
                    newMech.motorPreset = "KrakenX60";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "prismatic")
                        {
                            newMech.targetJoint = j.name;
                            newMech.parentLink = j.parentLink;
                            break;
                        }
                    }
                }
                else if (newMech.role == "arm")
                {
                    newMech.gearRatio = 65.0;
                    newMech.motorPreset = "KrakenX60";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newMech.targetJoint = j.name;
                            newMech.parentLink = j.parentLink;
                            break;
                        }
                    }
                }
                else if (newMech.role == "wrist")
                {
                    newMech.gearRatio = 25.0;
                    newMech.motorPreset = "Falcon500";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newMech.targetJoint = j.name;
                            newMech.parentLink = j.parentLink;
                            break;
                        }
                    }
                }
                else if (newMech.role == "turret")
                {
                    newMech.gearRatio = 40.0;
                    newMech.motorPreset = "KrakenX44";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newMech.targetJoint = j.name;
                            newMech.parentLink = j.parentLink;
                            break;
                        }
                    }
                }
                else if (newMech.role == "flywheel")
                {
                    newMech.gearRatio = 1.0;
                    newMech.flywheelInertiaKgm2 = 0.0025;
                    newMech.spinAxis = { 1.0, 0.0, 0.0 };
                    newMech.motorPreset = "KrakenX60";
                    if (!state.manifest.links.empty())
                    {
                        newMech.parentLink = state.manifest.links[0].name;
                    }
                }
                else if (newMech.role == "intake")
                {
                    newMech.gearRatio = 3.0;
                    newMech.motorPreset = "KrakenX44";
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            newMech.targetJoint = j.name;
                            newMech.parentLink = j.parentLink;
                            break;
                        }
                    }
                }
                else if (newMech.role == "differential")
                {
                    newMech.differentialType = "symmetric_bevel";
                    newMech.gearRatios = { 50.0, 50.0 };
                    newMech.coulombFrictions = { 0.0, 0.0 };
                    newMech.viscousDampings = { 0.0, 0.0 };
                    newMech.motorPreset = "KrakenX60";

                    std::vector<std::string> revJoints;
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            revJoints.push_back(j.name);
                        }
                    }
                    if (revJoints.size() >= 2)
                    {
                        newMech.targetJoints = { revJoints[0], revJoints[1] };
                        const auto* pJ = state.manifest.FindJoint(revJoints[0]);
                        if (pJ) newMech.parentLink = pJ->parentLink;
                    }
                    else if (!state.manifest.joints.empty())
                    {
                        newMech.targetJoints = { state.manifest.joints[0].name, state.manifest.joints[0].name };
                        newMech.parentLink = state.manifest.joints[0].parentLink;
                    }
                }

                // Seed leader motor controller with next available CAN ID
                uint32_t nextCanId = 9;
                for (const auto& m : mechanisms)
                {
                    for (const auto& c : m.controllers)
                    {
                        if (c.canId >= nextCanId)
                        {
                            nextCanId = c.canId + 1;
                        }
                    }
                }
                Robot::Schema::ControllerConfig ctrl;
                ctrl.canId = nextCanId;
                ctrl.vendor = "CTRE_TalonFX";
                ctrl.canBus = "rio";
                ctrl.role = "leader";
                ctrl.inverted = false;
                newMech.controllers.push_back(ctrl);

                if (newMech.role == "differential")
                {
                    Robot::Schema::ControllerConfig ctrlB;
                    ctrlB.canId = nextCanId + 1;
                    ctrlB.vendor = "CTRE_TalonFX";
                    ctrlB.canBus = "rio";
                    ctrlB.role = "follower";
                    ctrlB.inverted = false;
                    newMech.controllers.push_back(ctrlB);
                }

                mechanisms.push_back(newMech);
                std::snprintf(newMechNameBuf, sizeof(newMechNameBuf), "Mechanism%zu", mechanisms.size() + 1);
                state.RunLinter();
            }
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Configured Mechanisms (%zu):", mechanisms.size());

        if (mechanisms.empty())
        {
            ImGui::TextWrapped("No mechanisms configured. Click '+ Add New Mechanism' above or 'Auto-Generate Starter Sidecar' in CAD Hierarchy.");
            return;
        }

        // Cache all CAD link names for parent/visual dropdowns
        std::vector<const char*> allLinkNames;
        for (const auto& l : state.manifest.links)
        {
            allLinkNames.push_back(l.name.c_str());
        }

        // -------------------------------------------------------------
        // 2. Mechanisms List (Cards)
        // -------------------------------------------------------------
        int mechToDelete = -1;

        for (size_t i = 0; i < mechanisms.size(); ++i)
        {
            auto& mech = mechanisms[i];
            ImGui::PushID(static_cast<int>(i));

            std::string headerLabel = "[" + mech.role + "] " + mech.name + "##CardHeader";
            bool isCardOpen = ImGui::CollapsingHeader(headerLabel.c_str(), ImGuiTreeNodeFlags_DefaultOpen);
            bool isFocused = ImGui::IsItemActive() || ImGui::IsItemClicked() || ImGui::IsItemHovered();
            if (isCardOpen && (isFocused || state.selectedMechanism.empty() || state.selectedMechanism == mech.name))
            {
                state.selectedMechanism = mech.name;
                state.selectedJoint = mech.targetJoint;
                const auto* boundJ = state.manifest.FindJoint(mech.targetJoint);
                if (boundJ && !boundJ->childLink.empty())
                {
                    state.selectedLink = boundJ->childLink;
                }
                else
                {
                    state.selectedLink = mech.visualLink.empty() ? mech.parentLink : mech.visualLink;
                }
            }

            if (isCardOpen)
            {
                // Top Action Row
                ImGui::TextDisabled("Mechanism Nickname (custom code / telemetry identifier):");
                char nameBuf[64];
                std::snprintf(nameBuf, sizeof(nameBuf), "%s", mech.name.c_str());
                ImGui::SetNextItemWidth(220.0f);
                if (ImGui::InputText("##MechanismName", nameBuf, sizeof(nameBuf)))
                {
                    mech.name = nameBuf;
                    state.selectedMechanism = mech.name;
                    state.RunLinter();
                }

                ImGui::SameLine(0.0f, 12.0f);
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
                if (ImGui::Button("Delete Mechanism"))
                {
                    mechToDelete = static_cast<int>(i);
                }
                ImGui::PopStyleColor(2);

                // Role Dropdown
                int roleIdx = 0;
                for (int r = 0; r < static_cast<int>(IM_ARRAYSIZE(kMechanismRoles)); ++r)
                {
                    if (mech.role == kMechanismRoles[r])
                    {
                        roleIdx = r;
                        break;
                    }
                }
                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::Combo("Mechanism Role", &roleIdx, kMechanismRoles, IM_ARRAYSIZE(kMechanismRoles)))
                {
                    mech.role = kMechanismRoles[roleIdx];
                    state.RunLinter();
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // ---------------------------------------------------------
                // A. Kinematic Joint & Link Bindings (CAD-Aware Filtering)
                // ---------------------------------------------------------
                if (mech.role == "differential")
                {
                    if (mech.targetJoints.size() < 2) mech.targetJoints.resize(2);
                    if (mech.gearRatios.size() < 2) mech.gearRatios.resize(2, 50.0);
                    if (mech.coulombFrictions.size() < 2) mech.coulombFrictions.resize(2, 0.0);
                    if (mech.viscousDampings.size() < 2) mech.viscousDampings.resize(2, 0.0);

                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "2-DOF Differential Kinematics & Joint Bindings (Revolute Only):");

                    // Filter CAD joints: strictly revolute/continuous
                    std::vector<const char*> revJointNames;
                    for (const auto& joint : state.manifest.joints)
                    {
                        if (joint.type == "revolute" || joint.type == "continuous")
                        {
                            revJointNames.push_back(joint.name.c_str());
                        }
                    }
                    if (revJointNames.empty())
                    {
                        for (const auto& joint : state.manifest.joints)
                        {
                            revJointNames.push_back(joint.name.c_str());
                        }
                    }

                    // Pitch Joint (Joint 0)
                    int pitchIdx = -1;
                    for (size_t j = 0; j < revJointNames.size(); ++j)
                    {
                        if (mech.targetJoints[0] == revJointNames[j]) { pitchIdx = static_cast<int>(j); break; }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Pitch Joint (Output A)##DiffPitch", &pitchIdx, revJointNames.data(), static_cast<int>(revJointNames.size())))
                    {
                        mech.targetJoints[0] = revJointNames[pitchIdx];
                        state.selectedJoint = mech.targetJoints[0];
                        state.RunLinter();
                    }

                    // Roll Joint (Joint 1)
                    int rollIdx = -1;
                    for (size_t j = 0; j < revJointNames.size(); ++j)
                    {
                        if (mech.targetJoints[1] == revJointNames[j]) { rollIdx = static_cast<int>(j); break; }
                    }
                    ImGui::SetNextItemWidth(200.0f);
                    if (ImGui::Combo("Roll Joint (Output B)##DiffRoll", &rollIdx, revJointNames.data(), static_cast<int>(revJointNames.size())))
                    {
                        mech.targetJoints[1] = revJointNames[rollIdx];
                        state.RunLinter();
                    }

                    // Differential Type
                    int diffTypeIdx = 0;
                    for (int t = 0; t < static_cast<int>(IM_ARRAYSIZE(kDifferentialTypes)); ++t)
                    {
                        if (mech.differentialType == kDifferentialTypes[t]) { diffTypeIdx = t; break; }
                    }
                    ImGui::SetNextItemWidth(170.0f);
                    if (ImGui::Combo("Differential Type##DiffType", &diffTypeIdx, kDifferentialTypes, IM_ARRAYSIZE(kDifferentialTypes)))
                    {
                        mech.differentialType = kDifferentialTypes[diffTypeIdx];
                        state.RunLinter();
                    }

                    // Limit configuration for Differential Joints
                    if (!mech.targetJoints[0].empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Pitch Joint Output A Limits ('%s'):", mech.targetJoints[0].c_str());
                        RenderJointLimitEditor(state, mech.targetJoints[0], "DiffPitch");
                    }
                    if (!mech.targetJoints[1].empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Roll Joint Output B Limits ('%s'):", mech.targetJoints[1].c_str());
                        RenderJointLimitEditor(state, mech.targetJoints[1], "DiffRoll");
                    }

                    // Parent Link Combo
                    int parentLinkIdx = -1;
                    for (size_t l = 0; l < allLinkNames.size(); ++l)
                    {
                        if (mech.parentLink == allLinkNames[l]) { parentLinkIdx = static_cast<int>(l); break; }
                    }
                    if (!allLinkNames.empty())
                    {
                        ImGui::SetNextItemWidth(200.0f);
                        if (ImGui::Combo("Parent Link##DiffParent", &parentLinkIdx, allLinkNames.data(), static_cast<int>(allLinkNames.size())))
                        {
                            if (parentLinkIdx >= 0 && parentLinkIdx < static_cast<int>(allLinkNames.size()))
                            {
                                mech.parentLink = allLinkNames[parentLinkIdx];
                                state.RunLinter();
                            }
                        }
                    }
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Kinematic Joint & Link Bindings:");

                    // Filter CAD joints by degree of freedom
                    std::vector<const char*> filteredJointNames;
                    int selectedJointIdx = -1;
                    bool isElevator = (mech.role == "elevator");

                    for (size_t j = 0; j < state.manifest.joints.size(); ++j)
                    {
                        const auto& joint = state.manifest.joints[j];
                        bool match = isElevator ? (joint.type == "prismatic") :
                            (joint.type == "revolute" || joint.type == "continuous");

                        if (match || state.manifest.joints.size() <= 3) // Fallback if no exact type match
                        {
                            if (joint.name == mech.targetJoint)
                            {
                                selectedJointIdx = static_cast<int>(filteredJointNames.size());
                            }
                            filteredJointNames.push_back(joint.name.c_str());
                        }
                    }

                    if (!filteredJointNames.empty())
                    {
                        ImGui::SetNextItemWidth(200.0f);
                        if (ImGui::Combo("Target Joint", &selectedJointIdx, filteredJointNames.data(), static_cast<int>(filteredJointNames.size())))
                        {
                            if (selectedJointIdx >= 0 && selectedJointIdx < static_cast<int>(filteredJointNames.size()))
                            {
                                mech.targetJoint = filteredJointNames[selectedJointIdx];
                                const auto* jPtr = state.manifest.FindJoint(mech.targetJoint);
                                if (jPtr && !jPtr->parentLink.empty())
                                {
                                    mech.parentLink = jPtr->parentLink;
                                }
                                state.RunLinter();
                            }
                        }
                    }
                    else
                    {
                        char targetJointBuf[64];
                        std::snprintf(targetJointBuf, sizeof(targetJointBuf), "%s", mech.targetJoint.c_str());
                        ImGui::SetNextItemWidth(200.0f);
                        if (ImGui::InputText("Target Joint", targetJointBuf, sizeof(targetJointBuf)))
                        {
                            mech.targetJoint = targetJointBuf;
                            state.RunLinter();
                        }
                    }

                    if (!mech.targetJoint.empty())
                    {
                        ImGui::Spacing();
                        RenderJointLimitEditor(state, mech.targetJoint, "MechLimit");
                    }

                    // Parent Link Combo
                    int parentLinkIdx = -1;
                    for (size_t l = 0; l < allLinkNames.size(); ++l)
                    {
                        if (mech.parentLink == allLinkNames[l])
                        {
                            parentLinkIdx = static_cast<int>(l);
                            break;
                        }
                    }
                    if (!allLinkNames.empty())
                    {
                        ImGui::SetNextItemWidth(200.0f);
                        if (ImGui::Combo("Parent Link", &parentLinkIdx, allLinkNames.data(), static_cast<int>(allLinkNames.size())))
                        {
                            if (parentLinkIdx >= 0 && parentLinkIdx < static_cast<int>(allLinkNames.size()))
                            {
                                mech.parentLink = allLinkNames[parentLinkIdx];
                                state.RunLinter();
                            }
                        }
                    }
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // ---------------------------------------------------------
                // B. Actuator Selection (CAN Motor vs PWM Servo)
                // ---------------------------------------------------------
                if (mech.role == "differential")
                {
                    if (mech.motorPresets.size() < 2)
                    {
                        mech.motorPresets.resize(2, mech.motorPreset.empty() ? "KrakenX60" : mech.motorPreset);
                    }
                    if (mech.actuatorTypes.size() < 2)
                    {
                        mech.actuatorTypes.resize(2, mech.actuatorType.empty() ? "motor" : mech.actuatorType);
                    }
                    if (mech.customMotors.size() < 2)
                    {
                        mech.customMotors.resize(2);
                    }
                    if (mech.servos.size() < 2)
                    {
                        mech.servos.resize(2);
                        mech.servos[0].pwmChannel = 0;
                        mech.servos[1].pwmChannel = 1;
                    }
                    if (mech.customServos.size() < 2)
                    {
                        mech.customServos.resize(2);
                    }
                    if (mech.statorCurrentLimits.size() < 2)
                    {
                        mech.statorCurrentLimits.resize(2, mech.statorCurrentLimitA);
                    }
                    if (mech.supplyCurrentLimits.size() < 2)
                    {
                        mech.supplyCurrentLimits.resize(2, mech.supplyCurrentLimitA);
                    }
                    if (mech.controllers.size() < 2)
                    {
                        uint32_t canA = 9;
                        uint32_t canB = 10;
                        if (!mech.controllers.empty()) canA = mech.controllers[0].canId;
                        Robot::Schema::ControllerConfig cA;
                        cA.vendor = "CTRE_TalonFX";
                        cA.canId = canA;
                        cA.canBus = "rio";
                        cA.role = "leader";
                        Robot::Schema::ControllerConfig cB;
                        cB.vendor = "CTRE_TalonFX";
                        cB.canId = canB;
                        cB.canBus = "rio";
                        cB.role = "leader";
                        mech.controllers = { cA, cB };
                    }

                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Actuation & Input Power Sources (2 Inputs Required for Differential):");

                    const char* kSourceTitles[2] = {
                        "Input Power Source 1 (Motor / Actuator A)",
                        "Input Power Source 2 (Motor / Actuator B)"
                    };

                    for (int src = 0; src < 2; ++src)
                    {
                        ImGui::PushID(src);
                        ImGui::Spacing();
                        if (ImGui::TreeNodeEx((void*)(intptr_t)src, ImGuiTreeNodeFlags_DefaultOpen, "%s", kSourceTitles[src]))
                        {
                            int actMode = (mech.actuatorTypes[src] == "servo") ? 1 : 0;
                            ImGui::RadioButton("CAN DC Motor", &actMode, 0);
                            ImGui::SameLine(0.0f, 12.0f);
                            ImGui::RadioButton("PWM Servo", &actMode, 1);
                            mech.actuatorTypes[src] = (actMode == 1) ? "servo" : "motor";

                            if (actMode == 0)
                            {
                                // Motor Preset
                                int motorIdx = -1;
                                for (int m = 0; m < static_cast<int>(IM_ARRAYSIZE(kMotorPresets)) - 1; ++m)
                                {
                                    if (mech.motorPresets[src] == kMotorPresets[m]) { motorIdx = m; break; }
                                }
                                if (motorIdx == -1 || mech.motorPresets[src] == "Custom")
                                {
                                    motorIdx = static_cast<int>(IM_ARRAYSIZE(kMotorPresets)) - 1;
                                }

                                ImGui::SetNextItemWidth(170.0f);
                                if (ImGui::Combo("Motor Preset", &motorIdx, kMotorPresets, IM_ARRAYSIZE(kMotorPresets)))
                                {
                                    mech.motorPresets[src] = kMotorPresets[motorIdx];
                                    state.RunLinter();
                                }

                                // Custom Motor Specs
                                if (mech.motorPresets[src] == "Custom")
                                {
                                    ImGui::Indent(12.0f);
                                    ImGui::TextDisabled("Custom Motor Specification (Input %s):", (src == 0 ? "A" : "B"));
                                    float vNom = static_cast<float>(mech.customMotors[src].nominalVoltageV);
                                    if (SliderWithDirectInput("Nominal Voltage (V)##DiffCustVolt", &vNom, 4.0f, 24.0f, "%.1f"))
                                        mech.customMotors[src].nominalVoltageV = static_cast<double>(vNom);

                                    float stallTorque = static_cast<float>(mech.customMotors[src].stallTorqueNm);
                                    if (SliderWithDirectInput("Stall Torque (N*m)##DiffCustTorque", &stallTorque, 0.10f, 15.0f, "%.3f"))
                                        mech.customMotors[src].stallTorqueNm = static_cast<double>(stallTorque);

                                    float freeSpeed = static_cast<float>(mech.customMotors[src].freeSpeedRpm);
                                    if (SliderWithDirectInput("Free Speed (RPM)##DiffCustSpeed", &freeSpeed, 500.0f, 25000.0f, "%.0f"))
                                        mech.customMotors[src].freeSpeedRpm = static_cast<double>(freeSpeed);

                                    float stallCurr = static_cast<float>(mech.customMotors[src].stallCurrentA);
                                    if (SliderWithDirectInput("Stall Current (A)##DiffCustStallCurr", &stallCurr, 10.0f, 500.0f, "%.0f"))
                                        mech.customMotors[src].stallCurrentA = static_cast<double>(stallCurr);

                                    float freeCurr = static_cast<float>(mech.customMotors[src].freeCurrentA);
                                    if (SliderWithDirectInput("Free Current (A)##DiffCustFreeCurr", &freeCurr, 0.1f, 10.0f, "%.2f"))
                                        mech.customMotors[src].freeCurrentA = static_cast<double>(freeCurr);
                                    ImGui::Unindent(12.0f);
                                }

                                // Primary Controller for this input source
                                ImGui::Spacing();
                                ImGui::TextDisabled("Primary Motor Controller (Input %s):", (src == 0 ? "A" : "B"));
                                auto& ctrl = mech.controllers[src];

                                ImGui::SetNextItemWidth(125.0f);
                                int vendorIdx = 0;
                                for (int v = 0; v < static_cast<int>(IM_ARRAYSIZE(kControllerVendors)); ++v)
                                {
                                    if (ctrl.vendor == kControllerVendors[v]) { vendorIdx = v; break; }
                                }
                                if (ImGui::Combo("##CtrlVendor", &vendorIdx, kControllerVendors, IM_ARRAYSIZE(kControllerVendors)))
                                {
                                    ctrl.vendor = kControllerVendors[vendorIdx];
                                }

                                ImGui::SameLine(0.0f, 8.0f);
                                ImGui::SetNextItemWidth(55.0f);
                                int canIdVal = static_cast<int>(ctrl.canId);
                                if (ImGui::InputInt("##CtrlCanId", &canIdVal, 0))
                                {
                                    ctrl.canId = static_cast<uint32_t>(std::max(0, canIdVal));
                                    state.RunLinter();
                                }
                                if (ImGui::IsItemHovered()) ImGui::SetTooltip("CAN ID");

                                ImGui::SameLine(0.0f, 8.0f);
                                ImGui::SetNextItemWidth(130.0f);
                                int busIdx = (ctrl.canBus == "canivore") ? 1 : 0;
                                if (ImGui::Combo("##CtrlBus", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                                {
                                    ctrl.canBus = kCanBuses[busIdx];
                                    state.RunLinter();
                                }

                                ImGui::SameLine(0.0f, 8.0f);
                                ImGui::Checkbox("Inv##CtrlInv", &ctrl.inverted);
                                if (ImGui::IsItemHovered()) ShowMotorInvertTooltip(ctrl.vendor, ctrl.inverted);

                                // Current Limits
                                float statorLim = static_cast<float>(mech.statorCurrentLimits[src]);
                                if (SliderWithDirectInput("Stator Limit (A)##DiffStator", &statorLim, 10.0f, 140.0f, "%.0f"))
                                    mech.statorCurrentLimits[src] = static_cast<double>(statorLim);

                                float supplyLim = static_cast<float>(mech.supplyCurrentLimits[src]);
                                if (SliderWithDirectInput("Supply Limit (A)##DiffSupply", &supplyLim, 10.0f, 90.0f, "%.0f"))
                                    mech.supplyCurrentLimits[src] = static_cast<double>(supplyLim);
                            }
                            else
                            {
                                // PWM Servo for this source
                                auto& sc = mech.servos[src];
                                int servoIdx = -1;
                                for (int s = 0; s < static_cast<int>(IM_ARRAYSIZE(kServoIds)) - 1; ++s)
                                {
                                    if (sc.preset == kServoIds[s]) { servoIdx = s; break; }
                                }
                                if (servoIdx == -1 || sc.preset == "Custom")
                                {
                                    servoIdx = static_cast<int>(IM_ARRAYSIZE(kServoIds)) - 1;
                                }

                                ImGui::SetNextItemWidth(210.0f);
                                if (ImGui::Combo("Servo Preset##DiffServo", &servoIdx, kServoPresets, IM_ARRAYSIZE(kServoPresets)))
                                {
                                    sc.preset = kServoIds[servoIdx];
                                    state.RunLinter();
                                }

                                int pwmCh = static_cast<int>(sc.pwmChannel);
                                ImGui::SetNextItemWidth(45.0f);
                                if (ImGui::InputInt("PWM Channel##DiffPwm", &pwmCh, 0))
                                {
                                    sc.pwmChannel = static_cast<uint32_t>(std::clamp(pwmCh, 0, 15));
                                    state.RunLinter();
                                }

                                const char* servoModes[] = { "positional", "continuous" };
                                int modeIdx = (sc.mode == "continuous") ? 1 : 0;
                                ImGui::SameLine(0.0f, 10.0f);
                                ImGui::SetNextItemWidth(110.0f);
                                if (ImGui::Combo("Control Mode##DiffServoMode", &modeIdx, servoModes, IM_ARRAYSIZE(servoModes)))
                                {
                                    sc.mode = servoModes[modeIdx];
                                }

                                float sVolt = static_cast<float>(sc.supplyVoltageV);
                                if (SliderWithDirectInput("Supply Voltage (V)##DiffServoVolt", &sVolt, 4.8f, 8.4f, "%.1f"))
                                    sc.supplyVoltageV = static_cast<double>(sVolt);

                                float sRange = static_cast<float>(sc.travelRangeDeg);
                                if (SliderWithDirectInput("Travel Range (deg)##DiffServoRange", &sRange, 90.0f, 360.0f, "%.0f"))
                                    sc.travelRangeDeg = static_cast<double>(sRange);
                            }

                            ImGui::TreePop();
                        }
                        ImGui::PopID();
                    }

                    // Additional Follower Controllers (for 4-motor differential wrists)
                    ImGui::Spacing();
                    if (mech.controllers.size() > 2)
                    {
                        ImGui::TextDisabled("Additional Follower Controllers (%zu):", mech.controllers.size() - 2);
                        int ctrlToDelete = -1;
                        if (ImGui::BeginTable("DiffFollowersTable", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit))
                        {
                            ImGui::TableSetupColumn("Type / Vendor", ImGuiTableColumnFlags_WidthFixed, 115.0f);
                            ImGui::TableSetupColumn("CAN ID", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                            ImGui::TableSetupColumn("CAN Bus", ImGuiTableColumnFlags_WidthFixed, 135.0f);
                            ImGui::TableSetupColumn("Target Input", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                            ImGui::TableSetupColumn("Invert", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                            ImGui::TableSetupColumn(" ", ImGuiTableColumnFlags_WidthFixed, 25.0f);
                            ImGui::TableHeadersRow();

                            for (size_t c = 2; c < mech.controllers.size(); ++c)
                            {
                                auto& ctrl = mech.controllers[c];
                                ImGui::PushID(static_cast<int>(c));
                                ImGui::TableNextRow();

                                // Vendor
                                ImGui::TableNextColumn();
                                ImGui::SetNextItemWidth(-1.0f);
                                int vIdx = 0;
                                for (int v = 0; v < static_cast<int>(IM_ARRAYSIZE(kControllerVendors)); ++v)
                                {
                                    if (ctrl.vendor == kControllerVendors[v]) { vIdx = v; break; }
                                }
                                if (ImGui::Combo("##FolVendor", &vIdx, kControllerVendors, IM_ARRAYSIZE(kControllerVendors)))
                                {
                                    ctrl.vendor = kControllerVendors[vIdx];
                                }

                                // CAN ID
                                ImGui::TableNextColumn();
                                int cid = static_cast<int>(ctrl.canId);
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::InputInt("##FolCanId", &cid, 0))
                                {
                                    ctrl.canId = static_cast<uint32_t>(std::max(0, cid));
                                    state.RunLinter();
                                }

                                // Bus
                                ImGui::TableNextColumn();
                                int bIdx = (ctrl.canBus == "canivore") ? 1 : 0;
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::Combo("##FolBus", &bIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                                {
                                    ctrl.canBus = kCanBuses[bIdx];
                                    state.RunLinter();
                                }

                                // Target Input
                                ImGui::TableNextColumn();
                                const char* targetInputs[] = { "Follows Input A", "Follows Input B" };
                                int targetInputIdx = (ctrl.role == "follower_b") ? 1 : 0;
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::Combo("##FolTarget", &targetInputIdx, targetInputs, IM_ARRAYSIZE(targetInputs)))
                                {
                                    ctrl.role = (targetInputIdx == 1) ? "follower_b" : "follower_a";
                                }

                                // Invert
                                ImGui::TableNextColumn();
                                ImGui::Checkbox("##FolInv", &ctrl.inverted);
                                if (ImGui::IsItemHovered()) ShowMotorInvertTooltip(ctrl.vendor, ctrl.inverted);

                                // Delete
                                ImGui::TableNextColumn();
                                if (ImGui::SmallButton("X"))
                                {
                                    ctrlToDelete = static_cast<int>(c);
                                }

                                ImGui::PopID();
                            }
                            ImGui::EndTable();
                        }
                        if (ctrlToDelete >= 2 && ctrlToDelete < static_cast<int>(mech.controllers.size()))
                        {
                            mech.controllers.erase(mech.controllers.begin() + ctrlToDelete);
                            state.RunLinter();
                        }
                    }

                    if (ImGui::Button("+ Add Follower Controller (e.g. for dual-motor per side)"))
                    {
                        uint32_t nextCanId = 11;
                        for (const auto& m : state.sidecar.mechanisms)
                        {
                            for (const auto& c : m.controllers)
                            {
                                if (c.canId >= nextCanId) nextCanId = c.canId + 1;
                            }
                        }
                        Robot::Schema::ControllerConfig folCtrl;
                        folCtrl.canId = nextCanId;
                        folCtrl.vendor = "CTRE_TalonFX";
                        folCtrl.canBus = "rio";
                        folCtrl.role = "follower_a";
                        folCtrl.inverted = false;
                        mech.controllers.push_back(folCtrl);
                        state.RunLinter();
                    }
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Actuation & Motor / Servo:");

                int actMode = (mech.actuatorType == "servo") ? 1 : 0;
                ImGui::RadioButton("CAN DC Motor", &actMode, 0);
                ImGui::SameLine(0.0f, 12.0f);
                ImGui::RadioButton("PWM Servo", &actMode, 1);
                mech.actuatorType = (actMode == 1) ? "servo" : "motor";

                if (actMode == 0)
                {
                    // Motor Preset Dropdown
                    int motorIdx = -1;
                    for (int m = 0; m < static_cast<int>(IM_ARRAYSIZE(kMotorPresets)) - 1; ++m)
                    {
                        if (mech.motorPreset == kMotorPresets[m])
                        {
                            motorIdx = m;
                            break;
                        }
                    }
                    if (motorIdx == -1 || mech.motorPreset == "Custom")
                    {
                        motorIdx = static_cast<int>(IM_ARRAYSIZE(kMotorPresets)) - 1; // Custom
                    }

                    ImGui::SetNextItemWidth(180.0f);
                    if (ImGui::Combo("Motor Preset", &motorIdx, kMotorPresets, IM_ARRAYSIZE(kMotorPresets)))
                    {
                        mech.motorPreset = kMotorPresets[motorIdx];
                        state.RunLinter();
                    }

                    // Parametric Custom Motor Editor
                    if (mech.motorPreset == "Custom")
                    {
                        ImGui::Spacing();
                        ImGui::Indent(12.0f);
                        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Motor Specification:");

                        float vNom = static_cast<float>(mech.customMotor.nominalVoltageV);
                        if (SliderWithDirectInput("Nominal Voltage (V)##MechCustomMotor", &vNom, 4.0f, 24.0f, "%.1f"))
                        {
                            mech.customMotor.nominalVoltageV = static_cast<double>(vNom);
                        }

                        float stallTorque = static_cast<float>(mech.customMotor.stallTorqueNm);
                        if (SliderWithDirectInput("Stall Torque (N*m)##MechCustomMotor", &stallTorque, 0.10f, 15.0f, "%.3f"))
                        {
                            mech.customMotor.stallTorqueNm = static_cast<double>(stallTorque);
                        }

                        float freeSpeed = static_cast<float>(mech.customMotor.freeSpeedRpm);
                        if (SliderWithDirectInput("Free Speed (RPM)##MechCustomMotor", &freeSpeed, 500.0f, 25000.0f, "%.0f"))
                        {
                            mech.customMotor.freeSpeedRpm = static_cast<double>(freeSpeed);
                        }

                        float stallCurr = static_cast<float>(mech.customMotor.stallCurrentA);
                        if (SliderWithDirectInput("Stall Current (A)##MechCustomMotor", &stallCurr, 5.0f, 500.0f, "%.1f"))
                        {
                            mech.customMotor.stallCurrentA = static_cast<double>(stallCurr);
                        }

                        float freeCurr = static_cast<float>(mech.customMotor.freeCurrentA);
                        if (SliderWithDirectInput("Free Current (A)##MechCustomMotor", &freeCurr, 0.05f, 10.0f, "%.2f"))
                        {
                            mech.customMotor.freeCurrentA = static_cast<double>(freeCurr);
                        }

                        ImGui::Unindent(12.0f);
                    }

                    // Electrical Limits
                    ImGui::Spacing();
                    ImGui::TextDisabled("Electrical Current Limits:");

                    float statorLim = static_cast<float>(mech.statorCurrentLimitA);
                    if (SliderWithDirectInput("Stator Limit (A)##MechStator", &statorLim, 10.0f, 140.0f, "%.0f"))
                    {
                        mech.statorCurrentLimitA = static_cast<double>(statorLim);
                    }

                    float supplyLim = static_cast<float>(mech.supplyCurrentLimitA);
                    if (SliderWithDirectInput("Supply Limit (A)##MechSupply", &supplyLim, 10.0f, 90.0f, "%.0f"))
                    {
                        mech.supplyCurrentLimitA = static_cast<double>(supplyLim);
                    }

                    // Hardware Controllers Table
                    ImGui::Spacing();
                    ImGui::TextDisabled("Hardware Motor Controllers (%zu):", mech.controllers.size());
                    ImGui::SameLine(0.0f, 6.0f);
                    ShowMotorInvertHelpMarker();

                    int ctrlToDelete = -1;
                    if (ImGui::BeginTable("ControllersTable", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit))
                    {
                        ImGui::TableSetupColumn("Type / Vendor", ImGuiTableColumnFlags_WidthFixed, 115.0f);
                        ImGui::TableSetupColumn("CAN ID", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                        ImGui::TableSetupColumn("CAN Bus", ImGuiTableColumnFlags_WidthFixed, 135.0f);
                        ImGui::TableSetupColumn("Role", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                        ImGui::TableSetupColumn("Invert", ImGuiTableColumnFlags_WidthFixed, 45.0f);
                        ImGui::TableSetupColumn(" ", ImGuiTableColumnFlags_WidthFixed, 25.0f);
                        ImGui::TableHeadersRow();

                        for (size_t c = 0; c < mech.controllers.size(); ++c)
                        {
                            auto& ctrl = mech.controllers[c];
                            ImGui::PushID(static_cast<int>(c));
                            ImGui::TableNextRow();

                            // 1. Vendor
                            ImGui::TableNextColumn();
                            ImGui::SetNextItemWidth(-1.0f);
                            int vendorIdx = 0;
                            for (int v = 0; v < static_cast<int>(IM_ARRAYSIZE(kControllerVendors)); ++v)
                            {
                                if (ctrl.vendor == kControllerVendors[v]) { vendorIdx = v; break; }
                            }
                            if (ImGui::Combo("##Vendor", &vendorIdx, kControllerVendors, IM_ARRAYSIZE(kControllerVendors)))
                            {
                                ctrl.vendor = kControllerVendors[vendorIdx];
                            }

                            // 2. CAN ID
                            ImGui::TableNextColumn();
                            int canIdVal = static_cast<int>(ctrl.canId);
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::InputInt("##CanId", &canIdVal, 0))
                            {
                                ctrl.canId = static_cast<uint32_t>(std::max(0, canIdVal));
                                state.RunLinter();
                            }

                            // 3. CAN Bus
                            ImGui::TableNextColumn();
                            int busIdx = (ctrl.canBus == "canivore") ? 1 : 0;
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::Combo("##Bus", &busIdx, kCanBusLabels, IM_ARRAYSIZE(kCanBusLabels)))
                            {
                                ctrl.canBus = kCanBuses[busIdx];
                                state.RunLinter();
                            }

                            // 4. Role
                            ImGui::TableNextColumn();
                            int roleCIdx = (ctrl.role == "follower") ? 1 : 0;
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::Combo("##Role", &roleCIdx, kControllerRoles, IM_ARRAYSIZE(kControllerRoles)))
                            {
                                ctrl.role = kControllerRoles[roleCIdx];
                            }

                            // 5. Invert
                            ImGui::TableNextColumn();
                            ImGui::Checkbox("##Inv", &ctrl.inverted);
                            if (ImGui::IsItemHovered()) ShowMotorInvertTooltip(ctrl.vendor, ctrl.inverted);

                            // 6. Delete
                            ImGui::TableNextColumn();
                            if (mech.controllers.size() > 1)
                            {
                                if (ImGui::SmallButton("X"))
                                {
                                    ctrlToDelete = static_cast<int>(c);
                                }
                            }

                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }

                    if (ctrlToDelete >= 0 && ctrlToDelete < static_cast<int>(mech.controllers.size()))
                    {
                        mech.controllers.erase(mech.controllers.begin() + ctrlToDelete);
                        state.RunLinter();
                    }

                    if (ImGui::Button("+ Add Follower Controller"))
                    {
                        Robot::Schema::ControllerConfig follower;
                        follower.canId = mech.controllers.empty() ? 10 : (mech.controllers.back().canId + 1);
                        follower.vendor = mech.controllers.empty() ? "CTRE_TalonFX" : mech.controllers[0].vendor;
                        follower.canBus = mech.controllers.empty() ? "rio" : mech.controllers[0].canBus;
                        follower.role = "follower";
                        follower.inverted = false;
                        mech.controllers.push_back(follower);
                        state.RunLinter();
                    }
                }
                else
                {
                    // Servo Preset Dropdown
                    int servoIdx = -1;
                    for (int s = 0; s < static_cast<int>(IM_ARRAYSIZE(kServoIds)) - 1; ++s)
                    {
                        if (mech.servo.preset == kServoIds[s])
                        {
                            servoIdx = s;
                            break;
                        }
                    }
                    if (servoIdx == -1 || mech.servo.preset == "Custom")
                    {
                        servoIdx = static_cast<int>(IM_ARRAYSIZE(kServoIds)) - 1; // Custom
                    }

                    ImGui::SetNextItemWidth(250.0f);
                    if (ImGui::Combo("Servo Preset", &servoIdx, kServoPresets, IM_ARRAYSIZE(kServoPresets)))
                    {
                        mech.servo.preset = kServoIds[servoIdx];
                        state.RunLinter();
                    }

                    // Parametric Custom Servo Editor
                    if (mech.servo.preset == "Custom")
                    {
                        ImGui::Spacing();
                        ImGui::Indent(12.0f);
                        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Custom Servo Specification:");

                        float vNom = static_cast<float>(mech.customServo.nominalVoltageV);
                        if (SliderWithDirectInput("Operating Voltage (V)##MechCustomServo", &vNom, 4.8f, 8.4f, "%.1f"))
                        {
                            mech.customServo.nominalVoltageV = static_cast<double>(vNom);
                        }

                        float stallTorque = static_cast<float>(mech.customServo.stallTorqueKgCm);
                        if (SliderWithDirectInput("Stall Torque (kg*cm)##MechCustomServo", &stallTorque, 1.0f, 80.0f, "%.1f"))
                        {
                            mech.customServo.stallTorqueKgCm = static_cast<double>(stallTorque);
                        }

                        float speedSec = static_cast<float>(mech.customServo.speedSecPer60Deg);
                        if (SliderWithDirectInput("Speed (s/60 deg)##MechCustomServo", &speedSec, 0.030f, 0.400f, "%.3f"))
                        {
                            mech.customServo.speedSecPer60Deg = static_cast<double>(speedSec);
                        }
                        double equivRpm = (mech.customServo.speedSecPer60Deg > 0.001) ? (10.0 / mech.customServo.speedSecPer60Deg) : 0.0;
                        ImGui::TextDisabled("  Equivalent Speed: %.0f RPM", equivRpm);

                        float stallCurr = static_cast<float>(mech.customServo.stallCurrentA);
                        if (SliderWithDirectInput("Stall Current (A)##MechCustomServo", &stallCurr, 0.5f, 10.0f, "%.2f"))
                        {
                            mech.customServo.stallCurrentA = static_cast<double>(stallCurr);
                        }

                        float freeCurr = static_cast<float>(mech.customServo.freeCurrentA);
                        if (SliderWithDirectInput("Free Current (A)##MechCustomServo", &freeCurr, 0.02f, 1.0f, "%.3f"))
                        {
                            mech.customServo.freeCurrentA = static_cast<double>(freeCurr);
                        }

                        float travelRange = static_cast<float>(mech.customServo.travelRangeDeg);
                        if (SliderWithDirectInput("Travel Range (deg)##MechCustomServo", &travelRange, 90.0f, 360.0f, "%.0f"))
                        {
                            mech.customServo.travelRangeDeg = static_cast<double>(travelRange);
                        }

                        ImGui::Unindent(12.0f);
                    }

                    // Servo Control Parameters
                    ImGui::Spacing();
                    ImGui::TextDisabled("Servo PWM Control & Channels:");

                    int pwmCh = static_cast<int>(mech.servo.pwmChannel);
                    ImGui::SetNextItemWidth(45.0f);
                    if (ImGui::InputInt("PWM Channel##Mech", &pwmCh, 0))
                    {
                        mech.servo.pwmChannel = static_cast<uint32_t>(std::clamp(pwmCh, 0, 15));
                        state.RunLinter();
                    }

                    const char* servoModes[] = { "positional", "continuous" };
                    int modeIdx = (mech.servo.mode == "continuous") ? 1 : 0;
                    ImGui::SameLine(0.0f, 10.0f);
                    ImGui::SetNextItemWidth(110.0f);
                    if (ImGui::Combo("Control Mode##Mech", &modeIdx, servoModes, IM_ARRAYSIZE(servoModes)))
                    {
                        mech.servo.mode = servoModes[modeIdx];
                    }

                    float sVolt = static_cast<float>(mech.servo.supplyVoltageV);
                    if (SliderWithDirectInput("Supply Voltage (V)##MechServo", &sVolt, 4.8f, 8.4f, "%.1f"))
                    {
                        mech.servo.supplyVoltageV = static_cast<double>(sVolt);
                    }

                    float sRange = static_cast<float>(mech.servo.travelRangeDeg);
                    if (SliderWithDirectInput("Travel Range (deg)##MechServo", &sRange, 90.0f, 360.0f, "%.0f"))
                    {
                        mech.servo.travelRangeDeg = static_cast<double>(sRange);
                    }
                }
            }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // ---------------------------------------------------------
                // C. Transmission & Mechanical Dynamics
                // ---------------------------------------------------------
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Transmission & Mechanical Dynamics:");

                if (mech.role == "differential")
                {
                    if (mech.gearRatios.size() < 2) mech.gearRatios.resize(2, 50.0);
                    if (mech.coulombFrictions.size() < 2) mech.coulombFrictions.resize(2, 0.0);
                    if (mech.viscousDampings.size() < 2) mech.viscousDampings.resize(2, 0.0);

                    if (mech.differentialType == "custom_matrix")
                    {
                        if (mech.couplingMatrix.size() != 2 || mech.couplingMatrix[0].size() != 2 || mech.couplingMatrix[1].size() != 2)
                        {
                            mech.couplingMatrix = { { 50.0, 50.0 }, { 50.0, -50.0 } };
                        }
                        ImGui::TextDisabled("2x2 Coupling Matrix [omega_motors = M * omega_joints]:");
                        float m00 = static_cast<float>(mech.couplingMatrix[0][0]);
                        float m01 = static_cast<float>(mech.couplingMatrix[0][1]);
                        float m10 = static_cast<float>(mech.couplingMatrix[1][0]);
                        float m11 = static_cast<float>(mech.couplingMatrix[1][1]);
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputFloat("M[0][0]##M00", &m00, 0.0f, 0.0f, "%.2f")) mech.couplingMatrix[0][0] = m00;
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputFloat("M[0][1]##M01", &m01, 0.0f, 0.0f, "%.2f")) mech.couplingMatrix[0][1] = m01;

                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputFloat("M[1][0]##M10", &m10, 0.0f, 0.0f, "%.2f")) mech.couplingMatrix[1][0] = m10;
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(70.0f);
                        if (ImGui::InputFloat("M[1][1]##M11", &m11, 0.0f, 0.0f, "%.2f")) mech.couplingMatrix[1][1] = m11;
                    }
                    else
                    {
                        float gp = static_cast<float>(mech.gearRatios[0]);
                        if (SliderWithDirectInput("Pitch Reduction (Gp:1)##DiffGp", &gp, 1.0f, 250.0f, "%.2f"))
                        {
                            mech.gearRatios[0] = static_cast<double>(gp);
                        }
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pitch reduction Gp:1 (e.g. 25:1 = enter 25.0 for 25x torque. Do NOT enter 1/25).");

                        float gr = static_cast<float>(mech.gearRatios[1]);
                        if (SliderWithDirectInput("Roll Reduction (Gr:1)##DiffGr", &gr, 1.0f, 250.0f, "%.2f"))
                        {
                            mech.gearRatios[1] = static_cast<double>(gr);
                        }
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Roll reduction Gr:1 (e.g. 25:1 = enter 25.0 for 25x torque. Do NOT enter 1/25).");
                    }

                    float eff = static_cast<float>(mech.efficiency);
                    if (SliderWithDirectInput("Mechanical Efficiency##DiffEff", &eff, 0.50f, 1.0f, "%.2f"))
                    {
                        mech.efficiency = static_cast<double>(eff);
                    }

                    float fPitch = static_cast<float>(mech.coulombFrictions[0]);
                    if (SliderWithDirectInput("Pitch Friction (N*m)##DiffFPitch", &fPitch, 0.0f, 5.0f, "%.3f"))
                    {
                        mech.coulombFrictions[0] = static_cast<double>(fPitch);
                    }

                    float fRoll = static_cast<float>(mech.coulombFrictions[1]);
                    if (SliderWithDirectInput("Roll Friction (N*m)##DiffFRoll", &fRoll, 0.0f, 5.0f, "%.3f"))
                    {
                        mech.coulombFrictions[1] = static_cast<double>(fRoll);
                    }

                    float dPitch = static_cast<float>(mech.viscousDampings[0]);
                    if (SliderWithDirectInput("Pitch Damping##DiffDPitch", &dPitch, 0.0f, 2.0f, "%.3f"))
                    {
                        mech.viscousDampings[0] = static_cast<double>(dPitch);
                    }

                    float dRoll = static_cast<float>(mech.viscousDampings[1]);
                    if (SliderWithDirectInput("Roll Damping##DiffDRoll", &dRoll, 0.0f, 2.0f, "%.3f"))
                    {
                        mech.viscousDampings[1] = static_cast<double>(dRoll);
                    }
                }
                else
                {
                    float ratio = static_cast<float>(mech.gearRatio);
                    if (SliderWithDirectInput("Gear Reduction (G:1)##MechRatio", &ratio, 0.1f, 250.0f, "%.2f"))
                    {
                        mech.gearRatio = static_cast<double>(ratio);
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Gear reduction G:1 (e.g. 9:1 = enter 9.0 for 9x motor torque and 1/9th speed. Do NOT enter 1/9). Values > 1 increase torque; values < 1 increase speed.");

                    float eff = static_cast<float>(mech.efficiency);
                    if (SliderWithDirectInput("Mechanical Efficiency##MechEff", &eff, 0.50f, 1.0f, "%.2f"))
                    {
                        mech.efficiency = static_cast<double>(eff);
                    }

                    float friction = static_cast<float>(mech.coulombFriction);
                    if (SliderWithDirectInput("Coulomb Friction##MechFriction", &friction, 0.0f, 5.0f, "%.3f"))
                    {
                        mech.coulombFriction = static_cast<double>(friction);
                    }

                    float damping = static_cast<float>(mech.viscousDamping);
                    if (SliderWithDirectInput("Viscous Damping##MechDamping", &damping, 0.0f, 2.0f, "%.3f"))
                    {
                        mech.viscousDamping = static_cast<double>(damping);
                    }
                }

                // ---------------------------------------------------------
                // D. Role-Specific Parameters
                // ---------------------------------------------------------
                if (mech.role == "differential")
                {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "2-DOF Differential Wrist Kinematics:");
                    ImGui::TextDisabled("Coupled Output Model: omega_motorA = Gp * omega_pitch + Gr * omega_roll");
                    ImGui::TextDisabled("                      omega_motorB = Gp * omega_pitch - Gr * omega_roll");
                }
                else if (mech.role == "elevator")
                {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Elevator Spool & Spring Assist:");

                    float drumR = static_cast<float>(mech.drumRadiusM);
                    if (SliderWithDirectInput("Spool Radius (m)##ElevDrum", &drumR, 0.005f, 0.100f, "%.4f"))
                    {
                        mech.drumRadiusM = static_cast<double>(drumR);
                    }
                    double drumDiamInches = (mech.drumRadiusM * 2.0) / 0.0254;
                    ImGui::TextDisabled("  Effective Diameter: %.3f in (%.1f mm)", drumDiamInches, drumDiamInches * 25.4);

                    float sForce = static_cast<float>(mech.springAssist.forceN);
                    if (SliderWithDirectInput("Spring Assist (N)##ElevSpring", &sForce, 0.0f, 250.0f, "%.1f"))
                    {
                        mech.springAssist.forceN = static_cast<double>(sForce);
                        mech.springAssist.type = (sForce > 0.0) ? "constant_force" : "none";
                    }
                    ImGui::TextDisabled("  Counterbalance Force: %.1f lbf", mech.springAssist.forceN / 4.44822);
                }
                else if (mech.role == "flywheel")
                {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Flywheel Rotor & Dynamic Mount Offset (Fixing Q4):");

                    float inertia = static_cast<float>(mech.flywheelInertiaKgm2);
                    if (SliderWithDirectInput("Inertia (kg*m^2)##FlywheelInertia", &inertia, 0.0001f, 0.0500f, "%.5f"))
                    {
                        mech.flywheelInertiaKgm2 = static_cast<double>(inertia);
                    }

                    // Visual Link Dropdown
                    int visLinkIdx = -1;
                    for (size_t l = 0; l < allLinkNames.size(); ++l)
                    {
                        if (mech.visualLink == allLinkNames[l])
                        {
                            visLinkIdx = static_cast<int>(l);
                            break;
                        }
                    }
                    if (!allLinkNames.empty())
                    {
                        ImGui::SetNextItemWidth(200.0f);
                        if (ImGui::Combo("Visual Mesh Link##Flywheel", &visLinkIdx, allLinkNames.data(), static_cast<int>(allLinkNames.size())))
                        {
                            if (visLinkIdx >= 0 && visLinkIdx < static_cast<int>(allLinkNames.size()))
                            {
                                mech.visualLink = allLinkNames[visLinkIdx];
                            }
                        }
                    }

                    // Spin Axis
                    float axisX = static_cast<float>(mech.spinAxis.x);
                    float axisY = static_cast<float>(mech.spinAxis.y);
                    float axisZ = static_cast<float>(mech.spinAxis.z);
                    ImGui::TextDisabled("Spin Axis (+X=Right, +Y=Up, +Z=Front):");
                    SliderWithDirectInput("Axis X##Spin", &axisX, -1.0f, 1.0f, "%.2f", 75.0f, 45.0f);
                    SliderWithDirectInput("Axis Y##Spin", &axisY, -1.0f, 1.0f, "%.2f", 75.0f, 45.0f);
                    SliderWithDirectInput("Axis Z##Spin", &axisZ, -1.0f, 1.0f, "%.2f", 75.0f, 45.0f);
                    double len = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
                    if (len > 1e-4)
                    {
                        mech.spinAxis = { axisX / len, axisY / len, axisZ / len };
                    }

                    // Dynamic Mount Offset (Addressing Q4!)
                    float offX = static_cast<float>(mech.mountOffsetM.x);
                    float offY = static_cast<float>(mech.mountOffsetM.y);
                    float offZ = static_cast<float>(mech.mountOffsetM.z);
                    ImGui::TextDisabled("Dynamic Mount Offset (+X=Right, +Y=Up, +Z=Front) (m):");
                    if (SliderWithDirectInput("Mount X (Right [+]) (m)##Flywheel", &offX, -0.5f, 0.5f, "%.3f", 75.0f, 50.0f))
                    {
                        mech.mountOffsetM.x = static_cast<double>(offX);
                    }
                    if (SliderWithDirectInput("Mount Y (Up [+]) (m)##Flywheel", &offY, -0.5f, 0.5f, "%.3f", 75.0f, 50.0f))
                    {
                        mech.mountOffsetM.y = static_cast<double>(offY);
                    }
                    if (SliderWithDirectInput("Mount Z (Front [+]) (m)##Flywheel", &offZ, -0.5f, 0.5f, "%.3f", 75.0f, 50.0f))
                    {
                        mech.mountOffsetM.z = static_cast<double>(offZ);
                    }
                }

                ImGui::Spacing();
            }

            ImGui::PopID();
        }

        if (mechToDelete >= 0 && mechToDelete < static_cast<int>(mechanisms.size()))
        {
            mechanisms.erase(mechanisms.begin() + mechToDelete);
            state.RunLinter();
        }
    }
}
