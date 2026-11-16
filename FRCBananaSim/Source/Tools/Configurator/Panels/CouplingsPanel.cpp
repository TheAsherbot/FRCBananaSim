/**
 * @file CouplingsPanel.cpp
 * @brief Implementation of the Couplings & Kinematic Linkages inspector panel.
 */

#include "Tools/Configurator/Panels/CouplingsPanel.h"
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

    static const char* kCouplingTypes[] = {
        "power_coupling", "elevator_cascade", "four_bar_loop"
    };

    static const char* kCouplingTypeLabels[] = {
        "Power Coupling (Gear/Belt/Chain)",
        "Elevator Cascade (Multi-Stage)",
        "Four-Bar Linkage Loop"
    };

    static const char* kTransmissionTypes[] = {
        "gear", "belt", "chain", "cable", "virtual_fourbar"
    };

    static const char* kSolverModes[] = {
        "analytic_coupled", "jolt_constraint"
    };

    void CouplingsPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "MECHANICAL COUPLINGS & LINKAGES");
        ImGui::Separator();

        if (!state.isSidecarLoaded && !state.isCadLoaded)
        {
            ImGui::TextWrapped("No robot loaded. Open a CAD folder or load a reference robot to configure couplings.");
            return;
        }

        auto& couplings = state.sidecar.couplings;

        // -------------------------------------------------------------
        // 1. Add New Coupling Section
        // -------------------------------------------------------------
        if (ImGui::CollapsingHeader("Add New Coupling", ImGuiTreeNodeFlags_None))
        {
            ImGui::TextDisabled("Coupling Name:");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputText("##NewCoupName", newCouplingNameBuf, sizeof(newCouplingNameBuf));

            ImGui::TextDisabled("Coupling Type:");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##NewCoupType", &newCouplingTypeIdx, kCouplingTypeLabels, IM_ARRAYSIZE(kCouplingTypeLabels));

            ImGui::Spacing();
            if (ImGui::Button("+ Create Coupling", ImVec2(-1.0f, 0.0f)))
            {
                Robot::Schema::CouplingConfig newCoup;
                newCoup.name = newCouplingNameBuf;
                newCoup.type = kCouplingTypes[newCouplingTypeIdx];
                newCoup.solverMode = "analytic_coupled";

                if (newCoup.type == "power_coupling")
                {
                    if (state.manifest.joints.size() >= 2)
                    {
                        newCoup.powerCoupling.masterJoint = state.manifest.joints[0].name;
                        newCoup.powerCoupling.slaveJoint = state.manifest.joints[1].name;
                    }
                    else if (!state.manifest.joints.empty())
                    {
                        newCoup.powerCoupling.masterJoint = state.manifest.joints[0].name;
                    }
                    newCoup.powerCoupling.transmissionType = "gear";
                    newCoup.powerCoupling.ratio = 1.0;
                    newCoup.powerCoupling.efficiency = 0.95;
                    newCoup.powerCoupling.inverted = false;
                    newCoup.powerCoupling.backlashRad = 0.0;
                }
                else if (newCoup.type == "elevator_cascade")
                {
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "prismatic")
                        {
                            Robot::Schema::StageCouplingConfig s;
                            s.joint = j.name;
                            s.multiplier = 1.0;
                            newCoup.cascadeStages.push_back(s);
                            break;
                        }
                    }
                }
                else if (newCoup.type == "four_bar_loop")
                {
                    std::vector<std::string> revJoints;
                    for (const auto& j : state.manifest.joints)
                    {
                        if (j.type == "revolute" || j.type == "continuous")
                        {
                            revJoints.push_back(j.name);
                        }
                    }
                    if (revJoints.size() >= 3)
                    {
                        newCoup.fourBar.crankJoint = revJoints[0];
                        newCoup.fourBar.couplerJoint = revJoints[1];
                        newCoup.fourBar.followerJoint = revJoints[2];
                    }
                    if (!state.manifest.links.empty())
                    {
                        newCoup.fourBar.closureParentLink = state.manifest.links[0].name;
                        if (state.manifest.links.size() >= 2)
                        {
                            newCoup.fourBar.closureChildLink = state.manifest.links[1].name;
                        }
                    }
                }

                couplings.push_back(newCoup);
                std::snprintf(newCouplingNameBuf, sizeof(newCouplingNameBuf), "Coupling%zu", couplings.size() + 1);
                state.RunLinter();
            }
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Configured Couplings & Linkages (%zu):", couplings.size());

        if (couplings.empty())
        {
            ImGui::TextWrapped("No mechanical couplings configured. Add a power coupling, elevator cascade, or four-bar loop closure above.");
            return;
        }

        // Cache all CAD joint and link names
        std::vector<const char*> allJointNames;
        std::vector<const char*> prismaticJointNames;
        std::vector<const char*> revoluteJointNames;
        for (const auto& j : state.manifest.joints)
        {
            allJointNames.push_back(j.name.c_str());
            if (j.type == "prismatic")
            {
                prismaticJointNames.push_back(j.name.c_str());
            }
            if (j.type == "revolute" || j.type == "continuous")
            {
                revoluteJointNames.push_back(j.name.c_str());
            }
        }

        std::vector<const char*> allLinkNames;
        for (const auto& l : state.manifest.links)
        {
            allLinkNames.push_back(l.name.c_str());
        }

        // -------------------------------------------------------------
        // 2. Couplings Cards List
        // -------------------------------------------------------------
        int coupToDelete = -1;

        for (size_t i = 0; i < couplings.size(); ++i)
        {
            auto& coup = couplings[i];
            ImGui::PushID(static_cast<int>(i));

            std::string headerLabel = "[" + coup.type + "] " + coup.name + "##CardHeader";
            bool isCardOpen = ImGui::CollapsingHeader(headerLabel.c_str(), ImGuiTreeNodeFlags_DefaultOpen);
            bool isFocused = ImGui::IsItemActive() || ImGui::IsItemClicked() || ImGui::IsItemHovered();
            if (isCardOpen && isFocused)
            {
                if (coup.type == "power_coupling")
                {
                    state.selectedJoint = coup.powerCoupling.masterJoint;
                }
                else if (coup.type == "four_bar_loop")
                {
                    state.selectedJoint = coup.fourBar.crankJoint;
                }
            }

            if (isCardOpen)
            {
                float availW = ImGui::GetContentRegionAvail().x;
                float delBtnW = 115.0f;

                // Top Action Row
                ImGui::TextDisabled("Coupling Nickname:");
                char nameBuf[64];
                std::snprintf(nameBuf, sizeof(nameBuf), "%s", coup.name.c_str());
                ImGui::SetNextItemWidth(std::max(80.0f, availW - delBtnW - 8.0f));
                if (ImGui::InputText("##CouplingName", nameBuf, sizeof(nameBuf)))
                {
                    coup.name = nameBuf;
                    state.RunLinter();
                }

                ImGui::SameLine(0.0f, 8.0f);
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
                if (ImGui::Button("Delete Coupling", ImVec2(delBtnW, 0.0f)))
                {
                    coupToDelete = static_cast<int>(i);
                }
                ImGui::PopStyleColor(2);

                // Type selector & Solver Mode (stacked full-width to prevent horizontal clipping)
                ImGui::TextDisabled("Coupling Type:");
                int typeIdx = 0;
                for (int t = 0; t < static_cast<int>(IM_ARRAYSIZE(kCouplingTypes)); ++t)
                {
                    if (coup.type == kCouplingTypes[t])
                    {
                        typeIdx = t;
                        break;
                    }
                }
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##CouplingType", &typeIdx, kCouplingTypes, IM_ARRAYSIZE(kCouplingTypes)))
                {
                    coup.type = kCouplingTypes[typeIdx];
                    state.RunLinter();
                }

                ImGui::TextDisabled("Solver Kinematics Mode:");
                int solverIdx = (coup.solverMode == "jolt_constraint") ? 1 : 0;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##SolverMode", &solverIdx, kSolverModes, IM_ARRAYSIZE(kSolverModes)))
                {
                    coup.solverMode = kSolverModes[solverIdx];
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // -----------------------------------------------------
                // A. Power Coupling Form
                // -----------------------------------------------------
                if (coup.type == "power_coupling")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Power Transmission Kinematics:");

                    // Master Joint Dropdown
                    int masterIdx = -1;
                    for (size_t j = 0; j < allJointNames.size(); ++j)
                    {
                        if (coup.powerCoupling.masterJoint == allJointNames[j])
                        {
                            masterIdx = static_cast<int>(j);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Master Joint (Driver):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##MasterJoint", &masterIdx, allJointNames.data(), static_cast<int>(allJointNames.size())))
                    {
                        coup.powerCoupling.masterJoint = allJointNames[masterIdx];
                        state.selectedJoint = coup.powerCoupling.masterJoint;
                        state.RunLinter();
                    }

                    // Slave Joint Dropdown
                    int slaveIdx = -1;
                    for (size_t j = 0; j < allJointNames.size(); ++j)
                    {
                        if (coup.powerCoupling.slaveJoint == allJointNames[j])
                        {
                            slaveIdx = static_cast<int>(j);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Slave Joint (Driven):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##SlaveJoint", &slaveIdx, allJointNames.data(), static_cast<int>(allJointNames.size())))
                    {
                        coup.powerCoupling.slaveJoint = allJointNames[slaveIdx];
                        state.RunLinter();
                    }

                    // Transmission Type
                    int transIdx = 0;
                    for (int t = 0; t < static_cast<int>(IM_ARRAYSIZE(kTransmissionTypes)); ++t)
                    {
                        if (coup.powerCoupling.transmissionType == kTransmissionTypes[t])
                        {
                            transIdx = t;
                            break;
                        }
                    }
                    ImGui::TextDisabled("Transmission Type:");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##TransType", &transIdx, kTransmissionTypes, IM_ARRAYSIZE(kTransmissionTypes)))
                    {
                        coup.powerCoupling.transmissionType = kTransmissionTypes[transIdx];
                    }

                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Ratio & Physical Properties:");

                    float ratioVal = static_cast<float>(coup.powerCoupling.ratio);
                    if (SliderWithDirectInput("Velocity Ratio (slave / master)##CoupRatio", &ratioVal, -20.0f, 20.0f, "%.3f", 85.0f))
                    {
                        coup.powerCoupling.ratio = static_cast<double>(ratioVal);
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Kinematic velocity ratio: omega_slave = ratio * omega_master.\nE.g. 1.0 = 1:1, 0.5 = 2:1 reduction, 2.0 = 1:2 overdrive.\nTip: Double-click, Ctrl+Click, or Right-click to type directly (e.g. 1/9, 0.111)");
                    }

                    ImGui::Checkbox("Invert Slave Direction##CoupInv", &coup.powerCoupling.inverted);
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Inverts rotation direction of slave joint relative to master joint (e.g. reverse spur gear mesh vs belt/chain)");
                    }

                    float effVal = static_cast<float>(coup.powerCoupling.efficiency);
                    if (SliderWithDirectInput("Transmission Efficiency##CoupEff", &effVal, 0.50f, 1.00f, "%.2f", 85.0f))
                    {
                        coup.powerCoupling.efficiency = static_cast<double>(effVal);
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Mechanical transmission torque/power efficiency (0.50 to 1.00).\nNOTE: Does NOT reduce kinematic speed! It dictates torque and friction loss (P_out = eff * P_in).\nLower efficiency means the master motor experiences higher reaction torque.");
                    }

                    float backlashVal = static_cast<float>(coup.powerCoupling.backlashRad);
                    if (SliderWithDirectInput("Backlash / Deadband (rad)##CoupBacklash", &backlashVal, 0.0f, 0.10f, "%.4f", 85.0f))
                    {
                        coup.powerCoupling.backlashRad = static_cast<double>(backlashVal);
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Mechanical backlash / deadband window in radians.\nTorque is only transmitted once displacement exceeds half-backlash.");
                    }
                }
                // -----------------------------------------------------
                // B. Elevator Cascade Form
                // -----------------------------------------------------
                else if (coup.type == "elevator_cascade")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Cascade Linear Stage Multipliers (x_k = multiplier * x_1):");
                    ImGui::Spacing();

                    if (ImGui::Button("+ Add Cascade Stage", ImVec2(-1.0f, 0.0f)))
                    {
                        Robot::Schema::StageCouplingConfig newStage;
                        if (!prismaticJointNames.empty())
                        {
                            newStage.joint = prismaticJointNames[0];
                        }
                        else if (!allJointNames.empty())
                        {
                            newStage.joint = allJointNames[0];
                        }
                        newStage.multiplier = static_cast<double>(coup.cascadeStages.size() + 1);
                        coup.cascadeStages.push_back(newStage);
                        state.RunLinter();
                    }

                    int stageToDelete = -1;
                    if (ImGui::BeginTable("CascadeStagesTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                    {
                        ImGui::TableSetupColumn("Stage", ImGuiTableColumnFlags_WidthFixed, 42.0f);
                        ImGui::TableSetupColumn("Target Joint (Prismatic)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("Multiplier (k)", ImGuiTableColumnFlags_WidthFixed, 85.0f);
                        ImGui::TableSetupColumn("Actions", ImGuiTableColumnFlags_WidthFixed, 35.0f);
                        ImGui::TableHeadersRow();

                        for (size_t s = 0; s < coup.cascadeStages.size(); ++s)
                        {
                            auto& stage = coup.cascadeStages[s];
                            ImGui::TableNextRow();

                            // Col 0: Index
                            ImGui::TableSetColumnIndex(0);
                            ImGui::Text("  #%zu", s + 1);

                            // Col 1: Joint dropdown
                            ImGui::TableSetColumnIndex(1);
                            ImGui::PushID(static_cast<int>(s));
                            int stageJointIdx = -1;
                            const auto& jointList = prismaticJointNames.empty() ? allJointNames : prismaticJointNames;
                            for (size_t j = 0; j < jointList.size(); ++j)
                            {
                                if (stage.joint == jointList[j])
                                {
                                    stageJointIdx = static_cast<int>(j);
                                    break;
                                }
                            }
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::Combo("##StageJoint", &stageJointIdx, jointList.data(), static_cast<int>(jointList.size())))
                            {
                                stage.joint = jointList[stageJointIdx];
                                state.selectedJoint = stage.joint;
                                state.RunLinter();
                            }

                            // Col 2: Multiplier
                            ImGui::TableSetColumnIndex(2);
                            float multVal = static_cast<float>(stage.multiplier);
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::InputFloat("##StageMult", &multVal, 0.5f, 1.0f, "%.2f"))
                            {
                                stage.multiplier = static_cast<double>(multVal);
                            }

                            // Col 3: Delete
                            ImGui::TableSetColumnIndex(3);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                            if (ImGui::Button("X##DelStage", ImVec2(-1.0f, 0.0f)))
                            {
                                stageToDelete = static_cast<int>(s);
                            }
                            ImGui::PopStyleColor();
                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }

                    if (stageToDelete >= 0 && stageToDelete < static_cast<int>(coup.cascadeStages.size()))
                    {
                        coup.cascadeStages.erase(coup.cascadeStages.begin() + stageToDelete);
                        state.RunLinter();
                    }
                }
                // -----------------------------------------------------
                // C. Four-Bar Linkage Form
                // -----------------------------------------------------
                else if (coup.type == "four_bar_loop")
                {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Closed Kinematic Loop Joint Chain:");

                    const auto& revList = revoluteJointNames.empty() ? allJointNames : revoluteJointNames;

                    // Crank Joint
                    int crankIdx = -1;
                    for (size_t j = 0; j < revList.size(); ++j)
                    {
                        if (coup.fourBar.crankJoint == revList[j])
                        {
                            crankIdx = static_cast<int>(j);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Crank Joint (Driven):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##CrankJoint", &crankIdx, revList.data(), static_cast<int>(revList.size())))
                    {
                        coup.fourBar.crankJoint = revList[crankIdx];
                        state.selectedJoint = coup.fourBar.crankJoint;
                        state.RunLinter();
                    }

                    // Coupler Joint
                    int couplerIdx = -1;
                    for (size_t j = 0; j < revList.size(); ++j)
                    {
                        if (coup.fourBar.couplerJoint == revList[j])
                        {
                            couplerIdx = static_cast<int>(j);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Coupler Joint (Mid):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##CouplerJoint", &couplerIdx, revList.data(), static_cast<int>(revList.size())))
                    {
                        coup.fourBar.couplerJoint = revList[couplerIdx];
                        state.RunLinter();
                    }

                    // Follower Joint
                    int followerIdx = -1;
                    for (size_t j = 0; j < revList.size(); ++j)
                    {
                        if (coup.fourBar.followerJoint == revList[j])
                        {
                            followerIdx = static_cast<int>(j);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Follower Joint (Output):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##FollowerJoint", &followerIdx, revList.data(), static_cast<int>(revList.size())))
                    {
                        coup.fourBar.followerJoint = revList[followerIdx];
                        state.RunLinter();
                    }

                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Loop Closure Constraints & Anchors:");

                    // Closure Parent Link
                    int pLinkIdx = -1;
                    for (size_t l = 0; l < allLinkNames.size(); ++l)
                    {
                        if (coup.fourBar.closureParentLink == allLinkNames[l])
                        {
                            pLinkIdx = static_cast<int>(l);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Closure Parent Link:");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##ClosurePLink", &pLinkIdx, allLinkNames.data(), static_cast<int>(allLinkNames.size())))
                    {
                        coup.fourBar.closureParentLink = allLinkNames[pLinkIdx];
                        state.RunLinter();
                    }

                    // Closure Child Link
                    int cLinkIdx = -1;
                    for (size_t l = 0; l < allLinkNames.size(); ++l)
                    {
                        if (coup.fourBar.closureChildLink == allLinkNames[l])
                        {
                            cLinkIdx = static_cast<int>(l);
                            break;
                        }
                    }
                    ImGui::TextDisabled("Closure Child Link:");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##ClosureCLink", &cLinkIdx, allLinkNames.data(), static_cast<int>(allLinkNames.size())))
                    {
                        coup.fourBar.closureChildLink = allLinkNames[cLinkIdx];
                        state.RunLinter();
                    }

                    // Parent Anchor
                    float pAnchor[3] = {
                        static_cast<float>(coup.fourBar.closureParentAnchorM.x),
                        static_cast<float>(coup.fourBar.closureParentAnchorM.y),
                        static_cast<float>(coup.fourBar.closureParentAnchorM.z)
                    };
                    ImGui::TextDisabled("Parent Anchor [X=Right, Y=Up, Z=Front] (m):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputFloat3("##pAnchor", pAnchor, "%.3f"))
                    {
                        coup.fourBar.closureParentAnchorM = glm::dvec3(pAnchor[0], pAnchor[1], pAnchor[2]);
                    }

                    // Child Anchor
                    float cAnchor[3] = {
                        static_cast<float>(coup.fourBar.closureChildAnchorM.x),
                        static_cast<float>(coup.fourBar.closureChildAnchorM.y),
                        static_cast<float>(coup.fourBar.closureChildAnchorM.z)
                    };
                    ImGui::TextDisabled("Child Anchor [X=Right, Y=Up, Z=Front] (m):");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputFloat3("##cAnchor", cAnchor, "%.3f"))
                    {
                        coup.fourBar.closureChildAnchorM = glm::dvec3(cAnchor[0], cAnchor[1], cAnchor[2]);
                    }

                    // Closure Axis
                    float cAxis[3] = {
                        static_cast<float>(coup.fourBar.closureAxis.x),
                        static_cast<float>(coup.fourBar.closureAxis.y),
                        static_cast<float>(coup.fourBar.closureAxis.z)
                    };
                    ImGui::TextDisabled("Closure Axis [X=Right, Y=Up, Z=Front]:");
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputFloat3("##cAxis", cAxis, "%.2f"))
                    {
                        coup.fourBar.closureAxis = glm::dvec3(cAxis[0], cAxis[1], cAxis[2]);
                    }
                }
            }

            ImGui::PopID();
            ImGui::Spacing();
        }

        if (coupToDelete >= 0 && coupToDelete < static_cast<int>(couplings.size()))
        {
            couplings.erase(couplings.begin() + coupToDelete);
            state.RunLinter();
        }
    }
}
