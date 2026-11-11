/**
 * @file CADHierarchyPanel.cpp
 * @brief Implementation of the CAD hierarchy tree view panel.
 */

#include "Tools/Configurator/Panels/CADHierarchyPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>
#include <string>
#include <algorithm>

namespace FRCBananaSim::Tools::Configurator
{
    void CADHierarchyPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextDisabled("CAD HIERARCHY");
        ImGui::Separator();

        // 1. Search Filter
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputTextWithHint("##HierarchySearch", "Search links/joints...", searchBuffer, sizeof(searchBuffer));
        std::string filter(searchBuffer);
        std::transform(filter.begin(), filter.end(), filter.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (ImGui::Button("Auto-Generate Starter Sidecar", ImVec2(-1.0f, 26.0f)))
        {
            state.AutoGenerateStarterSidecar();
        }

        ImGui::Spacing();

        if (!state.isCadLoaded)
        {
            ImGui::TextWrapped("No CAD manifest loaded.\nUse 'Open Folder' in the toolbar or load Reference2026 to view links and joints.");
            return;
        }

        // 2. Electromechanical Subsystems Section
        if (ImGui::CollapsingHeader("Subsystems & Calibrations", ImGuiTreeNodeFlags_DefaultOpen))
        {
            bool isSwerveSelected = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::SwerveDrivetrain);
            if (ImGui::Selectable("Swerve Drivetrain (4 Modules)", isSwerveSelected))
            {
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::SwerveDrivetrain;
                state.selectedLink.clear();
                state.selectedJoint.clear();
            }

            bool isMechSelected = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::Mechanisms);
            std::string mechLabel = "Mechanisms (" + std::to_string(state.sidecar.mechanisms.size()) + " Defined)";
            if (ImGui::Selectable(mechLabel.c_str(), isMechSelected))
            {
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::Mechanisms;
                state.selectedLink.clear();
                state.selectedJoint.clear();
            }

            bool isCouplingSelected = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::Couplings);
            std::string coupLabel = "Couplings & Linkages (" + std::to_string(state.sidecar.couplings.size()) + " Defined)";
            if (ImGui::Selectable(coupLabel.c_str(), isCouplingSelected))
            {
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::Couplings;
                state.selectedLink.clear();
                state.selectedJoint.clear();
            }

            bool isSensorSelected = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::Sensors);
            std::string sensorLabel = "Sensor Suite (" + std::to_string(state.sidecar.sensors.size()) + " Defined)";
            if (ImGui::Selectable(sensorLabel.c_str(), isSensorSelected))
            {
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::Sensors;
                state.selectedLink.clear();
                state.selectedJoint.clear();
            }

            bool isMassSelected = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::MassReconciliation);
            if (ImGui::Selectable("Mass Reconciliation (Bench Scale)", isMassSelected))
            {
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::MassReconciliation;
                state.selectedLink.clear();
                state.selectedJoint.clear();
            }
        }

        // 3. Links Section
        if (ImGui::CollapsingHeader("Links & Bodies", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (const auto& link : state.manifest.links)
            {
                std::string lNameLower = link.name;
                std::transform(lNameLower.begin(), lNameLower.end(), lNameLower.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

                if (!filter.empty() && lNameLower.find(filter) == std::string::npos)
                {
                    continue;
                }

                bool isSelected = (state.selectedLink == link.name &&
                    state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::CADSelection);
                std::string label = link.name;
                if (link.isRoot)
                {
                    label += " [ROOT]";
                }

                if (ImGui::Selectable(label.c_str(), isSelected))
                {
                    state.selectedLink = link.name;
                    state.selectedJoint.clear();
                    state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::CADSelection;
                }

                ImGui::Indent(14.0f);
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Mass: %.2f kg | Colliders: %zu",
                    link.massKg, link.colliders.size());
                ImGui::Unindent(14.0f);
            }
        }

        // 3. Kinematic Joints Section
        if (ImGui::CollapsingHeader("Kinematic Joints", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (const auto& joint : state.manifest.joints)
            {
                std::string jNameLower = joint.name;
                std::transform(jNameLower.begin(), jNameLower.end(), jNameLower.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

                if (!filter.empty() && jNameLower.find(filter) == std::string::npos)
                {
                    continue;
                }

                bool isSelected = (state.selectedJoint == joint.name &&
                    state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::CADSelection);
                std::string label = joint.name + " (" + joint.type + ")";

                if (ImGui::Selectable(label.c_str(), isSelected))
                {
                    state.selectedJoint = joint.name;
                    state.selectedLink.clear();
                    state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::CADSelection;
                }

                ImGui::Indent(14.0f);
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s -> %s",
                    joint.parentLink.c_str(), joint.childLink.c_str());
                if (joint.limits.hasLimits)
                {
                    ImGui::TextColored(ImVec4(0.5f, 0.8f, 0.5f, 1.0f), "[%.2f, %.2f] %s",
                        joint.limits.lower, joint.limits.upper, joint.limits.unit.c_str());
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.4f, 1.0f), "[Continuous / Unbounded]");
                }
                ImGui::Unindent(14.0f);
            }
        }

        // 4. Contact Zones Section
        if (!state.manifest.contactZones.empty() && ImGui::CollapsingHeader("Contact Zones"))
        {
            for (const auto& zone : state.manifest.contactZones)
            {
                ImGui::BulletText("%s (%s on %s)",
                    zone.name.c_str(), zone.role.c_str(), zone.parentLink.c_str());
            }
        }
    }
}
