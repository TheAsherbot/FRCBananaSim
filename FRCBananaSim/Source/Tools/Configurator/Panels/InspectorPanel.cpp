/**
 * @file InspectorPanel.cpp
 * @brief Implementation of the property inspector panel with subsystem tabs.
 */

#include "Tools/Configurator/Panels/InspectorPanel.h"
#include "Tools/Configurator/Panels/SwervePanel.h"
#include "Tools/Configurator/Panels/MechanismPanel.h"
#include "Tools/Configurator/Panels/CouplingsPanel.h"
#include "Tools/Configurator/Panels/SensorPanel.h"
#include "Tools/Configurator/Panels/MassReconciliationPanel.h"
#include "Tools/Configurator/Panels/BumperPanel.h"
#include "Tools/Configurator/Panels/ContactZonePanel.h"
#include "Tools/Configurator/ConfiguratorState.h"
#include "Tools/Configurator/Panels/ConfiguratorWidgets.h"
#include "Robot/Schema/RobotSchemaLoader.h"

#include <imgui/imgui.h>

namespace FRCBananaSim::Tools::Configurator
{
    InspectorPanel::InspectorPanel()
        : swervePanel(std::make_unique<SwervePanel>()),
          mechanismPanel(std::make_unique<MechanismPanel>()),
          couplingsPanel(std::make_unique<CouplingsPanel>()),
          sensorPanel(std::make_unique<SensorPanel>()),
          massReconciliationPanel(std::make_unique<MassReconciliationPanel>()),
          bumperPanel(std::make_unique<BumperPanel>()),
          contactZonePanel(std::make_unique<ContactZonePanel>())
    {
    }

    InspectorPanel::~InspectorPanel() = default;

    void InspectorPanel::Render(ConfiguratorState& state)
    {
        ImGui::PushTextWrapPos(0.0f);

        ImGui::TextDisabled("SUBSYSTEM & PROPERTY INSPECTOR");
        ImGui::Separator();

        // 2-Line Subsystem Navigation Bar
        float availW = ImGui::GetContentRegionAvail().x;
        float spacing = ImGui::GetStyle().ItemSpacing.x;
        float btnW4 = std::max(60.0f, (availW - 3.0f * spacing) / 4.0f);
        float btnW3 = std::max(70.0f, (availW - 2.0f * spacing) / 3.0f);
        float btnH = 26.0f;

        auto renderNavButton = [&](const char* label, ConfiguratorState::ActiveInspectorTab tab, float width)
        {
            bool isActive = (state.activeInspectorTab == tab);
            if (isActive)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.44f, 0.75f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.52f, 0.88f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.15f, 0.38f, 0.65f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.18f, 0.22f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.28f, 0.34f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.18f, 0.44f, 0.75f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.75f, 0.78f, 0.82f, 1.0f));
            }

            if (ImGui::Button(label, ImVec2(width, btnH)))
            {
                state.activeInspectorTab = tab;
            }

            ImGui::PopStyleColor(4);
        };

        // Line 1: CAD Selection | Swerve Drive | Mechanisms | Bumpers
        renderNavButton("CAD Selection", ConfiguratorState::ActiveInspectorTab::CADSelection, btnW4);
        ImGui::SameLine();
        renderNavButton("Swerve Drive", ConfiguratorState::ActiveInspectorTab::SwerveDrivetrain, btnW4);
        ImGui::SameLine();
        renderNavButton("Mechanisms", ConfiguratorState::ActiveInspectorTab::Mechanisms, btnW4);
        ImGui::SameLine();
        renderNavButton("Bumpers", ConfiguratorState::ActiveInspectorTab::Bumpers, btnW4);

        // Line 2: Couplings | Sensors | Contact Zones | Mass & Scale
        renderNavButton("Couplings", ConfiguratorState::ActiveInspectorTab::Couplings, btnW4);
        ImGui::SameLine();
        renderNavButton("Sensors", ConfiguratorState::ActiveInspectorTab::Sensors, btnW4);
        ImGui::SameLine();
        renderNavButton("Contact Zones", ConfiguratorState::ActiveInspectorTab::ContactZones, btnW4);
        ImGui::SameLine();
        renderNavButton("Mass & Scale", ConfiguratorState::ActiveInspectorTab::MassReconciliation, btnW4);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Render Active Panel
        switch (state.activeInspectorTab)
        {
        case ConfiguratorState::ActiveInspectorTab::CADSelection:
            RenderCADSelection(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::SwerveDrivetrain:
            swervePanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::Mechanisms:
            mechanismPanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::Bumpers:
            bumperPanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::Couplings:
            couplingsPanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::Sensors:
            sensorPanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::ContactZones:
            contactZonePanel->Render(state);
            break;
        case ConfiguratorState::ActiveInspectorTab::MassReconciliation:
            massReconciliationPanel->Render(state);
            break;
        }

        ImGui::PopTextWrapPos();
    }

    void InspectorPanel::RenderCADSelection(ConfiguratorState& state)
    {
        if (!state.isCadLoaded)
        {
            ImGui::TextWrapped("No CAD manifest loaded. Select an item in the CAD hierarchy to inspect.");
            return;
        }

        // 1. Inspect Selected Link
        if (!state.selectedLink.empty())
        {
            const auto* link = state.manifest.FindLink(state.selectedLink);
            if (link)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Link: %s", link->name.c_str());
                if (link->isRoot)
                {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[Root Link / Chassis]");
                }
                ImGui::Separator();

                ImGui::Text("Material:     %s", link->material.c_str());

                auto oIt = state.sidecar.massReconciliation.linkMassOverrides.find(link->name);
                if (state.sidecar.massReconciliation.enabled && oIt != state.sidecar.massReconciliation.linkMassOverrides.end() && oIt->second > 0.0)
                {
                    ImGui::Text("Mass (CAD):   %.3f kg", link->massKg);
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Mass (Active): %.3f kg [Individual Override]", oIt->second);
                }
                else if (state.sidecar.massReconciliation.enabled && state.sidecar.massReconciliation.linkMassMultiplier != 1.0)
                {
                    ImGui::Text("Mass (CAD):   %.3f kg", link->massKg);
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Mass (Scaled): %.3f kg [x%.3f Multiplier]",
                        link->massKg * state.sidecar.massReconciliation.linkMassMultiplier,
                        state.sidecar.massReconciliation.linkMassMultiplier);
                }
                else
                {
                    ImGui::Text("Mass:         %.3f kg", link->massKg);
                }

                glm::vec3 comRobot = Robot::Schema::RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(link->centerOfMassM));
                ImGui::Text("Center of Mass (+X Right, +Y Up, +Z Front):");
                ImGui::Text("  [X=%.3f, Y=%.3f, Z=%.3f] m",
                    comRobot.x, comRobot.y, comRobot.z);

                ImGui::Spacing();
                ImGui::Text("Rotational Inertia (kg*m^2):");
                ImGui::Text("  Ixx=%.4f  Iyy=%.4f  Izz=%.4f",
                    link->inertiaTensor.ixx, link->inertiaTensor.iyy, link->inertiaTensor.izz);
                ImGui::Text("  Ixy=%.4f  Ixz=%.4f  Iyz=%.4f",
                    link->inertiaTensor.ixy, link->inertiaTensor.ixz, link->inertiaTensor.iyz);

                ImGui::Spacing();
                ImGui::Text("Visual Mesh:   %s", link->visualMesh.empty() ? "(None)" : link->visualMesh.c_str());
                ImGui::Text("Colliders:    %zu defined", link->colliders.size());

                for (size_t i = 0; i < link->colliders.size(); ++i)
                {
                    const auto& col = link->colliders[i];
                    if (ImGui::TreeNode((void*)(intptr_t)i, "Collider %zu: %s (%s)", i, col.name.c_str(), col.type.c_str()))
                    {
                        glm::vec3 colOrigRobot = Robot::Schema::RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(col.originM));
                        glm::vec3 colDimsRobot = glm::abs(Robot::Schema::RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(col.dimensionsM)));
                        ImGui::Text("Origin (+X Right, +Y Up, +Z Front):     [%.3f, %.3f, %.3f] m", colOrigRobot.x, colOrigRobot.y, colOrigRobot.z);
                        ImGui::Text("Dimensions [X(Width), Y(Height), Z(Length)]: [%.3f, %.3f, %.3f] m",
                            colDimsRobot.x, colDimsRobot.y, colDimsRobot.z);
                        ImGui::TreePop();
                    }
                }
                return;
            }
        }

        // 2. Inspect Selected Joint
        if (!state.selectedJoint.empty())
        {
            const auto* joint = state.manifest.FindJoint(state.selectedJoint);
            if (joint)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Joint: %s", joint->name.c_str());
                ImGui::Text("Type:        %s", joint->type.c_str());
                ImGui::Separator();

                ImGui::Text("Parent Link: %s", joint->parentLink.c_str());
                ImGui::Text("Child Link:  %s", joint->childLink.c_str());

                glm::vec3 jOrigRobot = Robot::Schema::RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(joint->originM));
                glm::vec3 jAxisRobot = Robot::Schema::RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(joint->axis));

                ImGui::Spacing();
                ImGui::Text("Anchor Origin (+X Right, +Y Up, +Z Front):");
                ImGui::Text("  [X=%.3f, Y=%.3f, Z=%.3f] m", jOrigRobot.x, jOrigRobot.y, jOrigRobot.z);
                ImGui::Text("Motion Axis (+X Right, +Y Up, +Z Front):");
                ImGui::Text("  [X=%.3f, Y=%.3f, Z=%.3f]", jAxisRobot.x, jAxisRobot.y, jAxisRobot.z);

                ImGui::Spacing();
                RenderJointLimitEditor(state, joint->name, "InspectorJointLimit");
                return;
            }
        }

        // 3. Fallback when nothing selected
        ImGui::TextWrapped("Click any Link or Joint in the CAD Hierarchy tree to inspect physical and kinematic properties, or switch tabs above to configure Swerve or Mass Reconciliation.");
    }
}
