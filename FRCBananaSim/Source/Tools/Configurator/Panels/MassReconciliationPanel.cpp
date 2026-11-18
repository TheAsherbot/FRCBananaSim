/**
 * @file MassReconciliationPanel.cpp
 * @brief Implementation of the physical bench mass reconciliation and ballast panel.
 */

#include "Tools/Configurator/Panels/MassReconciliationPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>
#include "ConfiguratorWidgets.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Tools::Configurator
{

    void MassReconciliationPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "PHYSICAL BENCH MASS RECONCILIATION");
        ImGui::Separator();

        if (!state.isSidecarLoaded && !state.isCadLoaded)
        {
            ImGui::TextWrapped("No robot loaded. Open a CAD folder or load a reference robot to configure mass reconciliation.");
            return;
        }

        auto& mr = state.sidecar.massReconciliation;

        // 1. Enable / Disable Master Toggle
        if (ImGui::Checkbox("Enable Mass Reconciliation (Written to sidecar.json)", &mr.enabled))
        {
            state.RunLinter();
        }

        if (!mr.enabled)
        {
            ImGui::Spacing();
            ImGui::TextDisabled("Status: Disabled (Simulation uses uncalibrated raw CAD masses).");
            ImGui::TextWrapped("Check the box above to reconcile CAD mass with scale bench measurements.");
            return;
        }

        ImGui::Spacing();

        // 2. CAD Total Mass vs Bench Measurement
        double totalCadMass = 0.0;
        for (const auto& link : state.manifest.links)
        {
            totalCadMass += link.massKg;
        }

        double totalSimulatedMass = 0.0;
        for (const auto& link : state.manifest.links)
        {
            auto oIt = mr.linkMassOverrides.find(link.name);
            if (oIt != mr.linkMassOverrides.end() && oIt->second > 0.0)
            {
                totalSimulatedMass += oIt->second;
            }
            else
            {
                totalSimulatedMass += link.massKg * mr.linkMassMultiplier;
            }
        }
        if (mr.unmodeledBallast.massKg > 0.0)
        {
            totalSimulatedMass += mr.unmodeledBallast.massKg;
        }

        if (ImGui::CollapsingHeader("Scale Measurement vs CAD Mass", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("CAD Raw Total Mass:      %.3f kg (%.2f lbs)", totalCadMass, totalCadMass * 2.20462);
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Simulated Total Mass:   %.3f kg (%.2f lbs)",
                totalSimulatedMass, totalSimulatedMass * 2.20462);

            SliderWithDirectInput("Bench Scale Mass (kg)", &benchMassKg, 1.0f, 120.0f, "%.3f", 120.0f, 65.0f);

            double deltaM = static_cast<double>(benchMassKg) - totalCadMass;
            if (std::abs(deltaM) < 0.01)
            {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Unmodeled Mass Delta:  %+.3f kg (Exact Match)", deltaM);
            }
            else if (deltaM > 0.0)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Unmodeled Mass Delta:  %+.3f kg (Robot heavier than CAD)", deltaM);
            }
            else
            {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Unmodeled Mass Delta:  %+.3f kg (CAD heavier than robot)", deltaM);
            }
        }

        // 3. Mode A: Proportional Link Mass Scaling
        if (ImGui::CollapsingHeader("Mode A: Proportional Link Mass Scaling", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("Scales the mass and rotational inertia of all links proportionally by a uniform multiplier.");

            float mult = static_cast<float>(mr.linkMassMultiplier);
            if (SliderWithDirectInput("Link Mass Multiplier", &mult, 0.50f, 2.50f, "%.4f", 120.0f, 65.0f))
            {
                mr.linkMassMultiplier = static_cast<double>(mult);
            }

            if (totalCadMass > 0.0)
            {
                if (ImGui::Button("Compute Multiplier from Bench Scale (Bench / CAD)"))
                {
                    mr.linkMassMultiplier = static_cast<double>(benchMassKg) / totalCadMass;
                }
            }

            ImGui::Text("Effective Scaled Robot Mass: %.3f kg", totalCadMass * mr.linkMassMultiplier);
        }

        // 4. Mode B: Unmodeled Ballast (Point-Mass Inertial Offset)
        if (ImGui::CollapsingHeader("Mode B: Unmodeled Ballast Point-Mass", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("Injects an inertial point-mass offset directly into a target link's rigid body dynamics (e.g. unmodeled battery, wiring, or ballast weights).");

            auto& ballast = mr.unmodeledBallast;

            // Target Link Picker
            std::vector<std::string> linkNames;
            int selectedLinkIdx = -1;
            for (size_t i = 0; i < state.manifest.links.size(); ++i)
            {
                linkNames.push_back(state.manifest.links[i].name);
                if (state.manifest.links[i].name == ballast.targetLink)
                {
                    selectedLinkIdx = static_cast<int>(i);
                }
            }
            if (selectedLinkIdx == -1 && !linkNames.empty())
            {
                ballast.targetLink = linkNames[0];
                selectedLinkIdx = 0;
            }

            std::vector<const char*> linkNamePtrs;
            for (const auto& name : linkNames)
            {
                linkNamePtrs.push_back(name.c_str());
            }

            if (!linkNamePtrs.empty())
            {
                ImGui::SetNextItemWidth(180.0f);
                if (ImGui::Combo("Target Link##Ballast", &selectedLinkIdx, linkNamePtrs.data(), static_cast<int>(linkNamePtrs.size())))
                {
                    ballast.targetLink = linkNames[selectedLinkIdx];
                    state.RunLinter();
                }
            }

            // Ballast Mass
            float bMass = static_cast<float>(ballast.massKg);
            if (SliderWithDirectInput("Ballast Mass (kg)", &bMass, 0.0f, 50.0f, "%.3f", 120.0f, 65.0f))
            {
                ballast.massKg = static_cast<double>(std::max(0.0f, bMass));
            }

            // Ballast Center of Mass Offset (+X=Right, +Y=Up, +Z=Front)
            float offX = static_cast<float>(ballast.offsetM.x);
            float offY = static_cast<float>(ballast.offsetM.y);
            float offZ = static_cast<float>(ballast.offsetM.z);

            ImGui::TextDisabled("Ballast Offset (+X=Right, +Y=Up, +Z=Front):");
            if (SliderWithDirectInput("Offset X (Left [-] / Right [+]) (m)", &offX, -2.0f, 2.0f, "%.3f", 100.0f, 60.0f)) ballast.offsetM.x = offX;
            if (SliderWithDirectInput("Offset Y (Down [-] / Up [+]) (m)", &offY, -2.0f, 2.0f, "%.3f", 100.0f, 60.0f)) ballast.offsetM.y = offY;
            if (SliderWithDirectInput("Offset Z (Back [-] / Front [+]) (m)", &offZ, -2.0f, 2.0f, "%.3f", 100.0f, 60.0f)) ballast.offsetM.z = offZ;

            double deltaM = static_cast<double>(benchMassKg) - totalCadMass;
            if (ImGui::Button("Set Ballast = Deficit"))
            {
                ballast.massKg = std::max(0.0, deltaM);
            }
            ImGui::SameLine(0.0f, 10.0f);
            if (ImGui::Button("Zero Ballast"))
            {
                ballast.massKg = 0.0;
                ballast.offsetM = { 0.0, 0.0, 0.0 };
            }
        }

        // 5. Mode C: Individual Component Mass Overrides
        if (ImGui::CollapsingHeader("Mode C: Individual Component Mass Overrides", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TextWrapped("Directly override the mass of specific individual links or components (e.g. fabricated parts differing from CAD). Links without an individual override use the global multiplier or raw CAD mass.");

            if (!state.manifest.links.empty())
            {
                std::vector<const char*> allLinks;
                for (const auto& l : state.manifest.links)
                {
                    allLinks.push_back(l.name.c_str());
                }

                if (overrideLinkIdx >= static_cast<int>(allLinks.size()))
                {
                    overrideLinkIdx = 0;
                }

                ImGui::TextDisabled("Select Link to Override:");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::Combo("##OverrideLinkPicker", &overrideLinkIdx, allLinks.data(), static_cast<int>(allLinks.size()));

                const auto& curLink = state.manifest.links[overrideLinkIdx];
                double cadM = curLink.massKg;
                auto oIt = mr.linkMassOverrides.find(curLink.name);
                bool hasOverride = (oIt != mr.linkMassOverrides.end() && oIt->second > 0.0);
                double currentEffective = hasOverride ? oIt->second : (cadM * mr.linkMassMultiplier);

                ImGui::Text("CAD Mass: %.3f kg (%.2f lbs) | Effective: %.3f kg", cadM, cadM * 2.20462, currentEffective);

                SliderWithDirectInput("Override Mass (kg)##IndivOverride", &overrideMassKg, 0.01f, 50.0f, "%.3f", 120.0f, 65.0f);

                if (ImGui::Button("Set Override for Selected Link"))
                {
                    mr.linkMassOverrides[curLink.name] = static_cast<double>(overrideMassKg);
                    state.RunLinter();
                }

                if (hasOverride)
                {
                    ImGui::SameLine(0.0f, 8.0f);
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                    if (ImGui::Button("Clear Override for Selected"))
                    {
                        mr.linkMassOverrides.erase(curLink.name);
                        state.RunLinter();
                    }
                    ImGui::PopStyleColor();
                }

                // Table of Active Overrides
                if (!mr.linkMassOverrides.empty())
                {
                    ImGui::Spacing();
                    ImGui::TextDisabled("Active Individual Overrides (%zu):", mr.linkMassOverrides.size());

                    std::string linkToErase;
                    if (ImGui::BeginTable("OverridesTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                    {
                        ImGui::TableSetupColumn("Link Name", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("CAD Mass", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                        ImGui::TableSetupColumn("Override Mass", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                        ImGui::TableHeadersRow();

                        for (auto& [lnk, mVal] : mr.linkMassOverrides)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::TextUnformatted(lnk.c_str());

                            ImGui::TableSetColumnIndex(1);
                            const auto* l = state.manifest.FindLink(lnk);
                            if (l) ImGui::Text("%.3f kg", l->massKg);
                            else ImGui::TextDisabled("N/A");

                            ImGui::TableSetColumnIndex(2);
                            float editM = static_cast<float>(mVal);
                            ImGui::PushID(lnk.c_str());
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::InputFloat("##EditM", &editM, 0.1f, 1.0f, "%.3f"))
                            {
                                mVal = static_cast<double>(std::max(0.001f, editM));
                            }

                            ImGui::TableSetColumnIndex(3);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                            if (ImGui::Button("X##DelOverride", ImVec2(-1.0f, 0.0f)))
                            {
                                linkToErase = lnk;
                            }
                            ImGui::PopStyleColor();
                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }

                    if (!linkToErase.empty())
                    {
                        mr.linkMassOverrides.erase(linkToErase);
                        state.RunLinter();
                    }
                }
            }
        }

        // 5. Non-Destructive Invariant Guarantee Note
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Non-Destructive Guarantee:");
        ImGui::TextWrapped(
            "CAD manifest files (manifest.json, Meshes/*.glb) are never modified. "
            "Reconciliation settings are stored strictly in sidecar.json under 'massReconciliation'. "
            "Ballast is injected purely as an inertial point-mass into Jolt rigid body dynamics. "
            "No physics collision shapes are generated for ballast, preventing phantom snagging."
        );
    }
}
