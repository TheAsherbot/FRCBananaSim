/**
 * @file ContactZonePanel.cpp
 * @brief Subsystem inspector panel for authoring, inspecting, and editing game-piece contact zones.
 */

#include "Tools/Configurator/Panels/ContactZonePanel.h"
#include "Tools/Configurator/ConfiguratorState.h"
#include "Tools/Configurator/Panels/ConfiguratorWidgets.h"

#include <imgui/imgui.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Tools::Configurator
{
    void ContactZonePanel::Render(ConfiguratorState& state)
    {
        ImGui::PushTextWrapPos(0.0f);

        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.5f, 1.0f), "GAME-PIECE CONTACT ZONES & INTERACTION VOLUMES");
        ImGui::TextDisabled("Define compliant intake capture volumes and shooter squeeze nips for hybrid game-piece physics.");
        ImGui::Separator();

        auto effectiveZones = state.GetEffectiveContactZones();

        // 1. Zone Selection & Add/Remove Bar
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "CONTACT ZONES LIST");

        if (ImGui::Button("+ Add Custom Contact Zone", ImVec2(220.0f, 26.0f)))
        {
            Robot::Schema::ContactZoneDefinition newZone;
            newZone.name = "contact_zone_" + std::to_string(effectiveZones.size() + 1);
            newZone.role = "intake";
            newZone.parentLink = state.manifest.links.empty() ? "base_link" : state.manifest.links[0].name;
            newZone.shape = "box";
            newZone.originM = glm::dvec3(0.0, 0.0, 0.0);
            newZone.dimensionsM = glm::dvec3(0.30, 0.20, 0.10); // NWU: X=0.30 (Robot Z length), Y=0.20 (Robot X width), Z=0.10 (Robot Y height)
            newZone.tangentAxis = glm::dvec3(1.0, 0.0, 0.0);    // NWU +X is Robot +Z (Forward)
            newZone.compressionAxis = glm::dvec3(0.0, 0.0, 1.0);// NWU +Z is Robot +Y (Up)
            newZone.nominalGapM = 0.08;

            state.sidecar.contactZones.push_back(newZone);
            state.selectedContactZone = newZone.name;
            state.RunLinter();

            // Refresh effective zones so the new zone is immediately active this frame
            effectiveZones = state.GetEffectiveContactZones();
        }

        ImGui::Spacing();

        if (effectiveZones.empty())
        {
            ImGui::TextColored(ImVec4(0.85f, 0.70f, 0.20f, 1.0f),
                "No contact zones defined in CAD manifest or sidecar. Click '+ Add Custom Contact Zone' above to create one.");
            ImGui::PopTextWrapPos();
            return;
        }

        // Zone selector list
        if (state.selectedContactZone.empty() ||
            std::find_if(effectiveZones.begin(), effectiveZones.end(),
                [&](const auto& z) { return z.name == state.selectedContactZone; }) == effectiveZones.end())
        {
            state.selectedContactZone = effectiveZones[0].name;
        }

        int currentZoneIdx = 0;
        std::vector<std::string> zoneDisplayNames;
        for (size_t i = 0; i < effectiveZones.size(); ++i)
        {
            const auto& z = effectiveZones[i];
            bool inCad = (state.manifest.FindContactZone(z.name) != nullptr);
            bool inSidecar = (state.sidecar.FindContactZone(z.name) != nullptr);

            std::string tag = inCad ? (inSidecar ? " [CAD OVERRIDE]" : " [CAD]") : " [CUSTOM]";
            std::string label = z.name + " (" + z.role + ")" + tag;
            zoneDisplayNames.push_back(label);

            if (z.name == state.selectedContactZone)
            {
                currentZoneIdx = static_cast<int>(i);
            }
        }

        std::vector<const char*> zoneItems;
        for (const auto& s : zoneDisplayNames) zoneItems.push_back(s.c_str());

        ImGui::SetNextItemWidth(360.0f);
        if (ImGui::Combo("Select Zone", &currentZoneIdx, zoneItems.data(), static_cast<int>(zoneItems.size())))
        {
            if (currentZoneIdx >= 0 && currentZoneIdx < static_cast<int>(effectiveZones.size()))
            {
                state.selectedContactZone = effectiveZones[currentZoneIdx].name;
            }
        }

        if (currentZoneIdx < 0 || currentZoneIdx >= static_cast<int>(effectiveZones.size()))
        {
            currentZoneIdx = 0;
            if (!effectiveZones.empty())
            {
                state.selectedContactZone = effectiveZones[0].name;
            }
        }

        ImGui::SameLine();
        const auto& activeZone = effectiveZones[currentZoneIdx];
        std::string activeZoneName = activeZone.name;
        bool isActiveInSidecar = (state.sidecar.FindContactZone(activeZoneName) != nullptr);
        bool isActiveInCad = (state.manifest.FindContactZone(activeZoneName) != nullptr);

        if (isActiveInSidecar)
        {
            if (isActiveInCad)
            {
                if (ImGui::Button("Revert to CAD Default"))
                {
                    auto it = std::remove_if(state.sidecar.contactZones.begin(), state.sidecar.contactZones.end(),
                        [&](const auto& z) { return z.name == activeZoneName; });
                    state.sidecar.contactZones.erase(it, state.sidecar.contactZones.end());
                    state.RunLinter();
                    ImGui::PopTextWrapPos();
                    return;
                }
            }
            else
            {
                if (ImGui::Button("Delete Custom Zone"))
                {
                    auto it = std::remove_if(state.sidecar.contactZones.begin(), state.sidecar.contactZones.end(),
                        [&](const auto& z) { return z.name == activeZoneName; });
                    state.sidecar.contactZones.erase(it, state.sidecar.contactZones.end());
                    auto remaining = state.GetEffectiveContactZones();
                    state.selectedContactZone = remaining.empty() ? "" : remaining[0].name;
                    state.RunLinter();
                    ImGui::PopTextWrapPos();
                    return;
                }
            }
        }

        ImGui::Separator();
        ImGui::Spacing();

        if (state.selectedContactZone.empty())
        {
            ImGui::PopTextWrapPos();
            return;
        }

        // Ensure zone is editable: if CAD-only and user modifies it, promote to sidecar override
        auto ensureSidecarOverride = [&]() -> Robot::Schema::ContactZoneDefinition&
        {
            auto* existing = state.sidecar.FindContactZone(state.selectedContactZone);
            if (existing)
            {
                return *existing;
            }
            // Promote CAD zone to sidecar override
            const auto* cadZone = state.manifest.FindContactZone(state.selectedContactZone);
            if (cadZone)
            {
                state.sidecar.contactZones.push_back(*cadZone);
            }
            else
            {
                Robot::Schema::ContactZoneDefinition newZone;
                newZone.name = state.selectedContactZone;
                state.sidecar.contactZones.push_back(newZone);
            }
            return state.sidecar.contactZones.back();
        };

        auto* editZone = state.sidecar.FindContactZone(state.selectedContactZone);
        const auto* cadZone = state.manifest.FindContactZone(state.selectedContactZone);
        if (!editZone && !cadZone)
        {
            ImGui::PopTextWrapPos();
            return;
        }
        Robot::Schema::ContactZoneDefinition currentView = editZone ? *editZone : *cadZone;

        // 2. Identification & Role
        ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "ZONE IDENTIFICATION & ROLE");

        char nameBuf[128];
        std::snprintf(nameBuf, sizeof(nameBuf), "%s", currentView.name.c_str());
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::InputText("Zone Name##CZ", nameBuf, sizeof(nameBuf), ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (std::strlen(nameBuf) > 0)
            {
                auto& z = ensureSidecarOverride();
                z.name = nameBuf;
                state.selectedContactZone = nameBuf;
                state.RunLinter();
            }
        }

        // Role Selector
        const char* roleOptions[] = { "intake", "shooter", "indexer" };
        int roleIdx = (currentView.role == "shooter") ? 1 : ((currentView.role == "indexer") ? 2 : 0);
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::Combo("Interaction Role##CZ", &roleIdx, roleOptions, 3))
        {
            auto& z = ensureSidecarOverride();
            z.role = roleOptions[roleIdx];
            state.RunLinter();
        }

        // Parent Link Picker
        std::vector<const char*> linkNames;
        int selectedLinkIdx = -1;
        for (size_t l = 0; l < state.manifest.links.size(); ++l)
        {
            linkNames.push_back(state.manifest.links[l].name.c_str());
            if (currentView.parentLink == state.manifest.links[l].name)
            {
                selectedLinkIdx = static_cast<int>(l);
            }
        }
        ImGui::SetNextItemWidth(280.0f);
        if (!linkNames.empty())
        {
            if (ImGui::Combo("Parent Link##CZ", &selectedLinkIdx, linkNames.data(), static_cast<int>(linkNames.size())))
            {
                if (selectedLinkIdx >= 0 && selectedLinkIdx < static_cast<int>(linkNames.size()))
                {
                    auto& z = ensureSidecarOverride();
                    z.parentLink = linkNames[selectedLinkIdx];
                    state.RunLinter();
                }
            }
        }
        else
        {
            ImGui::TextDisabled("No CAD links available in manifest.");
        }

        ImGui::Spacing();
        ImGui::Separator();

        // 3. Geometry & Dimensions
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "BOUNDING VOLUME GEOMETRY (+X=Right, +Y=Up, +Z=Front):");

        // Convert stored NWU to Robot Frame (+X Right, +Y Up, +Z Front) for intuitive editing
        // NWU: +X=Fwd (Robot +Z), +Y=Left (Robot -X), +Z=Up (Robot +Y)
        // Therefore:
        // Robot X (Right) = -NWU.y
        // Robot Y (Up)    = +NWU.z
        // Robot Z (Front) = +NWU.x
        float originRob[3] = {
            static_cast<float>(-currentView.originM.y),
            static_cast<float>(currentView.originM.z),
            static_cast<float>(currentView.originM.x)
        };
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::DragFloat3("Origin Offset (X, Y, Z) (m)##CZ", originRob, 0.005f, -2.0f, 2.0f, "%.3f m"))
        {
            auto& z = ensureSidecarOverride();
            // Convert back to NWU: NWU.x = Robot.Z, NWU.y = -Robot.X, NWU.z = Robot.Y
            z.originM = glm::dvec3(originRob[2], -originRob[0], originRob[1]);
            state.RunLinter();
        }

        // Dimensions (Width X, Height Y, Length Z) in Robot Frame
        float dimsRob[3] = {
            static_cast<float>(currentView.dimensionsM.y), // Width (X)
            static_cast<float>(currentView.dimensionsM.z), // Height (Y)
            static_cast<float>(currentView.dimensionsM.x)  // Length (Z)
        };
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::DragFloat3("Dimensions (X, Y, Z) (m)##CZ", dimsRob, 0.005f, 0.01f, 1.5f, "%.3f m"))
        {
            auto& z = ensureSidecarOverride();
            z.dimensionsM = glm::dvec3(
                std::max(0.01f, dimsRob[2]), // NWU.x = Length Z
                std::max(0.01f, dimsRob[0]), // NWU.y = Width X
                std::max(0.01f, dimsRob[1])  // NWU.z = Height Y
            );
            state.RunLinter();
        }

        ImGui::Spacing();
        ImGui::Separator();

        // 4. Directional Kinematics (Tangent & Compression Axes)
        ImGui::TextColored(ImVec4(0.35f, 0.9f, 1.0f, 1.0f), "TRANSIT & SQUEEZE KINEMATICS (+X=Right, +Y=Up, +Z=Front):");

        // Tangent Axis (Transit Direction) in Robot Frame
        float tanAxisRob[3] = {
            static_cast<float>(-currentView.tangentAxis.y),
            static_cast<float>(currentView.tangentAxis.z),
            static_cast<float>(currentView.tangentAxis.x)
        };
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::DragFloat3("Transit Tangent (X, Y, Z)##CZ", tanAxisRob, 0.02f, -1.0f, 1.0f, "%.3f"))
        {
            auto& z = ensureSidecarOverride();
            z.tangentAxis = glm::dvec3(tanAxisRob[2], -tanAxisRob[0], tanAxisRob[1]);
            state.RunLinter();
        }

        ImGui::SameLine();
        if (ImGui::Button("Normalize##Tan"))
        {
            auto& z = ensureSidecarOverride();
            if (glm::length(z.tangentAxis) > 1e-4) z.tangentAxis = glm::normalize(z.tangentAxis);
            state.RunLinter();
        }

        // Quick Presets for Tangent Axis (Robot Frame: +Z=Front, +X=Right, +Y=Up)
        ImGui::TextDisabled("Tangent Presets:");
        ImGui::SameLine();
        if (ImGui::SmallButton("+Z (Front)")) { auto& z = ensureSidecarOverride(); z.tangentAxis = { 1, 0, 0 }; state.RunLinter(); } // NWU +X is Robot +Z
        ImGui::SameLine();
        if (ImGui::SmallButton("-Z (Back)"))  { auto& z = ensureSidecarOverride(); z.tangentAxis = { -1, 0, 0 }; state.RunLinter(); } // NWU -X is Robot -Z
        ImGui::SameLine();
        if (ImGui::SmallButton("+Y (Up)"))    { auto& z = ensureSidecarOverride(); z.tangentAxis = { 0, 0, 1 }; state.RunLinter(); } // NWU +Z is Robot +Y
        ImGui::SameLine();
        if (ImGui::SmallButton("-Y (Down)"))  { auto& z = ensureSidecarOverride(); z.tangentAxis = { 0, 0, -1 }; state.RunLinter(); } // NWU -Z is Robot -Y
        ImGui::SameLine();
        if (ImGui::SmallButton("+X (Right)")) { auto& z = ensureSidecarOverride(); z.tangentAxis = { 0, -1, 0 }; state.RunLinter(); } // NWU -Y is Robot +X

        // Compression Axis (Squeeze Direction) in Robot Frame
        float compAxisRob[3] = {
            static_cast<float>(-currentView.compressionAxis.y),
            static_cast<float>(currentView.compressionAxis.z),
            static_cast<float>(currentView.compressionAxis.x)
        };
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::DragFloat3("Squeeze Compression (X, Y, Z)##CZ", compAxisRob, 0.02f, -1.0f, 1.0f, "%.3f"))
        {
            auto& z = ensureSidecarOverride();
            z.compressionAxis = glm::dvec3(compAxisRob[2], -compAxisRob[0], compAxisRob[1]);
            state.RunLinter();
        }

        ImGui::SameLine();
        if (ImGui::Button("Normalize##Comp"))
        {
            auto& z = ensureSidecarOverride();
            if (glm::length(z.compressionAxis) > 1e-4) z.compressionAxis = glm::normalize(z.compressionAxis);
            state.RunLinter();
        }

        // Quick Presets for Compression Axis (Robot Frame)
        ImGui::TextDisabled("Compression Presets:");
        ImGui::SameLine();
        if (ImGui::SmallButton("+Y (Up)##CP"))   { auto& z = ensureSidecarOverride(); z.compressionAxis = { 0, 0, 1 }; state.RunLinter(); } // NWU +Z is Robot +Y
        ImGui::SameLine();
        if (ImGui::SmallButton("-Y (Down)##CP")) { auto& z = ensureSidecarOverride(); z.compressionAxis = { 0, 0, -1 }; state.RunLinter(); } // NWU -Z is Robot -Y
        ImGui::SameLine();
        if (ImGui::SmallButton("+X (Right)##CP")){ auto& z = ensureSidecarOverride(); z.compressionAxis = { 0, -1, 0 }; state.RunLinter(); } // NWU -Y is Robot +X
        ImGui::SameLine();
        if (ImGui::SmallButton("+Z (Front)##CP")){ auto& z = ensureSidecarOverride(); z.compressionAxis = { 1, 0, 0 }; state.RunLinter(); } // NWU +X is Robot +Z

        // Orthogonality Indicator
        double tLen = glm::length(currentView.tangentAxis);
        double cLen = glm::length(currentView.compressionAxis);
        if (tLen > 1e-4 && cLen > 1e-4)
        {
            double dotVal = std::abs(glm::dot(currentView.tangentAxis / tLen, currentView.compressionAxis / cLen));
            if (dotVal <= 0.08)
            {
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
                    "● Axes are Orthogonal: Tangent & Squeeze directions are perpendicular (dot = %.3f).", dotVal);
            }
            else
            {
                ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.2f, 1.0f),
                    "▲ Warning: Non-orthogonal axes (dot = %.3f). Tangent and Squeeze axes must be perpendicular!", dotVal);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // 5. Nominal Nip Gap
        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), "ROLLER COMPRESSION NIP GAP");

        float gapMm = static_cast<float>(currentView.nominalGapM * 1000.0);
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::SliderFloat("Nominal Nip Gap (mm)##CZ", &gapMm, 5.0f, 250.0f, "%.1f mm"))
        {
            auto& z = ensureSidecarOverride();
            z.nominalGapM = static_cast<double>(gapMm) / 1000.0;
            state.RunLinter();
        }
        ImGui::TextDisabled("Distance between roller surface and hood (or between dual rollers): %.3f m (%.2f in)",
            currentView.nominalGapM, currentView.nominalGapM * 39.3701);

        ImGui::Spacing();
        ImGui::Separator();

        // 6. Sensor Association
        ImGui::TextColored(ImVec4(0.2f, 0.85f, 1.0f, 1.0f), "OPTICAL BEAM-BREAK SENSOR ASSOCIATION");

        bool hasAssociatedSensor = false;
        std::string boundSensorName;
        uint32_t boundDioChannel = 0;

        for (const auto& sens : state.sidecar.sensors)
        {
            if (sens.contactZone == currentView.name)
            {
                hasAssociatedSensor = true;
                boundSensorName = sens.name;
                boundDioChannel = sens.dioChannel;
                break;
            }
        }

        if (hasAssociatedSensor)
        {
            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f),
                "● Bound to Beam Break: '%s' on DIO Channel %u", boundSensorName.c_str(), boundDioChannel);
        }
        else
        {
            ImGui::TextDisabled("No Beam Break sensor is currently monitoring this contact zone.");
            if (ImGui::Button("+ Create Associated Beam Break Sensor", ImVec2(280.0f, 26.0f)))
            {
                Robot::Schema::SensorConfig bb;
                bb.name = currentView.name + "_beam_break";
                bb.type = "beam_break";
                bb.bus = "rio";
                bb.dioChannel = 2;
                bb.normallyClosed = false;
                bb.debounceTimeS = 0.010;
                bb.contactZone = currentView.name;
                bb.parentLink = currentView.parentLink;

                state.sidecar.sensors.push_back(bb);
                state.selectedSensor = bb.name;
                state.activeInspectorTab = ConfiguratorState::ActiveInspectorTab::Sensors;
                state.RunLinter();
            }
        }

        ImGui::PopTextWrapPos();
    }
}
