/**
 * @file BumperPanel.cpp
 * @brief Subsystem inspector panel for configuring FRC bumpers, collision geometry, physical mass, and visual presets.
 */

#include "Tools/Configurator/Panels/BumperPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"
#include "Tools/Configurator/Panels/ConfiguratorWidgets.h"

#include <imgui/imgui.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Tools::Configurator
{
    void BumperPanel::Render(ConfiguratorState& state)
    {
        ImGui::PushTextWrapPos(0.0f);

        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "FRC BUMPER CONFIGURATION & VIRTUAL COLLIDERS");
        ImGui::TextDisabled("Define physical collision bounds, mass, and 3D visual preview for robots without CAD-modeled bumpers.");
        ImGui::Separator();

        if (!state.isCadLoaded)
        {
            ImGui::TextColored(ImVec4(0.85f, 0.70f, 0.20f, 1.0f),
                "Note: No CAD loaded. Virtual bumpers previewed around robot origin (0,0,0). Load CAD to bind to specific links.");
            ImGui::Spacing();
        }

        // Check if CAD already had a bumper collider in manifest
        bool cadHasBumpers = false;
        std::string cadBumperLink;
        for (const auto& l : state.manifest.links)
        {
            for (const auto& col : l.colliders)
            {
                if (col.name.find("bumper") != std::string::npos || col.material == "bumper_fabric")
                {
                    cadHasBumpers = true;
                    cadBumperLink = l.name;
                    break;
                }
            }
        }

        if (cadHasBumpers)
        {
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f),
                "Note: CAD manifest contains existing bumper geometry on '%s'.\n"
                "Enabling Virtual Bumpers will override CAD bumpers in physical simulation.",
                cadBumperLink.c_str());
            ImGui::Spacing();
        }

        // Enable Toggle
        bool enabled = state.sidecar.bumpers.enabled;
        if (ImGui::Checkbox("Enable Virtual Bumpers & Physics Colliders##BumperEnable", &enabled))
        {
            state.sidecar.bumpers.enabled = enabled;
            state.sidecarVersion++;
            state.RunLinter();
        }

        if (!enabled)
        {
            ImGui::Spacing();
            ImGui::TextDisabled("Virtual bumpers are disabled. The robot will simulate using raw CAD collision bounds only.");
            return;
        }

        ImGui::Spacing();

        // 1. Quick Presets & Sizing Helpers
        // -------------------------------------------------------------
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Quick Presets & Sizing Helpers:");

        auto applyBumperPreset = [&](double lenM, double widthM, double heightM, double thickM)
        {
            state.sidecar.bumpers.lengthM = lenM;
            state.sidecar.bumpers.widthM = widthM;
            state.sidecar.bumpers.heightM = heightM;
            state.sidecar.bumpers.thicknessM = thickM;
            state.sidecarVersion++;
            state.RunLinter();
        };

        if (ImGui::Button("28x28 Frame (34.5x34.5\" Bumpers)"))
        {
            applyBumperPreset(0.8763, 0.8763, 0.127, 0.0825);
        }
        AutoWrapSameLine(220.0f);
        if (ImGui::Button("30x30 Frame (36.5x36.5\" Bumpers)"))
        {
            applyBumperPreset(0.9271, 0.9271, 0.127, 0.0825);
        }
        AutoWrapSameLine(220.0f);
        if (ImGui::Button("26x26 Frame (32.5x32.5\" Bumpers)"))
        {
            applyBumperPreset(0.8255, 0.8255, 0.127, 0.0825);
        }

        if (!state.sidecar.swerve.modules.empty())
        {
            if (ImGui::Button("Fit to Swerve Module Footprint##FitSwerve"))
            {
                double maxAbsX = 0.0;
                double maxAbsZ = 0.0;
                for (const auto& mod : state.sidecar.swerve.modules)
                {
                    maxAbsX = std::max(maxAbsX, std::abs(mod.posM.x));
                    maxAbsZ = std::max(maxAbsZ, std::max(std::abs(mod.posM.z), std::abs(mod.posM.y)));
                }
                double wRad = state.sidecar.swerve.wheelRadiusM > 0.0 ? state.sidecar.swerve.wheelRadiusM : 0.0508;
                double thick = state.sidecar.bumpers.thicknessM > 0.0 ? state.sidecar.bumpers.thicknessM : 0.0825;
                // Frame margin: modules extent + tire radius + 25mm frame margin + bumper thickness
                double fitWidth = 2.0 * (maxAbsX + wRad + 0.025) + 2.0 * thick;
                double fitLen = 2.0 * (maxAbsZ + wRad + 0.025) + 2.0 * thick;
                state.sidecar.bumpers.widthM = fitWidth;
                state.sidecar.bumpers.lengthM = fitLen;
                state.sidecarVersion++;
                state.RunLinter();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Automatically calculates bumper width (X) and length (Z) to wrap around all 4 swerve modules plus tire clearance.");
            }
        }

        ImGui::Separator();

        // 2. Target Chassis Link
        // -------------------------------------------------------------
        std::vector<const char*> linkNames;
        int targetIdx = -1;
        for (size_t i = 0; i < state.manifest.links.size(); ++i)
        {
            linkNames.push_back(state.manifest.links[i].name.c_str());
            if (state.sidecar.bumpers.targetLink.empty())
            {
                if (state.manifest.links[i].isRoot) targetIdx = static_cast<int>(i);
            }
            else if (state.manifest.links[i].name == state.sidecar.bumpers.targetLink)
            {
                targetIdx = static_cast<int>(i);
            }
        }
        if (targetIdx == -1 && !linkNames.empty()) targetIdx = 0;

        if (!linkNames.empty())
        {
            ImGui::SetNextItemWidth(220.0f);
            if (ImGui::Combo("Attachment Link##BumperTarget", &targetIdx, linkNames.data(), static_cast<int>(linkNames.size())))
            {
                if (targetIdx >= 0 && targetIdx < static_cast<int>(linkNames.size()))
                {
                    state.sidecar.bumpers.targetLink = linkNames[targetIdx];
                    state.sidecarVersion++;
                    state.RunLinter();
                }
            }
        }
        else
        {
            ImGui::TextDisabled("Attachment Link: [Mounted at robot origin (0,0,0) - Load CAD to select links]");
        }

        // 3. Bumper Geometry & Shape
        // -------------------------------------------------------------
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Bumper Geometry & Perimeter Shape (+X=Right, +Y=Up, +Z=Front):");

        const char* kShapes[] = { "box", "perimeter", "u_shape" };
        const char* kShapeLabels[] = {
            "Solid Box (Enclosed Cuboid)",
            "4-Sided Hollow Perimeter",
            "U-Shape / Front Cutout (Ground Intake)"
        };
        int currentShapeIdx = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (state.sidecar.bumpers.shape == kShapes[i]) { currentShapeIdx = i; break; }
        }

        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::Combo("Perimeter Shape##BumperShape", &currentShapeIdx, kShapeLabels, 3))
        {
            state.sidecar.bumpers.shape = kShapes[currentShapeIdx];
            state.sidecarVersion++;
            state.RunLinter();
        }

        float widthX = static_cast<float>(state.sidecar.bumpers.widthM);
        if (SliderWithDirectInput("Outer Width (X / Left-Right) (m)##BumperWidth", &widthX, 0.30f, 1.60f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.widthM = widthX;
            state.sidecarVersion++;
            state.RunLinter();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%.2f in)", widthX * 39.3701f);

        float heightY = static_cast<float>(state.sidecar.bumpers.heightM);
        if (SliderWithDirectInput("Bumper Height (Y / Up-Down) (m)##BumperHeight", &heightY, 0.05f, 0.30f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.heightM = heightY;
            state.sidecarVersion++;
            state.RunLinter();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%.2f in - std 5.0\")", heightY * 39.3701f);

        float lenZ = static_cast<float>(state.sidecar.bumpers.lengthM);
        if (SliderWithDirectInput("Outer Length (Z / Front-Back) (m)##BumperLen", &lenZ, 0.30f, 1.60f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.lengthM = lenZ;
            state.sidecarVersion++;
            state.RunLinter();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%.2f in)", lenZ * 39.3701f);

        if (state.sidecar.bumpers.shape != "box")
        {
            float thick = static_cast<float>(state.sidecar.bumpers.thicknessM);
            if (SliderWithDirectInput("Wall Thickness (m)##BumperThick", &thick, 0.02f, 0.20f, "%.3f m", 110.0f))
            {
                state.sidecar.bumpers.thicknessM = thick;
                state.sidecarVersion++;
                state.RunLinter();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(%.2f in - std 3.25\")", thick * 39.3701f);
        }

        if (state.sidecar.bumpers.shape == "u_shape")
        {
            float cutoutW = static_cast<float>(state.sidecar.bumpers.cutoutWidthM);
            if (SliderWithDirectInput("Front Cutout Width (X / Left-Right) (m)##BumperCutout", &cutoutW, 0.05f, 1.20f, "%.3f m", 110.0f))
            {
                state.sidecar.bumpers.cutoutWidthM = cutoutW;
                state.sidecarVersion++;
                state.RunLinter();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(%.2f in)", cutoutW * 39.3701f);
        }

        // 4. Elevation & Mounting Offsets
        // -------------------------------------------------------------
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Elevation & Mounting Offsets (+X=Right, +Y=Up, +Z=Front):");

        float offX = static_cast<float>(state.sidecar.bumpers.offsetM.x);
        if (SliderWithDirectInput("Center Offset X (Left [-] / Right [+]) (m)##BumperOffX", &offX, -1.0f, 1.0f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.offsetM.x = offX;
            state.sidecarVersion++;
            state.RunLinter();
        }

        float offY = static_cast<float>(state.sidecar.bumpers.offsetM.y);
        if (SliderWithDirectInput("Center Offset Y (Elevation / Up [+]) (m)##BumperOffY", &offY, -1.0f, 1.0f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.offsetM.y = offY;
            state.sidecarVersion++;
            state.RunLinter();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%.2f in)", offY * 39.3701f);

        float offZ = static_cast<float>(state.sidecar.bumpers.offsetM.z);
        if (SliderWithDirectInput("Center Offset Z (Back [-] / Front [+]) (m)##BumperOffZ", &offZ, -1.0f, 1.0f, "%.3f m", 110.0f))
        {
            state.sidecar.bumpers.offsetM.z = offZ;
            state.sidecarVersion++;
            state.RunLinter();
        }

        // FRC Bumper Zone Legal Indicator (0.5 to 7.5 in off floor)
        float bottomIn = (offY - heightY * 0.5f) * 39.3701f;
        float topIn = (offY + heightY * 0.5f) * 39.3701f;
        if (bottomIn >= 0.5f && topIn <= 7.5f)
        {
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f),
                "FRC Bumper Zone: Bottom = %.1f\", Top = %.1f\" [LEGAL in 0.5\" - 7.5\" Zone (Y Above Floor)]", bottomIn, topIn);
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
                "FRC Bumper Zone: Bottom = %.1f\", Top = %.1f\" [WARNING: Outside standard 0.5\" - 7.5\" Zone (Y Above Floor)]", bottomIn, topIn);
        }

        // 5. Mass & Physics Properties
        // -------------------------------------------------------------
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "Bumper Mass & Physics Material:");

        float massKg = static_cast<float>(state.sidecar.bumpers.massKg);
        if (SliderWithDirectInput("Bumper Mass (kg)##BumperMass", &massKg, 0.0f, 15.0f, "%.2f kg", 110.0f))
        {
            state.sidecar.bumpers.massKg = massKg;
            state.sidecarVersion++;
            state.RunLinter();
        }
        ImGui::SameLine();
        float massLbs = massKg * 2.20462f;
        if (massLbs <= 15.0f)
        {
            ImGui::TextDisabled("(%.1f lbs - FRC Legal <= 15.0 lbs)", massLbs);
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%.1f lbs - EXCEEDS 15.0 LB RULE)", massLbs);
        }

        // Material Combo
        const char* kMaterials[] = { "bumper_fabric", "colson_performa", "aluminum_6061", "polycarbonate" };
        int matIdx = 0;
        for (int m = 0; m < 4; ++m)
        {
            if (state.sidecar.bumpers.material == kMaterials[m]) { matIdx = m; break; }
        }
        ImGui::SetNextItemWidth(180.0f);
        if (ImGui::Combo("Friction Material##BumperMat", &matIdx, kMaterials, 4))
        {
            state.sidecar.bumpers.material = kMaterials[matIdx];
            state.sidecarVersion++;
            state.RunLinter();
        }

        // 6. Viewport Alliance Color
        // -------------------------------------------------------------
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f), "3D Viewport Alliance Color:");

        bool isRed = (state.sidecar.bumpers.color != "blue");
        if (isRed)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
        }
        if (ImGui::Button("Red Alliance##BumperRed", ImVec2(120.0f, 26.0f)))
        {
            state.sidecar.bumpers.color = "red";
            state.sidecarVersion++;
        }
        if (isRed) ImGui::PopStyleColor(2);

        ImGui::SameLine();

        bool isBlue = (state.sidecar.bumpers.color == "blue");
        if (isBlue)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.85f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.55f, 0.95f, 1.0f));
        }
        if (ImGui::Button("Blue Alliance##BumperBlue", ImVec2(120.0f, 26.0f)))
        {
            state.sidecar.bumpers.color = "blue";
            state.sidecarVersion++;
        }
        if (isBlue) ImGui::PopStyleColor(2);

        ImGui::PopTextWrapPos();
    }
}
