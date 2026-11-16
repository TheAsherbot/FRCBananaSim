/**
 * @file CADViewportPanel.cpp
 * @brief Implementation of the interactive 3D CAD preview viewport with orbit camera,
 *        left-handed coordinate system (+X Right, +Y Up, +Z Front), bumper wireframes,
 *        and joint arc visualization.
 */

#include "Tools/Configurator/Panels/CADViewportPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"
#include "Robot/Schema/RobotSchemaLoader.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Tools::Configurator
{
    using Robot::Schema::RobotSchemaLoader;

    CADViewportPanel::CADViewportPanel()
    {
        SetPreset(CameraPreset::Fusion360);
        LoadSettings();
        ResetCamera();
    }

    void CADViewportPanel::SetPreset(CameraPreset preset)
    {
        cameraConfig.preset = preset;
        if (preset == CameraPreset::Fusion360)
        {
            cameraConfig.invertScrollZoom = true; // Inverted: Scroll forward/up = zoom in
            cameraConfig.orbitSensitivity = 0.40f;
            cameraConfig.panSensitivity = 0.0025f;
            cameraConfig.zoomSensitivity = 0.20f;
        }
        else if (preset == CameraPreset::Blender)
        {
            cameraConfig.invertScrollZoom = true;
            cameraConfig.orbitSensitivity = 0.40f;
            cameraConfig.panSensitivity = 0.0025f;
            cameraConfig.zoomSensitivity = 0.20f;
        }
        else if (preset == CameraPreset::StandardCAD)
        {
            cameraConfig.invertScrollZoom = true;
            cameraConfig.orbitSensitivity = 0.40f;
            cameraConfig.panSensitivity = 0.0025f;
            cameraConfig.zoomSensitivity = 0.20f;
        }
        SaveSettings();
    }

    void CADViewportPanel::LoadSettings(const std::string& filepath)
    {
        std::string configPath = filepath;
        if (configPath.empty())
        {
            if (std::filesystem::exists("FRCBananaSim/Resources/Config/configurator_settings.json"))
            {
                configPath = "FRCBananaSim/Resources/Config/configurator_settings.json";
            }
            else if (std::filesystem::exists("../FRCBananaSim/Resources/Config/configurator_settings.json"))
            {
                configPath = "../FRCBananaSim/Resources/Config/configurator_settings.json";
            }
            else if (std::filesystem::exists("configurator_settings.json"))
            {
                configPath = "configurator_settings.json";
            }
        }

        if (!configPath.empty() && std::filesystem::exists(configPath))
        {
            try
            {
                std::ifstream f(configPath);
                if (f.is_open())
                {
                    nlohmann::json j;
                    f >> j;
                    if (j.contains("preset")) cameraConfig.preset = static_cast<CameraPreset>(j["preset"].get<int>());
                    if (j.contains("invertScrollZoom")) cameraConfig.invertScrollZoom = j["invertScrollZoom"].get<bool>();
                    if (j.contains("invertOrbitX")) cameraConfig.invertOrbitX = j["invertOrbitX"].get<bool>();
                    if (j.contains("invertOrbitY")) cameraConfig.invertOrbitY = j["invertOrbitY"].get<bool>();
                    if (j.contains("orbitSensitivity")) cameraConfig.orbitSensitivity = j["orbitSensitivity"].get<float>();
                    if (j.contains("panSensitivity")) cameraConfig.panSensitivity = j["panSensitivity"].get<float>();
                    if (j.contains("zoomSensitivity")) cameraConfig.zoomSensitivity = j["zoomSensitivity"].get<float>();
                    if (j.contains("fovDegrees")) cameraConfig.fovDegrees = j["fovDegrees"].get<float>();
                }
            }
            catch (...) {}
        }
    }

    void CADViewportPanel::SaveSettings(const std::string& filepath) const
    {
        std::string configPath = filepath;
        if (configPath.empty())
        {
            if (std::filesystem::exists("FRCBananaSim/Resources/Config"))
            {
                configPath = "FRCBananaSim/Resources/Config/configurator_settings.json";
            }
            else if (std::filesystem::exists("../FRCBananaSim/Resources/Config"))
            {
                configPath = "../FRCBananaSim/Resources/Config/configurator_settings.json";
            }
            else
            {
                configPath = "configurator_settings.json";
            }
        }

        try
        {
            nlohmann::json j;

            if (std::filesystem::exists(configPath))
            {
                std::ifstream inF(configPath);
                if (inF.is_open())
                {
                    inF >> j;
                }
            }

            j["preset"] = static_cast<int>(cameraConfig.preset);
            j["invertScrollZoom"] = cameraConfig.invertScrollZoom;
            j["invertOrbitX"] = cameraConfig.invertOrbitX;
            j["invertOrbitY"] = cameraConfig.invertOrbitY;
            j["orbitSensitivity"] = cameraConfig.orbitSensitivity;
            j["panSensitivity"] = cameraConfig.panSensitivity;
            j["zoomSensitivity"] = cameraConfig.zoomSensitivity;
            j["fovDegrees"] = cameraConfig.fovDegrees;

            std::ofstream f(configPath);
            if (f.is_open())
            {
                f << j.dump(2);
            }
        }
        catch (...) {}
    }

    void CADViewportPanel::ResetCamera()
    {
        cameraTarget = glm::vec3(0.0f, 0.25f, 0.0f);
        cameraDistance = 2.2f;
        cameraYaw = 45.0f;
        cameraPitch = 25.0f;
    }

    ImVec2 CADViewportPanel::ProjectToScreen(const glm::vec3& worldPos, const glm::mat4& viewProj,
        const ImVec2& viewOrigin, const ImVec2& viewSize)
    {
        glm::vec4 clip = viewProj * glm::vec4(worldPos, 1.0f);
        if (clip.w <= 0.01f)
        {
            return ImVec2(-10000.0f, -10000.0f); // Behind near clipping plane
        }

        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        float x = viewOrigin.x + (ndc.x * 0.5f + 0.5f) * viewSize.x;
        float y = viewOrigin.y + (-ndc.y * 0.5f + 0.5f) * viewSize.y;
        return ImVec2(x, y);
    }

    void CADViewportPanel::DrawWireframeBox(ImDrawList* drawList, const glm::vec3& center,
        const glm::vec3& halfExtents, const glm::quat& orientation, const glm::mat4& viewProj,
        const ImVec2& viewOrigin, const ImVec2& viewSize, ImU32 color)
    {
        const glm::vec3 localCorners[8] = {
            glm::vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
            glm::vec3( halfExtents.x, -halfExtents.y, -halfExtents.z),
            glm::vec3( halfExtents.x,  halfExtents.y, -halfExtents.z),
            glm::vec3(-halfExtents.x,  halfExtents.y, -halfExtents.z),
            glm::vec3(-halfExtents.x, -halfExtents.y,  halfExtents.z),
            glm::vec3( halfExtents.x, -halfExtents.y,  halfExtents.z),
            glm::vec3( halfExtents.x,  halfExtents.y,  halfExtents.z),
            glm::vec3(-halfExtents.x,  halfExtents.y,  halfExtents.z)
        };

        ImVec2 pts[8];
        for (int i = 0; i < 8; ++i)
        {
            glm::vec3 worldCorner = center + (orientation * localCorners[i]);
            pts[i] = ProjectToScreen(worldCorner, viewProj, viewOrigin, viewSize);
        }

        auto drawEdge = [&](int i, int j) {
            if (pts[i].x > -5000.0f && pts[j].x > -5000.0f)
            {
                drawList->AddLine(pts[i], pts[j], color, 1.5f);
            }
        };

        // Bottom quad
        drawEdge(0, 1); drawEdge(1, 2); drawEdge(2, 3); drawEdge(3, 0);
        // Top quad
        drawEdge(4, 5); drawEdge(5, 6); drawEdge(6, 7); drawEdge(7, 4);
        // Vertical pillars
        drawEdge(0, 4); drawEdge(1, 5); drawEdge(2, 6); drawEdge(3, 7);
    }

    void CADViewportPanel::DrawJointAxis(ImDrawList* drawList, const glm::vec3& origin,
        const glm::vec3& axis, float lowerLimit, float upperLimit, bool hasLimits,
        bool isRevolute, const glm::mat4& viewProj, const ImVec2& viewOrigin,
        const ImVec2& viewSize, bool isSelected)
    {
        ImU32 axisColor = isSelected ? IM_COL32(255, 215, 0, 255) : IM_COL32(0, 255, 128, 200);
        float lineThick = isSelected ? 3.0f : 2.0f;

        // Draw axis stem
        glm::vec3 normAxis = (glm::length(axis) > 1e-6f) ? glm::normalize(axis) : glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 axisEnd = origin + normAxis * 0.25f;
        ImVec2 p0 = ProjectToScreen(origin, viewProj, viewOrigin, viewSize);
        ImVec2 p1 = ProjectToScreen(axisEnd, viewProj, viewOrigin, viewSize);

        if (p0.x > -5000.0f && p1.x > -5000.0f)
        {
            drawList->AddLine(p0, p1, axisColor, lineThick);
            drawList->AddCircleFilled(p0, 4.0f, axisColor);
        }

        // Draw range of motion arc for revolute joint
        if (isRevolute && hasLimits && isSelected && p0.x > -5000.0f)
        {
            // Build reference perpendicular vector to axis
            glm::vec3 ref(0.0f, 1.0f, 0.0f);
            if (std::abs(glm::dot(ref, normAxis)) > 0.9f)
            {
                ref = glm::vec3(1.0f, 0.0f, 0.0f);
            }
            glm::vec3 u = glm::normalize(glm::cross(normAxis, ref));
            glm::vec3 v = glm::normalize(glm::cross(normAxis, u));

            constexpr int segments = 16;
            float arcRadius = 0.18f;
            float step = (upperLimit - lowerLimit) / static_cast<float>(segments);

            ImVec2 prevPt;
            bool hasPrev = false;

            for (int s = 0; s <= segments; ++s)
            {
                float ang = lowerLimit + s * step;
                glm::vec3 arcPt = origin + (u * std::cos(ang) + v * std::sin(ang)) * arcRadius;
                ImVec2 scr = ProjectToScreen(arcPt, viewProj, viewOrigin, viewSize);
                if (scr.x > -5000.0f)
                {
                    if (hasPrev)
                    {
                        drawList->AddLine(prevPt, scr, IM_COL32(255, 215, 0, 180), 2.0f);
                    }
                    prevPt = scr;
                    hasPrev = true;
                }
                else
                {
                    hasPrev = false;
                }
            }
        }
    }

    void CADViewportPanel::DrawOrientationGizmo(ImDrawList* drawList, const glm::mat4& view,
        const ImVec2& viewOrigin, const ImVec2& viewSize)
    {
        ImVec2 center(viewOrigin.x + 55.0f, viewOrigin.y + viewSize.y - 55.0f);
        constexpr float gizmoRadius = 38.0f;
        drawList->AddCircleFilled(center, gizmoRadius, IM_COL32(20, 22, 28, 210));
        drawList->AddCircle(center, gizmoRadius, IM_COL32(55, 60, 72, 220), 32, 1.5f);

        glm::mat3 rot(view);
        struct AxisGizmo
        {
            glm::vec3 worldDir;
            ImU32 color;
            const char* label;
            float depth;
            ImVec2 scrPt;
        };

        AxisGizmo axes[3] = {
            { glm::vec3(1.0f, 0.0f, 0.0f), IM_COL32(235, 60, 60, 255), "X", 0.0f, ImVec2(0,0) },
            { glm::vec3(0.0f, 1.0f, 0.0f), IM_COL32(60, 230, 60, 255), "Y", 0.0f, ImVec2(0,0) },
            { glm::vec3(0.0f, 0.0f, 1.0f), IM_COL32(70, 140, 255, 255), "Z", 0.0f, ImVec2(0,0) }
        };

        float armLen = 25.0f;
        for (int i = 0; i < 3; ++i)
        {
            glm::vec3 camDir = rot * axes[i].worldDir;
            axes[i].depth = camDir.z;
            axes[i].scrPt = ImVec2(center.x + camDir.x * armLen, center.y - camDir.y * armLen);
        }

        // Sort so farthest axes are drawn first (in LH, larger depth = further away)
        int order[3] = { 0, 1, 2 };
        if (axes[order[0]].depth < axes[order[1]].depth) std::swap(order[0], order[1]);
        if (axes[order[1]].depth < axes[order[2]].depth) std::swap(order[1], order[2]);
        if (axes[order[0]].depth < axes[order[1]].depth) std::swap(order[0], order[1]);

        for (int i = 0; i < 3; ++i)
        {
            const auto& a = axes[order[i]];
            drawList->AddLine(center, a.scrPt, a.color, 2.5f);
            drawList->AddCircleFilled(a.scrPt, 7.0f, a.color);
            drawList->AddText(ImVec2(a.scrPt.x - 3.5f, a.scrPt.y - 6.5f), IM_COL32(15, 15, 20, 255), a.label);
        }
    }

    void CADViewportPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextDisabled("3D CAD PREVIEW VIEWPORT");
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset View"))
        {
            ResetCamera();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Camera Settings"))
        {
            showCameraSettings = !showCameraSettings;
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.6f, 0.7f, 0.85f, 1.0f), "|  Axes: +X Right (Red) | +Y Up (Green) | +Z Front (Blue)");

        // Camera Settings Popover / Window
        if (showCameraSettings)
        {
            ImGui::SetNextWindowSize(ImVec2(360.0f, 330.0f), ImGuiCond_FirstUseEver);
            if (ImGui::Begin("Camera Navigation Settings", &showCameraSettings))
            {
                ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Navigation Preset");
                int currentPreset = static_cast<int>(cameraConfig.preset);
                const char* presets[] = {
                    "Fusion 360 (MMB Pan, Shift+MMB Rotate)",
                    "Blender (MMB Rotate, Shift+MMB Pan)",
                    "Standard CAD (LMB Rotate, RMB Pan)"
                };
                if (ImGui::Combo("##PresetCombo", &currentPreset, presets, IM_ARRAYSIZE(presets)))
                {
                    SetPreset(static_cast<CameraPreset>(currentPreset));
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Controls & Sensitivity");
                if (ImGui::Checkbox("Invert Mouse Wheel Zoom", &cameraConfig.invertScrollZoom))
                {
                    SaveSettings();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Reverse scroll wheel zoom direction.");
                }

                if (ImGui::Checkbox("Invert Orbit (Horizontal X)", &cameraConfig.invertOrbitX))
                {
                    SaveSettings();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Reverse left/right orbit direction.");
                }

                if (ImGui::Checkbox("Invert Orbit (Vertical Y)", &cameraConfig.invertOrbitY))
                {
                    SaveSettings();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Reverse up/down orbit direction.");
                }

                if (ImGui::SliderFloat("Orbit Sensitivity", &cameraConfig.orbitSensitivity, 0.10f, 1.20f, "%.2f"))
                {
                    SaveSettings();
                }
                if (ImGui::SliderFloat("Pan Sensitivity", &cameraConfig.panSensitivity, 0.0005f, 0.0080f, "%.4f"))
                {
                    SaveSettings();
                }
                if (ImGui::SliderFloat("Zoom Sensitivity", &cameraConfig.zoomSensitivity, 0.05f, 0.60f, "%.2f"))
                {
                    SaveSettings();
                }
                if (ImGui::SliderFloat("Field of View", &cameraConfig.fovDegrees, 30.0f, 90.0f, "%.0f deg"))
                {
                    SaveSettings();
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                if (ImGui::Button("Reset View"))
                {
                    ResetCamera();
                }
                ImGui::SameLine();
                if (ImGui::Button("Restore Defaults"))
                {
                    SetPreset(CameraPreset::Fusion360);
                }
                ImGui::SameLine();
                if (ImGui::Button("Close"))
                {
                    showCameraSettings = false;
                }
            }
            ImGui::End();
        }

        ImVec2 viewOrigin = ImGui::GetCursorScreenPos();
        ImVec2 viewSize = ImGui::GetContentRegionAvail();

        if (viewSize.x < 50.0f || viewSize.y < 50.0f)
        {
            return;
        }

        ImGuiIO& io = ImGui::GetIO();

        // Invisible canvas to capture mouse interactions
        ImGui::InvisibleButton("ViewportCanvas", viewSize,
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
        bool isHovered = ImGui::IsItemHovered();

        // 1. Camera Orbit / Pan / Zoom Interaction
        if (isHovered || ImGui::IsItemActive())
        {
            // Zoom via scroll wheel
            if (io.MouseWheel != 0.0f)
            {
                // Inverted (Fusion 360 Inverted / Standard Games): Wheel up (positive) -> Zooms IN (distance decreases)
                float zoomSign = cameraConfig.invertScrollZoom ? -1.0f : 1.0f;
                cameraDistance = std::clamp(cameraDistance + io.MouseWheel * zoomSign * cameraConfig.zoomSensitivity, 0.2f, 25.0f);
            }

            bool doOrbit = false;
            bool doPan = false;

            if (cameraConfig.preset == CameraPreset::Fusion360)
            {
                // Fusion 360: MMB Pan, Shift + MMB Orbit
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
                {
                    if (io.KeyShift)
                    {
                        doOrbit = true;
                    }
                    else
                    {
                        doPan = true;
                    }
                }
            }
            else if (cameraConfig.preset == CameraPreset::Blender)
            {
                // Blender: MMB Orbit, Shift + MMB Pan
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
                {
                    if (io.KeyShift)
                    {
                        doPan = true;
                    }
                    else
                    {
                        doOrbit = true;
                    }
                }
            }
            else if (cameraConfig.preset == CameraPreset::StandardCAD)
            {
                // Standard: LMB Orbit, RMB Pan
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    doOrbit = true;
                }
                else if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
                {
                    doPan = true;
                }
            }

            if (doOrbit)
            {
                float signX = cameraConfig.invertOrbitX ? -1.0f : 1.0f;
                float signY = cameraConfig.invertOrbitY ? -1.0f : 1.0f;
                cameraYaw += signX * io.MouseDelta.x * cameraConfig.orbitSensitivity;
                cameraPitch = std::clamp(cameraPitch + signY * io.MouseDelta.y * cameraConfig.orbitSensitivity, -85.0f, 85.0f);
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            else if (doPan)
            {
                float radYaw = glm::radians(cameraYaw);
                float radPitch = glm::radians(cameraPitch);
                glm::vec3 camPos = cameraTarget + glm::vec3(
                    cameraDistance * std::cos(radPitch) * std::sin(radYaw),
                    cameraDistance * std::sin(radPitch),
                    cameraDistance * std::cos(radPitch) * std::cos(radYaw)
                );
                glm::vec3 forward = glm::normalize(cameraTarget - camPos);
                glm::vec3 right = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), forward));
                glm::vec3 up = glm::normalize(glm::cross(forward, right));

                float panSpeed = cameraConfig.panSensitivity * cameraDistance;
                cameraTarget += (-io.MouseDelta.x * right + io.MouseDelta.y * up) * panSpeed;
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            }
        }

        // 2. Compute Left-Handed View-Projection Matrix (+X Right, +Y Up, +Z Front)
        float radYaw = glm::radians(cameraYaw);
        float radPitch = glm::radians(cameraPitch);
        glm::vec3 camPos = cameraTarget + glm::vec3(
            cameraDistance * std::cos(radPitch) * std::sin(radYaw),
            cameraDistance * std::sin(radPitch),
            cameraDistance * std::cos(radPitch) * std::cos(radYaw)
        );

        glm::mat4 view = glm::lookAtLH(camPos, cameraTarget, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 proj = glm::perspectiveLH(glm::radians(cameraConfig.fovDegrees), viewSize.x / viewSize.y, 0.05f, 50.0f);
        glm::mat4 viewProj = proj * view;

        ImDrawList* drawList = ImGui::GetWindowDrawList();

        // 3. Render Viewport Background & Frame
        drawList->AddRectFilled(viewOrigin, ImVec2(viewOrigin.x + viewSize.x, viewOrigin.y + viewSize.y),
            IM_COL32(24, 25, 30, 255));
        drawList->AddRect(viewOrigin, ImVec2(viewOrigin.x + viewSize.x, viewOrigin.y + viewSize.y),
            IM_COL32(60, 62, 70, 255));

        // 4. Ground Grid (Y = 0.0 plane)
        constexpr float gridHalfSize = 1.2f;
        constexpr float gridStep = 0.20f;
        ImU32 gridColor = IM_COL32(45, 48, 56, 255);

        for (float z = -gridHalfSize; z <= gridHalfSize + 1e-4f; z += gridStep)
        {
            ImVec2 p0 = ProjectToScreen(glm::vec3(-gridHalfSize, 0.0f, z), viewProj, viewOrigin, viewSize);
            ImVec2 p1 = ProjectToScreen(glm::vec3( gridHalfSize, 0.0f, z), viewProj, viewOrigin, viewSize);
            if (p0.x > -5000.0f && p1.x > -5000.0f)
            {
                drawList->AddLine(p0, p1, gridColor);
            }
        }
        for (float x = -gridHalfSize; x <= gridHalfSize + 1e-4f; x += gridStep)
        {
            ImVec2 p0 = ProjectToScreen(glm::vec3(x, 0.0f, -gridHalfSize), viewProj, viewOrigin, viewSize);
            ImVec2 p1 = ProjectToScreen(glm::vec3(x, 0.0f,  gridHalfSize), viewProj, viewOrigin, viewSize);
            if (p0.x > -5000.0f && p1.x > -5000.0f)
            {
                drawList->AddLine(p0, p1, gridColor);
            }
        }

        // 5. Origin Coordinate Triad with Directional Arrowheads (+X Right, +Y Up, +Z Front)
        ImVec2 originPt = ProjectToScreen(glm::vec3(0.0f), viewProj, viewOrigin, viewSize);
        ImVec2 xAxisPt = ProjectToScreen(glm::vec3(0.25f, 0.0f, 0.0f), viewProj, viewOrigin, viewSize);
        ImVec2 yAxisPt = ProjectToScreen(glm::vec3(0.0f, 0.25f, 0.0f), viewProj, viewOrigin, viewSize);
        ImVec2 zAxisPt = ProjectToScreen(glm::vec3(0.0f, 0.0f, 0.25f), viewProj, viewOrigin, viewSize);

        auto drawAxisArrow = [&](const ImVec2& p0, const ImVec2& p1, ImU32 col, const char* label, const ImVec4& textCol)
        {
            if (p0.x <= -5000.0f || p1.x <= -5000.0f) return;
            drawList->AddLine(p0, p1, col, 2.5f);
            ImVec2 dir(p1.x - p0.x, p1.y - p0.y);
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len > 4.0f)
            {
                dir.x /= len; dir.y /= len;
                ImVec2 perp(-dir.y, dir.x);
                float aSize = 8.0f;
                ImVec2 left(p1.x - dir.x * aSize + perp.x * aSize * 0.5f, p1.y - dir.y * aSize + perp.y * aSize * 0.5f);
                ImVec2 right(p1.x - dir.x * aSize - perp.x * aSize * 0.5f, p1.y - dir.y * aSize - perp.y * aSize * 0.5f);
                drawList->AddTriangleFilled(p1, left, right, col);
                drawList->AddText(ImVec2(p1.x + dir.x * 6.0f + 2.0f, p1.y + dir.y * 6.0f - 7.0f), ImGui::ColorConvertFloat4ToU32(textCol), label);
            }
        };

        if (originPt.x > -5000.0f)
        {
            drawAxisArrow(originPt, xAxisPt, IM_COL32(235, 60, 60, 255), "+X (Right)", ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
            drawAxisArrow(originPt, yAxisPt, IM_COL32(60, 235, 60, 255), "+Y (Up)", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
            drawAxisArrow(originPt, zAxisPt, IM_COL32(70, 140, 255, 255), "+Z (Front)", ImVec4(0.45f, 0.70f, 1.0f, 1.0f));
        }

        // 6. Compute Link World Poses via Kinematic Joint Tree in NWU
        if (state.isCadLoaded)
        {
            struct LinkPoseNWU
            {
                glm::dvec3 pos{ 0.0 };
                glm::dquat rot{ 1.0, 0.0, 0.0, 0.0 };
            };
            std::unordered_map<std::string, LinkPoseNWU> linkPosesNWU;

            std::string rootLinkName = "base_link";
            for (const auto& link : state.manifest.links)
            {
                if (link.isRoot)
                {
                    rootLinkName = link.name;
                    break;
                }
            }
            linkPosesNWU[rootLinkName] = { glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0) };

            bool expanded = true;
            while (expanded)
            {
                expanded = false;
                for (const auto& joint : state.manifest.joints)
                {
                    auto pIt = linkPosesNWU.find(joint.parentLink);
                    if (pIt != linkPosesNWU.end() && linkPosesNWU.find(joint.childLink) == linkPosesNWU.end())
                    {
                        glm::dvec3 childPos = pIt->second.pos + (pIt->second.rot * joint.originM);
                        glm::dquat childRot = pIt->second.rot;
                        linkPosesNWU[joint.childLink] = { childPos, childRot };
                        expanded = true;
                    }
                }
            }

            // Flywheel visual links attachment from sidecar if defined
            for (const auto& mech : state.sidecar.mechanisms)
            {
                if ((mech.role == "flywheel" || mech.role == "intake") && !mech.visualLink.empty() && !mech.parentLink.empty())
                {
                    auto pIt = linkPosesNWU.find(mech.parentLink);
                    if (pIt != linkPosesNWU.end() && linkPosesNWU.find(mech.visualLink) == linkPosesNWU.end())
                    {
                        linkPosesNWU[mech.visualLink] = {
                            pIt->second.pos + (pIt->second.rot * mech.mountOffsetM),
                            pIt->second.rot
                        };
                    }
                }
            }

            for (const auto& link : state.manifest.links)
            {
                if (linkPosesNWU.find(link.name) == linkPosesNWU.end())
                {
                    linkPosesNWU[link.name] = { glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0) };
                }
            }

            // Render CAD Link Colliders (+X=Right, +Y=Up, +Z=Front)
            for (const auto& link : state.manifest.links)
            {
                bool isSelected = (state.selectedLink == link.name);
                ImU32 colColor = isSelected ? IM_COL32(255, 215, 0, 255) : IM_COL32(80, 180, 220, 180);
                const auto& lPose = linkPosesNWU[link.name];

                for (const auto& col : link.colliders)
                {
                    glm::dvec3 colCenterNWU = lPose.pos + (lPose.rot * col.originM);
                    glm::dquat colRotNWU = lPose.rot * col.rotationQuat;

                    glm::vec3 worldColCenter = RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(colCenterNWU));
                    glm::quat worldColRot = RobotSchemaLoader::ConvertNWUQuatToRobotFrame(glm::quat(colRotNWU));
                    glm::vec3 dims = glm::abs(RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(col.dimensionsM)));
                    glm::vec3 halfExtents = dims * 0.5f;

                    if (halfExtents.x < 0.005f) halfExtents.x = 0.02f;
                    if (halfExtents.y < 0.005f) halfExtents.y = 0.02f;
                    if (halfExtents.z < 0.005f) halfExtents.z = 0.02f;

                    DrawWireframeBox(drawList, worldColCenter, halfExtents, worldColRot, viewProj, viewOrigin, viewSize, colColor);
                }
            }

            // 6b. Render Virtual Bumpers in Viewport (+X=Right, +Y=Up, +Z=Front)
            if (state.sidecar.bumpers.enabled)
            {
                const auto& b = state.sidecar.bumpers;
                std::string bTarget = b.targetLink.empty() ?
                    (!state.manifest.links.empty() ? state.manifest.links[0].name : "") :
                    b.targetLink;

                auto targetIt = linkPosesNWU.find(bTarget);
                if (targetIt != linkPosesNWU.end())
                {
                    const auto& tPose = targetIt->second;
                    glm::dvec3 tPosRobot = RobotSchemaLoader::ConvertNWUToRobotFrame(tPose.pos);
                    glm::dquat tRotRobot = RobotSchemaLoader::ConvertNWUQuatToRobotFrame(tPose.rot);

                    bool isBlue = (b.color == "blue");
                    bool isTabActive = (state.activeInspectorTab == ConfiguratorState::ActiveInspectorTab::Bumpers);
                    ImU32 bumperColor = isTabActive ?
                        (isBlue ? IM_COL32(80, 160, 255, 255) : IM_COL32(255, 80, 80, 255)) :
                        (isBlue ? IM_COL32(40, 120, 240, 220) : IM_COL32(235, 45, 45, 220));

                    std::vector<std::pair<glm::dvec3, glm::dvec3>> renderBoxesRobot;
                    auto addBumperBox = [&](const glm::dvec3& origRob, const glm::dvec3& dimsRob)
                    {
                        renderBoxesRobot.push_back({ origRob, glm::abs(dimsRob) });
                    };

                    if (b.shape == "box")
                    {
                        addBumperBox(b.offsetM, glm::dvec3(b.widthM, b.heightM, b.lengthM));
                    }
                    else if (b.shape == "perimeter")
                    {
                        // Front (+Z)
                        addBumperBox(
                            glm::dvec3(b.offsetM.x, b.offsetM.y, b.offsetM.z + b.lengthM * 0.5 - b.thicknessM * 0.5),
                            glm::dvec3(b.widthM, b.heightM, b.thicknessM));
                        // Back (-Z)
                        addBumperBox(
                            glm::dvec3(b.offsetM.x, b.offsetM.y, b.offsetM.z - b.lengthM * 0.5 + b.thicknessM * 0.5),
                            glm::dvec3(b.widthM, b.heightM, b.thicknessM));
                        // Left (-X)
                        double sideLen = std::max(0.01, b.lengthM - 2.0 * b.thicknessM);
                        addBumperBox(
                            glm::dvec3(b.offsetM.x - b.widthM * 0.5 + b.thicknessM * 0.5, b.offsetM.y, b.offsetM.z),
                            glm::dvec3(b.thicknessM, b.heightM, sideLen));
                        // Right (+X)
                        addBumperBox(
                            glm::dvec3(b.offsetM.x + b.widthM * 0.5 - b.thicknessM * 0.5, b.offsetM.y, b.offsetM.z),
                            glm::dvec3(b.thicknessM, b.heightM, sideLen));
                    }
                    else if (b.shape == "u_shape")
                    {
                        // Back (-Z)
                        addBumperBox(
                            glm::dvec3(b.offsetM.x, b.offsetM.y, b.offsetM.z - b.lengthM * 0.5 + b.thicknessM * 0.5),
                            glm::dvec3(b.widthM, b.heightM, b.thicknessM));
                        // Left (-X)
                        double sideLen = std::max(0.01, b.lengthM - 2.0 * b.thicknessM);
                        addBumperBox(
                            glm::dvec3(b.offsetM.x - b.widthM * 0.5 + b.thicknessM * 0.5, b.offsetM.y, b.offsetM.z),
                            glm::dvec3(b.thicknessM, b.heightM, sideLen));
                        // Right (+X)
                        addBumperBox(
                            glm::dvec3(b.offsetM.x + b.widthM * 0.5 - b.thicknessM * 0.5, b.offsetM.y, b.offsetM.z),
                            glm::dvec3(b.thicknessM, b.heightM, sideLen));
                        // Front corners if cutout is narrower than the inside width
                        double innerWidth = b.widthM - 2.0 * b.thicknessM;
                        if (b.cutoutWidthM < innerWidth && innerWidth > 0.02)
                        {
                            double cornerW = (innerWidth - b.cutoutWidthM) * 0.5 + b.thicknessM;
                            // Front-Left (-X, +Z)
                            addBumperBox(
                                glm::dvec3(b.offsetM.x - (b.widthM - cornerW) * 0.5, b.offsetM.y, b.offsetM.z + b.lengthM * 0.5 - b.thicknessM * 0.5),
                                glm::dvec3(cornerW, b.heightM, b.thicknessM));
                            // Front-Right (+X, +Z)
                            addBumperBox(
                                glm::dvec3(b.offsetM.x + (b.widthM - cornerW) * 0.5, b.offsetM.y, b.offsetM.z + b.lengthM * 0.5 - b.thicknessM * 0.5),
                                glm::dvec3(cornerW, b.heightM, b.thicknessM));
                        }
                    }

                    for (const auto& box : renderBoxesRobot)
                    {
                        glm::dvec3 worldCenterRob = tPosRobot + (tRotRobot * box.first);
                        glm::dquat worldRotRob = tRotRobot;

                        glm::vec3 worldCenter = glm::vec3(worldCenterRob);
                        glm::quat worldRot = glm::quat(worldRotRob);
                        glm::vec3 dims = glm::vec3(box.second);
                        glm::vec3 halfExtents = dims * 0.5f;

                        DrawWireframeBox(drawList, worldCenter, halfExtents, worldRot, viewProj, viewOrigin, viewSize, bumperColor);
                    }
                }
            }

            // 7. Render Kinematic Joints & Motion Arcs (+X=Right, +Y=Up, +Z=Front)
            for (const auto& joint : state.manifest.joints)
            {
                bool isSelected = (state.selectedJoint == joint.name);
                auto pIt = linkPosesNWU.find(joint.parentLink);
                glm::dvec3 pPos = (pIt != linkPosesNWU.end()) ? pIt->second.pos : glm::dvec3(0.0);
                glm::dquat pRot = (pIt != linkPosesNWU.end()) ? pIt->second.rot : glm::dquat(1.0, 0.0, 0.0, 0.0);

                glm::dvec3 worldJointAnchorNWU = pPos + (pRot * joint.originM);
                glm::dvec3 worldJointAxisNWU = pRot * joint.axis;

                glm::vec3 worldAnchor = RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(worldJointAnchorNWU));
                glm::vec3 worldAxis = RobotSchemaLoader::ConvertNWUToRobotFrame(glm::vec3(worldJointAxisNWU));

                bool isRev = (joint.type == "revolute" || joint.type == "continuous");
                DrawJointAxis(drawList, worldAnchor, worldAxis,
                    static_cast<float>(joint.limits.lower),
                    static_cast<float>(joint.limits.upper),
                    joint.limits.hasLimits,
                    isRev,
                    viewProj, viewOrigin, viewSize, isSelected);
            }
        }

        // 8. Draw Orientation Triad Gizmo (Bottom-Left)
        DrawOrientationGizmo(drawList, view, viewOrigin, viewSize);

        // 9. Viewport HUD Overlay
        const char* navHelp = "Fusion 360: MMB Pan | Shift+MMB Rotate | Scroll Zoom";
        if (cameraConfig.preset == CameraPreset::Blender)
        {
            navHelp = "Blender: MMB Rotate | Shift+MMB Pan | Scroll Zoom";
        }
        else if (cameraConfig.preset == CameraPreset::StandardCAD)
        {
            navHelp = "Standard CAD: LMB Rotate | RMB Pan | Scroll Zoom";
        }

        drawList->AddText(ImVec2(viewOrigin.x + 10.0f, viewOrigin.y + 10.0f),
            IM_COL32(200, 205, 215, 255), navHelp);

        if (!state.selectedMechanism.empty())
        {
            std::string selText = "Mechanism: " + state.selectedMechanism;
            if (!state.selectedJoint.empty()) selText += " [Joint: " + state.selectedJoint + "]";
            if (!state.selectedLink.empty()) selText += " [Link: " + state.selectedLink + "]";
            drawList->AddText(ImVec2(viewOrigin.x + 10.0f, viewOrigin.y + 28.0f),
                IM_COL32(255, 215, 0, 255), selText.c_str());
        }
        else if (!state.selectedLink.empty())
        {
            std::string selText = "Selected Link: " + state.selectedLink;
            drawList->AddText(ImVec2(viewOrigin.x + 10.0f, viewOrigin.y + 28.0f),
                IM_COL32(255, 215, 0, 255), selText.c_str());
        }
        else if (!state.selectedJoint.empty())
        {
            std::string selText = "Selected Joint: " + state.selectedJoint;
            drawList->AddText(ImVec2(viewOrigin.x + 10.0f, viewOrigin.y + 28.0f),
                IM_COL32(255, 215, 0, 255), selText.c_str());
        }
    }
}
