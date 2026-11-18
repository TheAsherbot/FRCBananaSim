/**
 * @file CADViewportPanel.h
 * @brief Interactive 3D CAD preview viewport with orbit/pan/zoom camera and joint axis visualization.
 */

#pragma once

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <imgui/imgui.h>
#include <string>

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @enum CameraPreset
     * @brief Preconfigured 3D CAD mouse navigation paradigms.
     */
    enum class CameraPreset
    {
        Fusion360 = 0,    ///< Pan: MMB, Rotate: Shift + MMB, Zoom: Wheel (Scroll up zooms in)
        Blender,          ///< Pan: Shift + MMB, Rotate: MMB, Zoom: Wheel
        StandardCAD       ///< Pan: RMB, Rotate: LMB, Zoom: Wheel
    };

    /**
     * @enum CanonicalView
     * @brief Standard orthogonal viewpoints (+X Right, -X Left, +Y Top, -Y Bottom, +Z Front, -Z Back).
     */
    enum class CanonicalView
    {
        None = 0,
        Right,  ///< +X (Looking from Right to Left)
        Left,   ///< -X (Looking from Left to Right)
        Top,    ///< +Y (Looking from Top down)
        Bottom, ///< -Y (Looking from Bottom up)
        Front,  ///< +Z (Looking from Front to Back)
        Back    ///< -Z (Looking from Back to Front)
    };

    /**
     * @struct CameraConfig
     * @brief Runtime viewport camera navigation, sensitivity, and perspective options.
     */
    struct CameraConfig
    {
        CameraPreset preset = CameraPreset::Fusion360;
        bool invertScrollZoom = true;   ///< True (Default): Scroll up zooms in, scroll down zooms out
        bool invertOrbitX = false;      ///< When false, dragging right orbits model right (turntable)
        bool invertOrbitY = false;      ///< When false, dragging down tilts model down
        float orbitSensitivity = 0.40f;
        float panSensitivity = 0.0025f;
        float zoomSensitivity = 0.20f;
        float fovDegrees = 45.0f;
        bool isOrthographic = false;    ///< False: 3D Perspective, True: 2D Orthographic (isometric/flat)
    };
    class CADViewportPanel
    {
    public:
        CADViewportPanel();


        /**
         * @brief Renders the interactive 3D CAD preview viewport.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

        /**
         * @brief Resets the camera to default perspective viewing the robot chassis.
         */
        void ResetCamera();

        /**
         * @brief Switches the camera control scheme to a specific preset.
         * @param preset The desired navigation preset.
         * @param save Whether to persist to settings JSON immediately.
         */
        void SetPreset(CameraPreset preset, bool save = true);

        /**
         * @brief Opens the camera navigation settings configuration dialog.
         */
        void OpenCameraSettings() { showCameraSettings = true; }

        /**
         * @brief Returns mutable reference to active camera configuration.
         */
        CameraConfig& GetCameraConfig() { return cameraConfig; }

        /**
         * @brief Returns const reference to active camera configuration.
         */
        const CameraConfig& GetCameraConfig() const { return cameraConfig; }

        /**
         * @brief Snaps camera to an exact orthogonal viewpoint (+X, -X, +Y, -Y, +Z, -Z).
         * @param view Canonical view direction.
         * @param makeOrtho If true (default), automatically switches projection to orthographic.
         */
        void SetCanonicalView(CanonicalView view, bool makeOrtho = true);

        /**
         * @brief Toggles between Perspective and Orthographic projection.
         */
        void ToggleOrthographic();

        /**
         * @brief Returns true if viewport is in Orthographic projection mode.
         */
        bool IsOrthographic() const;

        /**
         * @brief Returns human-readable label for current viewpoint and projection mode.
         */
        std::string GetCurrentViewName() const;

        /**
         * @brief Loads persisted camera settings from JSON configuration.
         */
        void LoadSettings(const std::string& filepath = "");

        /**
         * @brief Persists active camera settings to JSON configuration.
         */
        void SaveSettings(const std::string& filepath = "") const;

        // Camera pose accessors
        float GetYaw() const { return cameraYaw; }
        float GetPitch() const { return cameraPitch; }
        float GetDistance() const { return cameraDistance; }
        const glm::vec3& GetTarget() const { return cameraTarget; }

        void SetYaw(float yaw) { cameraYaw = yaw; }
        void SetPitch(float pitch) { cameraPitch = pitch; }
        void SetDistance(float dist) { cameraDistance = dist; }
        void SetTarget(const glm::vec3& target) { cameraTarget = target; }

        /**
         * @brief Renders the interactive orientation gizmo in the CAD viewport.
         * @return True if a gizmo axis bubble was clicked this frame.
         */
        bool DrawOrientationGizmo(ImDrawList* drawList, const glm::mat4& view, const ImVec2& viewOrigin, const ImVec2& viewSize);

    private:
        glm::vec3 cameraTarget{ 0.0f, 0.25f, 0.0f };
        float cameraDistance{ 2.2f };
        float cameraYaw{ 45.0f };   // Degrees
        float cameraPitch{ 25.0f }; // Degrees

        CameraConfig cameraConfig;
        bool showCameraSettings{ false };

        ImVec2 ProjectToScreen(const glm::vec3& worldPos, const glm::mat4& viewProj, const ImVec2& viewOrigin, const ImVec2& viewSize);
        void DrawWireframeBox(ImDrawList* drawList, const glm::vec3& center, const glm::vec3& halfExtents, const glm::quat& orientation, const glm::mat4& viewProj, const ImVec2& viewOrigin, const ImVec2& viewSize, ImU32 color);
        void DrawJointAxis(ImDrawList* drawList, const glm::vec3& origin, const glm::vec3& axis, float lowerLimit, float upperLimit, bool hasLimits, bool isRevolute, const glm::mat4& viewProj, const ImVec2& viewOrigin, const ImVec2& viewSize, bool isSelected);
    };
}

