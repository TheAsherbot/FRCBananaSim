/** *
@file SidecarConfiguratorApplication.cpp
 * @brief Standalone Dear ImGui application for the FRC Sidecar Configurator & CAD Ingestion Suite.
 */

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui/imgui.h>
#include <imgui/backend/imgui_impl_glfw.h>
#include <imgui/backend/imgui_impl_opengl3.h>

#include <fstream>
#include <filesystem>
#include <iostream>

#include "Tools/Configurator/ConfiguratorState.h"
#include "Tools/Configurator/Panels/CADHierarchyPanel.h"
#include "Tools/Configurator/Panels/CADViewportPanel.h"
#include "Tools/Configurator/Panels/InspectorPanel.h"
#include "Tools/Configurator/Panels/LinterPanel.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef APIENTRY
#undef APIENTRY
#endif
#include <windows.h>
#include <shlobj.h>

static std::string ShowBrowseFolderDialog()
{
    std::string result;
    BROWSEINFOA bi = { 0 };
    bi.lpszTitle = "Select CAD Robot Folder (containing manifest.json)";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolderA(&bi);
    if (pidl != nullptr)
    {
        char path[MAX_PATH];
        if (SHGetPathFromIDListA(pidl, path))
 
        {
            result = path;
        }
        CoTaskMemFree(pidl);
    }
    return result;
}
#endif

#include <filesystem>
#include <iostream>

using namespace FRCBananaSim::Tools::Configurator;

int main(int argc, char* argv[])
{
    // 1. Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(1600, 920, "FRC Banana Simulator — Sidecar Configurator & CAD Ingestion Suite", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable VSync

    // 2. Initialize GLAD
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // 3. Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.0f);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.15f, 0.15f, 0.18f, 1.0f);
    style.Colors[ImGuiCol_Header] = ImVec4(0.24f, 0.28f, 0.38f, 1.0f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.30f, 0.36f, 0.48f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.22f, 0.28f, 0.40f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.36f, 0.52f, 1.0f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // 4. Instantiate Application State & Panels
    ConfiguratorState state;
    CADHierarchyPanel hierarchyPanel;
    CADViewportPanel viewportPanel;
    InspectorPanel inspectorPanel;
    LinterPanel linterPanel;

    // Buffer for folder opening
    char folderPathBuf[512] = "FRCBananaSim/Resources/Robots/Reference2026";

    // Discover available robot folders in Resources/Robots
    std::string robotsDir = "FRCBananaSim/Resources/Robots";
    if (!std::filesystem::exists(robotsDir))
    {
        robotsDir = "../FRCBananaSim/Resources/Robots";
    }

    std::vector<std::pair<std::string, std::string>> availableRobots;
    if (std::filesystem::exists(robotsDir))
    {
        for (const auto& entry : std::filesystem::directory_iterator(robotsDir))
        {
            if (entry.is_directory() && std::filesystem::exists(entry.path() / "manifest.json"))
            {
                availableRobots.push_back({ entry.path().filename().string(), entry.path().string() });
            }
        }
    }

    // Load recent robots history (persisted in configurator_settings.json)
    std::vector<std::string> recentRobots = ConfiguratorState::GetRecentRobots();

    int selectedRobotIndex = 0;
    std::string initialRobotPath;
    if (!recentRobots.empty())
    {
        initialRobotPath = recentRobots[0];
    }
    else
    {
        for (size_t i = 0; i < availableRobots.size(); ++i)
        {
            if (availableRobots[i].first == "Reference2026")
            {
                selectedRobotIndex = static_cast<int>(i);
                initialRobotPath = availableRobots[i].second;
                break;
            }
        }
        if (initialRobotPath.empty() && !availableRobots.empty())
        {
            initialRobotPath = availableRobots[0].second;
        }
    }

    auto openRobot = [&](const std::string& path) {
        if (path.empty()) return;
        state.LoadCADFolder(path);
        std::snprintf(folderPathBuf, sizeof(folderPathBuf), "%s", path.c_str());

        // Update MRU and persist
        ConfiguratorState::AddRecentRobot(path);
        recentRobots = ConfiguratorState::GetRecentRobots();

        // Sync selectedRobotIndex with availableRobots if matching
        for (size_t i = 0; i < availableRobots.size(); ++i)
        {
            if (ConfiguratorState::ArePathsEquivalent(availableRobots[i].second, path) || availableRobots[i].first == path)
            {
                selectedRobotIndex = static_cast<int>(i);
                break;
            }
        }
    };

    // Auto-load initial robot if available
    if (!initialRobotPath.empty() && std::filesystem::exists(initialRobotPath))
    {
        openRobot(initialRobotPath);
    }
    else if (!availableRobots.empty())
    {
        openRobot(availableRobots[selectedRobotIndex].second);
    }

    // 5. Main Application Loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int fbWidth = 0, fbHeight = 0;
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

        // Fill entire screen with root dock frame
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(fbWidth), static_cast<float>(fbHeight)));
        ImGuiWindowFlags rootFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar;

        ImGui::Begin("RootConfiguratorWindow", nullptr, rootFlags);

        // -------------------------------------------------------------
        // A. Top Toolbar (Height ~ 44px)
        // -------------------------------------------------------------
        ImGui::BeginChild("TopToolbar", ImVec2(0.0f, 44.0f), false);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("CAD Folder:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(240.0f);
        ImGui::InputText("##FolderPath", folderPathBuf, sizeof(folderPathBuf));
        ImGui::SameLine();
        if (ImGui::Button("Open"))
        {
            openRobot(folderPathBuf);
        }
#ifdef _WIN32
        ImGui::SameLine();
        if (ImGui::Button("Browse..."))
        {
            std::string picked = ShowBrowseFolderDialog();
            if (!picked.empty())
            {
                openRobot(picked);
            }
        }
#endif

        if (!recentRobots.empty())
        {
            ImGui::SameLine(0.0f, 12.0f);
            ImGui::Text("Recent:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(170.0f);

            int selectedRecentIndex = -1;
            std::vector<std::string> recentDisplayLabels;
            std::vector<const char*> recentItems;

            for (size_t i = 0; i < recentRobots.size(); ++i)
            {
                std::filesystem::path p(recentRobots[i]);
                std::string stem = p.filename().string();
                if (stem.empty()) stem = p.string();
                recentDisplayLabels.push_back(std::to_string(i + 1) + ". " + stem);
                if (ConfiguratorState::ArePathsEquivalent(recentRobots[i], folderPathBuf))
                {
                    selectedRecentIndex = static_cast<int>(i);
                }
            }
            for (const auto& lbl : recentDisplayLabels)
            {
                recentItems.push_back(lbl.c_str());
            }

            const char* previewText = (selectedRecentIndex >= 0) ? recentItems[selectedRecentIndex] : "Select Recent...";
            if (ImGui::BeginCombo("##RecentRobotsCombo", previewText))
            {
                for (int i = 0; i < static_cast<int>(recentItems.size()); ++i)
                {
                    const bool isSelected = (selectedRecentIndex == i);
                    if (ImGui::Selectable(recentItems[i], isSelected))
                    {
                        openRobot(recentRobots[i]);
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("%s", recentRobots[i].c_str());
                    }
                    if (isSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Quickly switch between the last 3 opened robot folders without typing.");
            }
        }

        if (!availableRobots.empty())
        {
            ImGui::SameLine(0.0f, 12.0f);
            ImGui::Text("Samples:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            std::vector<const char*> robotNames;
            for (const auto& r : availableRobots)
            {
                robotNames.push_back(r.first.c_str());
            }
            if (ImGui::Combo("##PresetRobots", &selectedRobotIndex, robotNames.data(), static_cast<int>(robotNames.size())))
            {
                const auto& chosen = availableRobots[selectedRobotIndex];
                openRobot(chosen.second);
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Included sample CAD assemblies in Resources/Robots");
            }
        }

        ImGui::SameLine(0.0f, 12.0f);
        if (ImGui::Button("Camera Settings"))
        {
            viewportPanel.OpenCameraSettings();
        }
        ImGui::SameLine();
        if (ImGui::Button("Auto-Generate Starter"))
        {
            state.AutoGenerateStarterSidecar();
        }
        ImGui::SameLine();
        if (ImGui::Button("Save sidecar.json"))
        {
            state.SaveSidecar();
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear"))
        {
            state.Clear();
        }

        ImGui::EndChild();
        ImGui::Separator();

        // -------------------------------------------------------------
        // B. Main Middle Area (3 Columns: Hierarchy, Viewport, Inspector)
        // -------------------------------------------------------------
        float bottomHeight = 150.0f;
        float middleHeight = static_cast<float>(fbHeight) - 44.0f - bottomHeight - 24.0f;
        if (middleHeight < 200.0f) middleHeight = 200.0f;

        float hierarchyWidth = 340.0f;
        float inspectorWidth = 490.0f;
        float viewportWidth = static_cast<float>(fbWidth) - hierarchyWidth - inspectorWidth - 28.0f;
        if (viewportWidth < 200.0f) viewportWidth = 200.0f;

        // 1. Left Column: CAD Hierarchy
        ImGui::BeginChild("HierarchyChild", ImVec2(hierarchyWidth, middleHeight), true);
        hierarchyPanel.Render(state);
        ImGui::EndChild();

        ImGui::SameLine();

        // 2. Middle Column: 3D CAD Preview Viewport
        ImGui::BeginChild("ViewportChild", ImVec2(viewportWidth, middleHeight), true);
        viewportPanel.Render(state);
        ImGui::EndChild();

        ImGui::SameLine();

        // 3. Right Column: Subsystem Inspector
        ImGui::BeginChild("InspectorChild", ImVec2(0.0f, middleHeight), true);
        inspectorPanel.Render(state);
        ImGui::EndChild();

        ImGui::Separator();

        // -------------------------------------------------------------
        // C. Bottom Dock: Diagnostics & Schema Linter
        // -------------------------------------------------------------
        ImGui::BeginChild("LinterChild", ImVec2(0.0f, 0.0f), true);
        linterPanel.Render(state);
        ImGui::EndChild();

        ImGui::End(); // RootConfiguratorWindow

        // 6. Presentation
        ImGui::Render();
        glViewport(0, 0, fbWidth, fbHeight);
        glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // 7. Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
