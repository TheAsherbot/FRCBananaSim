/**
 * @file LinterPanel.cpp
 * @brief Implementation of the bottom-dock live schema linter panel.
 */

#include "Tools/Configurator/Panels/LinterPanel.h"
#include "Tools/Configurator/ConfiguratorState.h"

#include <imgui/imgui.h>

namespace FRCBananaSim::Tools::Configurator
{
    void LinterPanel::Render(ConfiguratorState& state)
    {
        ImGui::TextDisabled("LIVE SCHEMA LINTER & DIAGNOSTICS");
        ImGui::SameLine(0.0f, 15.0f);

        if (ImGui::SmallButton("Re-validate"))
        {
            state.RunLinter();
        }

        ImGui::SameLine(0.0f, 15.0f);
        if (state.linterErrors.empty() && state.linterWarnings.empty())
        {
            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "[OK] Schema v1.0.0 Valid | 0 Errors | 0 Warnings");
        }
        else if (!state.linterErrors.empty())
        {
            ImGui::TextColored(ImVec4(0.95f, 0.3f, 0.3f, 1.0f), "[ERROR] %zu Schema Errors Detected", state.linterErrors.size());
        }
        else
        {
            ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.2f, 1.0f), "[WARNING] %zu Schema Warnings Detected", state.linterWarnings.size());
        }

        ImGui::Separator();

        // Status bar message
        ImGui::TextDisabled("Status: %s", state.statusMessage.c_str());

        // Error / warning lists in child window
        ImGui::BeginChild("LinterMessageList", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_HorizontalScrollbar);

        for (const auto& err : state.linterErrors)
        {
            ImGui::TextColored(ImVec4(0.95f, 0.3f, 0.3f, 1.0f), "[ERROR]   %s", err.c_str());
        }

        for (const auto& warn : state.linterWarnings)
        {
            ImGui::TextColored(ImVec4(0.95f, 0.8f, 0.2f, 1.0f), "[WARNING] %s", warn.c_str());
        }

        if (state.linterErrors.empty() && state.linterWarnings.empty())
        {
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "No validation errors or topology conflicts found.");
        }

        ImGui::EndChild();
    }
}
