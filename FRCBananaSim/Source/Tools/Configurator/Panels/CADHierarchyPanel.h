/**
 * @file CADHierarchyPanel.h
 * @brief Left-dock hierarchy tree view for CAD links, colliders, joints, and contact zones.
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    class CADHierarchyPanel
    {
    public:
        CADHierarchyPanel() = default;

        /**
         * @brief Renders the CAD hierarchy tree view within an ImGui panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        char searchBuffer[64]{ 0 };
    };
}
