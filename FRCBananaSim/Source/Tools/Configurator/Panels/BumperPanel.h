/**
 * @file BumperPanel.h
 * @brief Inspector panel for configuring FRC bumpers, collision geometry, physical mass, and visual presets.
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class BumperPanel
     * @brief Inspector panel for configuring FRC bumpers, collision geometry, physical mass, and visual presets.
     */
    class BumperPanel
    {
    public:
        BumperPanel() = default;

        /**
         * @brief Renders the bumper configuration form in the active ImGui window.
         * @param state Active configurator document state.
         */
        void Render(ConfiguratorState& state);
    };
}
