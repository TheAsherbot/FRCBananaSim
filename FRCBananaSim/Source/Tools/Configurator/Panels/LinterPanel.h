/**
 * @file LinterPanel.h
 * @brief Bottom-dock diagnostics and live schema linter panel.
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    class LinterPanel
    {
    public:
        LinterPanel() = default;

        /**
         * @brief Renders the live schema linter diagnostics panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);
    };
}
