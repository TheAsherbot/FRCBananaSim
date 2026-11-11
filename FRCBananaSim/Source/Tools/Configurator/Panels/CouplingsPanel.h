/**
 * @file CouplingsPanel.h
 * @brief Subsystem inspector panel for mechanical couplings, cascade elevators, and four-bar linkages.
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class CouplingsPanel
     * @brief Form editor for configuring power couplings, multi-stage elevator cascades, and four-bar loop closures.
     */
    class CouplingsPanel
    {
    public:
        CouplingsPanel() = default;

        /**
         * @brief Renders the complete Couplings and Linkages configuration panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        char newCouplingNameBuf[64]{ "NewCoupling" };
        int newCouplingTypeIdx{ 0 }; // 0: power_coupling, 1: elevator_cascade, 2: four_bar_loop
    };
}
