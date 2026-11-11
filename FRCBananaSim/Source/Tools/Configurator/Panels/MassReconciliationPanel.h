/**
 * @file MassReconciliationPanel.h
 * @brief Subsystem inspector panel for physical bench mass reconciliation and unmodeled ballast.
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class MassReconciliationPanel
     * @brief Form editor for scale bench weight calibration, proportional link scaling, and ballast offsets.
     */
    class MassReconciliationPanel
    {
    public:
        MassReconciliationPanel() = default;

        /**
         * @brief Renders the bench mass reconciliation and ballast panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        float benchMassKg{ 54.31f };
        int overrideLinkIdx{ 0 };
        float overrideMassKg{ 1.0f };
    };
}
