/**
 * @file ContactZonePanel.h
 * @brief Inspector panel for authoring, inspecting, and editing game-piece contact zones (intakes, shooters, indexers).
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class ContactZonePanel
     * @brief Inspector panel for authoring, inspecting, and editing game-piece contact zones.
     */
    class ContactZonePanel
    {
    public:
        ContactZonePanel() = default;

        /**
         * @brief Renders the contact zone authoring and editing form in the active ImGui inspector.
         * @param state Active configurator document state.
         */
        void Render(ConfiguratorState& state);
    };
}
