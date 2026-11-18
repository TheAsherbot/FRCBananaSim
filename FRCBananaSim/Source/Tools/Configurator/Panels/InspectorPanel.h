/**
 * @file InspectorPanel.h
 * @brief Right-dock property inspector for selected CAD links, joints, and electromechanical components.
 */

#pragma once

#include <memory>

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;
    class SwervePanel;
    class MechanismPanel;
    class CouplingsPanel;
    class SensorPanel;
    class MassReconciliationPanel;
    class BumperPanel;
    class ContactZonePanel;

    class InspectorPanel
    {
    public:
        InspectorPanel();
        ~InspectorPanel();

        /**
         * @brief Renders the property inspector for active selection or active subsystem tab.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        void RenderCADSelection(ConfiguratorState& state);

        int lastRenderedTab{ 0 };
        std::unique_ptr<SwervePanel> swervePanel;
        std::unique_ptr<MechanismPanel> mechanismPanel;
        std::unique_ptr<CouplingsPanel> couplingsPanel;
        std::unique_ptr<SensorPanel> sensorPanel;
        std::unique_ptr<MassReconciliationPanel> massReconciliationPanel;
        std::unique_ptr<BumperPanel> bumperPanel;
        std::unique_ptr<ContactZonePanel> contactZonePanel;
    };
}
