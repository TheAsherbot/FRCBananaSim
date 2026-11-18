/**
 * @file MechanismPanel.h
 * @brief Subsystem inspector panel for 1-DOF robot mechanisms (elevators, arms, wrists, flywheels, intakes).
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class MechanismPanel
     * @brief Form editor for configuring 1-DOF mechanisms, motor/servo presets, gear ratios, controllers, and kinematics.
     */
    class MechanismPanel
    {
    public:
        MechanismPanel() = default;

        /**
         * @brief Renders the complete Mechanisms configuration panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        char newMechNameBuf[64]{ "NewMechanism" };
        int newMechRoleIdx{ 0 }; // 0: elevator, 1: arm, 2: wrist, 3: flywheel, 4: intake
    };
}
