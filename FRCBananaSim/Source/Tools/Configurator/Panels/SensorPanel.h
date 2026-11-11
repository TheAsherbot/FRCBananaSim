/**
 * @file SensorPanel.h
 * @brief Subsystem inspector panel for robot sensors (IMUs, CANcoders, Beam Breaks, Limit Switches, Analog Inputs).
 */

#pragma once

namespace FRCBananaSim::Tools::Configurator
{
    class ConfiguratorState;

    /**
     * @class SensorPanel
     * @brief Form editor for configuring IMUs, magnetic encoders, beam breaks, limit switches, and analog inputs.
     */
    class SensorPanel
    {
    public:
        SensorPanel() = default;

        /**
         * @brief Renders the complete Sensor Suite configuration panel.
         * @param state Active configurator state.
         */
        void Render(ConfiguratorState& state);

    private:
        char newSensorNameBuf[64]{ "NewSensor" };
        int newSensorTypeIdx{ 0 }; // 0: imu_pigeon2, 1: imu_navx, 2: cancoder, 3: beam_break, 4: limit_switch, 5: analog_input, 6: laser_can, 7: color_sensor, 8: custom
        char searchFilterBuf[64]{ "" };
    };
}
