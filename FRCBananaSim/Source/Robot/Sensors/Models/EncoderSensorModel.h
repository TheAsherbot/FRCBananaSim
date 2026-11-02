/**
 * @file EncoderSensorModel.h
 * @brief Transducer model converting continuous joint travel into quantized encoder ticks with velocity windowing.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace FRCBananaSim::Robot::Sensors::Models
{
    /**
     * @class EncoderSensorModel
     * @brief Models gear scaling, CPR quantization, zero offsets, and finite-difference velocity window filtering.
     */
    class EncoderSensorModel
    {
    public:
        /**
         * @brief Constructs an encoder transducer model.
         * @param cpr Counts per revolution (e.g. 4096 for CANcoder, 42 for NEO hall sensor; 0 for unquantized).
         * @param gearRatio Sensor-to-mechanism reduction ratio (sensor rotations per mechanism rotation).
         * @param inverted Inverts direction if true.
         * @param offsetRad Angular zero calibration offset in radians.
         * @param windowSize Number of historical physics ticks used for finite-difference velocity calculation.
         */
        explicit EncoderSensorModel(uint32_t cpr = 4096,
            double gearRatio = 1.0,
            bool inverted = false,
            double offsetRad = 0.0,
            uint32_t windowSize = 4);

        /**
         * @brief Updates encoder state from continuous joint position and velocity.
         * @param jointPosition Mechanism coordinate in radians or meters.
         * @param jointVelocity Mechanism velocity in rad/s or m/s.
         * @param dt Timestep duration in seconds.
         */
        void Update(double jointPosition, double jointVelocity, double dt);

        /**
         * @brief Resets encoder historical state and ring buffer.
         * @param jointPosition Optional reset position.
         */
        void Reset(double jointPosition = 0.0);

        // Accessors
        double GetPositionRad() const;
        double GetVelocityRadPerSec() const;
        double GetPositionRotations() const;
        double GetVelocityRps() const;
        double GetRawPositionRad() const;
        uint32_t GetCPR() const;
        double GetGearRatio() const;
        bool IsInverted() const;

    private:
        uint32_t cpr;
        double gearRatio;
        bool inverted;
        double offsetRad;
        uint32_t windowSize;

        double rawPositionRad = 0.0;
        double quantizedPositionRad = 0.0;
        double filteredVelocityRadPerSec = 0.0;
        double positionRotations = 0.0;
        double velocityRps = 0.0;

        // Velocity windowing ring buffer
        static constexpr size_t maxWindowSize = 16;
        std::array<double, maxWindowSize> positionHistory = { 0.0 };
        std::array<double, maxWindowSize> timeHistory = { 0.0 };
        size_t historyCount = 0;
        size_t headIndex = 0;
        double accumulatedTime = 0.0;
    };
}