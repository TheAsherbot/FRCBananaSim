/**
 * @file EncoderSensorModel.cpp
 * @brief Implementation of encoder CPR quantization and finite-difference velocity windowing.
 */

#include "Robot/Sensors/Models/EncoderSensorModel.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Robot::Sensors::Models
{
    EncoderSensorModel::EncoderSensorModel(uint32_t cpr,
        double gearRatio,
        bool inverted,
        double offsetRad,
        uint32_t windowSize)
        : cpr(cpr),
        gearRatio(gearRatio),
        inverted(inverted),
        offsetRad(offsetRad),
        windowSize(std::clamp(windowSize, 1u, static_cast<uint32_t>(maxWindowSize)))
    {}

    void EncoderSensorModel::Update(double jointPosition, double jointVelocity, double dt)
    {
        if (dt <= 1e-7)
        {
            return;
        }

        accumulatedTime += dt;

        // 1. Direction Inversion & Gear Ratio Scaling
        double sign = inverted ? -1.0 : 1.0;
        rawPositionRad = (sign * gearRatio * jointPosition) + offsetRad;

        // 2. Counts-Per-Revolution (CPR) Quantization
        if (cpr > 0)
        {
            constexpr double twoPi = 2.0 * std::numbers::pi;
            double ticks = std::round((rawPositionRad * static_cast<double>(cpr)) / twoPi);
            quantizedPositionRad = (ticks * twoPi) / static_cast<double>(cpr);
        }
        else
        {
            quantizedPositionRad = rawPositionRad;
        }

        // 3. Velocity Windowing Ring Buffer
        positionHistory[headIndex] = quantizedPositionRad;
        timeHistory[headIndex] = accumulatedTime;

        headIndex = (headIndex + 1) % maxWindowSize;
        if (historyCount < maxWindowSize)
        {
            ++historyCount;
        }

        // Compute finite-difference velocity across the configured window
        size_t activeWindow = std::min(static_cast<size_t>(windowSize), historyCount);
        if (activeWindow >= 2)
        {
            size_t newestIdx = (headIndex + maxWindowSize - 1) % maxWindowSize;
            size_t oldestIdx = (headIndex + maxWindowSize - activeWindow) % maxWindowSize;

            double deltaPos = positionHistory[newestIdx] - positionHistory[oldestIdx];
            double deltaTime = timeHistory[newestIdx] - timeHistory[oldestIdx];

            if (deltaTime > 1e-6)
            {
                filteredVelocityRadPerSec = deltaPos / deltaTime;
            }
            else
            {
                filteredVelocityRadPerSec = sign * gearRatio * jointVelocity;
            }
        }
        else
        {
            filteredVelocityRadPerSec = sign * gearRatio * jointVelocity;
        }

        // 4. Engineering Units Conversion (Rotations and RPS)
        constexpr double invTwoPi = 1.0 / (2.0 * std::numbers::pi);
        positionRotations = quantizedPositionRad * invTwoPi;
        velocityRps = filteredVelocityRadPerSec * invTwoPi;
    }

    void EncoderSensorModel::Reset(double jointPosition)
    {
        double sign = inverted ? -1.0 : 1.0;
        rawPositionRad = (sign * gearRatio * jointPosition) + offsetRad;
        quantizedPositionRad = rawPositionRad;
        filteredVelocityRadPerSec = 0.0;
        positionRotations = quantizedPositionRad / (2.0 * std::numbers::pi);
        velocityRps = 0.0;

        historyCount = 0;
        headIndex = 0;
        accumulatedTime = 0.0;
    }

    double EncoderSensorModel::GetPositionRad() const { return quantizedPositionRad; }
    double EncoderSensorModel::GetVelocityRadPerSec() const { return filteredVelocityRadPerSec; }
    double EncoderSensorModel::GetPositionRotations() const { return positionRotations; }
    double EncoderSensorModel::GetVelocityRps() const { return velocityRps; }
    double EncoderSensorModel::GetRawPositionRad() const { return rawPositionRad; }
    uint32_t EncoderSensorModel::GetCPR() const { return cpr; }
    double EncoderSensorModel::GetGearRatio() const { return gearRatio; }
    bool EncoderSensorModel::IsInverted() const { return inverted; }
}