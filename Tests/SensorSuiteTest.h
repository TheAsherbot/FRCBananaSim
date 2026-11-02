/**
 * @file SensorSuiteTest.h
 * @brief Automated test suite verifying quantized encoders, 6-axis IMU dynamics, limit switches, and beam breaks.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for the sensor subsystem.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunSensorSuiteTest(TestContext& ctx);
}