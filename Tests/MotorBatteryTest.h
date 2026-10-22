/**
 * @file MotorBatteryTest.h
 * @brief Declarations for DC motor and lumped battery regression tests.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit and regression tests for DC motor models and battery electrical dynamics.
     * @param ctx Test context tracking pass/fail assertions.
     */
    void RunMotorBatteryTest(TestContext& ctx);
}