/**
 * @file SwerveMassRatioTest.h
 * @brief Declarations for 60-second headless swerve mass-ratio stress test and Gate 1 verification.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs 60-second (12,000 steps) mass-ratio stress test validating Gate 1 stability criteria.
     * @param ctx Test context tracking pass/fail assertions.
     */
    void RunSwerveMassRatioTest(TestContext& ctx);
}