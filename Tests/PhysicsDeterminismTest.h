/**
 * @file PhysicsDeterminismTest.h
 * @brief Declarations for 200 Hz headless physics determinism test suite.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs headless Jolt simulation for 1,000 steps and validates exact repeatability.
     * @param ctx Test context tracking pass/fail counts.
     */
    void RunPhysicsDeterminismTest(TestContext& ctx);
}