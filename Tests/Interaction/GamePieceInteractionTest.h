/**
 * @file GamePieceInteractionTest.h
 * @brief Automated 2,000-Shot Monte Carlo Oracle Test Suite and Energy Conservation Validation (Block 6 Day 6).
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the complete 2,000-shot Monte Carlo oracle test suite, verifying exit velocities within 5%,
     *        flywheel RPM sag within 10%, momentum conservation, Galilean invariance, and zero NaNs/infinities.
     * @param ctx Test execution context.
     */
    void RunGamePieceInteractionTest(TestContext& ctx);
}
