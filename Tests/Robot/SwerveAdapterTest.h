/**
 * @file SwerveAdapterTest.h
 * @brief Test suite declarations for Phase 4 Jolt adapter and terrain probe verification (T12, T13, T15, T17, T21).
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Runs Phase 4 test suite validating Jolt middleware integration, midpoint handoff, wall pushing, and probe accuracy.
     * @param ctx Test context tracking pass/fail metrics.
     */
    void RunSwerveAdapterTest(TestContext& ctx);
}