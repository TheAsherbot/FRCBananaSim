/**
 * @file UnitsTest.h
 * @brief Declarations for unit conversion test suite.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit conversion regression tests.
     * @param ctx Test context tracking pass/fail counts.
     */
    void RunUnitsTest(TestContext& ctx);
}