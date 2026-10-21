/**
 * @file ParameterStoreTest.h
 * @brief Declarations for JSON parameter store and layered inheritance test suite.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs parameter store ingestion and layered overlay tests.
     * @param ctx Test context tracking pass/fail counts.
     */
    void RunParameterStoreTest(TestContext& ctx);
}