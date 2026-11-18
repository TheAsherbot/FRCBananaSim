/**
 * @file RoundTripPipelineTest.h
 * @brief Gate 5 integration test verifying end-to-end CAD ingestion, Configurator authoring,
 *        sidecar serialization, Jolt assembly, and 60-second physics execution.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the Gate 5 end-to-end round trip pipeline test.
     * @param ctx The test execution context.
     */
    void RunRoundTripPipelineTest(TestContext& ctx);
}
