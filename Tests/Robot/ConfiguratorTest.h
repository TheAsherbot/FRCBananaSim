/**
 * @file ConfiguratorTest.h
 * @brief Declaration of test suite for SidecarConfigurator document model, auto-generator, and linter.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes test suite verifying CAD ingestion, starter sidecar generation, and live linter diagnostics.
     * @param ctx Test execution context tracking assertions.
     */
    void RunConfiguratorTest(TestContext& ctx);
}
