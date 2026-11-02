/**
 * @file RobotAssemblerTest.h
 * @brief Test suite verifying Schema v1 full robot assembly, entity component integrity, and 8-stage pipeline stepping.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for RobotAssembler and 8-stage pipeline stepping.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunRobotAssemblerTest(TestContext& ctx);
}