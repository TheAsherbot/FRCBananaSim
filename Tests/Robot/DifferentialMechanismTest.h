/**
 * @file DifferentialMechanismTest.h
 * @brief Test suite declarations for 2-DOF coupled differential mechanisms.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the complete differential mechanism verification suite.
     * @param ctx Reference to test execution context.
     */
    void RunDifferentialMechanismTest(TestContext& ctx);
}