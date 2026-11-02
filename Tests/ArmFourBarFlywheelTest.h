/**
 * @file ArmFourBarFlywheelTest.h
 * @brief Test suite verifying articulated arm gravity holding, hard stop impact, 4-bar linkage, flywheel ODE, and dynamic CG transfer.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for rotational mechanisms, four-bars, couplings, and flywheels.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunArmFourBarFlywheelTest(TestContext& ctx);
}