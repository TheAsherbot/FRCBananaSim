/**
 * @file SimDeviceLockstepTest.h
 * @brief Test suite verifying SimDevice frame dispatching, lockstep coordination, and 1x vs 10x determinism.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for SimDeviceDispatcher and LockstepCoordinator.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunSimDeviceLockstepTest(TestContext& ctx);
}