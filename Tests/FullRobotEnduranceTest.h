/**
 * @file FullRobotEnduranceTest.h
 * @brief Headless 5-minute (60,000 steps at 200 Hz) endurance test for the full Schema v1 Reference 2026 robot.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the 5-minute endurance simulation test driving all subsystems concurrently.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunFullRobotEnduranceTest(TestContext& ctx);
}