/**
 * @file ServoMechanismTest.h
 * @brief Automated unit test suite verifying end-to-end servo actuation across arms, differential wrists, elevators, swerve azimuth, and electrical rail overload.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for servo-driven mechanisms and swerve azimuth.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunServoMechanismTest(TestContext& ctx);
}
