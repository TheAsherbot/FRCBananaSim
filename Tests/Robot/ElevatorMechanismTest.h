/**
 * @file ElevatorMechanismTest.h
 * @brief Test suite verifying elevator prismatic joints, semi-implicit step response, cascade tracking, continuous rigging, and zero-creep holding.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes all automated verification tests for the elevator mechanism system.
     * @param ctx Test execution context for recording pass/fail assertions.
     */
    void RunElevatorMechanismTest(TestContext& ctx);
}