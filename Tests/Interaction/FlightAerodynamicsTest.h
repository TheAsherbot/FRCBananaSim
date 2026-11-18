/**
 * @file FlightAerodynamicsTest.h
 * @brief Unit tests for linear chassis recoil, muzzle handoff, quadratic drag, and Magnus lift aerodynamics.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the complete test suite for Block 6 Day 3 Recoil and Flight Aerodynamics.
     * @param ctx Test execution context.
     */
    void RunFlightAerodynamicsTest(TestContext& ctx);
}
