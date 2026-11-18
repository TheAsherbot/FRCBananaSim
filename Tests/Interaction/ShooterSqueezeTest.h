/**
 * @file ShooterSqueezeTest.h
 * @brief Unit tests for Hunt-Crossley normal squeeze, tangential traction, flywheel RPM sag, and recoil.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the complete test suite for Block 6 Day 2 Shooter Squeeze and Flywheel RPM Sag.
     * @param ctx Test execution context.
     */
    void RunShooterSqueezeTest(TestContext& ctx);
}
