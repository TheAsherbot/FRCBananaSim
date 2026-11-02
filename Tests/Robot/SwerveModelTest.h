/**
 * @file SwerveModelTest.h
 * @brief Declarations for unit and physics regression tests validating the analytic 6-DOF swerve drivetrain model.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Executes the complete L1 analytic swerve drivetrain test suite (T1-T11, T14, T16, T18).
     * @param ctx Reference to test execution context.
     */
    void RunSwerveModelTest(TestContext& ctx);
}