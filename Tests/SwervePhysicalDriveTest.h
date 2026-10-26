/**
 * @file SwervePhysicalDriveTest.h
 * @brief Declarations for physical swerve drive and control boundary regression tests.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit and regression tests for physical swerve modules, motor drive torque, and SimControlBoundary.
     * @param ctx Test context tracking pass/fail assertions.
     */
    void RunSwervePhysicalDriveTest(TestContext& ctx);
}