/**
 * @file JointStabilityTest.h
 * @brief Declarations for Jolt hinge constraint and mass-ratio stability regression test suite.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit and regression tests for Jolt constraints, motor modes, and mass-ratio stability.
     * @param ctx Test context tracking pass/fail assertions.
     */
    void RunJointStabilityTest(TestContext& ctx);
}