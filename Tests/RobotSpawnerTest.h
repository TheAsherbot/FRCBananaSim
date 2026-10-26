/**
 * @file RobotSpawnerTest.h
 * @brief Declarations for declarative robot spawner and collision filtering regression tests.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit and regression tests for RobotSpawner JSON ingestion, entity hierarchy, and stability.
     * @param ctx Test context tracking pass/fail assertions.
     */
    void RunRobotSpawnerTest(TestContext& ctx);
}