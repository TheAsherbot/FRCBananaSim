/**
 * @file RobotSchemaTest.h
 * @brief Declarations for Schema v1 ingestion, validation, and NWU coordinate transform unit tests.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit tests for Robot Data Schema v1 loading, physical validation, and coordinate frame transformation.
     * @param ctx Test context tracking assertion passes and failures.
     */
    void RunRobotSchemaTest(TestContext& ctx);
}