/**
 * @file SchemaSerializationTest.h
 * @brief Declarations for RobotSidecarSerializer Schema v1.0.0 round-trip and serialization unit tests.
 */

#pragma once

namespace FRCBananaSim::Tests
{
    struct TestContext;

    /**
     * @brief Runs unit tests for RobotSidecar JSON serialization, fidelity, and round-trip verification.
     * @param ctx Test context tracking assertion passes and failures.
     */
    void RunSchemaSerializationTest(TestContext& ctx);
}
