/**
 * @file DrivetrainLoggerTest.h
 * @brief Unit tests for the high-throughput ring buffer and CSV export functionality.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Runs unit test suite for the DrivetrainLogger class.
     * @param ctx Test context tracking passed and failed assertions.
     */
    void RunDrivetrainLoggerTest(TestContext& ctx);
}