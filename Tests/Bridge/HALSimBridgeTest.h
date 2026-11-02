/**
 * @file HALSimBridgeTest.h
 * @brief Unit tests for RFC 6455 handshake, frame parsing, and WPILib JSON dispatching.
 */

#pragma once

#include "TestFramework.h"

namespace FRCBananaSim::Tests
{
    /**
     * @brief Runs complete test suite for the WPILib HALSim WebSocket bridge.
     * @param ctx Test context tracking passed/failed assertions.
     */
    void RunHALSimBridgeTest(TestContext& ctx);
}