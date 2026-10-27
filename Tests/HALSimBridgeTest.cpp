/**
 * @file HALSimBridgeTest.cpp
 * @brief Implementation of unit tests for the WPILib HALSim WebSocket bridge.
 */

#include "HALSimBridgeTest.h"
#include "Bridge/Sha1.h"
#include "Bridge/HALSimWebSocketServer.h"

namespace FRCBananaSim::Tests
{
    using namespace FRCBananaSim::Bridge;

    void RunHALSimBridgeTest(TestContext& ctx)
    {
        std::cout << "[RUN ] HALSimBridgeTest\n";

        // 1. RFC 6455 Handshake Acceptance Vector Verification
        {
            std::string clientKey = "dGhlIHNhbXBsZSBub25jZQ==";
            std::string expectedAccept = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=";
            std::string calculatedAccept = ComputeWebSocketAcceptKey(clientKey);

            TEST_EXPECT_EQ(ctx, calculatedAccept, expectedAccept);
        }

        // 2. WebSocket Text Frame Encoding and Masked Decoding
        {
            std::string testPayload = "{\"type\":\"DriverStation\",\"data\":{\">enabled\":true}}";
            std::vector<uint8_t> encodedFrame = HALSimWebSocketServer::EncodeTextFrame(testPayload);

            TEST_EXPECT_TRUE(ctx, encodedFrame.size() > testPayload.size());
            TEST_EXPECT_EQ(ctx, static_cast<int>(encodedFrame[0]), 0x81);

            std::string decodedPayload;
            uint8_t opcode = 0;
            bool success = HALSimWebSocketServer::DecodeFrame(encodedFrame, decodedPayload, opcode);

            TEST_EXPECT_TRUE(ctx, success);
            TEST_EXPECT_EQ(ctx, static_cast<int>(opcode), 0x01);
            TEST_EXPECT_EQ(ctx, decodedPayload, testPayload);
        }

        // 3. Masked Client Frame Decoding Verification
        {
            std::string payload = "Hello";
            std::vector<uint8_t> maskedFrame = {
                0x81, 0x85, // FIN + Text, Masked + Length 5
                0x37, 0xFA, 0x21, 0x3D, // Mask key
                static_cast<uint8_t>('H' ^ 0x37),
                static_cast<uint8_t>('e' ^ 0xFA),
                static_cast<uint8_t>('l' ^ 0x21),
                static_cast<uint8_t>('l' ^ 0x3D),
                static_cast<uint8_t>('o' ^ 0x37)
            };

            std::string decodedPayload;
            uint8_t opcode = 0;
            bool success = HALSimWebSocketServer::DecodeFrame(maskedFrame, decodedPayload, opcode);

            TEST_EXPECT_TRUE(ctx, success);
            TEST_EXPECT_EQ(ctx, decodedPayload, std::string("Hello"));
        }

        // 4. Inbound PWM JSON Message Dispatching
        {
            HALSimWebSocketServer server;
            std::string pwmJson = "{\"type\":\"PWM\",\"device\":\"4\",\"data\":{\">speed\":0.75}}";
            bool handled = server.DispatchIncomingJson(pwmJson);

            TEST_EXPECT_TRUE(ctx, handled);

            Control::SimInputs inputs;
            bool hasInputs = server.UpdateInputs(inputs);
            TEST_EXPECT_TRUE(ctx, hasInputs);
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[4].demand, 9.0, 0.01);
        }

        // 5. Inbound SimDevice (Talon FX) JSON Dispatching
        {
            HALSimWebSocketServer server;
            // Talon FX[2] = FrontLeft Drive (Index 4)
            std::string talonJson = "{\"type\":\"SimDevice\",\"device\":\"Talon FX[2]\",\"data\":{\">motorVoltage\":10.5}}";
            bool handled = server.DispatchIncomingJson(talonJson);

            TEST_EXPECT_TRUE(ctx, handled);

            Control::SimInputs inputs;
            bool hasInputs = server.UpdateInputs(inputs);
            TEST_EXPECT_TRUE(ctx, hasInputs);
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[4].demand, 10.5, 0.01);
        }

        // 6. Inbound SimDevice (Steer Talon FX) Dispatching
        {
            HALSimWebSocketServer server;
            // Talon FX[1] = FrontLeft Steer (Index 0)
            std::string steerJson = "{\"type\":\"SimDevice\",\"device\":\"Talon FX[1]\",\"data\":{\">motorOutputLead\":-0.5}}";
            bool handled = server.DispatchIncomingJson(steerJson);

            TEST_EXPECT_TRUE(ctx, handled);

            Control::SimInputs inputs;
            bool hasInputs = server.UpdateInputs(inputs);
            TEST_EXPECT_TRUE(ctx, hasInputs);
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[0].demand, -6.0, 0.01);
        }

        std::cout << "[DONE] HALSimBridgeTest\n";
    }
}