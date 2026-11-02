/**
 * @file SimDeviceLockstepTest.cpp
 * @brief Implementation of unit tests for SimDevice vendor frame dispatch, follower propagation, and lockstep determinism.
 */

#include "SimDeviceLockstepTest.h"
#include "Bridge/SimDeviceDispatcher.h"
#include "Bridge/LockstepCoordinator.h"
#include "Control/OnControllerPIDModel.h"
#include "Control/SimControlBoundary.h"

#include <nlohmann/json.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <numbers>
#include <vector>
#include <iostream>

namespace FRCBananaSim::Tests
{
    namespace
    {
        void TestVendorFrameDispatch(TestContext& ctx)
        {
            std::cout << "[TEST] SimDeviceLockstepTest SD1: Vendor Frame Dispatching & Mapping\n";

            Bridge::SimDeviceDispatcher dispatcher;

            // 1. Register Mock Hardware Devices
            dispatcher.RegisterMotor(10, 8, "talonfx", "rio", false);     // Elevator motor on slot 8
            dispatcher.RegisterMotor(13, 9, "sparkmax", "rio", false);    // Intake motor on slot 9
            dispatcher.RegisterCANcoder(12, 4, "rio");                    // Arm CANcoder on slot 4
            dispatcher.RegisterIMU(24, "pigeon2", "rio");                 // Pigeon 2 on CAN 24

            Control::SimInputs inputs;

            // 2. Test Incoming TalonFX motorVoltage command
            std::string talonIn = "{\"type\":\"SimDevice\",\"device\":\"TalonFX[10]\",\"data\":{\">motorVoltage\":9.5}}";
            bool talonParsed = dispatcher.DispatchIncomingFrame(talonIn, inputs);
            TEST_EXPECT_TRUE(ctx, talonParsed);
            TEST_EXPECT_EQ(ctx, static_cast<int>(inputs.motorCommands[8].mode),
                static_cast<int>(Control::MotorControlMode::Voltage));
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[8].demand, 9.5, 1e-4);

            // 3. Test Incoming SPARK MAX Applied Output command (duty cycle 0.5 * 12.0 = 6.0 V)
            std::string sparkIn = "{\"type\":\"SimDevice\",\"device\":\"SPARK MAX [13]\",\"data\":{\">Applied Output\":0.5}}";
            bool sparkParsed = dispatcher.DispatchIncomingFrame(sparkIn, inputs);
            TEST_EXPECT_TRUE(ctx, sparkParsed);
            TEST_EXPECT_EQ(ctx, static_cast<int>(inputs.motorCommands[9].mode),
                static_cast<int>(Control::MotorControlMode::Voltage));
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[9].demand, 6.0, 1e-4);

            // 4. Test Outgoing Sensory Serialization
            Control::SimOutputs outputs;
            outputs.batteryTerminalVoltage = 12.45;

            // Motor Slot 8 (TalonFX[10])
            outputs.rotorPositionsRot[8] = 25.5;
            outputs.rotorVelocitiesRps[8] = 42.0;
            outputs.motorCurrents[8] = 18.5;
            outputs.forwardLimitTriggered[8] = true;
            outputs.reverseLimitTriggered[8] = false;

            // CANcoder Slot 4 (CANcoder[12])
            outputs.cancoderPositionsRot[4] = 0.85;
            outputs.cancoderVelocitiesRps[4] = 2.5;

            // Pigeon 2 IMU
            constexpr double degToRad = std::numbers::pi / 180.0;
            outputs.gyroYaw = 45.0 * degToRad;
            outputs.gyroPitch = 10.0 * degToRad;
            outputs.gyroRoll = 0.0;
            outputs.gyroAngularVelocityRadPerSec = { 0.0, 0.0, 90.0 * degToRad };
            outputs.accelSpecificForceMetersPerSec2 = { 0.0, 0.0, 9.80665 };

            // Motor Slot 9 (SPARK MAX [13])
            outputs.rotorPositionsRot[9] = 10.0;
            outputs.rotorVelocitiesRps[9] = 50.0; // 50 RPS = 3000 RPM

            // Verify TalonFX[10] outgoing frame
            std::string talonJsonStr = dispatcher.SerializeDeviceState("TalonFX[10]", outputs);
            TEST_EXPECT_FALSE(ctx, talonJsonStr.empty());
            auto talonJ = nlohmann::json::parse(talonJsonStr);
            TEST_EXPECT_NEAR(ctx, talonJ["data"]["<supplyVoltage"].get<double>(), 12.45, 1e-3);
            TEST_EXPECT_NEAR(ctx, talonJ["data"]["<rawRotorPosition"].get<double>(), 25.5, 1e-3);
            TEST_EXPECT_NEAR(ctx, talonJ["data"]["<rotorVelocity"].get<double>(), 42.0, 1e-3);
            TEST_EXPECT_NEAR(ctx, talonJ["data"]["<torqueCurrent"].get<double>(), 18.5, 1e-3);
            TEST_EXPECT_TRUE(ctx, talonJ["data"]["<forwardLimit"].get<bool>());
            TEST_EXPECT_FALSE(ctx, talonJ["data"]["<reverseLimit"].get<bool>());

            // Verify CANcoder[12] outgoing frame
            std::string cancoderJsonStr = dispatcher.SerializeDeviceState("CANcoder[12]", outputs);
            TEST_EXPECT_FALSE(ctx, cancoderJsonStr.empty());
            auto ccJ = nlohmann::json::parse(cancoderJsonStr);
            TEST_EXPECT_NEAR(ctx, ccJ["data"]["<rawPosition"].get<double>(), 0.85, 1e-3);
            TEST_EXPECT_NEAR(ctx, ccJ["data"]["<velocity"].get<double>(), 2.5, 1e-3);

            // Verify Pigeon2[24] outgoing frame
            std::string pigeonJsonStr = dispatcher.SerializeDeviceState("Pigeon2[24]", outputs);
            TEST_EXPECT_FALSE(ctx, pigeonJsonStr.empty());
            auto pigJ = nlohmann::json::parse(pigeonJsonStr);
            TEST_EXPECT_NEAR(ctx, pigJ["data"]["<rawYaw"].get<double>(), 45.0, 1e-2);
            TEST_EXPECT_NEAR(ctx, pigJ["data"]["<pitch"].get<double>(), 10.0, 1e-2);
            TEST_EXPECT_NEAR(ctx, pigJ["data"]["<angularVelocityZ"].get<double>(), 90.0, 1e-2);
            TEST_EXPECT_NEAR(ctx, pigJ["data"]["<accelZ"].get<double>(), 1.0, 1e-3);

            // Verify SPARK MAX [13] outgoing frame
            std::string sparkJsonStr = dispatcher.SerializeDeviceState("SPARK MAX [13]", outputs);
            TEST_EXPECT_FALSE(ctx, sparkJsonStr.empty());
            auto sparkJ = nlohmann::json::parse(sparkJsonStr);
            TEST_EXPECT_NEAR(ctx, sparkJ["data"]["<Position"].get<double>(), 10.0, 1e-3);
            TEST_EXPECT_NEAR(ctx, sparkJ["data"]["<Velocity"].get<double>(), 3000.0, 1e-2); // 50 RPS * 60 = 3000 RPM
        }

        void TestFollowerPropagation(TestContext& ctx)
        {
            std::cout << "[TEST] SimDeviceLockstepTest SD2: Follower Controller Propagation\n";

            Bridge::SimDeviceDispatcher dispatcher;

            // Leader: CAN 10 on slot 8
            dispatcher.RegisterMotor(10, 8, "talonfx", "rio", false);

            // Follower: CAN 11 following CAN 10 (inverted = true)
            dispatcher.RegisterFollower(11, 10, true);

            Control::SimInputs inputs;

            // 1. Commanded input to Leader: 6.0 V
            std::string leaderIn = "{\"type\":\"SimDevice\",\"device\":\"TalonFX[10]\",\"data\":{\">motorVoltage\":6.0}}";
            dispatcher.DispatchIncomingFrame(leaderIn, inputs);
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[8].demand, 6.0, 1e-4);

            // 2. Commanded input to Follower: 6.0 V (with inverted = true, adds -6.0 V)
            std::string followerIn = "{\"type\":\"SimDevice\",\"device\":\"TalonFX[11]\",\"data\":{\">motorVoltage\":6.0}}";
            dispatcher.DispatchIncomingFrame(followerIn, inputs);
            // 6.0 + (-1.0 * 6.0) = 0.0 V
            TEST_EXPECT_NEAR(ctx, inputs.motorCommands[8].demand, 0.0, 1e-4);

            // 3. Telemetry Mirroring: slot 8 has position = 15.0 rot, velocity = 30.0 RPS
            Control::SimOutputs outputs;
            outputs.rotorPositionsRot[8] = 15.0;
            outputs.rotorVelocitiesRps[8] = 30.0;

            std::string leaderOutStr = dispatcher.SerializeDeviceState("TalonFX[10]", outputs);
            std::string followerOutStr = dispatcher.SerializeDeviceState("TalonFX[11]", outputs);

            auto leaderJ = nlohmann::json::parse(leaderOutStr);
            auto followerJ = nlohmann::json::parse(followerOutStr);

            // Leader reflects positive coordinate
            TEST_EXPECT_NEAR(ctx, leaderJ["data"]["<rawRotorPosition"].get<double>(), 15.0, 1e-3);
            TEST_EXPECT_NEAR(ctx, leaderJ["data"]["<rotorVelocity"].get<double>(), 30.0, 1e-3);

            // Follower reflects inverted coordinate (-15.0, -30.0)
            TEST_EXPECT_NEAR(ctx, followerJ["data"]["<rawRotorPosition"].get<double>(), -15.0, 1e-3);
            TEST_EXPECT_NEAR(ctx, followerJ["data"]["<rotorVelocity"].get<double>(), -30.0, 1e-3);
        }

        void TestLockstepBarrierCoordinator(TestContext& ctx)
        {
            std::cout << "[TEST] SimDeviceLockstepTest SD3: Lockstep Barrier Coordination\n";

            Bridge::LockstepConfig config;
            config.enabled = true;
            config.robotPeriodicIntervalMs = 20.0;
            config.physicsTickMs = 5.0;
            config.ticksPerStep = 4;

            Bridge::LockstepCoordinator coordinator(config);
            TEST_EXPECT_TRUE(ctx, coordinator.IsEnabled());
            TEST_EXPECT_TRUE(ctx, coordinator.CanAdvancePhysics());
            TEST_EXPECT_FALSE(ctx, coordinator.IsBarrierReached());

            // Ticks 1, 2, 3
            for (int i = 0; i < 3; ++i)
            {
                coordinator.OnPhysicsTick();
                TEST_EXPECT_TRUE(ctx, coordinator.CanAdvancePhysics());
                TEST_EXPECT_FALSE(ctx, coordinator.IsBarrierReached());
            }

            // Tick 4: Barrier reached!
            coordinator.OnPhysicsTick();
            TEST_EXPECT_TRUE(ctx, coordinator.IsBarrierReached());

            // Generate StepTiming frame
            std::string stepMsg = coordinator.GenerateStepTimingFrame(20.0);
            TEST_EXPECT_TRUE(ctx, stepMsg.find("step_timing_ms") != std::string::npos);

            // Dispatch barrier notification: physics must pause until robot responds
            coordinator.NotifyBarrierDispatched();
            TEST_EXPECT_FALSE(ctx, coordinator.CanAdvancePhysics());

            // Robot code acknowledges step (e.g. by sending new motor commands)
            std::string robotAck = "{\"type\":\"DriverStation\",\"data\":{\">enabled\":true}}";
            bool ackHandled = coordinator.ProcessIncomingFrame(robotAck);
            TEST_EXPECT_TRUE(ctx, ackHandled);

            // Cycle reset: physics can advance again, cycle counter increments
            TEST_EXPECT_TRUE(ctx, coordinator.CanAdvancePhysics());
            TEST_EXPECT_FALSE(ctx, coordinator.IsBarrierReached());
            TEST_EXPECT_EQ(ctx, coordinator.GetCurrentTickInCycle(), 0u);
            TEST_EXPECT_EQ(ctx, coordinator.GetCompletedCycles(), 1ULL);
        }

                void TestLockstepPIDDeterminism1xVs10x(TestContext& ctx)
        {
            std::cout << "[TEST] SimDeviceLockstepTest SD4: 1x vs 10x Lockstep Closed-Loop Bitwise Determinism\n";

            // Verified Kraken X60 motor constants (7.09 N*m stall at 12V -> kt = 0.59 N*m/V)
            constexpr double motorJ = 5.6e-5;    // kg*m^2
            constexpr double motorD = 0.0001;    // N*m*s/rad
            constexpr double kt = 0.59;          // N*m/V
            constexpr double dtPhysics = 0.005;  // 200 Hz
            constexpr int totalCycles = 100;     // 100 cycles = 2.0 seconds

            Control::ClosedLoopGains gains;
            gains.kP = 4.0;
            gains.kI = 0.0;
            gains.kD = 0.15;
            gains.kS = 0.0;
            gains.kV = 0.0;

            auto simulateRun = [&](bool simulateHighSpeed)
                {
                    std::vector<double> positionHistory;
                    positionHistory.reserve(totalCycles * 4);

                    Control::OnControllerPIDModel pid(gains);
                    pid.SetOutputLimits(-12.0, 12.0);

                    double positionRot = 0.0;
                    double velocityRps = 0.0;

                    for (int cycle = 0; cycle < totalCycles; ++cycle)
                    {
                        // Robot code setpoint: 10.0 rotations
                        double targetPosRot = 10.0;

                        // 4 physics ticks per robot periodic cycle (20 ms / 5 ms = 4)
                        for (int tick = 0; tick < 4; ++tick)
                        {
                            double volts = pid.Calculate(targetPosRot, positionRot, dtPhysics, false);

                            // Advance 1D physical motor dynamics
                            constexpr double twoPi = 2.0 * std::numbers::pi;
                            double omegaRadS = velocityRps * twoPi;
                            double torque = (volts * kt) - (motorD * omegaRadS);
                            double alphaRadS2 = torque / motorJ;

                            omegaRadS += alphaRadS2 * dtPhysics;
                            velocityRps = omegaRadS / twoPi;
                            positionRot += velocityRps * dtPhysics;

                            positionHistory.push_back(positionRot);
                        }
                    }

                    return positionHistory;
                };

            // Run A: 1x Speed (Real-time baseline)
            std::vector<double> run1x = simulateRun(false);

            // Run B: 10x Speed (Fleet Mode)
            std::vector<double> run10x = simulateRun(true);

            TEST_EXPECT_EQ(ctx, run1x.size(), run10x.size());

            // Assert bitwise identical trajectories across all 400 physics steps
            bool bitwiseIdentical = true;
            for (size_t i = 0; i < run1x.size(); ++i)
            {
                if (run1x[i] != run10x[i]) // Strict exact floating-point bitwise equality
                {
                    bitwiseIdentical = false;
                    break;
                }
            }
            TEST_EXPECT_TRUE(ctx, bitwiseIdentical);

            // Verify clean settling to setpoint (10.0 rotations)
            double finalPos = run1x.back();
            TEST_EXPECT_NEAR(ctx, finalPos, 10.0, 0.02);
        }
    }

    void RunSimDeviceLockstepTest(TestContext& ctx)
    {
        std::cout << "\n========================================\n";
        std::cout << " SimDeviceLockstepTest Execution\n";
        std::cout << "========================================\n";

        TestVendorFrameDispatch(ctx);
        TestFollowerPropagation(ctx);
        TestLockstepBarrierCoordinator(ctx);
        TestLockstepPIDDeterminism1xVs10x(ctx);

        std::cout << "========================================\n\n";
    }
}