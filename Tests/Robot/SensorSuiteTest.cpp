/**
 * @file SensorSuiteTest.cpp
 * @brief Implementation of automated unit tests for sensors, encoders, IMU lever-arms, DIOs, and analog inputs.
 */

#include "SensorSuiteTest.h"
#include "Robot/Sensors/Systems/SensorSystem.h"
#include "Robot/Sensors/Components/EncoderComponent.h"
#include "Robot/Sensors/Components/IMUComponent.h"
#include "Robot/Sensors/Components/LimitSwitchComponent.h"
#include "Robot/Sensors/Components/BeamBreakComponent.h"
#include "Robot/Sensors/Components/AnalogInputComponent.h"
#include "Robot/Sensors/Components/ContactZoneComponent.h"
#include "Physics/Components/JointComponent.h"
#include "Core/ECS/Core/Scene.h"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <iostream>

namespace FRCBananaSim::Tests
{
    namespace
    {
                void TestCancoderQuantization(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS1: CANcoder Quantization & Gear Scaling\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // Mock joint entity
            auto jointEntity = scene.CreateEntity();
            Physics::Components::JointComponent joint;
            joint.currentPosition = 0.0f;
            joint.currentVelocity = 1.0f; // 1 rad/s
            scene.AddComponent(jointEntity, joint);

            // 1:1 CANcoder entity on slot 0: CPR = 4096
            auto encoderEntity = scene.CreateEntity();
            Robot::Sensors::Components::EncoderComponent encoder;
            encoder.name = "ArmCANcoder";
            encoder.bindingType = Robot::Sensors::Components::EncoderBindingType::CANcoder;
            encoder.slot = 0;
            encoder.targetJoint = jointEntity;
            encoder.model = Robot::Sensors::Models::EncoderSensorModel(4096, 1.0, false, 0.0, 4);
            scene.AddComponent(encoderEntity, encoder);

            // 2.5:1 Geared CANcoder entity on slot 1
            auto gearedEntity = scene.CreateEntity();
            Robot::Sensors::Components::EncoderComponent gearedEncoder;
            gearedEncoder.name = "GearedCANcoder";
            gearedEncoder.bindingType = Robot::Sensors::Components::EncoderBindingType::CANcoder;
            gearedEncoder.slot = 1;
            gearedEncoder.targetJoint = jointEntity;
            gearedEncoder.model = Robot::Sensors::Models::EncoderSensorModel(4096, 2.5, false, 0.0, 4);
            scene.AddComponent(gearedEntity, gearedEncoder);

            constexpr double dt = 0.005;
            constexpr double expectedStepRot = 1.0 / 4096.0;

            // 1. Sub-tick micro-sweep: verify the minimum nonzero quantization jump is exactly 1/4096
            constexpr double tickStepRad = (2.0 * std::numbers::pi) / 4096.0;
            double detectedMinJump = 0.0;
            auto* jMicro = scene.GetComponent<Physics::Components::JointComponent>(jointEntity);
            jMicro->currentPosition = 0.0f;
            jMicro->currentVelocity = 0.0f;

            double prevRot = 0.0;
            for (int micro = 1; micro <= 20; ++micro)
            {
                jMicro->currentPosition = static_cast<float>(micro * (tickStepRad * 0.1));
                sensorSystem.FixedUpdate(dt);

                double rot = sensorSystem.GetOutputs().cancoderPositionsRot[0];
                double delta = rot - prevRot;
                if (delta > 1e-7 && detectedMinJump == 0.0)
                {
                    detectedMinJump = delta;
                }
                prevRot = rot;
            }
            TEST_EXPECT_NEAR(ctx, detectedMinJump, expectedStepRot, 1e-6);

            // 2. Sweep across 1 full revolution (2*pi rad)
            constexpr double omega = 1.0; // rad/s
            constexpr double totalTime = (2.0 * std::numbers::pi) / omega;
            const int totalSteps = static_cast<int>(std::round(totalTime / dt));

            double accumulatedVelocity = 0.0;
            int velocitySamples = 0;

            for (int step = 0; step < totalSteps; ++step)
            {
                auto* j = scene.GetComponent<Physics::Components::JointComponent>(jointEntity);
                j->currentPosition = static_cast<float>(step * omega * dt);
                j->currentVelocity = static_cast<float>(omega);

                sensorSystem.FixedUpdate(dt);

                const auto& out = sensorSystem.GetOutputs();
                double currentPosRot = out.cancoderPositionsRot[0];

                // Verify every position reading is an exact integer multiple of 1 / 4096
                double ticks = currentPosRot * 4096.0;
                TEST_EXPECT_NEAR(ctx, ticks, std::round(ticks), 1e-5);

                if (step >= 10)
                {
                    accumulatedVelocity += out.cancoderVelocitiesRps[0];
                    ++velocitySamples;
                }
            }

            // 3. Check position after 1 full rotation: 1.0 rotation with zero drift
            auto* j = scene.GetComponent<Physics::Components::JointComponent>(jointEntity);
            j->currentPosition = static_cast<float>(2.0 * std::numbers::pi);
            j->currentVelocity = static_cast<float>(omega);
            sensorSystem.FixedUpdate(dt);

            const auto& finalOut = sensorSystem.GetOutputs();
            TEST_EXPECT_NEAR(ctx, finalOut.cancoderPositionsRot[0], 1.0, expectedStepRot);

            // Geared encoder should read 2.5 rotations
            TEST_EXPECT_NEAR(ctx, finalOut.cancoderPositionsRot[1], 2.5, expectedStepRot * 2.5);

            // 4. Velocity readout:
            // Theoretical continuous speed: 1.0 rad/s / (2*pi) ~ 0.159155 rev/s
            // Instantaneous 4-sample (15 ms) window quantization resolution is 1 / (4096 * 0.015) ~ 0.0163 rev/s
            constexpr double expectedRps = omega / (2.0 * std::numbers::pi);
            TEST_EXPECT_NEAR(ctx, finalOut.cancoderVelocitiesRps[0], expectedRps, 0.02);

            // Average velocity over the full sweep converges smoothly
            double meanRps = accumulatedVelocity / static_cast<double>(velocitySamples);
            TEST_EXPECT_NEAR(ctx, meanRps, expectedRps, 0.002);
        }

        void TestIMUFlatAndRamp(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS2: 6-Axis IMU Flat Ground & 15-Degree Ramp\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // IMU entity with zero noise for deterministic baseline check
            auto imuEntity = scene.CreateEntity();
            Robot::Sensors::Components::IMUComponent imu;
            imu.name = "Pigeon2";
            Robot::Sensors::Models::IMUConfig config;
            config.gyroNoiseStdDevRadS = 0.0;
            config.gyroBiasRandomWalkRadS2 = 0.0;
            config.accelNoiseStdDevMS2 = 0.0;
            imu.model = Robot::Sensors::Models::IMUSensorModel(config);
            imu.useKinematicOverride = true;
            imu.overrideAngularVelocityNWU = glm::dvec3(0.0);
            imu.overrideLinearAccelNWU = glm::dvec3(0.0);
            imu.overrideOrientationNWU = glm::dquat(1.0, 0.0, 0.0, 0.0);
            scene.AddComponent(imuEntity, imu);

            constexpr double dt = 0.005;

            // 1. Flat ground at rest
            sensorSystem.FixedUpdate(dt);
            const auto& flatOut = sensorSystem.GetOutputs();

            TEST_EXPECT_NEAR(ctx, flatOut.accelSpecificForceMetersPerSec2.x, 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, flatOut.accelSpecificForceMetersPerSec2.y, 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, flatOut.accelSpecificForceMetersPerSec2.z, 9.80665, 1e-4);
            TEST_EXPECT_NEAR(ctx, flatOut.gyroYaw, 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, flatOut.gyroPitch, 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, flatOut.gyroRoll, 0.0, 1e-4);

            // 2. 15-degree ramp tilt (pitch around Y axis in NWU)
            constexpr double rampAngleDeg = 15.0;
            double rampAngleRad = rampAngleDeg * (std::numbers::pi / 180.0);
            glm::dquat rampOrientation = glm::angleAxis(rampAngleRad, glm::dvec3(0.0, 1.0, 0.0));

            auto* imuComp = scene.GetComponent<Robot::Sensors::Components::IMUComponent>(imuEntity);
            imuComp->model.Reset(rampOrientation);
            imuComp->overrideOrientationNWU = rampOrientation;

            sensorSystem.FixedUpdate(dt);
            const auto& rampOut = sensorSystem.GetOutputs();

            // Pitch should read 15.0 degrees (+/- 0.02 deg)
            double measuredPitchDeg = rampOut.gyroPitch * (180.0 / std::numbers::pi);
            TEST_EXPECT_NEAR(ctx, measuredPitchDeg, rampAngleDeg, 0.02);

            // Specific force magnitude at rest must always equal 1g (9.80665 m/s^2)
            double specForceMag = glm::length(rampOut.accelSpecificForceMetersPerSec2);
            TEST_EXPECT_NEAR(ctx, specForceMag, 9.80665, 1e-4);

            // Specific force Z component = 9.80665 * cos(15 deg) ~ 9.47248 m/s^2
            double expectedZ = 9.80665 * std::cos(rampAngleRad);
            TEST_EXPECT_NEAR(ctx, rampOut.accelSpecificForceMetersPerSec2.z, expectedZ, 1e-3);
        }

        void TestIMUSpinningCentripetal(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS3: Centripetal Acceleration on Spinning Chassis\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // Mount IMU at r = (0.25, 0.0, 0.0) m from chassis center of mass
            auto imuEntity = scene.CreateEntity();
            Robot::Sensors::Components::IMUComponent imu;
            imu.name = "Pigeon2";
            Robot::Sensors::Models::IMUConfig config;
            config.mountOffsetM = glm::dvec3(0.25, 0.0, 0.0);
            config.gyroNoiseStdDevRadS = 0.0;
            config.gyroBiasRandomWalkRadS2 = 0.0;
            config.accelNoiseStdDevMS2 = 0.0;
            imu.model = Robot::Sensors::Models::IMUSensorModel(config);
            imu.useKinematicOverride = true;

            // Spinning at constant omega_z = 4.0 rad/s
            constexpr double omegaZ = 4.0;
            imu.overrideAngularVelocityNWU = glm::dvec3(0.0, 0.0, omegaZ);
            imu.overrideLinearAccelNWU = glm::dvec3(0.0);
            imu.overrideOrientationNWU = glm::dquat(1.0, 0.0, 0.0, 0.0);
            scene.AddComponent(imuEntity, imu);

            constexpr double dt = 0.005;

            // Run 5 ticks so finite-difference angular acceleration settles to 0
            for (int i = 0; i < 5; ++i)
            {
                sensorSystem.FixedUpdate(dt);
            }

            const auto& out = sensorSystem.GetOutputs();

            // Centripetal acceleration: a_cent = omega x (omega x r) = (-4.00, 0.0, 0.0) m/s^2
            constexpr double expectedCentripetalX = -4.00;
            TEST_EXPECT_NEAR(ctx, out.accelSpecificForceMetersPerSec2.x, expectedCentripetalX, 0.04); // Within 1%
            TEST_EXPECT_NEAR(ctx, out.accelSpecificForceMetersPerSec2.y, 0.0, 0.01);
            TEST_EXPECT_NEAR(ctx, out.accelSpecificForceMetersPerSec2.z, 9.80665, 1e-4);
            TEST_EXPECT_NEAR(ctx, out.gyroYawRate, omegaZ, 1e-4);
        }

        void TestLimitSwitchesWithHysteresis(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS4: Limit Switches with Hysteresis & Motor Flags\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // Elevator joint entity
            auto jointEntity = scene.CreateEntity();
            Physics::Components::JointComponent joint;
            joint.currentPosition = 0.0f;
            scene.AddComponent(jointEntity, joint);

            // Reverse limit switch: DIO 0, Motor Slot 8, envelope [-0.01, 0.005], hysteresis 0.001
            auto revLimitEntity = scene.CreateEntity();
            Robot::Sensors::Components::LimitSwitchComponent revLimit;
            revLimit.name = "ElevatorLowerLimit";
            revLimit.targetJoint = jointEntity;
            revLimit.dioChannel = 0;
            revLimit.attachToMotorSlot = 8;
            revLimit.direction = Robot::Sensors::Components::LimitSwitchDirection::Reverse;
            revLimit.triggerMin = -0.01;
            revLimit.triggerMax = 0.005;
            revLimit.hysteresis = 0.001;
            revLimit.normallyClosed = false;
            scene.AddComponent(revLimitEntity, revLimit);

            // Forward limit switch: DIO 1, Motor Slot 8, envelope [1.195, 1.250], hysteresis 0.001
            auto fwdLimitEntity = scene.CreateEntity();
            Robot::Sensors::Components::LimitSwitchComponent fwdLimit;
            fwdLimit.name = "ElevatorUpperLimit";
            fwdLimit.targetJoint = jointEntity;
            fwdLimit.dioChannel = 1;
            fwdLimit.attachToMotorSlot = 8;
            fwdLimit.direction = Robot::Sensors::Components::LimitSwitchDirection::Forward;
            fwdLimit.triggerMin = 1.195;
            fwdLimit.triggerMax = 1.250;
            fwdLimit.hysteresis = 0.001;
            fwdLimit.normallyClosed = false;
            scene.AddComponent(fwdLimitEntity, fwdLimit);

            constexpr double dt = 0.005;

            // 1. Position = 0.0 m (at lower stop)
            sensorSystem.FixedUpdate(dt);
            auto out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 0)) != 0); // DIO 0 high
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 1)) == 0); // DIO 1 low
            TEST_EXPECT_TRUE(ctx, out.reverseLimitTriggered[8]);
            TEST_EXPECT_FALSE(ctx, out.forwardLimitTriggered[8]);

            // 2. Position = 0.6 m (mid stroke)
            auto* j = scene.GetComponent<Physics::Components::JointComponent>(jointEntity);
            j->currentPosition = 0.6f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 0)) == 0); // DIO 0 low
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 1)) == 0); // DIO 1 low
            TEST_EXPECT_FALSE(ctx, out.reverseLimitTriggered[8]);
            TEST_EXPECT_FALSE(ctx, out.forwardLimitTriggered[8]);

            // 3. Position = 1.20 m (upper stop)
            j->currentPosition = 1.20f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 0)) == 0); // DIO 0 low
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 1)) != 0); // DIO 1 high
            TEST_EXPECT_FALSE(ctx, out.reverseLimitTriggered[8]);
            TEST_EXPECT_TRUE(ctx, out.forwardLimitTriggered[8]);

            // 4. Hysteresis test:
            // Move slightly below triggerMin (1.195) to 1.1945 m (within hysteresis band of 0.001)
            j->currentPosition = 1.1945f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, out.forwardLimitTriggered[8]); // Stays triggered due to hysteresis

            // Move outside hysteresis band: 1.193 m (< 1.195 - 0.001 = 1.194)
            j->currentPosition = 1.193f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_FALSE(ctx, out.forwardLimitTriggered[8]); // Now clears cleanly
        }

        void TestBeamBreakDebounce(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS5: Beam Break Debounce & ContactZone Occlusion\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // ContactZone entity
            auto czEntity = scene.CreateEntity();
            Robot::Sensors::Components::ContactZoneComponent cz;
            cz.name = "ShooterBreach";
            cz.role = "shooter";
            cz.isEngaged = false;
            scene.AddComponent(czEntity, cz);

            // Beam break bound to contact zone: DIO 2, 10 ms debounce (2 ticks at 200 Hz)
            auto beamEntity = scene.CreateEntity();
            Robot::Sensors::Components::BeamBreakComponent beam;
            beam.name = "BreachBeamBreak";
            beam.sensorEntity = czEntity;
            beam.dioChannel = 2;
            beam.debounceTimeS = 0.010;
            beam.normallyClosed = false;
            scene.AddComponent(beamEntity, beam);

            constexpr double dt = 0.005;

            // 1. Initially clear
            sensorSystem.FixedUpdate(dt);
            auto out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 2)) == 0);

            // 2. Ball enters contact zone -> isEngaged = true
            auto* czComp = scene.GetComponent<Robot::Sensors::Components::ContactZoneComponent>(czEntity);
            czComp->isEngaged = true;

            // Tick 1 (5 ms < 10 ms debounce): still low
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 2)) == 0);

            // Tick 2 (10 ms >= 10 ms debounce): now high
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 2)) != 0);

            // 3. Ball exits contact zone -> isEngaged = false
            czComp->isEngaged = false;

            // Tick 1 (5 ms < 10 ms falling edge debounce): still high
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 2)) != 0);

            // Tick 2 (10 ms >= 10 ms): now low
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_TRUE(ctx, (out.digitalInputs & (1u << 2)) == 0);
        }

        void TestAnalogInputMapping(TestContext& ctx)
        {
            std::cout << "[TEST] SensorSuiteTest TS6: Analog Input Transducer Mapping\n";

            Core::ECS::Core::Scene scene;
            Robot::Sensors::Systems::SensorSystem sensorSystem(scene);

            // Joint potentiometer on channel 0: range [0.0, 1.0] m mapped to [0.0, 5.0] V
            auto jointEntity = scene.CreateEntity();
            Physics::Components::JointComponent joint;
            joint.currentPosition = 0.0f;
            scene.AddComponent(jointEntity, joint);

            auto analogPotEntity = scene.CreateEntity();
            Robot::Sensors::Components::AnalogInputComponent pot;
            pot.name = "ArmPotentiometer";
            pot.analogChannel = 0;
            pot.mode = Robot::Sensors::Components::AnalogInputMode::JointPosition;
            pot.targetJoint = jointEntity;
            pot.minInputValue = 0.0;
            pot.maxInputValue = 1.0;
            pot.minVoltage = 0.0;
            pot.maxVoltage = 5.0;
            scene.AddComponent(analogPotEntity, pot);

            // Custom voltage sensor on channel 1 (pressure sensor): 3.3V
            auto analogPressureEntity = scene.CreateEntity();
            Robot::Sensors::Components::AnalogInputComponent pressure;
            pressure.name = "PressureTransducer";
            pressure.analogChannel = 1;
            pressure.mode = Robot::Sensors::Components::AnalogInputMode::CustomVoltage;
            pressure.voltage = 3.3;
            scene.AddComponent(analogPressureEntity, pressure);

            constexpr double dt = 0.005;

            // 1. Joint at 0.0 m -> 0.0 V
            sensorSystem.FixedUpdate(dt);
            auto out = sensorSystem.GetOutputs();
            TEST_EXPECT_NEAR(ctx, out.analogInputs[0], 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, out.analogInputs[1], 3.3, 1e-4);

            // 2. Joint at 0.5 m -> 2.5 V
            auto* j = scene.GetComponent<Physics::Components::JointComponent>(jointEntity);
            j->currentPosition = 0.5f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_NEAR(ctx, out.analogInputs[0], 2.5, 1e-4);

            // 3. Joint at 1.0 m -> 5.0 V
            j->currentPosition = 1.0f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_NEAR(ctx, out.analogInputs[0], 5.0, 1e-4);

            // 4. Joint overshoot to 1.5 m -> clamped to 5.0 V
            j->currentPosition = 1.5f;
            sensorSystem.FixedUpdate(dt);
            out = sensorSystem.GetOutputs();
            TEST_EXPECT_NEAR(ctx, out.analogInputs[0], 5.0, 1e-4);
        }
    }

    void RunSensorSuiteTest(TestContext& ctx)
    {
        std::cout << "\n========================================\n";
        std::cout << " SensorSuiteTest Execution\n";
        std::cout << "========================================\n";

        TestCancoderQuantization(ctx);
        TestIMUFlatAndRamp(ctx);
        TestIMUSpinningCentripetal(ctx);
        TestLimitSwitchesWithHysteresis(ctx);
        TestBeamBreakDebounce(ctx);
        TestAnalogInputMapping(ctx);

        std::cout << "========================================\n\n";
    }
}