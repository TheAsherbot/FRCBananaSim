/**
 * @file MotorBatteryTest.cpp
 * @brief Test suite verifying motor presets, semi-implicit stability, voltage sag, and brownout protection.
 */

#include "MotorBatteryTest.h"
#include "TestFramework.h"
#include "Actuation/ServoModel.h"
#include "Actuation/DCMotorModel.h"
#include "Actuation/BatteryModel.h"
#include "Actuation/Components/MotorComponent.h"
#include "Actuation/Components/BatteryComponent.h"
#include "Core/Units/Units.h"

#include <iostream>
#include <cmath>

namespace FRCBananaSim::Tests
{
    using namespace Actuation;
    using namespace Actuation::Components;
    using namespace Core::Units::Literals;

    void RunMotorBatteryTest(TestContext& ctx)
    {
        std::cout << "[RUN ] MotorBatteryTest\n";
        constexpr double tolerance = 1e-4;

        // -------------------------------------------------------------
        // 1. Motor Preset Verification against WPILib Constants
        // -------------------------------------------------------------
        {
            // Kraken X60 (1:1 direct drive)
            DCMotorModel kraken = DCMotorModel::KrakenX60();
            TEST_EXPECT_NEAR(ctx, kraken.GetNominalVoltage(), 12.0_V, tolerance);
            TEST_EXPECT_NEAR(ctx, kraken.GetStallTorque(), 7.09_Nm, tolerance);
            TEST_EXPECT_NEAR(ctx, kraken.GetStallCurrent(), 366.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, kraken.GetFreeSpeed(), 6000.0_rpm, 0.05);
            TEST_EXPECT_NEAR(ctx, kraken.GetFreeCurrent(), 2.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, kraken.GetRotorInertia(), 5.6e-5, 1e-7);

            // Stall condition: at omega = 0, current must equal stall current
            double stallCurrent = kraken.CalculateCurrent(12.0_V, 0.0_radps);
            TEST_EXPECT_NEAR(ctx, stallCurrent, 366.0_A, 0.01);

            // Free speed condition: at free speed, current must equal free current
            double freeCurrent = kraken.CalculateCurrent(12.0_V, kraken.GetFreeSpeed());
            TEST_EXPECT_NEAR(ctx, freeCurrent, 2.0_A, 0.01);

            // Falcon 500
            DCMotorModel falcon = DCMotorModel::Falcon500();
            TEST_EXPECT_NEAR(ctx, falcon.GetStallTorque(), 4.69_Nm, tolerance);
            TEST_EXPECT_NEAR(ctx, falcon.GetStallCurrent(), 257.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, falcon.GetFreeSpeed(), 6380.0_rpm, 0.05);

            // NEO v1.1
            DCMotorModel neo = DCMotorModel::NEO1();
            TEST_EXPECT_NEAR(ctx, neo.GetStallTorque(), 2.60_Nm, tolerance);
            TEST_EXPECT_NEAR(ctx, neo.GetStallCurrent(), 105.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, neo.GetFreeSpeed(), 5676.0_rpm, 0.05);

            // NEO Vortex
            DCMotorModel vortex = DCMotorModel::NEOVortex();
            TEST_EXPECT_NEAR(ctx, vortex.GetStallTorque(), 3.60_Nm, tolerance);
            TEST_EXPECT_NEAR(ctx, vortex.GetStallCurrent(), 211.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, vortex.GetFreeSpeed(), 6784.0_rpm, 0.05);

            // goBILDA 5202 (FTC)
            DCMotorModel goBilda = DCMotorModel::GoBilda5202();
            TEST_EXPECT_NEAR(ctx, goBilda.GetStallTorque(), 0.25_Nm, tolerance);
            TEST_EXPECT_NEAR(ctx, goBilda.GetStallCurrent(), 9.2_A, tolerance);
            TEST_EXPECT_NEAR(ctx, goBilda.GetFreeSpeed(), 6000.0_rpm, 0.05);
        }

        // -------------------------------------------------------------
        // 2. Reflected Inertia & Gearing
        // -------------------------------------------------------------
        {
            constexpr double gearRatio = 6.12;
            constexpr double efficiency = 0.96;
            DCMotorModel gearedKraken = DCMotorModel::KrakenX60(gearRatio, efficiency);

            double externalInertia = 0.005; // 0.005 kg*m^2 at output
            double expectedReflected = (5.6e-5 * gearRatio * gearRatio) + externalInertia;
            TEST_EXPECT_NEAR(ctx, gearedKraken.GetReflectedInertia(externalInertia), expectedReflected, 1e-6);

            // Stall torque at output shaft: tau_out = G * eta * tau_stall
            double expectedOutputStall = gearRatio * efficiency * 7.09_Nm;
            TEST_EXPECT_NEAR(ctx, gearedKraken.GetGearRatio(), gearRatio, tolerance);
            TEST_EXPECT_NEAR(ctx, gearedKraken.GetGearboxEfficiency(), efficiency, tolerance);
        }

        // -------------------------------------------------------------
        // 3. Semi-Implicit Integration Stability (G = 100, dt = 5 ms)
        // -------------------------------------------------------------
        {
            // Extreme gear ratio that would destabilize explicit Euler back-EMF
            constexpr double extremeGearRatio = 100.0;
            constexpr double efficiency = 0.95;
            DCMotorModel highRatioMotor = DCMotorModel::NEO1(extremeGearRatio, efficiency);

            double angularVelocity = 0.0_radps;
            double loadInertia = 0.001; // 0.001 kg*m^2
            constexpr double dt = 0.005; // 200 Hz

            // Simulate 50 steps (250 ms) of acceleration from rest
            double previousVelocity = -1.0;
            bool monotonicallyIncreasing = true;
            bool noNaNs = true;

            for (int step = 0; step < 50; ++step)
            {
                double torque = highRatioMotor.CalculateTorque(12.0_V, angularVelocity, loadInertia, dt);
                if (std::isnan(torque) || std::isinf(torque))
                {
                    noNaNs = false;
                    break;
                }

                double effectiveInertia = highRatioMotor.GetReflectedInertia(loadInertia);
                angularVelocity += (torque / effectiveInertia) * dt;

                if (angularVelocity < previousVelocity)
                {
                    monotonicallyIncreasing = false;
                }
                previousVelocity = angularVelocity;
            }

            TEST_EXPECT_TRUE(ctx, noNaNs);
            TEST_EXPECT_TRUE(ctx, monotonicallyIncreasing);

                        // Max speed at output cannot exceed the theoretical zero-torque electrical speed: V_nom / (Ke * G)
            double maxOutputSpeed = highRatioMotor.GetNominalVoltage() / (highRatioMotor.GetBackEmfConstantKe() * extremeGearRatio);
            TEST_EXPECT_TRUE(ctx, angularVelocity <= maxOutputSpeed + 1e-3);
            TEST_EXPECT_NEAR(ctx, angularVelocity, maxOutputSpeed, 0.01);
        }

        // -------------------------------------------------------------
        // 4. Battery Voltage Sag & Lumped Resistance
        // -------------------------------------------------------------
        {
            BatteryModel battery(12.60_V, 0.023_ohm, 1.5_A);

            // Quiescent voltage: 12.60 - (1.5 * 0.023) = 12.5655 V
            double vQuiescent = battery.CalculateTerminalVoltage(0.0_A);
            TEST_EXPECT_NEAR(ctx, vQuiescent, 12.5655_V, tolerance);

            // 100A load: 12.60 - (101.5 * 0.023) = 10.2655 V
            double v100A = battery.CalculateTerminalVoltage(100.0_A);
            TEST_EXPECT_NEAR(ctx, v100A, 10.2655_V, tolerance);

            // 200A load: 12.60 - (201.5 * 0.023) = 7.9655 V
            double v200A = battery.CalculateTerminalVoltage(200.0_A);
            TEST_EXPECT_NEAR(ctx, v200A, 7.9655_V, tolerance);

            // Negative load safety clamp
            double vNegative = battery.CalculateTerminalVoltage(-50.0_A);
            TEST_EXPECT_NEAR(ctx, vNegative, 12.5655_V, tolerance);
        }

        // -------------------------------------------------------------
        // 5. Supply Current Limiting & Brownout Protection
        // -------------------------------------------------------------
        {
            BatteryModel battery(12.60_V, 0.023_ohm, 1.5_A, 60.0_A, 40.0_A, 6.8_V, 6.3_V);

            // Brownout warning triggers at < 6.8V
            TEST_EXPECT_FALSE(ctx, battery.IsBrownoutWarning(6.9_V));
            TEST_EXPECT_TRUE(ctx, battery.IsBrownoutWarning(6.75_V));

            // Brownout critical triggers at < 6.3V
            TEST_EXPECT_FALSE(ctx, battery.IsBrownoutCritical(6.4_V));
            TEST_EXPECT_TRUE(ctx, battery.IsBrownoutCritical(6.25_V));

            // Supply current limit throttling:
            // Stator demands 100A at 12V (1200W). Supply limit is 40A at 12V terminal (480W).
            // Throttled voltage should scale by 480 / 1200 = 0.40 -> 4.8V.
            double throttledVolts = battery.ApplySupplyCurrentLimit(12.0_V, 100.0_A, 12.0_V);
            TEST_EXPECT_NEAR(ctx, throttledVolts, 4.80_V, 0.01);

            // Zero current / idle must not produce zero division or NaN
            double idleVolts = battery.ApplySupplyCurrentLimit(12.0_V, 0.0_A, 12.0_V);
            TEST_EXPECT_NEAR(ctx, idleVolts, 12.0_V, tolerance);

            // Zero terminal voltage must return 0V safely
            double deadBusVolts = battery.ApplySupplyCurrentLimit(12.0_V, 50.0_A, 0.0_V);
            TEST_EXPECT_NEAR(ctx, deadBusVolts, 0.0_V, tolerance);
        }

        // -------------------------------------------------------------
        // 6. ECS Actuation Components Default Initialization
        // -------------------------------------------------------------
        {
            MotorComponent motorComp;
            TEST_EXPECT_NEAR(ctx, motorComp.commandedVoltage, 0.0_V, tolerance);
            TEST_EXPECT_NEAR(ctx, motorComp.statorCurrent, 0.0_A, tolerance);
            TEST_EXPECT_NEAR(ctx, motorComp.statorCurrentLimit, 60.0_A, tolerance);

            BatteryComponent batteryComp;
            TEST_EXPECT_NEAR(ctx, batteryComp.terminalVoltage, 12.60_V, tolerance);
            TEST_EXPECT_NEAR(ctx, batteryComp.totalCurrent, 1.5_A, tolerance);
            TEST_EXPECT_FALSE(ctx, batteryComp.isBrownoutWarning);
            TEST_EXPECT_FALSE(ctx, batteryComp.isBrownoutCritical);
        }

        // -------------------------------------------------------------
        // 7. Servo Datasheet Conversions & Tracking
        // -------------------------------------------------------------
        {
            // goBILDA Torque Preset: 0.20 s/60 deg -> (pi / 3) / 0.20 = 5.236 rad/s (~50 RPM)
            ServoModel servo = ServoModel::GoBildaTorque();
            TEST_EXPECT_NEAR(ctx, servo.GetNominalVoltage(), 6.0_V, tolerance);
            TEST_EXPECT_NEAR(ctx, servo.GetStallTorque(), 21.6_kgcm, 0.01);
            TEST_EXPECT_NEAR(ctx, servo.GetStallCurrent(), 2.5_A, tolerance);
            TEST_EXPECT_NEAR(ctx, servo.GetFreeSpeed(), 5.2359877_radps, 0.01);

            // Positional target setting: normalized 0.5 = center (0 rad)
            servo.SetTargetNormalized(0.5);
            TEST_EXPECT_NEAR(ctx, servo.GetTargetAngle(), 0.0_rad, tolerance);

            // Target at +45 degrees from 0 rad produces positive voltage & torque
            servo.SetTargetAngle(45.0_deg);

            // Step the locked rotor (currentAngle = 0 rad) for 10 ticks (50 ms) so the internal
            // trajectory profile advances past the 8-degree full-voltage saturation threshold
            double torque = 0.0;
            for (int tick = 0; tick < 10; ++tick)
            {
                torque = servo.CalculateTorque(0.0_rad, 0.0_radps, 0.0001, 0.005);
            }

            TEST_EXPECT_TRUE(ctx, torque > 0.0);
            TEST_EXPECT_NEAR(ctx, servo.GetLastAppliedVoltage(), 6.0_V, tolerance); // saturated to full volts

            // Stall current under locked-rotor condition with full voltage
            double current = servo.CalculateCurrent(0.0_radps);
            TEST_EXPECT_NEAR(ctx, current, 2.5_A, 0.05);
        }
        
        std::cout << "[DONE] MotorBatteryTest\n";
    }
}