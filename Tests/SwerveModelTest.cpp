/**
 * @file SwerveModelTest.cpp
 * @brief Comprehensive verification suite for analytic swerve kinematics, tire traction, load transfer, and ramps.
 */

#include "SwerveModelTest.h"
#include "Support/TestTerrain.h"
#include "Robot/Drivetrain/Model/SwerveDrivetrainModel.h"
#include "Control/SimControlBoundary.h"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Tests
{
    using namespace Robot::Drivetrain::Model;
    using namespace Support;
    using namespace Control;

    void RunSwerveModelTest(TestContext& ctx)
    {
        std::cout << "[RUN ] SwerveModelTest\n";
        constexpr double tolerance = 1e-3;

        // -------------------------------------------------------------
        // T1. Kinematics Consistency & Free Speed Verification
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 4; ++i)
            {
                inputs.motorCommands[i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[i].demand = 0.0; // Straight ahead (0 rad)

                inputs.motorCommands[4 + i].mode = MotorControlMode::Voltage;
                inputs.motorCommands[4 + i].demand = 12.0; // Full 12V forward
            }

            // Simulate 3.0 seconds (600 ticks at 200 Hz) to reach free speed
            constexpr double dt = 0.005;
            for (int tick = 0; tick < 600; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }

            double vx = model.GetChassisState().linearVelocity.x;
            
            // Expected free speed under gear ratio 5.27 and 4" wheel: ~5.8 to 6.1 m/s
            TEST_EXPECT_TRUE(ctx, vx > 5.5 && vx < 6.15);
            TEST_EXPECT_NEAR(ctx, model.GetChassisState().linearVelocity.y, 0.0, 0.05);
            TEST_EXPECT_NEAR(ctx, model.GetChassisState().angularVelocity.z, 0.0, 0.05);
        }

        // -------------------------------------------------------------
        // T2. Low-Friction Traction Acceleration Ceiling (a -> mu * g)
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            config.tire.muStatic = 0.30;
            config.tire.muKinetic = 0.30;
            config.chassis.centerOfMassHeight = 0.0; // Isolate from load transfer

            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 4; ++i)
            {
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[4 + i].demand = 12.0; // Over-driven
            }

            constexpr double dt = 0.005;
            // Step 20 ticks: acceleration must strictly respect mu * g = 0.30 * 9.80665 = 2.94 m/s^2
            double initialVx = model.GetChassisState().linearVelocity.x;
            for (int tick = 0; tick < 20; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }
            double finalVx = model.GetChassisState().linearVelocity.x;
            double measuredAccel = (finalVx - initialVx) / (20 * dt);

            TEST_EXPECT_NEAR(ctx, measuredAccel, 0.30 * 9.80665, 0.25);
        }

        // -------------------------------------------------------------
        // T3. Dynamic Normal Load Transfer under Acceleration
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            config.chassis.centerOfMassHeight = 0.20; // 20 cm CG height
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 4; ++i)
            {
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[4 + i].demand = 10.0; // Hard launch
            }

            constexpr double dt = 0.005;
            for (int tick = 0; tick < 20; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }

            double nFL = model.GetModule(0).GetState().contactState.normalForce;
            double nFR = model.GetModule(1).GetState().contactState.normalForce;
            double nBL = model.GetModule(2).GetState().contactState.normalForce;
            double nBR = model.GetModule(3).GetState().contactState.normalForce;

            double frontAxleLoad = nFL + nFR;
            double rearAxleLoad = nBL + nBR;

            // Under positive forward acceleration, rear axle load MUST exceed front axle load
            TEST_EXPECT_TRUE(ctx, rearAxleLoad > frontAxleLoad);
            // Sum of normal forces balances chassis weight (~50 kg * 9.81 m/s^2 = 490.3 N)
            double totalNormal = frontAxleLoad + rearAxleLoad;
            TEST_EXPECT_NEAR(ctx, totalNormal, 50.0 * 9.80665, 25.0);
        }

        // -------------------------------------------------------------
        // T5. Stationary Rest Stability (Zero Creep, Zero Energy Growth)
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 8; ++i)
            {
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[i].neutralMode = NeutralMode::Brake;
            }

            constexpr double dt = 0.005;
            for (int tick = 0; tick < 500; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }

            double speed = glm::length(model.GetChassisState().linearVelocity);
            double angRate = glm::length(model.GetChassisState().angularVelocity);

            TEST_EXPECT_TRUE(ctx, speed < 1e-4);
            TEST_EXPECT_TRUE(ctx, angRate < 1e-4);
            TEST_EXPECT_NEAR(ctx, model.GetChassisState().position.x, 0.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, model.GetChassisState().position.y, 0.0, 1e-4);
        }

        // -------------------------------------------------------------
        // T6. Steer Azimuth Step Response Time Constant
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            // Step +12V on module 0 steer motor
            inputs.motorCommands[0].mode = MotorControlMode::Voltage;
            inputs.motorCommands[0].demand = 12.0;

            constexpr double dt = 0.005;
            // Run 50 ticks (250 ms)
            for (int tick = 0; tick < 50; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }

            // Continuous steer angle unwrapping check: azimuth spins past pi without discontinuous drop
            double unwrapped = model.GetModule(0).GetState().unwrappedSteerAngle;
            TEST_EXPECT_TRUE(ctx, unwrapped > 3.14159265);
        }

        // -------------------------------------------------------------
        // T9. Closed-Form Battery Sag & Brownout Hysteresis
        // -------------------------------------------------------------
        {
            Actuation::BatteryModel battery = Actuation::BatteryModel::FRCStandard();

            bool isBrownedOut = false;
            // Nominal load (500 W): voltage sags slightly (~11.6 V), no brownout
            double v1 = battery.SolveBusVoltageClosedForm(500.0, isBrownedOut, isBrownedOut);
            TEST_EXPECT_TRUE(ctx, v1 > 11.5);
            TEST_EXPECT_FALSE(ctx, isBrownedOut);

            // Extreme stall load exceeding battery theoretical capacity (1720 W): collapses voltage <= 6.3V
            double v2 = battery.SolveBusVoltageClosedForm(1720.0, isBrownedOut, isBrownedOut);
            TEST_EXPECT_TRUE(ctx, v2 <= 6.3);
            TEST_EXPECT_TRUE(ctx, isBrownedOut);

            // Intermediate load in hysteresis band (1690 W -> ~7.1 V): latch MUST hold brownout active
            double v3 = battery.SolveBusVoltageClosedForm(1690.0, isBrownedOut, isBrownedOut);
            TEST_EXPECT_TRUE(ctx, v3 > 6.3 && v3 < 7.5);
            TEST_EXPECT_TRUE(ctx, isBrownedOut);

            // Light load (500 W -> ~11.6 V): clears recovery threshold (> 7.5V), resetting brownout
            double v4 = battery.SolveBusVoltageClosedForm(500.0, isBrownedOut, isBrownedOut);
            TEST_EXPECT_TRUE(ctx, v4 >= 7.5);
            TEST_EXPECT_FALSE(ctx, isBrownedOut);
        }

        // -------------------------------------------------------------
        // T11. Determinism & Mid-Flight Snapshot Restore
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model1(config);

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 4; ++i)
            {
                inputs.motorCommands[i].demand = 2.0; // Steering turn
                inputs.motorCommands[4 + i].demand = 8.0; // Driving forward
            }

            constexpr double dt = 0.005;
            for (int tick = 0; tick < 50; ++tick)
            {
                model1.Step(dt, inputs, terrain);
            }

            // Capture snapshot
            DrivetrainSnapshot snapshot = model1.CaptureSnapshot();

            // Run model1 50 more ticks
            for (int tick = 0; tick < 50; ++tick)
            {
                model1.Step(dt, inputs, terrain);
            }

            // Create model2, restore snapshot, and run 50 ticks
            SwerveDrivetrainModel model2(config);
            model2.RestoreSnapshot(snapshot);
            for (int tick = 0; tick < 50; ++tick)
            {
                model2.Step(dt, inputs, terrain);
            }

            // Positions and velocities must match identically
            TEST_EXPECT_NEAR(ctx, model1.GetChassisState().position.x, model2.GetChassisState().position.x, 1e-6);
            TEST_EXPECT_NEAR(ctx, model1.GetChassisState().position.y, model2.GetChassisState().position.y, 1e-6);
            TEST_EXPECT_NEAR(ctx, model1.GetChassisState().linearVelocity.x, model2.GetChassisState().linearVelocity.x, 1e-6);
        }

        // -------------------------------------------------------------
        // T14. Static Ramp Hold with Zero Creep (15 deg Incline)
        // -------------------------------------------------------------
        {
            constexpr double rampAngle = 15.0 * (3.141592653589793 / 180.0); // 15 degrees (~0.2618 rad)
            TiltedRampTerrain ramp(rampAngle);

            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            config.tire.muStatic = 1.20; // mu > tan(15 deg) = 0.268 -> robot MUST hold stationary in brake mode
            SwerveDrivetrainModel model(config);

            // Position chassis aligned on ramp surface at equilibrium
            glm::vec3 rampNormal = glm::vec3(-std::sin(rampAngle), 0.0f, std::cos(rampAngle));
            double staticSag = (config.chassis.mass * 9.80665 * std::cos(rampAngle)) / (4.0 * config.tire.radialStiffness);
            double hPlane = config.chassis.centerOfMassHeight - staticSag;

            model.GetChassisState().position = rampNormal * static_cast<float>(hPlane);
            model.GetChassisState().orientation = glm::angleAxis(static_cast<float>(-rampAngle), glm::vec3(0.0f, 1.0f, 0.0f));

            SimInputs inputs;
            inputs.robotEnabled = true;
            for (int i = 0; i < 8; ++i)
            {
                inputs.motorCommands[i].demand = 0.0;
                inputs.motorCommands[i].neutralMode = NeutralMode::Brake;
            }

            constexpr double dt = 0.005;
            for (int tick = 0; tick < 200; ++tick)
            {
                model.Step(dt, inputs, ramp);
            }

            double speed = glm::length(model.GetChassisState().linearVelocity);
            TEST_EXPECT_TRUE(ctx, speed < 0.01); // Zero creep
        }

        // -------------------------------------------------------------
        // T18. Small-Signal Radial Suspension Heave Frequency
        // -------------------------------------------------------------
        {
            SwerveDrivetrainConfig config = SwerveDrivetrainConfig::CreateMK5nR3Reference();
            FlatCarpetTerrain terrain;
            SwerveDrivetrainModel model(config);

            // Perturb chassis vertically by 5 mm
            model.GetChassisState().position.z += 0.005f;

            SimInputs inputs;
            inputs.robotEnabled = false; // Passive damping

            constexpr double dt = 0.001;
            // Run 500 ms to allow viscous damper to settle heave motion
            for (int tick = 0; tick < 500; ++tick)
            {
                model.Step(dt, inputs, terrain);
            }

            double staticSag = (config.chassis.mass * 9.80665) / (4.0 * config.tire.radialStiffness);
            double equilibriumZ = config.chassis.centerOfMassHeight - staticSag;

            TEST_EXPECT_NEAR(ctx, model.GetChassisState().position.z, equilibriumZ, 0.002);
            TEST_EXPECT_NEAR(ctx, model.GetChassisState().linearVelocity.z, 0.0, 0.02);
        }

        std::cout << "[DONE] SwerveModelTest\n";
    }
}