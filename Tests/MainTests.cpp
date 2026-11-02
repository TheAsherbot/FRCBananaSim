/**
 * @file MainTests.cpp
 * @brief Entry point for headless CTest suites including analytic swerve model, Jolt adapter, elevator mechanisms, arms/four-bars/flywheels, and full-robot endurance.
 */

#include "TestFramework.h"
#include "UnitsTest.h"
#include "ParameterStoreTest.h"
#include "PhysicsDeterminismTest.h"
#include "MotorBatteryTest.h"
#include "JointStabilityTest.h"
#include "RobotSpawnerTest.h"
#include "SwerveModelTest.h"
#include "SwerveAdapterTest.h"
#include "HALSimBridgeTest.h"
#include "DrivetrainLoggerTest.h"
#include "RobotSchemaTest.h"
#include "ElevatorMechanismTest.h"
#include "ArmFourBarFlywheelTest.h"
#include "SensorSuiteTest.h"
#include "SimDeviceLockstepTest.h"
#include "RobotAssemblerTest.h"
#include "FullRobotEnduranceTest.h"

#include <iostream>
#include <string>

using FRCBananaSim::Tests::TestContext;
using FRCBananaSim::Tests::RunUnitsTest;
using FRCBananaSim::Tests::RunParameterStoreTest;
using FRCBananaSim::Tests::RunPhysicsDeterminismTest;
using FRCBananaSim::Tests::RunMotorBatteryTest;
using FRCBananaSim::Tests::RunJointStabilityTest;
using FRCBananaSim::Tests::RunRobotSpawnerTest;
using FRCBananaSim::Tests::RunSwerveModelTest;
using FRCBananaSim::Tests::RunSwerveAdapterTest;
using FRCBananaSim::Tests::RunHALSimBridgeTest;
using FRCBananaSim::Tests::RunDrivetrainLoggerTest;
using FRCBananaSim::Tests::RunRobotSchemaTest;
using FRCBananaSim::Tests::RunElevatorMechanismTest;
using FRCBananaSim::Tests::RunArmFourBarFlywheelTest;
using FRCBananaSim::Tests::RunSensorSuiteTest;
using FRCBananaSim::Tests::RunSimDeviceLockstepTest;
using FRCBananaSim::Tests::RunRobotAssemblerTest;
using FRCBananaSim::Tests::RunFullRobotEnduranceTest;

int main(int argc, char* argv[])
{
    std::cout << "======================================\n";
    std::cout << " FRC Banana Simulator - Headless Tests\n";
    std::cout << "======================================\n";

    TestContext ctx;
    bool runAll = true;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--units")
        {
            runAll = false;
            RunUnitsTest(ctx);
        }
        else if (arg == "--params")
        {
            runAll = false;
            RunParameterStoreTest(ctx);
        }
        else if (arg == "--determinism")
        {
            runAll = false;
            RunPhysicsDeterminismTest(ctx);
        }
        else if (arg == "--motor-battery")
        {
            runAll = false;
            RunMotorBatteryTest(ctx);
        }
        else if (arg == "--joint-stability")
        {
            runAll = false;
            RunJointStabilityTest(ctx);
        }
        else if (arg == "--robot-spawner")
        {
            runAll = false;
            RunRobotSpawnerTest(ctx);
        }
        else if (arg == "--swerve-model")
        {
            runAll = false;
            RunSwerveModelTest(ctx);
        }
        else if (arg == "--swerve-adapter")
        {
            runAll = false;
            RunSwerveAdapterTest(ctx);
        }
        else if (arg == "--halsim-bridge")
        {
            runAll = false;
            RunHALSimBridgeTest(ctx);
        }
        else if (arg == "--drivetrain-logger")
        {
            runAll = false;
            RunDrivetrainLoggerTest(ctx);
        }
        else if (arg == "--schema")
        {
            runAll = false;
            RunRobotSchemaTest(ctx);
        }
        else if (arg == "--elevator-mechanism")
        {
            runAll = false;
            RunElevatorMechanismTest(ctx);
        }
        else if (arg == "--arm-fourbar-flywheel")
        {
            runAll = false;
            RunArmFourBarFlywheelTest(ctx);
        }
        else if (arg == "--sensors")
        {
            runAll = false;
            RunSensorSuiteTest(ctx);
        }
        else if (arg == "--simdevice-lockstep")
        {
            runAll = false;
            RunSimDeviceLockstepTest(ctx);
        }
        else if (arg == "--robot-assembler")
        {
            runAll = false;
            RunRobotAssemblerTest(ctx);
        }
        else if (arg == "--full-robot-endurance")
        {
            runAll = false;
            RunFullRobotEnduranceTest(ctx);
        }
    }

    if (runAll)
    {
        RunUnitsTest(ctx);
        RunParameterStoreTest(ctx);
        RunPhysicsDeterminismTest(ctx);
        RunMotorBatteryTest(ctx);
        RunJointStabilityTest(ctx);
        RunRobotSpawnerTest(ctx);
        RunSwerveModelTest(ctx);
        RunSwerveAdapterTest(ctx);
        RunHALSimBridgeTest(ctx);
        RunDrivetrainLoggerTest(ctx);
        RunRobotSchemaTest(ctx);
        RunElevatorMechanismTest(ctx);
        RunArmFourBarFlywheelTest(ctx);
        RunSensorSuiteTest(ctx);
        RunSimDeviceLockstepTest(ctx);
        RunRobotAssemblerTest(ctx);
        RunFullRobotEnduranceTest(ctx);
    }

    std::cout << "======================================\n";
    std::cout << " SUMMARY: " << ctx.passed << " Passed, "
              << ctx.failed << " Failed\n";
    std::cout << "======================================\n";

    return ctx.IsSuccessful() ? 0 : 1;
}