/**
 * @file MainTests.cpp
 * @brief Entry point for headless CTest suites (Units, Parameters, Determinism, Motor & Battery, Joint Stability, Swerve Physical Drive, Robot Spawner, Swerve Mass Ratio).
 */

#include "TestFramework.h"
#include "UnitsTest.h"
#include "ParameterStoreTest.h"
#include "PhysicsDeterminismTest.h"
#include "MotorBatteryTest.h"
#include "JointStabilityTest.h"
#include "SwervePhysicalDriveTest.h"
#include "RobotSpawnerTest.h"
#include "SwerveMassRatioTest.h"

#include <iostream>
#include <string>

using FRCBananaSim::Tests::TestContext;
using FRCBananaSim::Tests::RunUnitsTest;
using FRCBananaSim::Tests::RunParameterStoreTest;
using FRCBananaSim::Tests::RunPhysicsDeterminismTest;
using FRCBananaSim::Tests::RunMotorBatteryTest;
using FRCBananaSim::Tests::RunJointStabilityTest;
using FRCBananaSim::Tests::RunSwervePhysicalDriveTest;
using FRCBananaSim::Tests::RunRobotSpawnerTest;
using FRCBananaSim::Tests::RunSwerveMassRatioTest;

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
        else if (arg == "--swerve-drive")
        {
            runAll = false;
            RunSwervePhysicalDriveTest(ctx);
        }
        else if (arg == "--robot-spawner")
        {
            runAll = false;
            RunRobotSpawnerTest(ctx);
        }
        else if (arg == "--mass-ratio")
        {
            runAll = false;
            RunSwerveMassRatioTest(ctx);
        }
    }

    if (runAll)
    {
        RunUnitsTest(ctx);
        RunParameterStoreTest(ctx);
        RunPhysicsDeterminismTest(ctx);
        RunMotorBatteryTest(ctx);
        RunJointStabilityTest(ctx);
        RunSwervePhysicalDriveTest(ctx);
        RunRobotSpawnerTest(ctx);
        RunSwerveMassRatioTest(ctx);
    }

    std::cout << "======================================\n";
    std::cout << " Results: " << ctx.passed << " passed, " << ctx.failed << " failed.\n";
    std::cout << "======================================\n";

    return ctx.IsSuccessful() ? 0 : 1;
}