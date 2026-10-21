/**
 * @file MainTests.cpp
 * @brief Entry point for headless CTest suites (Units, Parameters, Determinism).
 */

#include "TestFramework.h"
#include "UnitsTest.h"
#include "ParameterStoreTest.h"
#include "PhysicsDeterminismTest.h"

#include <iostream>
#include <string>

using FRCBananaSim::Tests::TestContext;
using FRCBananaSim::Tests::RunUnitsTest;
using FRCBananaSim::Tests::RunParameterStoreTest;
using FRCBananaSim::Tests::RunPhysicsDeterminismTest;

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
    }

    if (runAll)
    {
        RunUnitsTest(ctx);
        RunParameterStoreTest(ctx);
        RunPhysicsDeterminismTest(ctx);
    }

    std::cout << "======================================\n";
    std::cout << " Results: " << ctx.passed << " passed, " << ctx.failed << " failed.\n";
    std::cout << "======================================\n";

    return ctx.IsSuccessful() ? 0 : 1;
}