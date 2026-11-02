/**
 * @file ParameterStoreTest.cpp
 * @brief Implementation of parameter store parsing, SI extraction, and merge tests.
 */

#include "ParameterStoreTest.h"
#include "TestFramework.h"
#include "Core/Parameters/ParameterStore.h"

#include <iostream>

namespace FRCBananaSim::Tests
{
    using Core::Parameters::ParameterStore;

    void RunParameterStoreTest(TestContext& ctx)
    {
        std::cout << "[RUN ] ParameterStoreTest\n";
        constexpr double tolerance = 1e-6;

        // 1. Ingestion of dimensional JSON
        const std::string defaultJson = R"({
            "robot": {
                "chassis": {
                    "mass": { "value": 50.0, "unit": "kg" },
                    "width": { "value": 28.0, "unit": "in" },
                    "length": { "value": 28.0, "unit": "in" },
                    "gearRatio": 6.75,
                    "hasBumpers": true
                },
                "shooter": {
                    "nominalRpm": { "value": 5500.0, "unit": "rpm" }
                }
            }
        })";

        ParameterStore defaultStore;
        TEST_EXPECT_TRUE(ctx, defaultStore.LoadFromString(defaultJson));
        TEST_EXPECT_TRUE(ctx, defaultStore.Has("robot.chassis.mass"));
        TEST_EXPECT_TRUE(ctx, defaultStore.Has("robot.chassis.width"));
        TEST_EXPECT_TRUE(ctx, defaultStore.Has("robot.chassis.gearRatio"));
        TEST_EXPECT_TRUE(ctx, defaultStore.Has("robot.chassis.hasBumpers"));

        // Raw value vs SI value
        TEST_EXPECT_NEAR(ctx, defaultStore.GetDouble("robot.chassis.width"), 28.0, tolerance);
        TEST_EXPECT_NEAR(ctx, defaultStore.GetSI("robot.chassis.width"), 28.0 * 0.0254, tolerance);
        TEST_EXPECT_EQ(ctx, defaultStore.GetUnit("robot.chassis.width"), "in");
        TEST_EXPECT_NEAR(ctx, defaultStore.GetSI("robot.chassis.mass"), 50.0, tolerance);
        TEST_EXPECT_NEAR(ctx, defaultStore.GetDouble("robot.chassis.gearRatio"), 6.75, tolerance);
        TEST_EXPECT_TRUE(ctx, defaultStore.GetBool("robot.chassis.hasBumpers"));

        // 2. Layered Merge (measured overrides defaults, but unmentioned defaults remain)
        const std::string measuredJson = R"({
            "robot": {
                "chassis": {
                    "mass": { "value": 53.2, "unit": "kg" }
                }
            }
        })";

        ParameterStore measuredStore;
        TEST_EXPECT_TRUE(ctx, measuredStore.LoadFromString(measuredJson));

        // Merge measured on top of default
        defaultStore.Merge(measuredStore);

        // Mass was updated
        TEST_EXPECT_NEAR(ctx, defaultStore.GetSI("robot.chassis.mass"), 53.2, tolerance);
        // Width and gear ratio were preserved from defaults
        TEST_EXPECT_NEAR(ctx, defaultStore.GetDouble("robot.chassis.width"), 28.0, tolerance);
        TEST_EXPECT_NEAR(ctx, defaultStore.GetDouble("robot.chassis.gearRatio"), 6.75, tolerance);

        std::cout << "[DONE] ParameterStoreTest\n";
    }
}