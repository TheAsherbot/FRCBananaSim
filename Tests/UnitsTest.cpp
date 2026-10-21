/**
 * @file UnitsTest.cpp
 * @brief Implementation of unit conversion tests against canonical SI constants.
 */

#include "UnitsTest.h"
#include "TestFramework.h"
#include "Core/Units/Units.h"

#include <iostream>
#include <numbers>

namespace FRCBananaSim::Tests
{
    using Core::Units::Units;

    void RunUnitsTest(TestContext& ctx)
    {
        std::cout << "[RUN ] UnitsTest\n";
        constexpr double tolerance = 1e-6;

        // Length
        TEST_EXPECT_NEAR(ctx, Units::Inches(1.0), 0.0254, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Feet(1.0), 0.3048, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Centimeters(100.0), 1.0, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Millimeters(1000.0), 1.0, tolerance);

        // Mass
        TEST_EXPECT_NEAR(ctx, Units::Pounds(1.0), 0.45359237, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Grams(1000.0), 1.0, tolerance);

        // Angle
        TEST_EXPECT_NEAR(ctx, Units::Degrees(180.0), std::numbers::pi, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Rotations(1.0), 2.0 * std::numbers::pi, tolerance);

        // Velocity & Rotational Speed
        TEST_EXPECT_NEAR(ctx, Units::RPM(60.0), 2.0 * std::numbers::pi, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::RPM(6000.0), 200.0 * std::numbers::pi, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::MilesPerHour(60.0), 26.8224, tolerance);

        // Torque & Force
        TEST_EXPECT_NEAR(ctx, Units::FootPounds(1.0), 1.3558179483314, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::PoundForce(1.0), 4.4482216152605, tolerance);

        // String parsing ToSI
        TEST_EXPECT_NEAR(ctx, Units::ToSI(10.0, "in"), 0.254, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::ToSI(50.0, "lbs"), 22.6796185, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::ToSI(6000.0, "rpm"), 200.0 * std::numbers::pi, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::ToSI(12.0, "V"), 12.0, tolerance);

        // Direct cross-conversion
        TEST_EXPECT_NEAR(ctx, Units::Convert(1.0, "in", "cm"), 2.54, tolerance);
        TEST_EXPECT_NEAR(ctx, Units::Convert(360.0, "deg", "rot"), 1.0, tolerance);

        // Validation
        TEST_EXPECT_TRUE(ctx, Units::IsValidUnit("in"));
        TEST_EXPECT_TRUE(ctx, Units::IsValidUnit("rpm"));
        TEST_EXPECT_FALSE(ctx, Units::IsValidUnit("banana_units"));

        std::cout << "[DONE] UnitsTest\n";
    }
}