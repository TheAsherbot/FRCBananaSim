/**
 * @file TestFramework.h
 * @brief Lightweight headless unit test assertion and reporting macros.
 */

#pragma once

#include <iostream>
#include <string>
#include <cmath>

namespace FRCBananaSim::Tests
{
    /**
     * @struct TestContext
     * @brief Aggregates assertion pass and fail counts for a test execution run.
     */
    struct TestContext
    {
        int passed{ 0 };
        int failed{ 0 };

        bool IsSuccessful() const
        {
            return failed == 0;
        }
    };
}

#define TEST_EXPECT_TRUE(ctx, condition) \
    do { \
        if (condition) { \
            ++(ctx).passed; \
        } else { \
            ++(ctx).failed; \
            std::cerr << "  [FAIL] " << __FILE__ << ":" << __LINE__ << ": Expected (" #condition ") to be true\n"; \
        } \
    } while (false)

#define TEST_EXPECT_FALSE(ctx, condition) \
    do { \
        if (!(condition)) { \
            ++(ctx).passed; \
        } else { \
            ++(ctx).failed; \
            std::cerr << "  [FAIL] " << __FILE__ << ":" << __LINE__ << ": Expected (" #condition ") to be false\n"; \
        } \
    } while (false)

#define TEST_EXPECT_NEAR(ctx, val1, val2, epsilon) \
    do { \
        double diff = std::abs(static_cast<double>(val1) - static_cast<double>(val2)); \
        if (diff <= static_cast<double>(epsilon)) { \
            ++(ctx).passed; \
        } else { \
            ++(ctx).failed; \
            std::cerr << "  [FAIL] " << __FILE__ << ":" << __LINE__ << ": " #val1 " (" << (val1) \
                      << ") != " #val2 " (" << (val2) << ") within epsilon " << (epsilon) \
                      << " (diff: " << diff << ")\n"; \
        } \
    } while (false)

#define TEST_EXPECT_EQ(ctx, val1, val2) \
    do { \
        if ((val1) == (val2)) { \
            ++(ctx).passed; \
        } else { \
            ++(ctx).failed; \
            std::cerr << "  [FAIL] " << __FILE__ << ":" << __LINE__ << ": " #val1 " (" << (val1) \
                      << ") != " #val2 " (" << (val2) << ")\n"; \
        } \
    } while (false)