/**
 * @file DrivetrainLoggerTest.cpp
 * @brief Implementation of unit tests for the DrivetrainLogger class.
 */

#include "DrivetrainLoggerTest.h"
#include "Telemetry/DrivetrainLogger.h"
#include <cstdio>
#include <fstream>

namespace FRCBananaSim::Tests
{
    using namespace FRCBananaSim::Telemetry;

    void RunDrivetrainLoggerTest(TestContext& ctx)
    {
        std::cout << "[RUN ] DrivetrainLoggerTest\n";

        // 1. Initial State & Recording Toggle
        {
            DrivetrainLogger logger(100);
            TEST_EXPECT_EQ(ctx, static_cast<int>(logger.GetSampleCount()), 0);
            TEST_EXPECT_EQ(ctx, static_cast<int>(logger.GetCapacity()), 100);
            TEST_EXPECT_FALSE(ctx, logger.IsRecording());

            DrivetrainLogSample sample;
            sample.timestamp = 0.05;
            logger.RecordSample(sample);
            TEST_EXPECT_EQ(ctx, static_cast<int>(logger.GetSampleCount()), 0);

            logger.StartRecording();
            TEST_EXPECT_TRUE(ctx, logger.IsRecording());

            logger.RecordSample(sample);
            TEST_EXPECT_EQ(ctx, static_cast<int>(logger.GetSampleCount()), 1);

            logger.StopRecording();
            TEST_EXPECT_FALSE(ctx, logger.IsRecording());
        }

        // 2. Ring Buffer Wrap-Around Correctness
        {
            DrivetrainLogger logger(5); // Capacity of 5
            logger.StartRecording();

            for (int i = 0; i < 8; ++i)
            {
                DrivetrainLogSample s;
                s.timestamp = static_cast<double>(i) * 0.1;
                s.commandedVoltage = static_cast<double>(i);
                logger.RecordSample(s);
            }

            TEST_EXPECT_EQ(ctx, static_cast<int>(logger.GetSampleCount()), 5);

            // Oldest sample should be i=3, newest should be i=7
            TEST_EXPECT_NEAR(ctx, logger.GetSample(0).commandedVoltage, 3.0, 1e-4);
            TEST_EXPECT_NEAR(ctx, logger.GetSample(4).commandedVoltage, 7.0, 1e-4);
        }

        // 3. CSV File Export
        {
            DrivetrainLogger logger(10);
            logger.StartRecording();

            DrivetrainLogSample s1;
            s1.timestamp = 0.005;
            s1.commandedVoltage = 12.0;
            s1.batteryVoltage = 10.5;
            s1.totalBusCurrent = 45.0;
            s1.wheelVelocitiesRadS[0] = 50.0;
            s1.wheelVelocitiesRadS[1] = 50.0;
            s1.wheelVelocitiesRadS[2] = 50.0;
            s1.wheelVelocitiesRadS[3] = 50.0;
            logger.RecordSample(s1);

            const std::string fullCsvPath = "test_full_log.csv";
            const std::string ftcCsvPath = "test_ftc_log.csv";

            bool fullSuccess = logger.ExportToCsv(fullCsvPath);
            TEST_EXPECT_TRUE(ctx, fullSuccess);

            bool ftcSuccess = logger.ExportToFtcSchemaCsv(ftcCsvPath);
            TEST_EXPECT_TRUE(ctx, ftcSuccess);

            // Verify FTC file content
            std::ifstream ftcFile(ftcCsvPath);
            std::string headerLine;
            std::getline(ftcFile, headerLine);
            TEST_EXPECT_EQ(ctx, headerLine, std::string("timestamp_s,commanded_volts,wheel_velocity_rad_s,battery_volts,current_draw_a"));

            ftcFile.close();
            std::remove(fullCsvPath.c_str());
            std::remove(ftcCsvPath.c_str());
        }

        std::cout << "[DONE] DrivetrainLoggerTest\n";
    }
}