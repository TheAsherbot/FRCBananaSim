/**
 * @file DrivetrainLogger.cpp
 * @brief Implementation of the pre-allocated telemetry ring buffer and CSV exporter.
 */

#include "Telemetry/DrivetrainLogger.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"
#include "Utils/Log.h"

#include <algorithm>
#include <fstream>
#include <iomanip>

namespace FRCBananaSim::Telemetry
{
    DrivetrainLogger::DrivetrainLogger(size_t capacity)
        : bufferCapacity(std::max<size_t>(1, capacity)),
        writeHead(0),
        sampleCount(0),
        isActivelyRecording(false)
    {
        buffer.resize(bufferCapacity);
    }

    void DrivetrainLogger::StartRecording()
    {
        isActivelyRecording = true;
    }

    void DrivetrainLogger::StopRecording()
    {
        isActivelyRecording = false;
    }

    bool DrivetrainLogger::IsRecording() const
    {
        return isActivelyRecording;
    }

    void DrivetrainLogger::Clear()
    {
        writeHead = 0;
        sampleCount = 0;
    }

    void DrivetrainLogger::RecordSample(const DrivetrainLogSample& sample)
    {
        if (!isActivelyRecording)
        {
            return;
        }

        buffer[writeHead] = sample;
        writeHead = (writeHead + 1) % bufferCapacity;

        if (sampleCount < bufferCapacity)
        {
            ++sampleCount;
        }
    }

    size_t DrivetrainLogger::GetSampleCount() const
    {
        return sampleCount;
    }

    size_t DrivetrainLogger::GetCapacity() const
    {
        return bufferCapacity;
    }

    const DrivetrainLogSample& DrivetrainLogger::GetSample(size_t logicalIndex) const
    {
        if (logicalIndex >= sampleCount)
        {
            static const DrivetrainLogSample emptySample{};
            return emptySample;
        }

        if (sampleCount < bufferCapacity)
        {
            return buffer[logicalIndex];
        }

        // Buffer wrapped around: writeHead points to the oldest entry
        size_t physicalIndex = (writeHead + logicalIndex) % bufferCapacity;
        return buffer[physicalIndex];
    }

    bool DrivetrainLogger::ExportToCsv(const std::string& filepath) const
    {
        SIM_PROFILE_SCOPE("DrivetrainLogger::ExportToCsv");

        std::ofstream file(filepath);
        if (!file.is_open())
        {
            SIM_LOG_ERROR("Failed to open file for CSV export: " << filepath);
            return false;
        }

        // CSV Header
        file << "timestamp_s,commanded_volts,battery_volts,total_current_a,"
            << "chassis_speed_mps,chassis_elevation_m,gyro_yaw_deg,gyro_pitch_deg,gyro_roll_deg,gyro_yaw_rate_degs,"
            << "steer_angle_fl_deg,steer_angle_fr_deg,steer_angle_bl_deg,steer_angle_br_deg,"
            << "wheel_vel_fl_rads,wheel_vel_fr_rads,wheel_vel_bl_rads,wheel_vel_br_rads,"
            << "drive_current_fl_a,drive_current_fr_a,drive_current_bl_a,drive_current_br_a,"
            << "normal_force_fl_n,normal_force_fr_n,normal_force_bl_n,normal_force_br_n,"
            << "slip_ratio_fl,slip_ratio_fr,slip_ratio_bl,slip_ratio_br\n";

        file << std::fixed << std::setprecision(5);

        for (size_t i = 0; i < sampleCount; ++i)
        {
            const auto& s = GetSample(i);
            file << s.timestamp << ","
                << s.commandedVoltage << ","
                << s.batteryVoltage << ","
                << s.totalBusCurrent << ","
                << s.chassisSpeed << ","
                << s.chassisElevation << ","
                << s.gyroYawDeg << ","
                << s.gyroPitchDeg << ","
                << s.gyroRollDeg << ","
                << s.gyroYawRateDegS << ","
                << s.steerAnglesDeg[0] << "," << s.steerAnglesDeg[1] << "," << s.steerAnglesDeg[2] << "," << s.steerAnglesDeg[3] << ","
                << s.wheelVelocitiesRadS[0] << "," << s.wheelVelocitiesRadS[1] << "," << s.wheelVelocitiesRadS[2] << "," << s.wheelVelocitiesRadS[3] << ","
                << s.driveCurrentsA[0] << "," << s.driveCurrentsA[1] << "," << s.driveCurrentsA[2] << "," << s.driveCurrentsA[3] << ","
                << s.normalForcesN[0] << "," << s.normalForcesN[1] << "," << s.normalForcesN[2] << "," << s.normalForcesN[3] << ","
                << s.slipRatios[0] << "," << s.slipRatios[1] << "," << s.slipRatios[2] << "," << s.slipRatios[3] << "\n";
        }

        SIM_LOG_INFO("Exported " << sampleCount << " telemetry samples to CSV: " << filepath);
        return true;
    }

    bool DrivetrainLogger::ExportToFtcSchemaCsv(const std::string& filepath) const
    {
        SIM_PROFILE_SCOPE("DrivetrainLogger::ExportToFtcSchemaCsv");

        std::ofstream file(filepath);
        if (!file.is_open())
        {
            SIM_LOG_ERROR("Failed to open file for FTC CSV export: " << filepath);
            return false;
        }

        // FTC standard calibration schema
        file << "timestamp_s,commanded_volts,wheel_velocity_rad_s,battery_volts,current_draw_a\n";
        file << std::fixed << std::setprecision(5);

        for (size_t i = 0; i < sampleCount; ++i)
        {
            const auto& s = GetSample(i);
            double avgWheelVel = (s.wheelVelocitiesRadS[0] + s.wheelVelocitiesRadS[1] +
                s.wheelVelocitiesRadS[2] + s.wheelVelocitiesRadS[3]) * 0.25;

            file << s.timestamp << ","
                << s.commandedVoltage << ","
                << avgWheelVel << ","
                << s.batteryVoltage << ","
                << s.totalBusCurrent << "\n";
        }

        SIM_LOG_INFO("Exported " << sampleCount << " samples in FTC hardware schema to: " << filepath);
        return true;
    }
}