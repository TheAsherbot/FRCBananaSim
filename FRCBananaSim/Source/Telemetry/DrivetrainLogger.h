/**
 * @file DrivetrainLogger.h
 * @brief High-throughput pre-allocated ring buffer and CSV exporter for drivetrain calibration and FTC/FRC telemetry matching.
 */

#pragma once

#include "Control/SimControlBoundary.h"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace FRCBananaSim::Telemetry
{
    /**
     * @struct DrivetrainLogSample
     * @brief High-fidelity snapshot of robot motion, electromechanics, and contact patch dynamics at a single timestep.
     */
    struct DrivetrainLogSample
    {
        double timestamp = 0.0;                 ///< Elapsed simulation time (s).
        double commandedVoltage = 0.0;          ///< Primary commanded drive voltage (V).
        double batteryVoltage = 12.60;          ///< Post-sag terminal voltage (V).
        double totalBusCurrent = 0.0;           ///< Total electrical current draw (A).

        double chassisSpeed = 0.0;              ///< Ground forward linear velocity (m/s).
        double chassisElevation = 0.0;          ///< World Z/Y vertical elevation (m).
        double gyroYawDeg = 0.0;                ///< IMU yaw angle (deg).
        double gyroPitchDeg = 0.0;              ///< IMU pitch angle (deg).
        double gyroRollDeg = 0.0;               ///< IMU roll angle (deg).
        double gyroYawRateDegS = 0.0;           ///< IMU yaw angular rate (deg/s).

        std::array<double, 4> steerAnglesDeg = { 0.0 };      ///< Steer azimuth angles for modules FL, FR, BL, BR (deg).
        std::array<double, 4> wheelVelocitiesRadS = { 0.0 }; ///< Wheel axle angular speeds for modules FL, FR, BL, BR (rad/s).
        std::array<double, 4> driveCurrentsA = { 0.0 };      ///< Stator winding currents for modules FL, FR, BL, BR (A).
        std::array<double, 4> normalForcesN = { 0.0 };       ///< Ground normal reaction loads for modules FL, FR, BL, BR (N).
        std::array<double, 4> slipRatios = { 0.0 };          ///< Tire elliptical slip saturation ratios for modules FL, FR, BL, BR.
    };

    /**
     * @class DrivetrainLogger
     * @brief Thread-safe, zero-allocation ring buffer recorder with CSV export matching hardware calibration schemas.
     */
    class DrivetrainLogger
    {
    public:
        /**
         * @brief Constructs the drivetrain logger.
         * @param capacity Maximum number of samples pre-allocated in memory (default 60,000 = 5 minutes at 200 Hz).
         */
        explicit DrivetrainLogger(size_t capacity = 60000);

        /**
         * @brief Begins active recording into the ring buffer.
         */
        void StartRecording();

        /**
         * @brief Stops recording while preserving captured buffer samples.
         */
        void StopRecording();

        /**
         * @brief Checks if logging is currently active.
         * @return True if recording.
         */
        bool IsRecording() const;

        /**
         * @brief Clears all captured samples and resets ring buffer indices.
         */
        void Clear();

        /**
         * @brief Records a sample if logging is active. Zero allocation and real-time safe.
         * @param sample Complete log frame snapshot.
         */
        void RecordSample(const DrivetrainLogSample& sample);

        /**
         * @brief Gets current number of stored samples in buffer.
         * @return Count of valid samples (up to capacity).
         */
        size_t GetSampleCount() const;

        /**
         * @brief Gets pre-allocated buffer capacity.
         * @return Maximum sample capacity.
         */
        size_t GetCapacity() const;

        /**
         * @brief Retrieves sample at logical index [0 .. SampleCount-1], where 0 is the oldest recorded sample.
         * @param logicalIndex Chronological index.
         * @return Const reference to sample.
         */
        const DrivetrainLogSample& GetSample(size_t logicalIndex) const;

        /**
         * @brief Exports all stored samples to a comprehensive multi-column CSV file.
         * @param filepath Destination filesystem path.
         * @return True if file was written successfully.
         */
        bool ExportToCsv(const std::string& filepath) const;

        /**
         * @brief Exports simplified 5-column CSV matching FTC home hardware data collection schema:
         * timestamp_s,commanded_volts,wheel_velocity_rad_s,battery_volts,current_draw_a.
         * @param filepath Destination filesystem path.
         * @return True if file was written successfully.
         */
        bool ExportToFtcSchemaCsv(const std::string& filepath) const;

    private:
        std::vector<DrivetrainLogSample> buffer;
        size_t bufferCapacity = 60000;
        size_t writeHead = 0;
        size_t sampleCount = 0;
        bool isActivelyRecording = false;
    };
}