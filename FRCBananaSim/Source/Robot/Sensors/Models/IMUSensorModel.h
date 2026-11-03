/**
 * @file IMUSensorModel.h
 * @brief High-fidelity 6-axis IMU transducer modeling gyroscope bias random walk, lever-arm acceleration, and specific force.
 */

#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <random>

namespace FRCBananaSim::Robot::Sensors::Models
{
    /**
     * @struct IMUConfig
     * @brief Mounting location, orientation, and stochastic noise parameters for an IMU.
     */
    struct IMUConfig
    {
        glm::dvec3 mountOffsetM = { 0.0, 0.0, 0.0 };              ///< Translation offset from chassis center of mass (m).
        glm::dquat mountRotationQuat = { 1.0, 0.0, 0.0, 0.0 };    ///< Mounting rotation quaternion in body coordinates.
        double gyroNoiseStdDevRadS = 0.0;                         ///< Gyroscope white noise standard deviation (rad/s).
        double gyroBiasRandomWalkRadS2 = 0.0;                     ///< Gyroscope bias random walk rate (rad/s^2).
        double accelNoiseStdDevMS2 = 0.0;                         ///< Accelerometer white noise standard deviation (m/s^2).
        uint64_t randomSeed = 1337;                               ///< Deterministic random seed for headless reproducibility.
    };

    /**
     * @class IMUSensorModel
     * @brief Simulates 3-axis gyroscope with Gauss-Markov drift and 3-axis accelerometer measuring specific force in WPILib NWU.
     */
    class IMUSensorModel
    {
    public:
        /**
         * @brief Constructs an IMU transducer model.
         * @param config Configuration parameters and noise characteristics.
         */
        explicit IMUSensorModel(const IMUConfig& config = IMUConfig{});

        /**
         * @brief Advances IMU simulation across timestep dt.
         * @param trueAngularVelocityNWU True spatial angular velocity in chassis body NWU frame (rad/s).
         * @param cgWorldAccelerationNWU True linear acceleration of chassis center of mass in world NWU frame (m/s^2).
         * @param chassisOrientationNWU Orientation quaternion of chassis body relative to world NWU frame.
         * @param dt Timestep duration in seconds.
         */
        void Update(const glm::dvec3& trueAngularVelocityNWU,
            const glm::dvec3& cgWorldAccelerationNWU,
            const glm::dquat& chassisOrientationNWU,
            double dt);

        /**
         * @brief Resets integrated yaw, pitch, roll, orientation, and bias states.
         * @param initialOrientation Optional starting orientation quaternion.
         */
        void Reset(const glm::dquat& initialOrientation = glm::dquat(1.0, 0.0, 0.0, 0.0));

        // Gyroscope Accessors (WPILib NWU Standard)
        double GetYawRad() const;
        double GetPitchRad() const;
        double GetRollRad() const;
        double GetYawDegrees() const;
        double GetPitchDegrees() const;
        double GetRollDegrees() const;
        double GetYawRateRadPerSec() const;
        glm::dvec3 GetAngularVelocityRadPerSec() const;
        glm::dquat GetOrientationQuat() const;

        // Accelerometer Accessors (Specific Force in NWU)
        glm::dvec3 GetSpecificForceMetersPerSec2() const;

    private:
        IMUConfig config;

        // Kinematic State
        glm::dquat orientationQuat = { 1.0, 0.0, 0.0, 0.0 };
        double continuousYawRad = 0.0;
        double pitchRad = 0.0;
        double rollRad = 0.0;

        glm::dvec3 measuredAngularVelocity = { 0.0, 0.0, 0.0 };
        glm::dvec3 measuredSpecificForce = { 0.0, 0.0, 9.80665 }; // +1g Z upward at rest

        glm::dvec3 previousAngularVelocity = { 0.0, 0.0, 0.0 };
        bool hasPreviousAngularVelocity = false;

        // Gauss-Markov Bias State
        glm::dvec3 gyroBias = { 0.0, 0.0, 0.0 };
        static constexpr double gyroBiasCorrelationTimeS = 300.0; // 5-minute bias drift time constant

        // Deterministic Pseudo-Random Number Generator
        std::mt19937_64 rng;
        std::normal_distribution<double> normalDist;
    };
}