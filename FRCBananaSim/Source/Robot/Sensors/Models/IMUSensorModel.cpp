/**
 * @file IMUSensorModel.cpp
 * @brief Implementation of 6-axis IMU gyroscope bias drift and specific force lever-arm dynamics.
 */

#include "Robot/Sensors/Models/IMUSensorModel.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Robot::Sensors::Models
{
    IMUSensorModel::IMUSensorModel(const IMUConfig& config)
        : config(config),
        rng(config.randomSeed),
        normalDist(0.0, 1.0)
    {}

    void IMUSensorModel::Update(const glm::dvec3& trueAngularVelocityNWU,
        const glm::dvec3& cgWorldAccelerationNWU,
        const glm::dquat& chassisOrientationNWU,
        double dt)
    {
        if (dt <= 1e-7)
        {
            return;
        }

        // 1. Angular Acceleration Evaluation (Finite Difference)
        glm::dvec3 angularAcceleration(0.0);
        if (hasPreviousAngularVelocity)
        {
            angularAcceleration = (trueAngularVelocityNWU - previousAngularVelocity) / dt;
        }
        previousAngularVelocity = trueAngularVelocityNWU;
        hasPreviousAngularVelocity = true;

        // 2. Gyroscope Gauss-Markov Bias Drift & White Noise
        if (config.gyroBiasRandomWalkRadS2 > 1e-9)
        {
            double decay = std::max(0.0, 1.0 - (dt / gyroBiasCorrelationTimeS));
            double biasDiffusion = config.gyroBiasRandomWalkRadS2 * std::sqrt(dt);

            gyroBias.x = (decay * gyroBias.x) + (biasDiffusion * normalDist(rng));
            gyroBias.y = (decay * gyroBias.y) + (biasDiffusion * normalDist(rng));
            gyroBias.z = (decay * gyroBias.z) + (biasDiffusion * normalDist(rng));
        }

        glm::dvec3 gyroNoise(0.0);
        if (config.gyroNoiseStdDevRadS > 1e-9)
        {
            double noiseScale = config.gyroNoiseStdDevRadS / std::sqrt(dt);
            gyroNoise = glm::dvec3(
                noiseScale * normalDist(rng),
                noiseScale * normalDist(rng),
                noiseScale * normalDist(rng)
            );
        }

        glm::dvec3 noisyBodyOmega = trueAngularVelocityNWU + gyroBias + gyroNoise;

        // Transform angular velocity from body frame to sensor mounting frame
        glm::dmat3 mountRotInv = glm::transpose(glm::mat3_cast(config.mountRotationQuat));
        measuredAngularVelocity = mountRotInv * noisyBodyOmega;

        // 3. Gyroscope Orientation Integration (WPILib NWU Frame)
        continuousYawRad += measuredAngularVelocity.z * dt;

        // Incremental orientation update
        double omegaMag = glm::length(measuredAngularVelocity);
        if (omegaMag > 1e-9)
        {
            glm::dvec3 axis = measuredAngularVelocity / omegaMag;
            double angle = omegaMag * dt;
            glm::dquat deltaQuat = glm::angleAxis(angle, axis);
            orientationQuat = glm::normalize(orientationQuat * deltaQuat);
        }

        // Extract continuous Euler angles (Yaw, Pitch, Roll)
        glm::dmat3 rMat = glm::mat3_cast(orientationQuat);
        pitchRad = std::asin(std::clamp(-rMat[0][2], -1.0, 1.0));
        rollRad = std::atan2(rMat[1][2], rMat[2][2]);

        // 4. Accelerometer Specific Force Calculation
        // World gravity vector in NWU (+Z is Up): g = (0, 0, -9.80665 m/s^2)
        const glm::dvec3 worldGravity(0.0, 0.0, -9.80665);

        // Specific force at CG in world frame: a_spec_world = a_CG - g_world
        glm::dvec3 specForceWorld = cgWorldAccelerationNWU - worldGravity;

        // Rotate into chassis body frame: a_spec_body = R_world_to_body * a_spec_world
        glm::dmat3 chassisRotInv = glm::transpose(glm::mat3_cast(chassisOrientationNWU));
        glm::dvec3 specForceBodyCG = chassisRotInv * specForceWorld;

        // Lever-arm tangential acceleration: a_tan = alpha x r
        glm::dvec3 aTangential = glm::cross(angularAcceleration, config.mountOffsetM);

        // Lever-arm centripetal acceleration: a_cent = omega x (omega x r)
        glm::dvec3 aCentripetal = glm::cross(trueAngularVelocityNWU,
            glm::cross(trueAngularVelocityNWU, config.mountOffsetM));

        glm::dvec3 totalBodySpecificForce = specForceBodyCG + aTangential + aCentripetal;

        // Transform into IMU mounting frame
        glm::dvec3 mountSpecificForce = mountRotInv * totalBodySpecificForce;

        // Accelerometer white noise injection
        if (config.accelNoiseStdDevMS2 > 1e-9)
        {
            double noiseScale = config.accelNoiseStdDevMS2 / std::sqrt(dt);
            mountSpecificForce.x += noiseScale * normalDist(rng);
            mountSpecificForce.y += noiseScale * normalDist(rng);
            mountSpecificForce.z += noiseScale * normalDist(rng);
        }

        measuredSpecificForce = mountSpecificForce;
    }

    void IMUSensorModel::Reset(const glm::dquat& initialOrientation)
    {
        orientationQuat = glm::normalize(initialOrientation);
        continuousYawRad = 0.0;
        pitchRad = 0.0;
        rollRad = 0.0;
        measuredAngularVelocity = glm::dvec3(0.0);
        measuredSpecificForce = glm::dvec3(0.0, 0.0, 9.80665);
        previousAngularVelocity = glm::dvec3(0.0);
        hasPreviousAngularVelocity = false;
        gyroBias = glm::dvec3(0.0);
    }

    double IMUSensorModel::GetYawRad() const { return continuousYawRad; }
    double IMUSensorModel::GetPitchRad() const { return pitchRad; }
    double IMUSensorModel::GetRollRad() const { return rollRad; }
    double IMUSensorModel::GetYawDegrees() const { return continuousYawRad * (180.0 / std::numbers::pi); }
    double IMUSensorModel::GetPitchDegrees() const { return pitchRad * (180.0 / std::numbers::pi); }
    double IMUSensorModel::GetRollDegrees() const { return rollRad * (180.0 / std::numbers::pi); }
    double IMUSensorModel::GetYawRateRadPerSec() const { return measuredAngularVelocity.z; }
    glm::dvec3 IMUSensorModel::GetAngularVelocityRadPerSec() const { return measuredAngularVelocity; }
    glm::dquat IMUSensorModel::GetOrientationQuat() const { return orientationQuat; }
    glm::dvec3 IMUSensorModel::GetSpecificForceMetersPerSec2() const { return measuredSpecificForce; }
}