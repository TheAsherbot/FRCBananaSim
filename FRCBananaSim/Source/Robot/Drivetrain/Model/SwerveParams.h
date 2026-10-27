/**
 * @file SwerveParams.h
 * @brief Configuration parameters, JSON loader, and presets for 6-DOF swerve drivetrain mechanics, tire contacts, and motors.
 */

#pragma once

#include "Actuation/DCMotorModel.h"
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <array>
#include <string>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct ChassisParams
     * @brief Mass properties and geometric dimensions of the complete robot.
     */
    struct ChassisParams
    {
        double mass = 50.0;                                            ///< Total nominal robot mass (kg).
        glm::vec3 dimensions{ 0.70, 0.70, 0.15 };                      ///< Width, Length, Height in meters (X=forward, Y=left, Z=up).
        glm::vec3 centerOfMassOffset{ 0.0, 0.0, 0.0 };                 ///< Center of mass offset from geometric hub center (m).
        double centerOfMassHeight = 0.20;                              ///< Height of CG above ground plane (m).
        glm::mat3 inertiaTensor{ 1.0 };                                ///< 3x3 rotational inertia tensor about CG in NWU frame (kg*m^2).

        /**
         * @brief Converts the NWU rotational inertia tensor to Jolt engine coordinate frame (+X Right, +Y Up, +Z Forward).
         * @return 3x3 inertia tensor mapped to Jolt body coordinates.
         */
        glm::mat3 GetJoltInertiaTensor() const;
    };

    /**
     * @struct TireParams
     * @brief Radial compliance, bristle contact stiffness, and friction envelope parameters.
     */
    struct TireParams
    {
        double wheelRadius = 0.0508;          ///< Standard 4-inch wheel radius (0.0508 m).
        double wheelWidth = 0.0381;           ///< Wheel tread width (0.0381 m = 1.5 inch).
        double wheelMass = 0.65;              ///< Wheel rim + tread mass (kg).
        double wheelAxleInertia = 0.00084;    ///< Rotational inertia about drive axle (kg*m^2).
        double radialStiffness = 1.2e5;       ///< Radial spring stiffness kt (N/m).
        double radialDamping = 1200.0;        ///< Radial viscous damping ct (N*s/m).
        double preSlideDisplacement = 0.001;  ///< Bristle elastic deflection limit z_max (m).
        double muStatic = 1.20;               ///< Peak static carpet-wheel friction coefficient.
        double muKinetic = 1.05;              ///< Dynamic sliding friction coefficient.
        double stribeckVelocity = 0.15;       ///< Stribeck transition slip velocity (m/s).
        double loadSensitivityBeta = 0.08;    ///< Normal load sensitivity exponent mu(N) = mu0 * (N/Nref)^(-beta).
        double rollingResistance = 0.015;     ///< Rolling resistance coefficient Crr.
        double lateralFrictionRatio = 1.0;    ///< Anisotropic carpet ratio mu_y / mu_x (1.0 = isotropic).
    };

    /**
     * @struct SteerModuleParams
     * @brief Steer azimuth assembly electromechanics and friction.
     */
    struct SteerModuleParams
    {
        Actuation::DCMotorModel motor = Actuation::DCMotorModel::KrakenX44(26.0909, 0.92); ///< MK5n steering motor.
        double steerHousingInertia = 0.00036;                                               ///< Physical azimuth housing inertia (kg*m^2).
        double coulombFrictionTorque = 0.08;                                                ///< Bearing and bevel gear static friction (N*m).
        double viscousDamping = 0.002;                                                      ///< Viscous drag (N*m / (rad/s)).
    };

    /**
     * @struct DriveModuleParams
     * @brief Drive wheel gearbox electromechanics and axle friction.
     */
    struct DriveModuleParams
    {
        Actuation::DCMotorModel motor = Actuation::DCMotorModel::KrakenX60(5.27, 0.96); ///< MK5n R3 drive motor.
        double coulombFrictionTorque = 1.8;                                              ///< Drive train static Coulomb friction at wheel axle (N*m).
    };

    /**
     * @struct ModulePlacement
     * @brief Spatial location and hardware IDs of an individual swerve module.
     * Expressed in WPILib NWU frame relative to robot center (+X Forward, +Y Left).
     */
    struct ModulePlacement
    {
        std::string name;
        uint32_t moduleIndex = 0;
        glm::vec2 locationOnChassis{ 0.0f, 0.0f }; ///< Offset from chassis center: (X = Forward, Y = Left).
        uint32_t driveCANId = 1;
        uint32_t steerCANId = 2;
        uint32_t canCoderId = 1;
    };

    /**
     * @struct SwerveDrivetrainConfig
     * @brief Complete configuration blueprint for the 4-module swerve drivetrain.
     */
    struct SwerveDrivetrainConfig
    {
        ChassisParams chassis;
        TireParams tire;
        SteerModuleParams steer;
        DriveModuleParams drive;
        std::array<ModulePlacement, 4> modules;

        /**
         * @brief Factory creating reference SDS MK5n R3 swerve configuration (Kraken X60 / X44).
         * @return Standard MK5n R3 configuration configured with reference mass and gearboxes.
         */
        static SwerveDrivetrainConfig CreateMK5nR3Reference();

        /**
         * @brief Deserializes a full swerve drivetrain configuration from a JSON configuration string.
         * Supports both Schema v1 and Schema v2 declarative formats with robust fallbacks.
         * @param jsonString Serialized JSON text.
         * @return Parsed SwerveDrivetrainConfig struct.
         */
        static SwerveDrivetrainConfig LoadFromJsonString(const std::string& jsonString);

        /**
         * @brief Deserializes a full swerve drivetrain configuration from a filesystem JSON file.
         * @param filePath Relative or absolute path to the robot JSON file.
         * @return Parsed SwerveDrivetrainConfig struct.
         */
        static SwerveDrivetrainConfig LoadFromJsonFile(const std::string& filePath);
    };
}