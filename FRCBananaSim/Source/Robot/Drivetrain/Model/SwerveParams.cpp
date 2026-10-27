/**
 * @file SwerveParams.cpp
 * @brief Implementation of swerve drivetrain parameter factories and reference presets.
 */

#include "Robot/Drivetrain/Model/SwerveParams.h"

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    SwerveDrivetrainConfig SwerveDrivetrainConfig::CreateMK5nR3Reference()
    {
        SwerveDrivetrainConfig config;

        // 1. Chassis Physical Properties
        config.chassis.mass = 50.0;
        config.chassis.dimensions = glm::vec3(0.70, 0.70, 0.15); // 70cm x 70cm x 15cm
        config.chassis.centerOfMassOffset = glm::vec3(0.0, 0.0, 0.0);
        config.chassis.centerOfMassHeight = 0.20;

        // Inertia tensor for a rectangular slab: Ix = M/12*(y^2 + z^2), Iy = M/12*(x^2 + z^2), Iz = M/12*(x^2 + y^2)
        // With NWU: X=Forward, Y=Left, Z=Up
        double m = config.chassis.mass;
        double dx = config.chassis.dimensions.x;
        double dy = config.chassis.dimensions.y;
        double dz = config.chassis.dimensions.z;

        double ixx = (m / 12.0) * (dy * dy + dz * dz); // Pitch axis
        double iyy = (m / 12.0) * (dx * dx + dz * dz); // Roll axis
        double izz = (m / 12.0) * (dx * dx + dy * dy); // Yaw axis

        config.chassis.inertiaTensor = glm::mat3(0.0);
        config.chassis.inertiaTensor[0][0] = ixx;
        config.chassis.inertiaTensor[1][1] = iyy;
        config.chassis.inertiaTensor[2][2] = izz;

        // 2. Tire & Contact Properties
        config.tire.wheelRadius = 0.0508; // 4 inch diameter (2 inch radius)
        config.tire.wheelWidth = 0.0381;
        config.tire.wheelMass = 0.65;
        config.tire.wheelAxleInertia = 0.00084;
        config.tire.radialStiffness = 1.2e5;       // 120 kN/m
        config.tire.radialDamping = 1200.0;        // 1200 N*s/m
        config.tire.preSlideDisplacement = 0.001;  // 1 mm
        config.tire.muStatic = 1.20;
        config.tire.muKinetic = 1.05;
        config.tire.stribeckVelocity = 0.15;
        config.tire.loadSensitivityBeta = 0.08;
        config.tire.rollingResistance = 0.015;
        config.tire.lateralFrictionRatio = 1.0;

        // 3. SDS MK5n R3 Actuation Setup
        // Steer ratio 287/11 = 26.09:1 with Kraken X44
        config.steer.motor = Actuation::DCMotorModel::KrakenX44(26.0909, 0.92);
        config.steer.steerHousingInertia = 0.00036;
        config.steer.coulombFrictionTorque = 0.08;
        config.steer.viscousDamping = 0.002;

        // Drive ratio 5.27:1 with Kraken X60
        config.drive.motor = Actuation::DCMotorModel::KrakenX60(5.27, 0.96);

        // 4. Module Placements in NWU (+X Forward, +Y Left):
        // Trackwidth = 0.56m, Wheelbase = 0.56m (Half-extents: 0.28m)
        constexpr double halfExtent = 0.28;

        // [0] FrontLeft: (+X, +Y)
        config.modules[0] = { "FrontLeft", 0, glm::vec2(+halfExtent, +halfExtent), 2, 1, 1 };
        // [1] FrontRight: (+X, -Y)
        config.modules[1] = { "FrontRight", 1, glm::vec2(+halfExtent, -halfExtent), 4, 3, 2 };
        // [2] BackLeft: (-X, +Y)
        config.modules[2] = { "BackLeft", 2, glm::vec2(-halfExtent, +halfExtent), 6, 5, 3 };
        // [3] BackRight: (-X, -Y)
        config.modules[3] = { "BackRight", 3, glm::vec2(-halfExtent, -halfExtent), 8, 7, 4 };

        return config;
    }
}