/**
 * @file RobotSidecar.h
 * @brief Formal data structures for Robot Data Schema v1 sidecar.json (Electromechanics & Sensors).
 */

#pragma once

#include "Robot/Schema/RobotManifest.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Robot::Schema
{
    struct RobotManifest;

    /**
     * @struct ElectricalConfig
     * @brief Lumped battery, bus resistance, brownout threshold, and regulated servo rail parameters.
     */
    struct ElectricalConfig
    {
        double openCircuitVoltageV = 12.6;
        double internalResistanceOhm = 0.023;
        double quiescentCurrentA = 1.5;
        double brownoutWarningV = 6.8;
        double brownoutShutoffV = 6.3;
        double brownoutRecoveryV = 7.5;

        // Regulated 6V / 5V Power Rail (RoboRIO 6V, REV SPM 6V, or FTC Hub)
        double regulatedRailVoltageV = 6.0;      ///< Default 6.0V
        double regulatedRailCurrentLimitA = 2.2;  ///< Default RoboRIO 2.2A limit
        double regulatedRailEfficiency = 0.88;    ///< Buck regulator efficiency
    };

    /**
     * @struct SwerveModuleConfig
     * @brief Electromechanical identification and CAN/PWM mapping for one swerve module.
     */
    struct SwerveModuleConfig
    {
        std::string name;
        glm::dvec3 posM = { 0.0, 0.0, 0.0 };
        uint32_t driveCanId = 0;
        uint32_t steerCanId = 0;
        uint32_t cancoderId = 0;
        std::string canBus = "rio";
        uint32_t drivePwmChannel = 0; ///< For servo-driven drive wheel
        uint32_t steerPwmChannel = 0; ///< For servo-driven steer azimuth
    };

    /**
     * @struct CustomMotorSpec
     * @brief Parametric electromechanical parameters for custom non-catalog DC motors.
     */
    struct CustomMotorSpec
    {
        double nominalVoltageV = 12.0;
        double stallTorqueNm = 5.0;
        double freeSpeedRpm = 6000.0;
        double stallCurrentA = 250.0;
        double freeCurrentA = 2.0;
    };

    /**
     * @struct CustomServoSpec
     * @brief Parametric electromechanical parameters for custom non-catalog PWM servos.
     */
    struct CustomServoSpec
    {
        double nominalVoltageV = 6.0;
        double stallTorqueKgCm = 20.0;
        double speedSecPer60Deg = 0.10;
        double stallCurrentA = 2.5;
        double freeCurrentA = 0.15;
        double travelRangeDeg = 300.0;
    };

    /**
     * @struct SwerveConfig
     * @brief Drivetrain chassis link, swerve module layout, and actuator configurations.
     */
    struct SwerveConfig
    {
        std::string chassisLink = "base_link";
        std::string driveActuatorType = "motor"; ///< "motor" or "servo"
        std::string steerActuatorType = "motor"; ///< "motor" or "servo"
        std::string driveMotorPreset = "KrakenX60";
        std::string steerMotorPreset = "KrakenX44";
        std::string driveServoPreset = "GoBildaSpeed";
        std::string steerServoPreset = "GoBildaTorque";
        CustomMotorSpec customDriveMotor;
        CustomMotorSpec customSteerMotor;
        CustomServoSpec customDriveServo;
        CustomServoSpec customSteerServo;
        double driveGearRatio = 5.27;
        double steerGearRatio = 26.0909;
        double driveEfficiency = 0.96;
        double steerEfficiency = 0.92;
        double wheelRadiusM = 0.0508;
        double driveStatorLimitA = 60.0;
        double driveSupplyLimitA = 40.0;
        double steerStatorLimitA = 40.0;
        double steerSupplyLimitA = 30.0;
        std::vector<SwerveModuleConfig> modules;
    };

    /**
     * @struct SpringAssistConfig
     * @brief Mechanical counterbalance spring configuration (e.g., constant-force elevator spring).
     */
    struct SpringAssistConfig
    {
        std::string type = "constant_force";
        double forceN = 0.0;
    };

    /**
     * @struct ControllerConfig
     * @brief Hardware motor controller specification (TalonFX, SparkMax, etc.).
     */
    struct ControllerConfig
    {
        std::string vendor = "CTRE_TalonFX";
        uint32_t canId = 0;
        std::string canBus = "rio";
        bool inverted = false;
        std::string role = "leader"; ///< "leader" or "follower"
    };

    /**
     * @struct ServoConfig
     * @brief Electromechanical parameters and PWM channel mapping for smart/hobby servos.
     */
    struct ServoConfig
    {
        std::string preset = "GoBildaTorque"; ///< e.g. "GoBildaTorque", "GoBildaSpeed", "SwyftSpeed"
        std::string mode = "positional";       ///< "positional" or "continuous"
        uint32_t pwmChannel = 0;               ///< PWM channel [0..15]
        double supplyVoltageV = 6.0;           ///< Regulated operating voltage (V)
        double travelRangeDeg = 300.0;         ///< Total rotational travel range (deg)
    };

    /**
     * @struct MechanismConfig
     * @brief High-level mechanism actuator parameters (elevators, arms, wrists, flywheels, intakes, differentials).
     */
    struct MechanismConfig
    {
        std::vector<std::string> targetJoints; ///< Array of 2 joint names (e.g. [pitch_joint, roll_joint]).
        std::string differentialType = "symmetric_bevel"; ///< "symmetric_bevel", "core_xy", "coupled_arm", "custom_matrix".
        std::vector<double> gearRatios;        ///< [Gp, Gr] for symmetric bevel.
        std::vector<std::vector<double>> couplingMatrix; ///< 2x2 matrix for custom_matrix.
        std::vector<double> coulombFrictions;  ///< [frictionJointA, frictionJointB].
        std::vector<double> viscousDampings;   ///< [dampingJointA, dampingJointB].
        std::vector<std::string> motorPresets; ///< For differential: [motorPresetA, motorPresetB].
        std::vector<CustomMotorSpec> customMotors; ///< For differential: [customMotorA, customMotorB].
        std::vector<CustomServoSpec> customServos; ///< For differential: [customServoA, customServoB].
        std::vector<std::string> actuatorTypes;    ///< For differential: [actuatorTypeA, actuatorTypeB].
        std::vector<double> statorCurrentLimits;   ///< For differential: [statorLimitA, statorLimitB].
        std::vector<double> supplyCurrentLimits;   ///< For differential: [supplyLimitA, supplyLimitB].

        std::string name;
        std::string role; ///< "elevator", "arm", "wrist", "turret", "four_bar", "flywheel", "intake", "differential"
        std::string actuatorType = "motor"; ///< "motor" (CAN motor) or "servo" (PWM servo)
        ServoConfig servo;                  ///< Primary servo configuration (for arm, elevator, wrist)
        std::vector<ServoConfig> servos;    ///< Multiple servo configs (for differential mechanisms)

        std::string targetJoint;
        std::string parentLink;
        std::string motorPreset = "KrakenX60";
        double gearRatio = 1.0;
        double efficiency = 1.0;
        double drumRadiusM = 0.0;            ///< Pitch radius for linear spool/sprocket (elevators)
        double rollerRadiusM = 0.0508;       ///< Roller radius for flywheels and active intake rollers
        double flywheelInertiaKgm2 = 0.0;    ///< Flywheel rotor mass moment of inertia (flywheels)
        glm::dvec3 spinAxis = { 0.0, 1.0, 0.0 }; ///< Local spin axis (flywheels)
        std::string visualLink;              ///< Visual mesh entity name to rotate with flywheel
        glm::dvec3 mountOffsetM = { 0.0, 0.0, 0.0 }; ///< Mount translation offset relative to joint/parent (flywheels/intakes)
        double statorCurrentLimitA = 60.0;
        double supplyCurrentLimitA = 40.0;
        double coulombFriction = 0.0;
        double viscousDamping = 0.0;
        CustomMotorSpec customMotor;
        CustomServoSpec customServo;
        SpringAssistConfig springAssist;
        std::vector<ControllerConfig> controllers;
    };

    /**
     * @struct StageCouplingConfig
     * @brief Cascade elevator stage kinematic relation (x_k = multiplier * x_1).
     */
    struct StageCouplingConfig
    {
        std::string joint;
        double multiplier = 1.0;
    };

    /**
     * @struct FourBarCouplingConfig
     * @brief Four-bar kinematic loop closure parameters.
     */
    struct FourBarCouplingConfig
    {
        std::string crankJoint;
        std::string couplerJoint;
        std::string followerJoint;
        std::string closureParentLink;
        std::string closureChildLink;
        glm::dvec3 closureParentAnchorM = { 0.0, 0.0, 0.0 };
        glm::dvec3 closureChildAnchorM = { 0.0, 0.0, 0.0 };
        glm::dvec3 closureAxis = { 0.0, 1.0, 0.0 };
    };

    /**
     * @struct PowerCouplingConfig
     * @brief Unified mechanical power transmission coupling between two joints (gears, timing belts, chains, cables).
     */
    struct PowerCouplingConfig
    {
        std::string masterJoint;
        std::string slaveJoint;
        std::string transmissionType = "gear"; ///< "gear", "belt", "chain", "cable", "virtual_fourbar"
        double ratio = 1.0;                    ///< Kinematic velocity ratio (omega_slave / omega_master)
        bool inverted = false;                 ///< Inverts direction of slave relative to master
        double efficiency = 0.95;              ///< Mechanical torque transmission efficiency (0.0 to 1.0)
        double backlashRad = 0.0;              ///< Mechanical backlash / deadband play (rad or meters)
    };

    /**
     * @struct CouplingConfig
     * @brief Generic mechanical coupling entry in sidecar.json.
     */
    struct CouplingConfig
    {
        std::string name;
        std::string type; ///< "power_coupling", "elevator_cascade", "four_bar_loop"
        std::string solverMode = "analytic_coupled"; ///< "analytic_coupled" or "jolt_constraint"

        PowerCouplingConfig powerCoupling;
        std::vector<StageCouplingConfig> cascadeStages;
        FourBarCouplingConfig fourBar;
    };

    /**
     * @struct SensorConfig
     * @brief Sensory transducer specification (IMUs, CANcoders, analog sensors, I2C/SPI devices, limit switches).
     */
    struct SensorConfig
    {
        std::string name;
        std::string type; ///< "imu_pigeon2", "imu_navx", "cancoder", "beam_break", "limit_switch", "analog_input", "laser_can", "color_sensor", "rev_through_bore", "andymark_lidar3", "custom"
        std::string bus = "rio"; ///< "rio", "canivore", "i2c_onboard", "i2c_mxp", "spi", "analog", "dio_pwm"
        uint32_t canId = 0;
        uint32_t dioChannel = 0;
        uint32_t secondaryDioChannel = 0;     ///< Secondary DIO channel (e.g. Quadrature Channel B for REV Through Bore)
        uint32_t analogChannel = 0;     ///< AnalogIn channel [0..7] for potentiometers / pressure transducers
        uint32_t i2cAddress = 0;        ///< 7-bit I2C device address
        uint32_t attachToControllerCanId = 0; ///< Motor controller CAN ID for hard-wired limit switches or SPARK data port sensors
        std::string parentLink;
        std::string targetJoint;
        std::string contactZone;
        std::string limitDirection;     ///< "forward" or "reverse"
        double triggerMin = 0.0;
        double triggerMax = 0.0;
        bool normallyClosed = false;
        double debounceTimeS = 0.010;
        bool inverted = false;
        glm::dvec3 mountOffsetM = { 0.0, 0.0, 0.0 };
        glm::dquat mountRotationQuat = { 1.0, 0.0, 0.0, 0.0 };
        double gyroNoiseStdDevRadS = 0.0;
        double gyroBiasRandomWalkRadS2 = 0.0;
        double accelNoiseStdDevMS2 = 0.0;
        double sensorToMechanismRatio = 1.0;
        uint32_t cpr = 4096;
        double offsetRad = 0.0;

        // Custom / vendor-specific metadata for arbitrary sensors
        std::unordered_map<std::string, double> customNumericParams;
        std::unordered_map<std::string, std::string> customStringParams;
    };

    /**
     * @struct UnmodeledBallastConfig
     * @brief Point-mass inertial ballast offset applied to a target link without a physical collision shape.
     */
    struct UnmodeledBallastConfig
    {
        std::string targetLink = "base_link";
        double massKg = 0.0;
        glm::dvec3 offsetM = { 0.0, 0.0, 0.0 };
    };

    /**
     * @struct MassReconciliationConfig
     * @brief Non-destructive physical bench scale adjustments and unmodeled ballast calibration.
     */
    struct MassReconciliationConfig
    {
        bool enabled = false;
        double linkMassMultiplier = 1.0;
        UnmodeledBallastConfig unmodeledBallast;
        std::unordered_map<std::string, double> linkMassOverrides; ///< Per-link mass override in kg
    };

    /**
     * @struct JointLimitOverride
     * @brief Override parameters for joint physical hardstops and compliance springs.
     */
    struct JointLimitOverride
    {
        bool hasLimits = true;
        double lower = 0.0;
        double upper = 0.0;
        std::string unit = "rad"; ///< "rad", "deg", "m"
        double springFrequencyHz = 40.0;
        double springDampingRatio = 1.0;
    };

    /**
     * @struct BumperConfig
     * @brief Parametric FRC bumper configuration for robots without CAD-modeled bumpers.
     */
    struct BumperConfig
    {
        bool enabled = false;                           ///< When true, physical bumper colliders and mass are injected.
        std::string targetLink = "";                    ///< Link to attach bumpers to (defaults to root/chassis).
        std::string shape = "box";                      ///< "box" (solid cuboid), "perimeter" (4 hollow sides), or "u_shape" (front cutout)
        double lengthM = 0.85;                          ///< Outer X length in meters (~33.5 in)
        double widthM = 0.85;                           ///< Outer Y width in meters (~33.5 in)
        double heightM = 0.127;                         ///< Bumper vertical height in meters (standard 5.0 in)
        double thicknessM = 0.0825;                     ///< Perimeter bumper thickness in meters (plywood + noodles, ~3.25 in)
        glm::dvec3 offsetM = { 0.0, 0.0, 0.09 };        ///< Center offset relative to targetLink NWU (default 0.09m above ground)
        double massKg = 6.0;                            ///< Total mass of the bumper set in kg (~13.2 lbs)
        std::string material = "bumper_fabric";         ///< Material for friction & restitution
        std::string color = "red";                      ///< "red", "blue", or custom hex

        // Cutout settings for "u_shape" (e.g. front ground intake opening)
        double cutoutWidthM = 0.40;                     ///< Width of the front opening cutout in meters (~15.7 in)
    };

    /**
     * @struct RobotSidecar
     * @brief Complete parsed sidecar.json electromechanical and sensor specification.
     */
    struct RobotSidecar
    {
        std::string schemaVersion = "1.0.0";
        std::string targetManifest = "manifest.json";
        ElectricalConfig electrical;
        SwerveConfig swerve;
        std::vector<MechanismConfig> mechanisms;
        std::vector<CouplingConfig> couplings;
        std::vector<SensorConfig> sensors;
        MassReconciliationConfig massReconciliation;
        std::unordered_map<std::string, JointLimitOverride> jointOverrides;
        BumperConfig bumpers;
        std::vector<ContactZoneDefinition> contactZones;

        /**
         * @brief Looks up a contact zone configuration by name.
         * @param zoneName Unique contact zone identifier.
         * @return Pointer to contact zone config, or nullptr if not found.
         */
        const ContactZoneDefinition* FindContactZone(const std::string& zoneName) const;
        ContactZoneDefinition* FindContactZone(const std::string& zoneName);

        /**
         * @brief Computes effective contact zones, factoring in sidecar overrides/additions over CAD defaults.
         * @param manifest Loaded robot CAD manifest.
         * @return Active effective contact zones.
         */
        std::vector<ContactZoneDefinition> GetEffectiveContactZones(const RobotManifest& manifest) const;

        /**
         * @brief Looks up a mechanism configuration by name.
         * @param mechanismName Unique mechanism identifier.
         * @return Pointer to mechanism config, or nullptr if not found.
         */
        const MechanismConfig* FindMechanism(const std::string& mechanismName) const;
        MechanismConfig* FindMechanism(const std::string& mechanismName);

        /**
         * @brief Looks up a sensor configuration by name.
         * @param sensorName Unique sensor identifier.
         * @return Pointer to sensor config, or nullptr if not found.
         */
        const SensorConfig* FindSensor(const std::string& sensorName) const;
        SensorConfig* FindSensor(const std::string& sensorName);

        /**
         * @brief Looks up a joint coupling by name.
         * @param couplingName Unique coupling identifier.
         * @return Pointer to coupling config, or nullptr if not found.
         */
        const CouplingConfig* FindCoupling(const std::string& couplingName) const;
        CouplingConfig* FindCoupling(const std::string& couplingName);

        /**
         * @brief Looks up a joint limit override by joint name.
         * @param jointName Unique joint identifier.
         * @return Pointer to override config, or nullptr if not found.
         */
        const JointLimitOverride* FindJointOverride(const std::string& jointName) const;
        JointLimitOverride* FindJointOverride(const std::string& jointName);

        /**
         * @brief Computes effective limits for a joint, factoring in sidecar overrides over CAD defaults.
         * @param jointName Unique joint identifier.
         * @param manifest Loaded robot CAD manifest.
         * @return Active effective physical limits.
         */
        JointLimitOverride GetEffectiveLimits(const std::string& jointName, const RobotManifest& manifest) const;
    };
}
