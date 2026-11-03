/**
 * @file RobotSidecar.h
 * @brief Formal data structures for Robot Data Schema v1 sidecar.json (Electromechanics & Sensors).
 */

#pragma once

#include <string>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @struct ElectricalConfig
     * @brief Lumped battery, bus resistance, and brownout threshold parameters.
     */
    struct ElectricalConfig
    {
        double openCircuitVoltageV = 12.6;
        double internalResistanceOhm = 0.023;
        double quiescentCurrentA = 1.5;
        double brownoutWarningV = 6.8;
        double brownoutShutoffV = 6.3;
        double brownoutRecoveryV = 7.5;
    };

    /**
     * @struct SwerveModuleConfig
     * @brief Electromechanical identification and CAN bus mapping for one swerve module.
     */
    struct SwerveModuleConfig
    {
        std::string name;
        glm::dvec3 posM = { 0.0, 0.0, 0.0 };
        uint32_t driveCanId = 0;
        uint32_t steerCanId = 0;
        uint32_t cancoderId = 0;
        std::string canBus = "rio";
    };

    /**
     * @struct SwerveConfig
     * @brief Drivetrain chassis link and swerve module layout parameters.
     */
    struct SwerveConfig
    {
        std::string chassisLink = "base_link";
        std::string driveMotorPreset = "KrakenX60";
        std::string steerMotorPreset = "KrakenX44";
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
     * @struct MechanismConfig
     * @brief High-level mechanism actuator parameters (elevators, arms, wrists, flywheels, intakes).
     */
    struct MechanismConfig
    {
        std::vector<std::string> targetJoints; ///< Array of 2 joint names (e.g. [pitch_joint, roll_joint]).
        std::string differentialType = "symmetric_bevel"; ///< "symmetric_bevel", "core_xy", "coupled_arm", "custom_matrix".
        std::vector<double> gearRatios;        ///< [Gp, Gr] for symmetric bevel.
        std::vector<std::vector<double>> couplingMatrix; ///< 2x2 matrix for custom_matrix.
        std::vector<double> coulombFrictions;  ///< [frictionJointA, frictionJointB].
        std::vector<double> viscousDampings;   ///< [dampingJointA, dampingJointB].

        std::string name;
        std::string role; ///< "elevator", "arm", "wrist", "four_bar", "flywheel", "intake"
        std::string targetJoint;
        std::string parentLink;
        std::string motorPreset = "KrakenX60";
        double gearRatio = 1.0;
        double efficiency = 1.0;
        double drumRadiusM = 0.0;            ///< Pitch radius for linear spool/sprocket (elevators)
        double flywheelInertiaKgm2 = 0.0;    ///< Flywheel rotor mass moment of inertia (flywheels)
        glm::dvec3 spinAxis = { 0.0, 1.0, 0.0 }; ///< Local spin axis (flywheels)
        std::string visualLink;              ///< Visual mesh entity name to rotate with flywheel
        double statorCurrentLimitA = 60.0;
        double supplyCurrentLimitA = 40.0;
        double coulombFriction = 0.0;
        double viscousDamping = 0.0;
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
        std::string type; ///< "imu_pigeon2", "imu_navx", "cancoder", "limit_switch", "beam_break", "analog_input", "laser_can", "color_sensor", "custom"
        std::string bus = "rio"; ///< "rio", "canivore", "i2c_onboard", "i2c_mxp", "spi", "analog"
        uint32_t canId = 0;
        uint32_t dioChannel = 0;
        uint32_t analogChannel = 0;     ///< AnalogIn channel [0..7] for potentiometers / pressure transducers
        uint32_t i2cAddress = 0;        ///< 7-bit I2C device address
        uint32_t attachToControllerCanId = 0; ///< Motor controller CAN ID for hard-wired limit switches
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

        /**
         * @brief Looks up a mechanism configuration by name.
         * @param mechanismName Unique mechanism identifier.
         * @return Pointer to mechanism config, or nullptr if not found.
         */
        const MechanismConfig* FindMechanism(const std::string& mechanismName) const;

        /**
         * @brief Looks up a sensor configuration by name.
         * @param sensorName Unique sensor identifier.
         * @return Pointer to sensor config, or nullptr if not found.
         */
        const SensorConfig* FindSensor(const std::string& sensorName) const;

        /**
         * @brief Looks up a joint coupling by name.
         * @param couplingName Unique coupling identifier.
         * @return Pointer to coupling config, or nullptr if not found.
         */
        const CouplingConfig* FindCoupling(const std::string& couplingName) const;
    };
}