/**
 * @file SwerveModuleModel.h
 * @brief Electromechanics, steer azimuth dynamics, and tire contact interface for an individual swerve module.
 */

#pragma once

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include "Robot/Drivetrain/Model/TireContactModel.h"
#include "Control/SimControlBoundary.h"
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct SwerveModuleState
     * @brief Runtime dynamic state of an individual module.
     */
    struct SwerveModuleState
    {
        double steerAngle = 0.0;           ///< Steer azimuth heading in radians wrapped to [-pi, +pi].
        double steerVelocity = 0.0;        ///< Steer azimuth angular rate (rad/s).
        double steerAppliedTorque = 0.0;   ///< Net torque applied at steer shaft (N*m).
        double steerStatorCurrent = 0.0;   ///< Instantaneous steer motor winding current (A).
        double lastSteerAngle = 0.0;       ///< Previous angle for unwrapped tracking (rad).
        double unwrappedSteerAngle = 0.0;  ///< Continuous unwrapped steer encoder position (rad).

        double wheelPosition = 0.0;        ///< Integrated drive axle angular position (rad).
        double wheelVelocity = 0.0;        ///< Drive axle angular velocity (rad/s).
        double driveAppliedTorque = 0.0;   ///< Net drive torque applied at axle (N*m).
        double driveStatorCurrent = 0.0;   ///< Instantaneous drive motor winding current (A).
        double linearVelocity = 0.0;       ///< Wheel linear rolling velocity (m/s).
        double distanceTraveled = 0.0;      ///< Total distance rolled along floor (m).

        TireContactState contactState;     ///< Ground tire bristle deflection and normal reaction load.
    };

    /**
     * @class SwerveModuleModel
     * @brief High-fidelity simulation for one swerve module.
     */
    class SwerveModuleModel
    {
    public:
        /**
         * @brief Constructs an individual swerve module model.
         * @param placement Placement geometry on chassis.
         * @param steerParams Steer motor, inertia, and bearing damping.
         * @param driveParams Drive motor and gearbox parameters.
         * @param tireParams Radial tire compliance and carpet friction parameters.
         */
        SwerveModuleModel(const ModulePlacement& placement, const SteerModuleParams& steerParams,
            const DriveModuleParams& driveParams, const TireParams& tireParams);

        /**
         * @brief Default constructor.
         */
        SwerveModuleModel();

        /**
         * @brief Integrates steer azimuth angular rate and angle using 1D Coulomb friction stick projection.
         * @param steerCommand Commanded voltage or control demand.
         * @param busVoltage Terminal battery bus voltage (V).
         * @param chassisYawRate Current angular rate of chassis about Z (rad/s).
         * @param deltaTime Integration sub-step duration in seconds.
         */
        void StepSteer(const Control::MotorCommand& steerCommand, double busVoltage,
            double chassisYawRate, double deltaTime);

        /**
         * @brief Computes effective linearized contact stiffness and intercept force for the 6x6 chassis solve.
         * @param driveCommand Drive motor control demand.
         * @param busVoltage Current battery bus voltage (V).
         * @param normalForce Resolved normal contact load N (N).
         * @param deltaTime Integration sub-step duration in seconds.
         * @return EffectiveTireStiffness struct holding Kx, Ky, and f0.
         */
        EffectiveTireStiffness ComputeTangentialStiffness(const Control::MotorCommand& driveCommand,
            double busVoltage, double normalForce, double deltaTime) const;

        /**
         * @brief Finalizes drive wheel rotation and updates bristle state based on resolved chassis velocity.
         * @param driveCommand Drive motor demand.
         * @param busVoltage Current battery bus voltage (V).
         * @param normalForce Resolved normal contact load N (N).
         * @param groundPatchVelocity Linear velocity of the chassis at the contact patch (m/s).
         * @param surfaceNormal Ground surface unit normal vector.
         * @param wheelHeadingWorld Forward rolling unit vector in world coordinates.
         * @param deltaTime Integration sub-step duration in seconds.
         * @return Resolved 3D contact force vector applied to chassis in world NWU coordinates (N).
         */
        glm::vec3 FinalizeDriveStep(const Control::MotorCommand& driveCommand, double busVoltage,
            double normalForce, const glm::vec3& groundPatchVelocity,
            const glm::vec3& surfaceNormal, const glm::vec3& wheelHeadingWorld, double deltaTime);

        /**
         * @brief Retrieves live module runtime state.
         * @return Const reference to SwerveModuleState.
         */
        const SwerveModuleState& GetState() const;

        /**
         * @brief Retrieves mutable module runtime state.
         * @return Mutable reference to SwerveModuleState.
         */
        SwerveModuleState& GetState();

        /**
         * @brief Retrieves placement specification on chassis.
         * @return Const reference to ModulePlacement.
         */
        const ModulePlacement& GetPlacement() const;

        /**
         * @brief Retrieves drive motor model.
         * @return Const reference to DCMotorModel.
         */
        const Actuation::DCMotorModel& GetDriveMotor() const;

        /**
         * @brief Retrieves steer motor model.
         * @return Const reference to DCMotorModel.
         */
        const Actuation::DCMotorModel& GetSteerMotor() const;

    private:
        ModulePlacement placement;
        SteerModuleParams steerParams;
        DriveModuleParams driveParams;
        TireContactModel tireModel;
        SwerveModuleState state;
    };
}