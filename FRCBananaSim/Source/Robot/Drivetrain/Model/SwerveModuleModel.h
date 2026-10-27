/**
 * @file SwerveModuleModel.h
 * @brief Electromechanical dynamics of an individual swerve module (steer ODE, drive motor, tire contact).
 */

#pragma once

#include "Robot/Drivetrain/Model/SwerveParams.h"
#include "Robot/Drivetrain/Model/TireContactModel.h"
#include "Control/SimControlBoundary.h"
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    /**
     * @struct SwerveModuleState
     * @brief Runtime kinematic and electromechanical states of a swerve module.
     */
    struct SwerveModuleState
    {
        double steerAngle = 0.0;           ///< Azimuth angle in radians (CCW-positive, [-pi, +pi]).
        double steerVelocity = 0.0;        ///< Azimuth angular rate in rad/s.
        double unwrappedSteerAngle = 0.0;  ///< Continuous unwrapped steer angle for encoder feedback.
        double lastSteerAngle = 0.0;       ///< Previous angle for delta unwrapping.

        double wheelPosition = 0.0;        ///< Accumulated wheel axle rotation in radians.
        double wheelVelocity = 0.0;        ///< Wheel axle rotational velocity in rad/s.
        double linearVelocity = 0.0;       ///< Ground tangential velocity (omega * r) in m/s.
        double distanceTraveled = 0.0;     ///< Ground linear distance in meters.

        double steerStatorCurrent = 0.0;   ///< Instantaneous steer motor stator current (A).
        double driveStatorCurrent = 0.0;   ///< Instantaneous drive motor stator current (A).
        double steerAppliedTorque = 0.0;   ///< Net mechanical torque delivered to steer azimuth (N*m).
        double driveAppliedTorque = 0.0;   ///< Net mechanical torque delivered to wheel axle (N*m).

        TireContactState contactState;     ///< Tire radial and tangential contact patch state.
    };

    /**
     * @class SwerveModuleModel
     * @brief Simulates steer heading ODE, drive motor coupling, and tire contact mechanics for one module.
     */
    class SwerveModuleModel
    {
    public:
        /**
         * @brief Constructs an individual swerve module physics model.
         * @param placement Placement geometry and hardware IDs on chassis.
         * @param steerParams Steer azimuth electromechanical parameters.
         * @param driveParams Drive wheel electromechanical parameters.
         * @param tireParams Tire compliance and friction parameters.
         */
        SwerveModuleModel(const ModulePlacement& placement, const SteerModuleParams& steerParams,
            const DriveModuleParams& driveParams, const TireParams& tireParams);

        /**
         * @brief Default constructor.
         */
        SwerveModuleModel();

        /**
         * @brief Advances steer azimuth ODE using exact 1D stick projection and motor dynamics.
         * @param steerCommand Steer motor demand (Voltage, DutyCycle, etc.).
         * @param busVoltage Current battery bus voltage (V).
         * @param chassisYawRate Vertical rotation rate of the chassis (rad/s).
         * @param deltaTime Integration sub-step duration in seconds.
         */
        void StepSteer(const Control::MotorCommand& steerCommand, double busVoltage,
            double chassisYawRate, double deltaTime);

        /**
         * @brief Computes effective linearized tire stiffness terms for the implicit chassis solver.
         * @param driveCommand Drive motor demand.
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
         * @param deltaTime Integration sub-step duration in seconds.
         * @return Resolved 3D contact force vector applied to chassis in world NWU coordinates (N).
         */
        glm::vec3 FinalizeDriveStep(const Control::MotorCommand& driveCommand, double busVoltage,
            double normalForce, const glm::vec3& groundPatchVelocity,
            const glm::vec3& surfaceNormal, double deltaTime);

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