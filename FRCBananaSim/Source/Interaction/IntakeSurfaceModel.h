/**
 * @file IntakeSurfaceModel.h
 * @brief Surface velocity traction drag and kinematic capture physics model for active intake rollers.
 */

#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Interaction
{
    /**
     * @struct IntakeParameters
     * @brief Physical and geometric parameters governing compliant roller intake traction and capture.
     */
    struct IntakeParameters
    {
        double rollerRadiusM = 0.0508;              ///< Intake roller outer radius (m)
        double frictionMu = 0.70;                   ///< Surface Coulomb friction coefficient
        double normalForceN = 45.0;                 ///< Compliant normal grip preload force (N)
        double viscousDampingNsPerM = 8.0;          ///< Viscous compliance traction damping (N*s/m)
        double maxGripForceN = 120.0;               ///< Saturation traction force limit (N)
        double centerlineCaptureThresholdM = 0.0;   ///< Centerline threshold along tangent axis for capture handoff (m)
    };

    /**
     * @struct IntakeTractionResult
     * @brief Traction force, motor reaction torque, and capture state calculated during intake contact.
     */
    struct IntakeTractionResult
    {
        glm::vec3 intakeForceN{ 0.0f };             ///< Compliant traction force vector pulling piece inward (N)
        double loadTorqueNm = 0.0;                  ///< Equal-and-opposite braking reaction torque on roller motor (N*m)
        glm::vec3 relativeVelocity{ 0.0f };         ///< Relative slip velocity vector v_roller - v_piece (m/s)
        bool isPastCenterline = false;              ///< True if piece center of mass crossed capture centerline
    };

    /**
     * @class IntakeSurfaceModel
     * @brief Mathematical model for intake zone overlap, surface velocity drag, and kinematic capture criteria.
     */
    class IntakeSurfaceModel
    {
    public:
        /**
         * @brief Tests if a game piece bounding volume overlaps an oriented 3D intake contact zone box.
         * @param pieceWorldPos Game piece world center of mass position (m).
         * @param pieceRadiusM Game piece effective outer radius (m).
         * @param zoneWorldPos Intake contact zone center in world space (m).
         * @param zoneWorldRot Intake contact zone orientation quaternion in world space.
         * @param zoneDimensionsM Full dimensions (Dx, Dy, Dz) of the contact zone (m).
         * @return True if game piece bounding volume overlaps the intake volume.
         */
        static bool TestZoneOverlap(const glm::vec3& pieceWorldPos,
                                    double pieceRadiusM,
                                    const glm::vec3& zoneWorldPos,
                                    const glm::quat& zoneWorldRot,
                                    const glm::vec3& zoneDimensionsM);

        /**
         * @brief Evaluates roller surface velocity in world space: v_roller = (omega * r) * t_world + v_parent.
         * @param angularVelocityRadPerS Angular speed of roller (rad/s).
         * @param rollerRadiusM Roller outer radius (m).
         * @param tangentAxisWorld Unit vector pointing inward along intake throat in world space.
         * @param parentLinkVelocity Linear velocity of parent link at the roller (m/s).
         * @return Roller surface velocity vector in world space (m/s).
         */
        static glm::vec3 CalculateRollerSurfaceVelocity(double angularVelocityRadPerS,
                                                        double rollerRadiusM,
                                                        const glm::vec3& tangentAxisWorld,
                                                        const glm::vec3& parentLinkVelocity = glm::vec3(0.0f));

        /**
         * @brief Evaluates compliant friction drag force and equal-and-opposite motor load torque.
         * @param params Intake geometry and grip tuning parameters.
         * @param rollerSurfaceVelocity Roller surface velocity in world space (m/s).
         * @param pieceVelocity Game piece linear velocity in world space (m/s).
         * @param tangentAxisWorld Unit vector pointing inward along intake throat in world space.
         * @param progressAlongTangentM Projected distance of piece center of mass along tangent axis (m).
         * @return IntakeTractionResult containing intake force vector, load torque, and capture flag.
         */
        static IntakeTractionResult CalculateIntakeTraction(const IntakeParameters& params,
                                                            const glm::vec3& rollerSurfaceVelocity,
                                                            const glm::vec3& pieceVelocity,
                                                            const glm::vec3& tangentAxisWorld,
                                                            double progressAlongTangentM);
    };
}
