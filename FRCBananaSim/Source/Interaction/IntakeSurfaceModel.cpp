/**
 * @file IntakeSurfaceModel.cpp
 * @brief Implementation of active intake compliant traction drag and kinematic capture model.
 */

#include "Interaction/IntakeSurfaceModel.h"
#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Interaction
{
    bool IntakeSurfaceModel::TestZoneOverlap(const glm::vec3& pieceWorldPos,
                                            double pieceRadiusM,
                                            const glm::vec3& zoneWorldPos,
                                            const glm::quat& zoneWorldRot,
                                            const glm::vec3& zoneDimensionsM)
    {
        glm::vec3 deltaWorld = pieceWorldPos - zoneWorldPos;
        glm::quat invRot = glm::inverse(zoneWorldRot);
        glm::vec3 deltaLocal = invRot * deltaWorld;
        glm::vec3 halfExtents = 0.5f * glm::abs(zoneDimensionsM);
        float r = static_cast<float>(pieceRadiusM > 0.001 ? pieceRadiusM : 0.10);

        bool overlapX = std::abs(deltaLocal.x) <= (halfExtents.x + r);
        bool overlapY = std::abs(deltaLocal.y) <= (halfExtents.y + r);
        bool overlapZ = std::abs(deltaLocal.z) <= (halfExtents.z + r);
        return overlapX && overlapY && overlapZ;
    }

    glm::vec3 IntakeSurfaceModel::CalculateRollerSurfaceVelocity(double angularVelocityRadPerS,
                                                                double rollerRadiusM,
                                                                const glm::vec3& tangentAxisWorld,
                                                                const glm::vec3& parentLinkVelocity)
    {
        glm::vec3 tNorm = glm::length(tangentAxisWorld) > 1e-4f ? glm::normalize(tangentAxisWorld) : glm::vec3(0.0f, 0.0f, 1.0f);
        float surfaceSpeed = static_cast<float>(angularVelocityRadPerS * rollerRadiusM);
        return parentLinkVelocity + tNorm * surfaceSpeed;
    }

    IntakeTractionResult IntakeSurfaceModel::CalculateIntakeTraction(const IntakeParameters& params,
                                                                    const glm::vec3& rollerSurfaceVelocity,
                                                                    const glm::vec3& pieceVelocity,
                                                                    const glm::vec3& tangentAxisWorld,
                                                                    double progressAlongTangentM)
    {
        IntakeTractionResult result;
        result.relativeVelocity = rollerSurfaceVelocity - pieceVelocity;
        result.isPastCenterline = (progressAlongTangentM >= params.centerlineCaptureThresholdM);

        float vRelMag = glm::length(result.relativeVelocity);
        if (vRelMag < 1e-4f)
        {
            result.intakeForceN = glm::vec3(0.0f);
            result.loadTorqueNm = 0.0;
            return result;
        }

        glm::vec3 vRelDir = result.relativeVelocity / vRelMag;

        // F_coulomb = mu * F_n * vRelDir
        // F_viscous = c_viscous * v_rel
        float fCoulomb = static_cast<float>(params.frictionMu * params.normalForceN);
        glm::vec3 fViscous = static_cast<float>(params.viscousDampingNsPerM) * result.relativeVelocity;
        glm::vec3 fRaw = fCoulomb * vRelDir + fViscous;

        float fRawMag = glm::length(fRaw);
        float maxGrip = static_cast<float>(params.maxGripForceN > 1.0 ? params.maxGripForceN : 120.0);
        float fClampedMag = std::min(maxGrip, fRawMag);

        if (fRawMag > 1e-6f)
        {
            result.intakeForceN = (fRaw / fRawMag) * fClampedMag;
        }
        else
        {
            result.intakeForceN = glm::vec3(0.0f);
        }

        // Equal-and-opposite reaction braking torque on roller motor: tau = ||F_intake|| * r_roller
        result.loadTorqueNm = static_cast<double>(fClampedMag) * params.rollerRadiusM;
        return result;
    }
}
