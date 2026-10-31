/**
 * @file RobotManifest.cpp
 * @brief Implementation of RobotManifest methods and inertia tensor positive-definiteness check.
 */

#include "Robot/Schema/RobotManifest.h"

namespace FRCBananaSim::Robot::Schema
{
    glm::dmat3 InertiaTensor::ToMatrix() const
    {
        return glm::dmat3(
            ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz
        );
    }

    bool InertiaTensor::IsPositiveDefinite() const
    {
        // Sylvester's Criterion Minor 1: Ixx > 0
        if (ixx <= 1e-9)
        {
            return false;
        }

        // Minor 2: det([Ixx Ixy; Ixy Iyy]) > 0
        double det2 = (ixx * iyy) - (ixy * ixy);
        if (det2 <= 1e-9)
        {
            return false;
        }

        // Minor 3: det(I) > 0
        double det3 = ixx * (iyy * izz - iyz * iyz)
            - ixy * (ixy * izz - iyz * ixz)
            + ixz * (ixy * iyz - iyy * ixz);
        return det3 > 1e-9;
    }

    const LinkDefinition* RobotManifest::FindLink(const std::string& linkName) const
    {
        for (const auto& link : links)
        {
            if (link.name == linkName)
            {
                return &link;
            }
        }
        return nullptr;
    }

    const JointDefinition* RobotManifest::FindJoint(const std::string& jointName) const
    {
        for (const auto& joint : joints)
        {
            if (joint.name == jointName)
            {
                return &joint;
            }
        }
        return nullptr;
    }

    const ContactZoneDefinition* RobotManifest::FindContactZone(const std::string& zoneName) const
    {
        for (const auto& zone : contactZones)
        {
            if (zone.name == zoneName)
            {
                return &zone;
            }
        }
        return nullptr;
    }

    const LinkDefinition* RobotManifest::GetRootLink() const
    {
        for (const auto& link : links)
        {
            if (link.isRoot)
            {
                return &link;
            }
        }
        return nullptr;
    }
}