/**
 * @file RobotSidecar.cpp
 * @brief Implementation of RobotSidecar lookup helper methods.
 */

#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/RobotManifest.h"

namespace FRCBananaSim::Robot::Schema
{
    const MechanismConfig* RobotSidecar::FindMechanism(const std::string& mechanismName) const
    {
        for (const auto& mech : mechanisms)
        {
            if (mech.name == mechanismName)
            {
                return &mech;
            }
        }
        return nullptr;
    }

    MechanismConfig* RobotSidecar::FindMechanism(const std::string& mechanismName)
    {
        for (auto& mech : mechanisms)
        {
            if (mech.name == mechanismName)
            {
                return &mech;
            }
        }
        return nullptr;
    }

    const SensorConfig* RobotSidecar::FindSensor(const std::string& sensorName) const
    {
        for (const auto& sensor : sensors)
        {
            if (sensor.name == sensorName)
            {
                return &sensor;
            }
        }
        return nullptr;
    }

    SensorConfig* RobotSidecar::FindSensor(const std::string& sensorName)
    {
        for (auto& sensor : sensors)
        {
            if (sensor.name == sensorName)
            {
                return &sensor;
            }
        }
        return nullptr;
    }

    const CouplingConfig* RobotSidecar::FindCoupling(const std::string& couplingName) const
    {
        for (const auto& coupling : couplings)
        {
            if (coupling.name == couplingName)
            {
                return &coupling;
            }
        }
        return nullptr;
    }

    CouplingConfig* RobotSidecar::FindCoupling(const std::string& couplingName)
    {
        for (auto& coupling : couplings)
        {
            if (coupling.name == couplingName)
            {
                return &coupling;
            }
        }
        return nullptr;
    }

    const JointLimitOverride* RobotSidecar::FindJointOverride(const std::string& jointName) const
    {
        auto it = jointOverrides.find(jointName);
        return (it != jointOverrides.end()) ? &it->second : nullptr;
    }

    JointLimitOverride* RobotSidecar::FindJointOverride(const std::string& jointName)
    {
        auto it = jointOverrides.find(jointName);
        return (it != jointOverrides.end()) ? &it->second : nullptr;
    }

    JointLimitOverride RobotSidecar::GetEffectiveLimits(const std::string& jointName, const RobotManifest& manifest) const
    {
        auto it = jointOverrides.find(jointName);
        if (it != jointOverrides.end())
        {
            return it->second;
        }

        const auto* j = manifest.FindJoint(jointName);
        if (j)
        {
            JointLimitOverride ov;
            ov.hasLimits = j->limits.hasLimits;
            ov.lower = j->limits.lower;
            ov.upper = j->limits.upper;
            ov.unit = j->limits.unit;
            ov.springFrequencyHz = (j->limits.springFrequencyHz > 0.0) ? j->limits.springFrequencyHz : 40.0;
            ov.springDampingRatio = (j->limits.springDampingRatio > 0.0) ? j->limits.springDampingRatio : 1.0;
            return ov;
        }

        JointLimitOverride def;
        def.hasLimits = false;
        return def;
    }

    const ContactZoneDefinition* RobotSidecar::FindContactZone(const std::string& zoneName) const
    {
        for (const auto& cz : contactZones)
        {
            if (cz.name == zoneName)
            {
                return &cz;
            }
        }
        return nullptr;
    }

    ContactZoneDefinition* RobotSidecar::FindContactZone(const std::string& zoneName)
    {
        for (auto& cz : contactZones)
        {
            if (cz.name == zoneName)
            {
                return &cz;
            }
        }
        return nullptr;
    }

    std::vector<ContactZoneDefinition> RobotSidecar::GetEffectiveContactZones(const RobotManifest& manifest) const
    {
        std::vector<ContactZoneDefinition> result;
        std::unordered_map<std::string, size_t> zoneIndexMap;

        // 1. Seed with CAD manifest contact zones
        for (const auto& cz : manifest.contactZones)
        {
            zoneIndexMap[cz.name] = result.size();
            result.push_back(cz);
        }

        // 2. Apply sidecar overrides or additions
        for (const auto& cz : contactZones)
        {
            auto it = zoneIndexMap.find(cz.name);
            if (it != zoneIndexMap.end())
            {
                // Override existing manifest zone with sidecar parameters
                result[it->second] = cz;
            }
            else
            {
                // Append new sidecar-defined zone
                zoneIndexMap[cz.name] = result.size();
                result.push_back(cz);
            }
        }

        return result;
    }
}