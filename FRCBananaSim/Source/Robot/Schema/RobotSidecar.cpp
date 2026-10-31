/**
 * @file RobotSidecar.cpp
 * @brief Implementation of RobotSidecar lookup helper methods.
 */

#include "Robot/Schema/RobotSidecar.h"

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
}