/**
 * @file RobotSidecarSerializer.h
 * @brief Serialization of RobotSidecar data structures into Schema v1.0.0 sidecar.json.
 */

#pragma once

#include "Robot/Schema/RobotSidecar.h"
#include <nlohmann/json.hpp>
#include <string>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @class RobotSidecarSerializer
     * @brief Serializes runtime RobotSidecar structs into formatted JSON conforming to Schema v1.0.0.
     */
    class RobotSidecarSerializer
    {
    public:
        /**
         * @brief Converts a RobotSidecar specification into an nlohmann::json object.
         * @param sidecar Runtime sidecar specification to serialize.
         * @return Populated JSON object matching Schema v1.0.0.
         */
        static nlohmann::json SerializeToJson(const RobotSidecar& sidecar);

        /**
         * @brief Converts a RobotSidecar specification into a formatted JSON string.
         * @param sidecar Runtime sidecar specification to serialize.
         * @param indent Indentation space count for pretty-printing (default 2).
         * @return Formatted JSON string.
         */
        static std::string SerializeToString(const RobotSidecar& sidecar, int indent = 2);

        /**
         * @brief Serializes a RobotSidecar specification directly to a file on disk.
         * @param sidecar Runtime sidecar specification to serialize.
         * @param filepath Target file path on disk.
         * @param indent Indentation space count for pretty-printing (default 2).
         * @return True if file was written successfully, false otherwise.
         */
        static bool SerializeToFile(const RobotSidecar& sidecar, const std::string& filepath, int indent = 2);
    };
}
