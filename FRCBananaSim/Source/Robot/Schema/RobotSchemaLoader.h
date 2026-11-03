/**
 * @file RobotSchemaLoader.h
 * @brief Schema v1 loader, topological validator, and NWU to Engine coordinate transformer.
 */

#pragma once

#include "Robot/Schema/RobotManifest.h"
#include "Robot/Schema/RobotSidecar.h"
#include "Robot/Schema/MaterialLibrary.h"
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @class RobotSchemaLoader
     * @brief Deserializer, physical validator, and coordinate transformer for Robot Data Schema v1 files.
     */
    class RobotSchemaLoader
    {
    public:
        /**
         * @brief Default constructor initializing clean diagnostics.
         */
        RobotSchemaLoader();

        /**
         * @brief Loads and validates an entire Schema v1 robot specification.
         * @param manifestPath Filepath to manifest.json.
         * @param sidecarPath Filepath to sidecar.json (or empty to infer from manifest directory).
         * @param materialsPath Filepath to materials.json (or empty to infer from Resources/Config/materials.json).
         * @return True if all files parsed and topological validation passed with zero errors.
         */
        bool LoadRobot(const std::string& manifestPath, const std::string& sidecarPath = "", const std::string& materialsPath = "");

        /**
         * @brief Deserializes a manifest.json file into internal RobotManifest data structures.
         * @param filepath Path to manifest.json.
         * @return True if parsing succeeded.
         */
        bool LoadManifestFromFile(const std::string& filepath);

        /**
         * @brief Deserializes a parsed nlohmann::json object into RobotManifest data structures.
         * @param json Parsed JSON object.
         * @return True if structure matches Schema v1.
         */
        bool LoadManifestFromJson(const nlohmann::json& json);

        /**
         * @brief Deserializes a sidecar.json file into internal RobotSidecar data structures.
         * @param filepath Path to sidecar.json.
         * @return True if parsing succeeded.
         */
        bool LoadSidecarFromFile(const std::string& filepath);

        /**
         * @brief Deserializes a parsed nlohmann::json object into RobotSidecar data structures.
         * @param json Parsed JSON object.
         * @return True if structure matches Schema v1.
         */
        bool LoadSidecarFromJson(const nlohmann::json& json);

        /**
         * @brief Loads materials and pairwise contact parameters from a materials.json file.
         * @param filepath Path to materials.json.
         * @return True if loaded successfully.
         */
        bool LoadMaterialsFromFile(const std::string& filepath);

        /**
         * @brief Validates schema versions, positive mass/inertia, and cross-file topological link/joint references.
         * @return True if zero fatal errors were encountered.
         */
        bool Validate();

        // Accessors for parsed specifications
        const RobotManifest& GetManifest() const;
        const RobotSidecar& GetSidecar() const;
        const MaterialLibrary& GetMaterials() const;
        MaterialLibrary& GetMaterials();

        // Diagnostic inspection
        bool HasErrors() const;
        bool HasWarnings() const;
        const std::vector<std::string>& GetErrors() const;
        const std::vector<std::string>& GetWarnings() const;
        void ClearDiagnostics();

        // Bidirectional Coordinate Transformation (WPILib NWU <-> Engine Frame)
        static glm::dvec3 ConvertNWUVectorToEngine(const glm::dvec3& vNWU);
        static glm::dvec3 ConvertEngineVectorToNWU(const glm::dvec3& vEngine);

        static glm::vec3 ConvertNWUVectorToEngine(const glm::vec3& vNWU);
        static glm::vec3 ConvertEngineVectorToNWU(const glm::vec3& vEngine);

        static glm::dmat3 ConvertNWUInertiaToEngine(const glm::dmat3& iNWU);
        static glm::dmat3 ConvertEngineInertiaToNWU(const glm::dmat3& iEngine);

        static glm::dquat ConvertNWUQuatToEngine(const glm::dquat& qNWU);
        static glm::dquat ConvertEngineQuatToNWU(const glm::dquat& qEngine);

        static glm::dmat3 GetNWUToEngineMatrix();
        static glm::dmat3 GetEngineToNWUMatrix();

    private:
        RobotManifest manifest;
        RobotSidecar sidecar;
        MaterialLibrary materialLibrary;

        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };
}