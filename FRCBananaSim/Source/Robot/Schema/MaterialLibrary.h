/**
 * @file MaterialLibrary.h
 * @brief Physical material property database and contact pair lookup registry for Schema v1.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @struct MaterialProperties
     * @brief Bulk physical properties for an individual material (density, base friction, restitution).
     */
    struct MaterialProperties
    {
        std::string name;
        double densityKgM3 = 2700.0;
        double friction = 0.7;
        double restitution = 0.0;
    };

    /**
     * @struct MaterialPairProperties
     * @brief Calibrated contact interaction parameters between two specific materials.
     */
    struct MaterialPairProperties
    {
        std::string materialA;
        std::string materialB;
        double staticFriction = 0.7;
        double dynamicFriction = 0.6;
        double restitution = 0.0;
        double rollingResistance = 0.0;
    };

    /**
     * @class MaterialLibrary
     * @brief Registry of surface materials and pairwise contact interaction parameters.
     */
    class MaterialLibrary
    {
    public:
        /**
         * @brief Default constructor populating standard FRC material presets and measured contact pairs.
         */
        MaterialLibrary();

        /**
         * @brief Loads and parses material definitions and pair matrices from a JSON file.
         * @param filepath Path to materials.json.
         * @return True if parsing succeeded.
         */
        bool LoadFromFile(const std::string& filepath);

        /**
         * @brief Ingests material definitions and pair matrices directly from a parsed nlohmann::json object.
         * @param json Parsed JSON object.
         * @return True if parsing succeeded.
         */
        bool LoadFromJson(const nlohmann::json& json);

        /**
         * @brief Registers or overwrites a named material profile.
         * @param props Material properties to register.
         */
        void RegisterMaterial(const MaterialProperties& props);

        /**
         * @brief Registers or overwrites a calibrated contact pair interaction.
         * @param pair Contact pair properties.
         */
        void RegisterMaterialPair(const MaterialPairProperties& pair);

        /**
         * @brief Retrieves single material properties by identifier.
         * @param name Unique material name.
         * @return Pointer to properties, or nullptr if not registered.
         */
        const MaterialProperties* GetMaterial(const std::string& name) const;

        /**
         * @brief Retrieves calibrated contact pair properties if explicitly registered.
         * @param materialA Name of first contacting material.
         * @param materialB Name of second contacting material.
         * @return Pointer to calibrated pair properties, or nullptr if no explicit pair is registered.
         */
        const MaterialPairProperties* GetMaterialPair(const std::string& materialA, const std::string& materialB) const;

        /**
         * @brief Resolves contact parameters for two materials: uses calibrated pair if available,
         * or computes physics-based mixed properties from individual materials.
         * @param materialA Name of first contacting material.
         * @param materialB Name of second contacting material.
         * @return Resolved contact interaction properties.
         */
        MaterialPairProperties ResolveContactProperties(const std::string& materialA, const std::string& materialB) const;

        /**
         * @brief Checks if a single material name exists in the database.
         * @param name Unique material name.
         * @return True if registered.
         */
        bool HasMaterial(const std::string& name) const;

        /**
         * @brief Returns total count of registered individual materials.
         * @return Material count.
         */
        size_t GetMaterialCount() const;

        /**
         * @brief Returns total count of registered contact pairs.
         * @return Contact pair count.
         */
        size_t GetMaterialPairCount() const;

        /**
         * @brief Clears all registered materials and pair matrices.
         */
        void Clear();

    private:
        /**
         * @brief Generates a canonical, order-independent hash key for a material pair.
         * @param a First material name.
         * @param b Second material name.
         * @return Canonical lookup string (min(a,b) + ":" + max(a,b)).
         */
        static std::string MakePairKey(const std::string& a, const std::string& b);

        /**
         * @brief Registers verified standard FRC engineering materials and measured contact pairs.
         */
        void InitializeDefaultMaterials();

        std::unordered_map<std::string, MaterialProperties> materials;
        std::unordered_map<std::string, MaterialPairProperties> materialPairs;
    };
}