/**
 * @file MaterialLibrary.h
 * @brief Physical material property database and lookup registry for Schema v1.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @struct MaterialProperties
     * @brief Physical properties defining friction, restitution, and volumetric density.
     */
    struct MaterialProperties
    {
        std::string name;
        double densityKgM3 = 2700.0;
        double friction = 0.7;
        double restitution = 0.0;
    };

    /**
     * @class MaterialLibrary
     * @brief Registry of surface and bulk material properties referenced by colliders and links.
     */
    class MaterialLibrary
    {
    public:
        /**
         * @brief Default constructor populating standard FRC material presets.
         */
        MaterialLibrary();

        /**
         * @brief Loads and parses material definitions from a JSON file.
         * @param filepath Path to materials.json.
         * @return True if parsing succeeded.
         */
        bool LoadFromFile(const std::string& filepath);

        /**
         * @brief Ingests material definitions directly from a parsed nlohmann::json object.
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
         * @brief Retrieves material properties by identifier.
         * @param name Unique material name.
         * @return Pointer to properties, or nullptr if not registered.
         */
        const MaterialProperties* GetMaterial(const std::string& name) const;

        /**
         * @brief Checks if a material name exists in the database.
         * @param name Unique material name.
         * @return True if registered.
         */
        bool HasMaterial(const std::string& name) const;

        /**
         * @brief Returns the total count of registered materials.
         * @return Number of registered materials.
         */
        size_t GetMaterialCount() const;

        /**
         * @brief Clears all registered materials.
         */
        void Clear();

    private:
        /**
         * @brief Registers verified standard FRC engineering materials.
         */
        void InitializeDefaultMaterials();

        std::unordered_map<std::string, MaterialProperties> materials;
    };
}