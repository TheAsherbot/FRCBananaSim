/**
 * @file ParameterStore.h
 * @brief Layered JSON configuration loader supporting unit conversions and defaults overriding.
 */

#pragma once

#include "ParameterValue.h"
#include <nlohmann/json.hpp>

#include <string>
#include <unordered_map>

namespace FRCBananaSim::Core::Parameters
{
    /**
     * @class ParameterStore
     * @brief Configuration dictionary providing layered parameter ingestion and unit conversions.
     */
    class ParameterStore
    {
    public:
        /**
         * @brief Default constructor creating an empty parameter store.
         */
        ParameterStore();

        /**
         * @brief Destructor.
         */
        ~ParameterStore();

        /**
         * @brief Loads parameters from a JSON file on disk.
         * @param filepath Path to the JSON configuration file.
         * @return True if read and parsed successfully, false otherwise.
         */
        bool LoadFromFile(const std::string& filepath);

        /**
         * @brief Loads parameters from a raw JSON string.
         * @param jsonString UTF-8 JSON text.
         * @return True if parsed successfully, false otherwise.
         */
        bool LoadFromString(const std::string& jsonString);

        /**
         * @brief Overlays parameters from another store on top of this one (defaults -> measured -> calibrated).
         * @param overlay Store whose matching keys take precedence.
         */
        void Merge(const ParameterStore& overlay);

        /**
         * @brief Checks if a parameter exists in the store.
         * @param key Flattened dot-separated path (e.g., "chassis.mass").
         * @return True if present, false otherwise.
         */
        bool Has(const std::string& key) const;

        /**
         * @brief Gets a numeric parameter in its raw configured units.
         * @param key Flattened path.
         * @param fallback Default value returned if key is missing.
         * @return Raw numeric magnitude.
         */
        double GetDouble(const std::string& key, double fallback = 0.0) const;

        /**
         * @brief Gets a numeric parameter converted into canonical SI base units.
         * @param key Flattened path.
         * @param fallback Default SI value returned if key is missing.
         * @return Value converted to SI units.
         */
        double GetSI(const std::string& key, double fallback = 0.0) const;

        /**
         * @brief Gets a text parameter.
         * @param key Flattened path.
         * @param fallback Default string returned if key is missing.
         * @return String value.
         */
        std::string GetString(const std::string& key, const std::string& fallback = "") const;

        /**
         * @brief Gets a boolean parameter.
         * @param key Flattened path.
         * @param fallback Default boolean returned if key is missing.
         * @return Boolean value.
         */
        bool GetBool(const std::string& key, bool fallback = false) const;

        /**
         * @brief Gets the declared unit string for a parameter.
         * @param key Flattened path.
         * @return Unit string (or empty if unassigned).
         */
        std::string GetUnit(const std::string& key) const;

        /**
         * @brief Directly queries a parameter definition.
         * @param key Flattened path.
         * @return Pointer to ParameterValue if present, nullptr otherwise.
         */
        const ParameterValue* GetParameter(const std::string& key) const;

        /**
         * @brief Manually sets or replaces a parameter entry.
         * @param key Flattened path.
         * @param value Parameter value to store.
         */
        void SetParameter(const std::string& key, const ParameterValue& value);

        /**
         * @brief Clears all entries from the store.
         */
        void Clear();

        /**
         * @brief Returns total number of registered parameters.
         * @return Count of parameters.
         */
        size_t Size() const;

    private:
        /**
         * @brief Recursively traverses JSON hierarchy and populates flattened dot-separated keys.
         * @param jsonNode Current JSON object or element.
         * @param currentPath Accumulated dot-separated path prefix.
         */
        void FlattenJsonObject(const nlohmann::json& jsonNode, const std::string& currentPath);

        std::unordered_map<std::string, ParameterValue> parameters;
    };
}