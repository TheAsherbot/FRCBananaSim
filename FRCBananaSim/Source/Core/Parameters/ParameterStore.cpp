/**
 * @file ParameterStore.cpp
 * @brief Implementation of hierarchical JSON parameter ingestion and layered merging.
 */

#include "ParameterStore.h"
#include "Core/Units/Units.h"
#include "Telemetry/TelemetryMacros.h"
#include "Utils/Log.h"

#include <fstream>
#include <sstream>

namespace FRCBananaSim::Core::Parameters
{
    using FRCBananaSim::Core::Units::Units;

    ParameterStore::ParameterStore()
    {}

    ParameterStore::~ParameterStore()
    {}

    bool ParameterStore::LoadFromFile(const std::string& filepath)
    {
        SIM_PROFILE_SCOPE("ParameterStore::LoadFromFile");

        std::ifstream file(filepath);
        if (!file.is_open())
        {
            SIM_LOG_ERROR("Failed to open parameter file '" << filepath.c_str() << "'");
            return false;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return LoadFromString(buffer.str());
    }

    bool ParameterStore::LoadFromString(const std::string& jsonString)
    {
        SIM_PROFILE_SCOPE("ParameterStore::LoadFromString");

        try
        {
            nlohmann::json root = nlohmann::json::parse(jsonString);
            FlattenJsonObject(root, "");
            return true;
        }
        catch (const nlohmann::json::exception& e)
        {
            SIM_LOG_ERROR("JSON parse error in ParameterStore: " << e.what());
            return false;
        }
    }

    void ParameterStore::Merge(const ParameterStore& overlay)
    {
        SIM_PROFILE_SCOPE("ParameterStore::Merge");

        for (const auto& [key, value] : overlay.parameters)
        {
            parameters[key] = value;
        }
    }

    bool ParameterStore::Has(const std::string& key) const
    {
        return parameters.find(key) != parameters.end();
    }

    double ParameterStore::GetDouble(const std::string& key, double fallback) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end() && it->second.isNumeric)
        {
            return it->second.rawValue;
        }
        return fallback;
    }

    double ParameterStore::GetSI(const std::string& key, double fallback) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end() && it->second.isNumeric)
        {
            return it->second.siValue;
        }
        return fallback;
    }

    std::string ParameterStore::GetString(const std::string& key, const std::string& fallback) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end() && it->second.isString)
        {
            return it->second.stringValue;
        }
        return fallback;
    }

    bool ParameterStore::GetBool(const std::string& key, bool fallback) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end() && it->second.isBoolean)
        {
            return it->second.boolValue;
        }
        return fallback;
    }

    std::string ParameterStore::GetUnit(const std::string& key) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end())
        {
            return it->second.unit;
        }
        return "";
    }

    const ParameterValue* ParameterStore::GetParameter(const std::string& key) const
    {
        auto it = parameters.find(key);
        if (it != parameters.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    void ParameterStore::SetParameter(const std::string& key, const ParameterValue& value)
    {
        parameters[key] = value;
    }

    void ParameterStore::Clear()
    {
        parameters.clear();
    }

    size_t ParameterStore::Size() const
    {
        return parameters.size();
    }

    void ParameterStore::FlattenJsonObject(const nlohmann::json& jsonNode, const std::string& currentPath)
    {
        if (jsonNode.is_object())
        {
            // Check if this object represents a dimensional value: { "value": X, "unit": "..." }
            if (jsonNode.contains("value") && (jsonNode.contains("unit") || jsonNode.size() <= 3))
            {
                const auto& valNode = jsonNode["value"];
                if (valNode.is_number())
                {
                    ParameterValue param;
                    param.rawValue = valNode.get<double>();
                    param.isNumeric = true;

                    if (jsonNode.contains("unit") && jsonNode["unit"].is_string())
                    {
                        param.unit = jsonNode["unit"].get<std::string>();
                        param.siValue = Units::ToSI(param.rawValue, param.unit);
                    }
                    else
                    {
                        param.siValue = param.rawValue;
                    }

                    parameters[currentPath] = param;
                    return;
                }
            }

            // Otherwise, recursively traverse child keys
            for (auto it = jsonNode.begin(); it != jsonNode.end(); ++it)
            {
                std::string childPath = currentPath.empty() ? it.key() : currentPath + "." + it.key();
                FlattenJsonObject(it.value(), childPath);
            }
        }
        else if (jsonNode.is_number())
        {
            ParameterValue param;
            param.rawValue = jsonNode.get<double>();
            param.siValue = param.rawValue;
            param.isNumeric = true;
            parameters[currentPath] = param;
        }
        else if (jsonNode.is_string())
        {
            ParameterValue param;
            param.stringValue = jsonNode.get<std::string>();
            param.isString = true;
            parameters[currentPath] = param;
        }
        else if (jsonNode.is_boolean())
        {
            ParameterValue param;
            param.boolValue = jsonNode.get<bool>();
            param.isBoolean = true;
            parameters[currentPath] = param;
        }
    }
}