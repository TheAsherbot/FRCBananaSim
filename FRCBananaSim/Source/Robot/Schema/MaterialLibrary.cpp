/**
 * @file MaterialLibrary.cpp
 * @brief Implementation of MaterialLibrary database and JSON deserializer.
 */

#include "Robot/Schema/MaterialLibrary.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace FRCBananaSim::Robot::Schema
{
    MaterialLibrary::MaterialLibrary()
    {
        InitializeDefaultMaterials();
    }

    void MaterialLibrary::InitializeDefaultMaterials()
    {
        RegisterMaterial({ "aluminum_6061", 2700.0, 0.45, 0.05 });
        RegisterMaterial({ "steel_4140", 7850.0, 0.50, 0.10 });
        RegisterMaterial({ "polycarbonate", 1200.0, 0.40, 0.15 });
        RegisterMaterial({ "hdpe", 960.0, 0.25, 0.10 });
        RegisterMaterial({ "bumper_fabric", 500.0, 0.70, 0.05 });
        RegisterMaterial({ "colson_performa", 1100.0, 1.10, 0.05 });
        RegisterMaterial({ "bilinear_carpet", 400.0, 0.95, 0.01 });
        RegisterMaterial({ "game_piece_foam", 80.0, 0.65, 0.20 });
    }

    bool MaterialLibrary::LoadFromFile(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            std::cerr << "[MaterialLibrary] Failed to open materials file: " << filepath << "\n";
            return false;
        }

        try
        {
            nlohmann::json json;
            file >> json;
            return LoadFromJson(json);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[MaterialLibrary] Error parsing JSON from " << filepath << ": " << e.what() << "\n";
            return false;
        }
    }

    bool MaterialLibrary::LoadFromJson(const nlohmann::json& json)
    {
        if (!json.contains("materials"))
        {
            std::cerr << "[MaterialLibrary] JSON root missing 'materials' field\n";
            return false;
        }

        const auto& matNode = json["materials"];
        if (matNode.is_array())
        {
            for (const auto& item : matNode)
            {
                MaterialProperties props;
                props.name = item.value("name", "");
                if (props.name.empty())
                {
                    continue;
                }
                props.densityKgM3 = item.value("density_kg_m3", 2700.0);
                props.friction = item.value("friction", 0.7);
                props.restitution = item.value("restitution", 0.0);
                RegisterMaterial(props);
            }
            return true;
        }
        else if (matNode.is_object())
        {
            for (auto it = matNode.begin(); it != matNode.end(); ++it)
            {
                MaterialProperties props;
                props.name = it.key();
                props.densityKgM3 = it.value().value("density_kg_m3", 2700.0);
                props.friction = it.value().value("friction", 0.7);
                props.restitution = it.value().value("restitution", 0.0);
                RegisterMaterial(props);
            }
            return true;
        }

        std::cerr << "[MaterialLibrary] 'materials' node is neither array nor object\n";
        return false;
    }

    void MaterialLibrary::RegisterMaterial(const MaterialProperties& props)
    {
        materials[props.name] = props;
    }

    const MaterialProperties* MaterialLibrary::GetMaterial(const std::string& name) const
    {
        auto it = materials.find(name);
        if (it != materials.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    bool MaterialLibrary::HasMaterial(const std::string& name) const
    {
        return materials.find(name) != materials.end();
    }

    size_t MaterialLibrary::GetMaterialCount() const
    {
        return materials.size();
    }

    void MaterialLibrary::Clear()
    {
        materials.clear();
    }
}