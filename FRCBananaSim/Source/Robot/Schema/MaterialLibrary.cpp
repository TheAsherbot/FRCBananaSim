/**
 * @file MaterialLibrary.cpp
 * @brief Implementation of MaterialLibrary database, pairwise contact matrix, and JSON deserializer.
 */

#include "Robot/Schema/MaterialLibrary.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace FRCBananaSim::Robot::Schema
{
    MaterialLibrary::MaterialLibrary()
    {
        InitializeDefaultMaterials();
    }

    std::string MaterialLibrary::MakePairKey(const std::string& a, const std::string& b)
    {
        return (a < b) ? (a + ":" + b) : (b + ":" + a);
    }

    void MaterialLibrary::InitializeDefaultMaterials()
    {
        // 1. Register Standard FRC Bulk Materials
        RegisterMaterial({ "aluminum_6061", 2700.0, 0.45, 0.05 });
        RegisterMaterial({ "steel_4140", 7850.0, 0.50, 0.10 });
        RegisterMaterial({ "polycarbonate", 1200.0, 0.40, 0.15 });
        RegisterMaterial({ "hdpe", 960.0, 0.25, 0.10 });
        RegisterMaterial({ "bumper_fabric", 500.0, 0.70, 0.05 });
        RegisterMaterial({ "colson_performa", 1100.0, 1.10, 0.05 });
        RegisterMaterial({ "bilinear_carpet", 400.0, 0.95, 0.01 });
        RegisterMaterial({ "game_piece_foam", 80.0, 0.65, 0.20 });

        // 2. Register Calibrated FRC Contact Pair Interfacial Matrix
        // Colson Performa Rubber on FRC Bilinear Carpet (High Traction)
        RegisterMaterialPair({ "colson_performa", "bilinear_carpet", 1.25, 1.10, 0.02, 0.015 });

        // Colson Rubber on Polycarbonate (Floor Intake Ramp / Slide)
        RegisterMaterialPair({ "colson_performa", "polycarbonate", 0.65, 0.55, 0.08, 0.005 });

        // Game Piece Foam on Polycarbonate (Intake Compression Hood)
        RegisterMaterialPair({ "game_piece_foam", "polycarbonate", 0.75, 0.68, 0.22, 0.0 });

        // Game Piece Foam on Colson Rubber (Active Intake Roller Capture)
        RegisterMaterialPair({ "colson_performa", "game_piece_foam", 0.95, 0.85, 0.18, 0.0 });

        // Bumper Fabric on Polycarbonate / Aluminum (Robot Collisions)
        RegisterMaterialPair({ "bumper_fabric", "polycarbonate", 0.55, 0.45, 0.10, 0.0 });
        RegisterMaterialPair({ "bumper_fabric", "aluminum_6061", 0.50, 0.42, 0.08, 0.0 });
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
        bool success = false;

        // 1. Ingest Individual Material Profiles
        if (json.contains("materials"))
        {
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
                success = true;
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
                success = true;
            }
        }

        // 2. Ingest Calibrated Contact Pair Matrix
        if (json.contains("materialPairs") && json["materialPairs"].is_array())
        {
            for (const auto& item : json["materialPairs"])
            {
                MaterialPairProperties pair;
                pair.materialA = item.value("materialA", "");
                pair.materialB = item.value("materialB", "");
                if (pair.materialA.empty() || pair.materialB.empty())
                {
                    continue;
                }

                pair.staticFriction = item.value("staticFriction", 0.7);
                pair.dynamicFriction = item.value("dynamicFriction", pair.staticFriction * 0.9);
                pair.restitution = item.value("restitution", 0.0);
                pair.rollingResistance = item.value("rollingResistance", 0.0);
                RegisterMaterialPair(pair);
            }
            success = true;
        }

        return success;
    }

    void MaterialLibrary::RegisterMaterial(const MaterialProperties& props)
    {
        materials[props.name] = props;
    }

    void MaterialLibrary::RegisterMaterialPair(const MaterialPairProperties& pair)
    {
        materialPairs[MakePairKey(pair.materialA, pair.materialB)] = pair;
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

    const MaterialPairProperties* MaterialLibrary::GetMaterialPair(const std::string& materialA,
        const std::string& materialB) const
    {
        auto it = materialPairs.find(MakePairKey(materialA, materialB));
        if (it != materialPairs.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    MaterialPairProperties MaterialLibrary::ResolveContactProperties(const std::string& materialA,
        const std::string& materialB) const
    {
        // 1. Direct hit in calibrated contact pair matrix
        const MaterialPairProperties* explicitPair = GetMaterialPair(materialA, materialB);
        if (explicitPair != nullptr)
        {
            return *explicitPair;
        }

        // 2. Fallback to physical mixing rules
        MaterialPairProperties fallback;
        fallback.materialA = materialA;
        fallback.materialB = materialB;

        const MaterialProperties* propA = GetMaterial(materialA);
        const MaterialProperties* propB = GetMaterial(materialB);

        if (propA != nullptr && propB != nullptr)
        {
            // Standard geometric mean mixing for friction and restitution
            fallback.staticFriction = std::sqrt(propA->friction * propB->friction);
            fallback.dynamicFriction = fallback.staticFriction * 0.9;
            fallback.restitution = std::sqrt(propA->restitution * propB->restitution);
        }
        else if (propA != nullptr)
        {
            fallback.staticFriction = propA->friction;
            fallback.dynamicFriction = propA->friction * 0.9;
            fallback.restitution = propA->restitution;
        }
        else if (propB != nullptr)
        {
            fallback.staticFriction = propB->friction;
            fallback.dynamicFriction = propB->friction * 0.9;
            fallback.restitution = propB->restitution;
        }

        return fallback;
    }

    bool MaterialLibrary::HasMaterial(const std::string& name) const
    {
        return materials.find(name) != materials.end();
    }

    size_t MaterialLibrary::GetMaterialCount() const
    {
        return materials.size();
    }

    size_t MaterialLibrary::GetMaterialPairCount() const
    {
        return materialPairs.size();
    }

    void MaterialLibrary::Clear()
    {
        materials.clear();
        materialPairs.clear();
    }
}