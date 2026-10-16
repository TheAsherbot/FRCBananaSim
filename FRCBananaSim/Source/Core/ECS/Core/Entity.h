/**
 * @file Entity.h
 * @brief Entity identifier definition and reserved sentinel constants.
 */

#pragma once

#include <cstdint>

namespace FRCBananaSim::Core::ECS::Core
{
    /**
     * @typedef Entity
     * @brief Unique identifier representing an entity in a Scene.
     */
    using Entity = std::uint32_t;

    /**
     * @brief Sentinel value indicating an invalid, null, or unassigned entity handle.
     */
    constexpr Entity invalidEntity = 0;
}

namespace FRCBananaSim::Core::ECS
{
    using Core::Entity;
    using Core::invalidEntity;
}