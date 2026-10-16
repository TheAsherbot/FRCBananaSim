/**
 * @file System.cpp
 * @brief Default empty implementations for ISystem lifecycle hooks.
 */

#include "System.h"

namespace FRCBananaSim::Core::ECS::Core
{
    ISystem::ISystem(Scene& scene)
        : scene(scene)
    {}

    ISystem::~ISystem() = default;

    void ISystem::Update()
    {}

    void ISystem::FixedUpdate(double /*dt*/)
    {}

    void ISystem::RenderUpdate()
    {}
}