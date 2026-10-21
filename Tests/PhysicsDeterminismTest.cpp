/**
 * @file PhysicsDeterminismTest.cpp
 * @brief Implementation of 200 Hz physics determinism test comparing state across identical runs.
 */

#include "PhysicsDeterminismTest.h"
#include "TestFramework.h"
#include "Physics/JoltContext.h"
#include "Physics/PhysicsConfig.h"

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <glm/vec3.hpp>
#include <chrono>
#include <iostream>
#include <vector>

namespace FRCBananaSim::Tests
{
    using Physics::JoltContext;
    using Physics::PhysicsConfig;
    using Physics::ObjectLayers::Moving;
    using Physics::ObjectLayers::NonMoving;

    struct SimulationSnapshot
    {
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> linearVelocities;
    };

    static SimulationSnapshot RunSimulationRun()
    {
        JoltContext context;
        context.Initialize();

        JPH::BodyInterface& bodyInterface = context.GetBodyInterface();

        // 1. Static Floor
        JPH::BoxShapeSettings floorShapeSettings(JPH::Vec3(15.0f, 0.1f, 15.0f));
        JPH::BodyCreationSettings floorSettings(
            floorShapeSettings.Create().Get(),
            JPH::RVec3(0.0f, -0.1f, 0.0f),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Static,
            NonMoving
        );
        floorSettings.mFriction = 0.8f;
        floorSettings.mRestitution = 0.1f;
        bodyInterface.CreateAndAddBody(floorSettings, JPH::EActivation::DontActivate);

        // 2. Dynamic bodies
        std::vector<JPH::BodyID> dynamicBodies;

        // Boxes
        for (int i = 0; i < 3; ++i)
        {
            JPH::BoxShapeSettings boxSettings(JPH::Vec3(0.5f, 0.5f, 0.5f));
            JPH::BodyCreationSettings settings(
                boxSettings.Create().Get(),
                JPH::RVec3(0.0f, 1.0f + static_cast<float>(i) * 1.2f, 0.0f),
                JPH::Quat::sIdentity(),
                JPH::EMotionType::Dynamic,
                Moving
            );
            settings.mFriction = 0.7f;
            settings.mRestitution = 0.1f;
            settings.mLinearDamping = PhysicsConfig::defaultLinearDamping;
            settings.mAngularDamping = PhysicsConfig::defaultAngularDamping;
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = 10.0f;

            JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
            dynamicBodies.push_back(id);
        }

        // Sphere
        {
            JPH::SphereShapeSettings sphereSettings(0.4f);
            JPH::BodyCreationSettings settings(
                sphereSettings.Create().Get(),
                JPH::RVec3(1.5f, 4.0f, 0.0f),
                JPH::Quat::sIdentity(),
                JPH::EMotionType::Dynamic,
                Moving
            );
            settings.mFriction = 0.6f;
            settings.mRestitution = 0.5f;
            settings.mLinearDamping = PhysicsConfig::defaultLinearDamping;
            settings.mAngularDamping = PhysicsConfig::defaultAngularDamping;
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = 0.5f;

            JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
            dynamicBodies.push_back(id);
        }

        // 3. Step for 1,000 ticks at 200 Hz (5.0 seconds simulated time)
        constexpr int totalTicks = 1000;
        constexpr float dt = static_cast<float>(PhysicsConfig::fixedDeltaTime);

        auto startTime = std::chrono::high_resolution_clock::now();

        for (int tick = 0; tick < totalTicks; ++tick)
        {
            context.Step(dt);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        double totalMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        double usPerStep = totalMicroseconds / totalTicks;
        std::cout << "  [PERF] 1,000 steps executed in " << (totalMicroseconds / 1000.0) << " ms ("
            << usPerStep << " us/step)\n";

        // 4. Capture snapshot
        SimulationSnapshot snapshot;
        for (const JPH::BodyID& id : dynamicBodies)
        {
            JPH::RVec3 pos = bodyInterface.GetPosition(id);
            JPH::Vec3 vel = bodyInterface.GetLinearVelocity(id);
            snapshot.positions.emplace_back(pos.GetX(), pos.GetY(), pos.GetZ());
            snapshot.linearVelocities.emplace_back(vel.GetX(), vel.GetY(), vel.GetZ());
        }

        context.Shutdown();
        return snapshot;
    }

    void RunPhysicsDeterminismTest(TestContext& ctx)
    {
        std::cout << "[RUN ] PhysicsDeterminismTest (1,000 steps at 200 Hz)...\n";
        constexpr double tolerance = 1e-5;

        // Run Run 1
        SimulationSnapshot run1 = RunSimulationRun();

        // Run Run 2 from scratch
        SimulationSnapshot run2 = RunSimulationRun();

        // Validate bitwise / floating-point parity
        TEST_EXPECT_EQ(ctx, run1.positions.size(), run2.positions.size());

        for (size_t i = 0; i < run1.positions.size(); ++i)
        {
            TEST_EXPECT_NEAR(ctx, run1.positions[i].x, run2.positions[i].x, tolerance);
            TEST_EXPECT_NEAR(ctx, run1.positions[i].y, run2.positions[i].y, tolerance);
            TEST_EXPECT_NEAR(ctx, run1.positions[i].z, run2.positions[i].z, tolerance);

            TEST_EXPECT_NEAR(ctx, run1.linearVelocities[i].x, run2.linearVelocities[i].x, tolerance);
            TEST_EXPECT_NEAR(ctx, run1.linearVelocities[i].y, run2.linearVelocities[i].y, tolerance);
            TEST_EXPECT_NEAR(ctx, run1.linearVelocities[i].z, run2.linearVelocities[i].z, tolerance);
        }

        std::cout << "[DONE] PhysicsDeterminismTest\n";
    }
}