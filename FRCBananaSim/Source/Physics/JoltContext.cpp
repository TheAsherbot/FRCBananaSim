/**
 * @file JoltContext.cpp
 * @brief Implementation of the Jolt Physics subsystem wrapper and collision filter tables.
 */

#include "JoltContext.h"
#include "Telemetry/TelemetryMacros.h"
#include "Utils/Log.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <cstdarg>

namespace FRCBananaSim::Physics
{
    /**
     * @brief Custom Jolt trace handler routing messages to the SIM_LOG system.
     * @param format Printf-style format string.
     */
    static void JoltTraceCallback(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        SIM_LOG_DEBUG("[Jolt] " << buffer);
    }

#ifdef JPH_ENABLE_ASSERTS
    /**
     * @brief Custom Jolt assertion failure handler.
     * @param expression Failed condition expression string.
     * @param message Optional diagnostic message.
     * @param file Source file name.
     * @param line Source line number.
     * @return True to trigger breakpoint in attached debugger.
     */
    static bool JoltAssertCallback(const char* expression, const char* message, const char* file, JPH::uint line)
    {
        SIM_LOG_ERROR("[Jolt Assert] %s:%u: (%s) %s", file, line, expression, (message ? message : ""));
        return true;
    }
#endif

    /**
     * @class BPLayerInterfaceImpl
     * @brief Maps ObjectLayers to BroadPhaseLayers.
     */
    class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
    {
    public:
        BPLayerInterfaceImpl()
        {
            objectToBroadPhase[ObjectLayers::NonMoving] = BroadPhaseLayers::NonMoving;
            objectToBroadPhase[ObjectLayers::Moving] = BroadPhaseLayers::Moving;
            objectToBroadPhase[ObjectLayers::Sensor] = BroadPhaseLayers::Sensor;
        }

        virtual JPH::uint GetNumBroadPhaseLayers() const override
        {
            return BroadPhaseLayers::NumLayers;
        }

        virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
        {
            if (inLayer < ObjectLayers::NumLayers)
            {
                return objectToBroadPhase[inLayer];
            }
            return BroadPhaseLayers::NonMoving;
        }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
        {
            switch (static_cast<JPH::BroadPhaseLayer::Type>(inLayer))
            {
            case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::NonMoving): return "NON_MOVING";
            case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::Moving):    return "MOVING";
            case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::Sensor):    return "SENSOR";
            default:                                                                    return "UNKNOWN";
            }
        }
#endif

    private:
        JPH::BroadPhaseLayer objectToBroadPhase[ObjectLayers::NumLayers];
    };

    /**
     * @class ObjectVsBroadPhaseLayerFilterImpl
     * @brief Determines whether an ObjectLayer should test for collision against a BroadPhaseLayer.
     */
    class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
        {
            switch (inLayer1)
            {
            case ObjectLayers::NonMoving:
                // Non-moving only collides with moving bodies
                return inLayer2 == BroadPhaseLayers::Moving;

            case ObjectLayers::Moving:
                // Moving collides with everything
                return true;

            case ObjectLayers::Sensor:
                // Sensors detect moving objects
                return inLayer2 == BroadPhaseLayers::Moving;

            default:
                return false;
            }
        }
    };

    /**
     * @class ObjectLayerPairFilterImpl
     * @brief Determines whether two ObjectLayers can produce contact constraints.
     */
    class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
    {
    public:
        virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override
        {
            switch (inLayer1)
            {
            case ObjectLayers::NonMoving:
                return inLayer2 == ObjectLayers::Moving;

            case ObjectLayers::Moving:
                return inLayer2 == ObjectLayers::NonMoving ||
                    inLayer2 == ObjectLayers::Moving ||
                    inLayer2 == ObjectLayers::Sensor;

            case ObjectLayers::Sensor:
                return inLayer2 == ObjectLayers::Moving;

            default:
                return false;
            }
        }
    };

    JoltContext::JoltContext()
        : isInitialized(false),
        tempAllocator(nullptr),
        jobSystem(nullptr),
        broadPhaseLayerInterface(nullptr),
        objectVsBroadPhaseLayerFilter(nullptr),
        objectLayerPairFilter(nullptr),
        physicsSystem(nullptr)
    {}

    JoltContext::~JoltContext()
    {
        if (isInitialized)
        {
            Shutdown();
        }
    }

    bool JoltContext::Initialize()
    {
        if (isInitialized)
        {
            return true;
        }

        SIM_PROFILE_SCOPE("JoltContext::Initialize");

        // 1. Install callbacks
        JPH::Trace = JoltTraceCallback;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = JoltAssertCallback;)

        // 2. Global allocation & type registrations
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();

        // 3. Allocators and job system
        tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(PhysicsConfig::tempAllocatorBytes);
        jobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);

        // 4. Broadphase & layer filter interfaces
        broadPhaseLayerInterface = std::make_unique<BPLayerInterfaceImpl>();
        objectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
        objectLayerPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

        // 5. PhysicsSystem instance
        physicsSystem = std::make_unique<JPH::PhysicsSystem>();
        physicsSystem->Init(
            PhysicsConfig::maxBodies,
            PhysicsConfig::numBodyMutexes,
            PhysicsConfig::maxBodyPairs,
            PhysicsConfig::maxContactConstraints,
            *broadPhaseLayerInterface,
            *objectVsBroadPhaseLayerFilter,
            *objectLayerPairFilter
        );

        // 6. Physics world configuration
        physicsSystem->SetGravity(JPH::Vec3(0.0f, -9.80665f, 0.0f));

        JPH::PhysicsSettings settings = physicsSystem->GetPhysicsSettings();
        settings.mNumVelocitySteps = PhysicsConfig::defaultVelocitySteps;
        settings.mNumPositionSteps = PhysicsConfig::defaultPositionSteps;
        settings.mBaumgarte = PhysicsConfig::defaultBaumgarte;
        settings.mDeterministicSimulation = true;
        physicsSystem->SetPhysicsSettings(settings);

        isInitialized = true;
        SIM_LOG_INFO("Jolt Physics initialized successfully (200 Hz fixed tick, 0.0 damping).");
        return true;
    }

    void JoltContext::Shutdown()
    {
        if (!isInitialized)
        {
            return;
        }

        SIM_PROFILE_SCOPE("JoltContext::Shutdown");

        physicsSystem.reset();
        objectLayerPairFilter.reset();
        objectVsBroadPhaseLayerFilter.reset();
        broadPhaseLayerInterface.reset();
        jobSystem.reset();
        tempAllocator.reset();

        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;

        isInitialized = false;
        SIM_LOG_INFO("Jolt Physics shut down cleanly.");
    }

    void JoltContext::Step(float deltaTime)
    {
        if (!isInitialized)
        {
            return;
        }

        SIM_PROFILE_SCOPE("JoltContext::Step");
        constexpr int collisionSteps = 1;
        physicsSystem->Update(deltaTime, collisionSteps, tempAllocator.get(), jobSystem.get());
    }

    void JoltContext::OptimizeBroadPhase()
    {
        if (isInitialized && physicsSystem)
        {
            physicsSystem->OptimizeBroadPhase();
        }
    }

    JPH::PhysicsSystem& JoltContext::GetPhysicsSystem()
    {
        return *physicsSystem;
    }

    JPH::BodyInterface& JoltContext::GetBodyInterface()
    {
        return physicsSystem->GetBodyInterface();
    }

    bool JoltContext::IsInitialized() const
    {
        return isInitialized;
    }
}