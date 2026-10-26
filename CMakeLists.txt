cmake_minimum_required(VERSION 3.24)
project(FRCBananaSim VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Link against static CRT (/MT in Release, /MTd in Debug) to match Jolt.lib
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Jolt Physics ABI configuration flags
add_compile_definitions(
    JPH_PROFILE_ENABLED
    JPH_DEBUG_RENDERER
    JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
    JPH_OBJECT_STREAM
)

if (MSVC)
    add_compile_options(/W3 /permissive- /utf-8)
endif()

# Global include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/FRCBananaSim/Source
    ${CMAKE_CURRENT_SOURCE_DIR}/External
    ${CMAKE_CURRENT_SOURCE_DIR}/External/nlohmann
    ${CMAKE_CURRENT_SOURCE_DIR}/External/glm
    ${CMAKE_CURRENT_SOURCE_DIR}/External/glfw/include
    ${CMAKE_CURRENT_SOURCE_DIR}/External/glad/include
    ${CMAKE_CURRENT_SOURCE_DIR}/External/imgui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/External/imgui/include/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/External/jolt/include
)

# Common core simulation sources
set(ENGINE_SOURCES
    FRCBananaSim/Source/Actuation/DCMotorModel.cpp
    FRCBananaSim/Source/Actuation/BatteryModel.cpp
    FRCBananaSim/Source/Actuation/ServoModel.cpp
    FRCBananaSim/Source/Drivetrain/Systems/SwerveDriveSystem.cpp
    FRCBananaSim/Source/Core/Engine.cpp
    FRCBananaSim/Source/Core/ECS/Core/IComponentStorage.cpp
    FRCBananaSim/Source/Core/ECS/Core/Scene.cpp
    FRCBananaSim/Source/Core/ECS/Core/System.cpp
    FRCBananaSim/Source/Core/ECS/Systems/TransformPropagationSystem.cpp
    FRCBananaSim/Source/Core/Units/Units.cpp
    FRCBananaSim/Source/Core/Parameters/ParameterStore.cpp
    FRCBananaSim/Source/Physics/JoltContext.cpp
    FRCBananaSim/Source/Physics/Systems/PhysicsSystem.cpp
    FRCBananaSim/Source/Physics/Systems/PhysicsDebugDrawSystem.cpp
    FRCBananaSim/Source/Physics/Systems/JointSystem.cpp
    FRCBananaSim/Source/Renderer/Camera.cpp
    FRCBananaSim/Source/Renderer/Input.cpp
    FRCBananaSim/Source/Renderer/Renderer.cpp
    FRCBananaSim/Source/Renderer/Systems/RenderSystem.cpp
    FRCBananaSim/Source/Telemetry/MemoryTracker.cpp
    FRCBananaSim/Source/Telemetry/ProfilerUI.cpp
    FRCBananaSim/Source/Telemetry/ProfilerWindow.cpp
    FRCBananaSim/Source/Telemetry/SessionLoader.cpp
    FRCBananaSim/Source/Telemetry/SessionManager.cpp
    FRCBananaSim/Source/Telemetry/TelemetryManager.cpp
    FRCBananaSim/Source/Telemetry/TelemetryOverlay.cpp
    FRCBananaSim/Source/Telemetry/TelemetryTimer.cpp
    FRCBananaSim/Source/Utils/DebugDraw.cpp
    FRCBananaSim/Source/Utils/FixedTimestep.cpp
    FRCBananaSim/Source/Utils/Log.cpp
    External/glad/src/glad.c
    External/imgui/include/imgui/imgui.cpp
    External/imgui/include/imgui/imgui_demo.cpp
    External/imgui/include/imgui/imgui_draw.cpp
    External/imgui/include/imgui/imgui_tables.cpp
    External/imgui/include/imgui/imgui_widgets.cpp
    External/imgui/include/imgui/backend/imgui_impl_glfw.cpp
    External/imgui/include/imgui/backend/imgui_impl_opengl3.cpp
)

# Target 1: Interactive Simulation Executable
add_executable(FRCBananaSim
    ${ENGINE_SOURCES}
    FRCBananaSim/Source/Main.cpp
)

target_link_directories(FRCBananaSim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/External/glfw/bin/$<CONFIG>
    ${CMAKE_CURRENT_SOURCE_DIR}/External/jolt/bin/$<CONFIG>
)

target_link_libraries(FRCBananaSim PRIVATE
    glfw3
    Jolt
    opengl32
    gdi32
)

# Target 2: Headless CTest Executable
enable_testing()

set(TEST_SOURCES
    FRCBananaSim/Source/Actuation/DCMotorModel.cpp
    FRCBananaSim/Source/Actuation/BatteryModel.cpp
    FRCBananaSim/Source/Actuation/ServoModel.cpp
    FRCBananaSim/Source/Drivetrain/Systems/SwerveDriveSystem.cpp
    FRCBananaSim/Source/Core/ECS/Core/IComponentStorage.cpp
    FRCBananaSim/Source/Core/ECS/Core/Scene.cpp
    FRCBananaSim/Source/Core/ECS/Core/System.cpp
    FRCBananaSim/Source/Core/ECS/Systems/TransformPropagationSystem.cpp
    FRCBananaSim/Source/Core/Units/Units.cpp
    FRCBananaSim/Source/Core/Parameters/ParameterStore.cpp
    FRCBananaSim/Source/Physics/JoltContext.cpp
    FRCBananaSim/Source/Physics/Systems/PhysicsSystem.cpp
    FRCBananaSim/Source/Physics/Systems/JointSystem.cpp
    FRCBananaSim/Source/Telemetry/MemoryTracker.cpp
    FRCBananaSim/Source/Telemetry/TelemetryManager.cpp
    FRCBananaSim/Source/Telemetry/TelemetryTimer.cpp
    FRCBananaSim/Source/Utils/DebugDraw.cpp
    FRCBananaSim/Source/Utils/FixedTimestep.cpp
    FRCBananaSim/Source/Utils/Log.cpp
    Tests/MainTests.cpp
    Tests/UnitsTest.cpp
    Tests/ParameterStoreTest.cpp
    Tests/PhysicsDeterminismTest.cpp
    Tests/MotorBatteryTest.cpp
    Tests/JointStabilityTest.cpp
    Tests/SwervePhysicalDriveTest.cpp
)

add_executable(FRCBananaSim_Tests ${TEST_SOURCES})

target_include_directories(FRCBananaSim_Tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Tests
)

target_link_directories(FRCBananaSim_Tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/External/jolt/bin/$<CONFIG>
)

target_link_libraries(FRCBananaSim_Tests PRIVATE
    Jolt
)

add_test(NAME UnitsTest COMMAND FRCBananaSim_Tests --units)
add_test(NAME ParameterStoreTest COMMAND FRCBananaSim_Tests --params)
add_test(NAME PhysicsDeterminismTest COMMAND FRCBananaSim_Tests --determinism)
add_test(NAME MotorBatteryTest COMMAND FRCBananaSim_Tests --motor-battery)
add_test(NAME JointStabilityTest COMMAND FRCBananaSim_Tests --joint-stability)
add_test(NAME SwervePhysicalDriveTest COMMAND FRCBananaSim_Tests --swerve-drive)
add_test(NAME AllTests COMMAND FRCBananaSim_Tests)