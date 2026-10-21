# Block 1 Implementation Plan: Foundations & Closeout (Oct 21-22, 2026) - Revision 2

## Goal Description
Complete **Block 1: Closeout** of the *FRC Simulation 10-Week Plan (Revision 4)* over Oct 21 and Oct 22, 2026. Zero-damping and swerve joint stability have already been tested and confirmed by the user. This revised plan focuses strictly on production-grade components:
1. Production Jolt Physics context wrapper configured with the validated settings (zero damping, tuned solver iterations, deterministic single-core job system).
2. ECS physics architecture (`RigidBodyComponent`, `ColliderComponent`, and `PhysicsSystem`) running at 200 Hz with telemetry profiling.
3. Strongly typed dimensional units conversion system.
4. Layered JSON parameter loader (`defaults -> measured -> calibrated`).
5. Root `CMakeLists.txt` with a headless `CTest` suite (`FRCBananaSim_Tests`) alongside full Visual Studio solution parity.
6. Interactive integration demo in `Main.cpp` verifying fixed-step physics telemetry and closing out Block 1.

---

## User Review Required
> [!IMPORTANT]
> **Solver Numbers Confirmation**: Per your testing, Jolt will be configured with default linear damping = `0.0f` and angular damping = `0.0f`. We plan to configure solver iterations to 10 velocity steps and 2 position steps (with `mDeterministicSimulation = true` and `mBaumgarte = 0.2f`). If you preferred a specific velocity/position step count from your tests (e.g. 8/2 or 10/4), let us know so we lock in your exact numbers.
>
> **File Modification Policy**: All code additions, updates, and CMake configurations will be provided inline in chat responses formatted strictly to project standards (4 spaces, Allman braces, lowerCamelCase members, Doxygen comments, `.h` declarations only, `.cpp` implementations, and `SIM_PROFILE_SCOPE` telemetry macros). You will review, copy, and paste all files into the workspace.

---

## Schedule & Milestones (2-Day Breakdown)

```mermaid
flowchart TD
    subgraph Day 1: Oct 21 - Jolt Core & ECS Physics Integration
        D1A["Task 1.1: Jolt Engine Context & Collision Filters"] --> D1B["Task 1.2: ECS Physics Components (RigidBody, Collider)"]
        D1B --> D1C["Task 1.3: PhysicsSystem (200 Hz FixedUpdate, Jolt <-> ECS Sync)"]
        D1C --> D1D["Task 1.4: Collider Debug Visualization (DebugDraw Hook)"]
        D1D --> D1E["Checkpoint 1: Interactive Physics Scene in Main.cpp"]
    end

    subgraph Day 2: Oct 22 - Units, Layered JSON Parameters & Headless CTests
        D2A["Task 2.1: Units Conversion Engine (Metric, Imperial, Rotational, Electrical)"] --> D2B["Task 2.2: Layered JSON Parameter Store (nlohmann/json)"]
        D2B --> D2C["Task 2.3: Root CMakeLists.txt & Headless Test Executable"]
        D2C --> D2D["Task 2.4: Headless CTest Verification (Units, Parameters, Determinism)"]
        D2D --> D2E["Checkpoint 2: Block 1 Closeout Sign-Off -> Handoff to Block 2 (Oct 23)"]
    end

    Day 1 --> Day 2
```

---

## Detailed Technical Design

### Day 1 (Oct 21): Jolt Physics Core & ECS Integration

#### 1. Jolt Engine Context (`Source/Physics/`)
Encapsulates Jolt Physics initialization, memory allocation, collision layer filtering, and stepping.

- **`Source/Physics/PhysicsConfig.h`**:
  - `constexpr double fixedTimeStep = 1.0 / 200.0;` (200 Hz = 5 ms).
  - `constexpr float defaultLinearDamping = 0.0f;`
  - `constexpr float defaultAngularDamping = 0.0f;`
  - `constexpr uint32_t defaultVelocitySteps = 10;`
  - `constexpr uint32_t defaultPositionSteps = 2;`
  - `constexpr float defaultBaumgarte = 0.2f;`
  - Collision object layers:
    - `ObjectLayers::NON_MOVING = 0` (static field geometry, obstacles)
    - `ObjectLayers::MOVING = 1` (robot chassis, links, game pieces)
    - `ObjectLayers::SENSOR = 2` (beam breaks, intake zones)
  - BroadPhase layers: `BroadPhaseLayers::NON_MOVING`, `MOVING`, `SENSOR`.

- **`Source/Physics/JoltContext.h` & `Source/Physics/JoltContext.cpp`**:
  - Initializes `JPH::RegisterTypes()`, `JPH::Factory`.
  - Instantiates `JPH::TempAllocatorImpl` (16 MB scratch pool).
  - Instantiates `JPH::JobSystemSingleThreaded` (zero thread synchronization overhead, deterministic execution per sim core).
  - Implements `BPLayerInterfaceImpl`, `ObjectVsBroadPhaseLayerFilterImpl`, and `ObjectLayerPairFilterImpl`.
  - Configures `JPH::PhysicsSystem` with `mDeterministicSimulation = true`.
  - Provides clean lifecycle: `Initialize()`, `Shutdown()`, `Step(float dt)`, `GetPhysicsSystem()`, `GetBodyInterface()`.

#### 2. ECS Physics Components (`Source/Physics/ECS/Components/`)
- **`RigidBodyComponent.h`**:
  - `JPH::BodyID bodyID`: Jolt body handle.
  - `MotionType motionType`: `Static`, `Kinematic`, `Dynamic`.
  - `float mass`: Mass in kg.
  - `float friction`: Surface friction coefficient (default: 0.7).
  - `float restitution`: Restitution coefficient (default: 0.1).
  - `float linearDamping`: Initialized to `PhysicsConfig::defaultLinearDamping` (0.0).
  - `float angularDamping`: Initialized to `PhysicsConfig::defaultAngularDamping` (0.0).
  - `glm::vec3 linearVelocity`, `glm::vec3 angularVelocity`.
  - `bool isSensor`: Trigger zone without contact response.

- **`ColliderComponent.h`**:
  - `ColliderType type`: `Box`, `Sphere`, `Cylinder`, `Capsule`.
  - `glm::vec3 dimensions`: Extents / radius / height in meters.
  - `glm::vec3 offset`: Local position offset relative to entity.
  - `glm::quat rotationOffset`: Local orientation offset.

#### 3. ECS Physics System (`Source/Physics/ECS/Systems/`)
- **`PhysicsSystem.h` & `Source/Physics/ECS/Systems/PhysicsSystem.cpp`**:
  - Inherits from `ISystem`.
  - Constructor takes `Scene&` and `JoltContext&`.
  - `FixedUpdate(double dt)`:
    - Profile scope: `SIM_PROFILE_SCOPE("PhysicsSystem::FixedUpdate")`.
    - 1. Push any kinematic entity movements (`TransformComponent`) to Jolt bodies.
    - 2. Advance Jolt simulation: `joltContext.Step(static_cast<float>(dt))`.
    - 3. Pull rigid body transforms from Jolt `BodyInterface` and update `TransformComponent` and `WorldTransformComponent`.
    - 4. Update linear and angular velocities in `RigidBodyComponent`.
  - Body creation / deletion helpers: automatically create Jolt body when `RigidBodyComponent` is attached; remove Jolt body when component or entity is destroyed.

#### 4. Collider Debug Visualization
- **`PhysicsDebugDrawSystem.h` & `Source/Physics/ECS/Systems/PhysicsDebugDrawSystem.cpp`**:
  - In `RenderUpdate()`:
    - Profile scope: `SIM_PROFILE_SCOPE("PhysicsDebugDrawSystem::RenderUpdate")`.
    - Iterate entities with `ColliderComponent` and `WorldTransformComponent`.
    - Draw wireframe shapes via `DebugDraw::Box()` and `DebugDraw::Sphere()`.

---

### Day 2 (Oct 22): Units, Layered JSON Parameter Store & Headless CTests

#### 5. Dimensional Units Engine (`Source/Core/Units/`)
Guarantees physical safety and explicit unit conversion across the codebase.

- **`Source/Core/Units/Units.h` & `Source/Core/Units/Units.cpp`**:
  - Conversions to SI base units:
    - **Length**: `Meters`, `Centimeters`, `Millimeters`, `Inches`, `Feet`.
    - **Mass**: `Kilograms`, `Grams`, `Pounds`, `Ounces`.
    - **Angle**: `Radians`, `Degrees`, `Rotations`.
    - **Linear Velocity**: `MetersPerSecond`, `FeetPerSecond`, `InchesPerSecond`.
    - **Angular Velocity**: `RadiansPerSecond`, `DegreesPerSecond`, `RPM`.
    - **Force & Torque**: `Newtons`, `PoundForce`, `NewtonMeters`, `FootPounds`.
    - **Electrical**: `Volts`, `Amperes`, `Ohms`, `Watts`.
  - String parsing helper: `Units::Parse(double value, const std::string& unitString)`.
  - Validation: Throws or logs detailed warning if unit string is unrecognized.

#### 6. Layered JSON Parameter Store (`Source/Core/Parameters/`)
Configuration store supporting unit-aware values and layered inheritance for calibration.

- **`External/nlohmann/json.hpp`**: Single-header JSON parsing library.
- **`Source/Core/Parameters/ParameterValue.h`**:
  - Holds `double rawValue`, `double siValue`, `std::string unit`, `std::string description`.
- **`Source/Core/Parameters/ParameterStore.h` & `Source/Core/Parameters/ParameterStore.cpp`**:
  - `LoadFromFile(const std::string& filepath)`: Parses hierarchical JSON files formatted as:
    ```json
    {
      "chassis": {
        "mass": { "value": 52.0, "unit": "kg" },
        "width": { "value": 28.0, "unit": "in" },
        "length": { "value": 28.0, "unit": "in" },
        "wheelDiameter": { "value": 4.0, "unit": "in" },
        "gearRatio": { "value": 6.75, "unit": "ratio" },
        "nominalVoltage": { "value": 12.0, "unit": "V" }
      }
    }
    ```
  - `Merge(const ParameterStore& overlay)`: Overlays parameters in place, supporting the `defaults -> measured -> calibrated` pipeline.
  - `GetDouble(const std::string& path, double fallback)`
  - `GetSI(const std::string& path, double fallback)`: Returns value converted to SI base units.
  - `GetString(const std::string& path, const std::string& fallback)`
  - `GetBool(const std::string& path, bool fallback)`

#### 7. CMake Build Skeleton & Headless CTest Target
- **`CMakeLists.txt`** (Root):
  - CMake 3.24+ project definition (`FRCBananaSim`).
  - Sets C++20 standard, Allman style warnings, MSVC flags (`/W3`, `/permissive-`).
  - Targets:
    - `FRCBananaSim`: Main visual simulation executable (linking GLFW, GLAD, ImGui, Jolt).
    - `FRCBananaSim_Tests`: Headless console test executable (linking Jolt, ECS, Units, Parameters).
  - CTest integration (`enable_testing()`, `add_test(...)`).
- **Visual Studio Parity**:
  - `FRCBananaSim.vcxproj` and `FRCBananaSim.slnx` updated with new files and include paths.

---

## Verification Plan

### Automated Headless Tests (`FRCBananaSim_Tests` via CTest)
Execute via: `ctest --output-on-failure` or direct CLI execution.

1. **Units Conversion Test Suite (`UnitsTest`)**:
   - Verify metric $\leftrightarrow$ imperial conversions (e.g. 1.0 inch $= 0.0254$ m, 1.0 lb $\approx 0.453592$ kg).
   - Verify rotational velocity conversions (e.g. 6000 RPM $= 200\pi$ rad/s).
   - Verify torque and force conversions (e.g. 1.0 ft·lbf $\approx 1.355818$ N·m).
   - Verify error handling on unknown unit strings.

2. **Parameter Store Test Suite (`ParameterStoreTest`)**:
   - Ingest sample JSON configuration fixture with explicit units.
   - Verify retrieval of raw values and automatic SI conversion.
   - Verify layered merging (`defaults.json` merged with `measured.json` overrides matching keys while preserving defaults).

3. **Physics Determinism Test Suite (`PhysicsDeterminismTest`)**:
   - Run 1,000 steps ($5.0$ s simulated time at 200 Hz) of multiple falling and bouncing rigid bodies.
   - Repeat simulation from the identical initial conditions.
   - Assert exact floating-point / bitwise match of body poses and velocities across runs.

### Manual & Interactive Verification
1. **Interactive Demo in `Main.cpp`**:
   - Run `FRCBananaSim.exe`.
   - Verify ground plane entity, dynamic falling bodies (boxes/spheres), and collider wireframes rendered via `DebugDraw`.
   - Cycle HUD (`F3`): Inspect fixed-step physics cost in the Telemetry overlay to verify the 200 Hz update executes in $\le 70\,\mu\text{s}$ per step.
2. **Dual Build Verification**:
   - Verify compilation succeeds cleanly via MSBuild (`msbuild FRCBananaSim.slnx /p:Configuration=Release /p:Platform=x64`).
   - Verify compilation succeeds cleanly via CMake (`cmake -B build -G "Visual Studio 18 2026" -A x64` and `cmake --build build --config Release`).

---

## Deliverables Checklist for Block 1 Completion
- [ ] Jolt core wrapper initialized with zero damping and chosen solver iterations.
- [ ] ECS `RigidBodyComponent`, `ColliderComponent`, and `PhysicsSystem` integrated and profiling.
- [ ] Units conversion engine fully implemented and tested.
- [ ] Layered JSON ParameterStore implemented and tested.
- [ ] Root `CMakeLists.txt` created with headless `FRCBananaSim_Tests` passing all CTests.
- [ ] Visual Studio solution building with zero errors and warnings.
- [ ] Ready for Block 2 kickoff on Oct 23.
