# FRC Simulation: 10-Week Plan, Revision 5 (Oct 21 - Dec 27, 2026)

## What changed from v4

| Change | Detail |
|---|---|
| **Real C++ Robot Code Bridge (HALSim WS + NT4)** | Added an embedded WPILib HALSim WebSocket server (`halsim_ws`) and NetworkTables 4 (`ntcore`) Limelight server so your unmodified C++ robot project (`./gradlew simulateNative`) drives the sim directly. No coding the robot twice. |
| **`SimDevice` vendor dispatch** | Maps CTRE Phoenix 6 (`TalonFXSimState`) and REVLib (`SparkMaxSim`) dynamic `SimDevice` fields over WebSockets to the motor and mechanism models defined in `sidecar.json`. |
| **Fleet mode runs C++ autonomous routines (Option A)** | Fleet mode spawns parallel headless robot processes on dedicated localhost ports (`3300..3300+N`) using a small `SimHooks::PauseTiming()` + `StepTiming(20_ms)` lockstep hook in `SimulationInit()`. |
| **Calibrator bypasses HALSim** | Offline parameter fitting and physics batch sweeps still inject `SimInputs` directly in-process with zero socket overhead. |
| **Schedule accounting updated** | The bridge adds ~3.5 days of real engineering across Blocks 2, 3, 8, and 10, absorbed by trimming the ImGui tuning panel to raw ImGui widgets and tightening the block scopes. |

> [!WARNING]
> **Option A Fleet throughput caveat:** Running 15–20 separate WPILib OS processes plus 7 Jolt worker threads across 8 CPU cores over localhost JSON WebSockets will likely bottleneck on Windows context switches and JSON parsing before Jolt is even stressed. In Block 8 we will benchmark 4, 8, 12, and 16–20 parallel auto processes and set the Fleet worker count to whichever gives the highest completed autonomous routines per second.

## Day accounting

| | Days |
|---|---|
| v4 baseline (Oct 21 - Dec 28) | 69 |
| **Added in v5** | |
| HALSim WebSocket server v0 + PWM/Encoder/Gyro bridge (Block 2) | +1 |
| Vendor `SimDevice` parser (TalonFX / SparkMax) + lockstep hook (Block 3) | +1 |
| Multi-port Fleet process spawner + Mock DriverStation sequencer (Block 8) | +1 |
| Embedded `ntcore` NT4 Limelight publisher (Block 10) | +0.5 |
| **Freed in v5** | |
| No standalone sim teleop/gamepad controller needed (real C++ code drives Block 2) | -1 |
| Control API boundary moved out of Block 12 into Block 2 | -1 |
| ImGui tuning panel (Block 12) reduced to bare `ImGui::DragFloat` / `ImPlot` | -1 |
| **v5 total (Oct 21 - Dec 28)** | **69.5** |

- Available days from Oct 21 through Dec 27 are 68, so the plan runs **about 1.5 days over** into the Dec 28 - Jan 8 pre-kickoff buffer.
- **There is no slack.** If a block slips, follow the cut order in Section 4 immediately rather than pushing downstream blocks past the Dec 27 AI cutoff.

## 1. Performance budgets

| Mode | Sims | Robots per sim | Control source | Rate / speed | Step budget |
|---|---|---|---|---|---|
| Fleet (Calibrator / Sweeps) | 15-20 across 7 cores | 1 | Direct `SimInputs` (no socket) | 200 Hz / 10x+ | ~175-230 us |
| Fleet (Auto Routines, Option A) | 6-20 across 7 cores (measured in Block 8) | 1 | Headless C++ robot processes (`halsim_ws` lockstep) | 200 Hz physics, 50 Hz robot / target 10x | ~175-230 us physics + IPC |
| Match | 1 | 6 (1 C++ robot + 5 scripted/bots, or up to 6 sockets) | `halsim_ws` + NT4 | 200 Hz / 1x live or 10x rollout | ~5,000 us (1x), ~500 us (10x) |
| Camera | 1-2 offscreen streams | 1 or 6 | Publishes to NT4 `/limelight/*` | 60-120 FPS | under 2 ms GPU |

Release bake-off numbers for Jolt: 51 us for the 100-body scene, 42 us for the swerve scene, 16 us for the four-bar scene. Full-robot Jolt cost and localhost WebSocket round-trip latency are not yet measured.

## 2. Architecture & Data Flow

The architecture is divided into 6 layers. Jolt owns rigid bodies, world geometry, and joint constraints; math models own motors, battery sag, intake pull, shooter squeeze, and flight aerodynamics.

Your **C++ Robot Code** (WPILib GradleRIO) runs unmodified as a native desktop process (`./gradlew simulateNative`) and talks to `FRCBananaSim` over **HALSim WebSockets** and **NetworkTables 4 (NT4)**.

```
┌────────────────────────────────────────────────────────────────────────────┐
│                  1. CAD & Parameter Ingestion (Load-Time)                  │
│                                                                            │
│  ┌──────────────────────┐   ┌───────────────────┐   ┌───────────────────┐  │
│  │  Fusion 360 Add-in   │   │  Sidecar Config   │   │  Parameter Store  │  │
│  │ (Python Mesh/Inertia)│   │ (Motors/Gears/IDs)│   │ (Defaults/Priors) │  │
│  └──────────┬───────────┘   └─────────┬─────────┘   └─────────┬─────────┘  │
│             │   manifest.json         │                       │            │
│             └───────────────►┌────────▼───────────────────────▼─────────┐  │
│                              │  C++ Importer & Mass Reconciliation Tool │  │
│                              └────────────────────┬─────────────────────┘  │
└───────────────────────────────────────────────────┼────────────────────────┘
                                                    │ Spawns Entities / Bodies
┌───────────────────────────────────────────────────▼────────────────────────┐
│           2. User C++ Robot Code & Hardware Interface Layer                │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │   User WPILib C++ Robot Process(es) (./gradlew simulateNative)       │  │
│  │   - 1x Live Mode (Port 3300) OR N Parallel Auto Workers (3300..3300+N│  │
│  │   - Swerve Kinematics, PathPlanner Autos, State Machines, PID Loops  │  │
│  │   - Lockstep Hook: SimHooks::PauseTiming() + StepTiming(20_ms)       │  │
│  └──────────────────┬───────────────────────────────▲───────────────────┘  │
│     halsim_ws JSON  │                               │ NT4 NetworkTables    │
│     (Lockstep/200Hz)│                               │ (/limelight/* topics)│
│                     ▼                               │                      │
│  ┌──────────────────────────────────┐   ┌───────────┴───────────────────┐  │
│  │  HALSim WebSocket Server Bridge  │   │ NetworkTables 4 Server (NT4)  │  │
│  │  - SimDevice: TalonFX & SparkMax │   │ - Limelight: tv, tx, ty, tid  │  │
│  │  - PWM, DIO, Encoders, Gyro/IMU  │   │ - botpose_wpiblue (MegaTag2)  │  │
│  │  - Mock DriverStation & Step Sync│   │ - Match Timing & Auto Chooser │  │
│  └──────────────────┬───────────────┘   └───────────▲───────────────────┘  │
│                     │ Translates to Structs         │                      │
│                     ▼                               │ Camera Poses & Tags  │
│  ┌──────────────────────────────────┐               │                      │
│  │ SimInputs (Volts, PWM, Solenoid) │               │                      │
│  │ SimOutputs (Encoders, IMU, DIO)  │               │                      │
│  └──────────────────┬───────────────┘               │                      │
└─────────────────────┼───────────────────────────────┼──────────────────────┘
                      │ Joint Demands                 │
┌─────────────────────▼───────────────────────────────┴──────────────────────┐
│           3. Robot Simulation Core & Hybrid Interaction Layer              │
│                                                                            │
│  SimInputs ──► [Implicit DC Motors & Lumped Battery Voltage Sag]           │
│                                           │                                │
│       ┌───────────────────────────────────┴────────────────────────┐       │
│       │ Dynamic Joint Torques & Mechanism Drives                   │       │
│       ▼                                                            ▼       │
│  ┌──────────────────────────────────┐   ┌───────────────────────────────┐  │
│  │ Swerve Kinematics & Chassis      │   │ Articulations: Elevators,     │  │
│  │ (4-Module Vector Squeeze / Slip) │   │ 4-Bars, Arms, Cascades, Stops │  │
│  └──────────────────┬───────────────┘   └───────────────┬───────────────┘  │
│                     │                                   │                  │
│  ┌──────────────────▼───────────────────────────────────▼───────────────┐  │
│  │ Hybrid Game-Piece Interaction Layer (Math Overrides)                 │  │
│  │ - State Machine: FREE ──► INTAKING ──► STORED ──► ENGAGED ──► LAUNCH │  │
│  │ - Intake Pull: Surface velocity drag & compliance capture            │  │
│  │ - Shooter Squeeze: Nonlinear spring compression & slip friction math │  │
│  │ - Flight Aero: Drag coefficient & Magnus effect spin-lift            │  │
│  └──────────────────────────────────┬───────────────────────────────────┘  │
└─────────────────────────────────────┼──────────────────────────────────────┘
                                      │ Injected Forces / Recoil / Constraints
┌─────────────────────────────────────▼──────────────────────────────────────┐
│           4. Engine Foundation & Jolt Physics Solver (200 Hz)              │
│                                                                            │
│  ┌───────────────────────────────┐     ┌────────────────────────────────┐  │
│  │      RT-PhysicsCore ECS       │◄───►│      Jolt Physics Backend      │  │
│  │ (Sparse Sets, World Hierarchy,│     │ (Rigid Bodies, Constraints,    │  │
│  │  Transform/RigidBody Systems) │     │  Collision Islands, CCD Steps) │  │
│  └───────────────────────────────┘     └────────────────┬───────────────┘  │
│                                                         │ Contact Events   │
│  SimOutputs (Encoders, IMU, Beam Breaks) ◄──────────────┘                  │
└─────────────────────────────────────┬──────────────────────────────────────┘
                                      │ Poses, Transforms, Contacts
        ┌─────────────────────────────┴─────────────────────────────┐
        │                                                           │
┌───────▼───────────────────────────┐   ┌───────────────────────────▼────────┐
│       5. Execution Runners        │   │    6. GPU Limelight Pipeline       │
│                                   │   │                                    │
│  ┌─────────────────────────────┐  │   │  ┌──────────────────────────────┐  │
│  │ Headless Fleet Runner       │  │   │  │ Offscreen FBOs (960p/480p)   │  │
│  │ - Option A: Multi-Port Auto │  │   │  └──────────────┬───────────────┘  │
│  │   Workers (Ports 3300..+N)  │  │   │                 ▼                  │
│  │ - Direct SimInputs for Calib│  │   │  ┌──────────────────────────────┐  │
│  └──────────────┬──────────────┘  │   │  │ OpenGL PBR Venue Renderer    │  │
│                 │ Swarm Sweeps    │   │  │ (Cook-Torrance, Arena Lights)│  │
│                 ▼                 │   │  └──────────────┬───────────────┘  │
│  ┌─────────────────────────────┐  │   │                 ▼                  │
│  │ Offline Calibrator          │  │   │  ┌──────────────────────────────┐  │
│  │ - Levenberg-Marquardt Fits  │  │   │  ┌──────────────────────────────┐  │
│  │ - Direct CSV Log Replay     │  │   │  │ Optics Post-Process Shaders  │  │
│  └─────────────────────────────┘  │   │  │ (Distortion, Motion Blur)    │  │
│                                   │   │  └──────────────┬───────────────┘  │
│  ┌─────────────────────────────┐  │   │                 ▼                  │
│  │ Headless / GUI Match Runner │  │   │  ┌──────────────────────────────┐  │
│  │ - 1x Match (6 Robots + FMS) │  │   │  │ Async PBO Readback & Latency │  │
│  │ - Bumper-to-Bumper Impact   │  │   │  │ AprilTag solvePnP ──► [NT4]  │  │
│  └─────────────────────────────┘  │   │  └──────────────────────────────┘  │
└───────────────────────────────────┘   └────────────────────────────────────┘
```

### Layer Breakdown

1. **CAD & Configuration Ingestion (Load-Time)**
   - **Fusion 360 Python Exporter**: Traverses assembly hierarchy, clusters rigid joints into kinematic links, queries `PhysicalProperties` (High accuracy) for link mass, center of mass, and body-frame inertia tensors ($I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}$), extracts joint types and motion limits, and tessellates collision/visual meshes.
   - **Sidecar Config (`sidecar.json`)**: Ingests metadata not encoded in CAD: motor presets, gear ratios, current limits, CAN device IDs (`canId`, `canBus`), collider proxies, and mechanism roles.
   - **Layered Parameter Store (`materials.json` / `calibrated.json`)**: Cascading parameter overrides (`defaults -> measured -> calibrated`) with explicit physical units and confidence intervals for friction ($\mu$), restitution ($e$), foam stiffness ($k, n$), damping ($c$), and carpet grip.
   - **C++ Importer & Mass Reconciliation**: Assembles ECS entities and Jolt constraints, cross-checks total/subsystem mass against real scale readings, and distributes unmodeled mass offsets.

2. **User C++ Robot Code & Hardware Interface Layer (Zero Code Duplication)**
   - **Unmodified Robot Code Execution**: Your C++ robot project enables `wpi.sim.addWebsocketsClient()` in `build.gradle` and compiles natively via `./gradlew simulateNative`.
   - **WPILib HALSim WebSocket Server**: `FRCBananaSim` hosts a WebSocket server (`ws://127.0.0.1:3300/wpilibws` in 1x mode, or ports `3300..3300+N` in Fleet mode). It parses incoming motor commands into `SimInputs` and sends `SimOutputs` (encoders, gyro yaw/pitch/roll, DIO beam breaks, battery voltage) back into the HAL.
   - **Dynamic `SimDevice` Dispatching**: CTRE Phoenix 6 (`TalonFXSimState`) and REVLib (`SparkMaxSim`) expose their simulation state as `SimDevice` entries over WebSockets. The bridge maps those device strings and CAN IDs to the corresponding motors in `sidecar.json`.
   - **Lockstep Time Stepping (Option A Fleet Mode)**: A lightweight hook in your robot project's `SimulationInit()` calls `frc::sim::PauseTiming()` when launched in Fleet mode. Each time `FRCBananaSim` finishes 4 physics substeps (20 ms), it sends a step sync message over the WebSocket; the robot process runs `frc::sim::StepTiming(20_ms)` inline and replies with the new motor voltages.
   - **NetworkTables 4 (NT4) Limelight Server**: `FRCBananaSim` embeds `ntcore` and publishes `/limelight/tv`, `tx`, `ty`, `ta`, `tid`, `/limelight/botpose_wpiblue`, and pipeline latency `cl`. Your C++ vision subsystem reads NT4 exactly as it does on the real robot.
   - **Mock DriverStation Sequencer**: Sends `DriverStation` state packets (`enabled`, `autonomous`, `teleop`, match time, alliance color, joystick axes) over the WebSocket so autonomous and teleop modes run without launching the official FRC Driver Station app.

3. **Robot Simulation Core & Hybrid Interaction Layer**
   - **Actuation & Electrics**: Implicit DC motor solver coupled with gearbox efficiency and reflected rotor inertia ($I_{\text{reflected}} = I_{\text{rotor}} \cdot G^2$). Lumped battery circuit calculates instantaneous bus voltage sag ($V_{\text{bus}} = V_{\text{oc}} - I_{\text{total}} R_{\text{int}}$) and triggers brownout thresholds under hard drivetrain acceleration.
   - **Mechanism Kinematics**: Solves forward and inverse swerve kinematics, elevator stage rigging ratios, arm gravity counter-torques, and four-bar closed loops.
   - **Hybrid Interaction Layer (Math-Driven Piece Contact)**:
     - **Game Piece State Machine**: Tracks game piece lifecycle (`FREE -> INTAKING -> STORED -> ENGAGED -> LAUNCHED -> FREE`).
     - **Intake Model**: Computes gripping force from roller tangential velocity, compression, and surface friction, pulling pieces into the hopper while respecting motor torque limits.
     - **Shooter Squeeze Model**: 1D nonlinear compression spring-damper ($F_n = k \delta^n + c \dot{\delta}$) and Coulomb slip friction ($F_t = \min(\mu F_n, F_{\text{stick}})$). Applies equal-and-opposite reaction torque to the flywheel (producing real RPM sag) and imparts exit velocity and spin to the piece.
     - **Flight Aerodynamics**: Injects quadratic air drag ($\frac{1}{2} \rho v^2 C_d A$) and Magnus effect rotational lift forces ($\mathbf{F}_M \propto \boldsymbol{\omega} \times \mathbf{v}$) while in free flight.

4. **Engine Foundation & Jolt Physics**
   - **`RT-PhysicsCore` ECS**: Manages entities, packed sparse-set component storage (`ComponentStorage`), and world transform hierarchies (`TransformComponent`).
   - **Jolt Physics Backend**: Single native physics engine driving 3D rigid bodies, colliders (Box, Sphere, Capsule, Cylinder, Convex Hulls), joints (Hinge, Slider, Fixed, 6-DOF), and the contact solver.
   - **Solver Scope**: Jolt handles chassis dynamics, joint constraints, wheel-to-carpet traction, robot-to-field boundaries, and bumper-to-bumper collisions.

5. **Execution Runners**
   - **Headless Fleet Runner**:
     - **Auto Routine Mode (Option A)**: Spawns parallel headless C++ robot processes on ports `3300..3300+N` (`HALSIMWS_PORT` env var) and steps them in lockstep with Jolt worlds to test and tune autonomous trajectories in parallel.
     - **Calibrator / Sweep Mode**: Bypasses sockets and feeds recorded voltage logs or parameter sweep setpoints directly into `SimInputs` at maximum CPU speed.
     - Features binary state snapshot/restore for sub-millisecond `reset(seed)`.
   - **Match Runner**: Single-instance runner simulating a full 6-robot match with active game pieces in real time (1x driver practice) or accelerated rollout.
   - **Offline Calibrator**: Replays real robot / FTC telemetry CSV logs, runs parallel parameter sweeps across the Fleet Runner, and optimizes model parameters using Levenberg-Marquardt with Bayesian priors.

6. **Limelight & Visual Pipeline (GPU)**
   - **Offscreen Camera FBOs**: Headless multi-camera rendering directly to GPU framebuffers matching hardware camera resolutions (e.g. 1280x960, 640x480).
   - **OpenGL PBR Renderer**: Renders field, robot, and game-piece glTF meshes with Cook-Torrance BRDF, realistic polycarbonate specular reflections, and arena spotlighting.
   - **Limelight Optics Shader**: Custom post-process pass simulating exact camera intrinsics ($f_x, f_y, c_x, c_y$), Brown-Conrady lens distortion ($k_1, k_2, k_3, p_1, p_2$), velocity-buffer motion blur (shutter angle/exposure), and sensor noise.
   - **Async Readback & Latency Buffer**: Double-buffered `GL_PIXEL_PACK_BUFFER` reads frames asynchronously without stalling physics worker threads, feeds a latency queue (20–45 ms delay) into AprilTag `solvePnP` or neural object detectors, and publishes results to the embedded **NT4 Server**.
   - **Interactive Dev Viewer**: Minimal ImGui overlay for live parameter tweaking, diagnostic camera streams, and real-time RPM sag / trajectory plotting.

## 3. Schedule

| # | Dates | Block |
|---|---|---|
| 1 | Oct 21-22 | Closeout |
| 2 | Oct 23-29 | Motors, battery, swerve & WPILib HALSim WS Bridge v0 |
| 3 | Oct 30-Nov 5 | **Mechanisms, schema, sensors & `SimDevice` Dispatch (restored v2 Week 4)** |
| 4 | Nov 6-10 | Fusion exporter |
| 5 | Nov 11-15 | Importer and mass reconciliation |
| 6 | Nov 16-22 | Interaction layer (shooter and intake math) |
| 7 | Nov 23-27 | Parameter store and calibrator (Thanksgiving, full capacity) |
| 8 | Nov 28-Dec 4 | Field, Match mode, Option A Fleet Runner & Mock DriverStation |
| 9 | Dec 5-11 | **Rendering 1: assets, PBR, venue lighting (restored v2 Week 7)** |
| 10 | Dec 12-18 | Rendering 2: camera optics & NT4 Limelight Server |
| 11 | Dec 19-25 | Rendering 3: AprilTag validation & Live NT4 Feed (Christmas, full capacity) |
| 12 | Dec 26-28 | Freeze and handoff |

### Block 1 (Oct 21-22): Closeout
- Project skeleton: CMake, Jolt wrapper, parameter-file loader with units.
- **Done when:** damping and iteration settings are chosen with numbers, and the skeleton builds.

### Block 2 (Oct 23-29): Motors, battery, swerve & WPILib HALSim WS Bridge v0
- Control boundary structs (`SimInputs`, `SimOutputs`) defined upfront so both HALSim and the calibrator share one interface.
- Implicit DC motor and gearbox model (reflected inertia), lumped battery with current limits.
- 4-module swerve chassis from hand-written JSON.
- **WPILib HALSim WebSocket Server v0** (`ws://127.0.0.1:3300/wpilibws`): Accepts connections from your C++ robot project (`./gradlew simulateNative`), parses motor voltage/PWM commands into `SimInputs`, sends `DriverStation` enable/joystick packets, and returns encoder positions/velocities and gyro yaw via `SimOutputs`.
- Dump the active `SimDevice` JSON tree from your 2026 C++ robot code on connect to record exact vendor property keys.
- FTC robot at home, evenings: log commanded voltage, encoder velocity and battery voltage as CSV. This is your first calibration data.
- **Done when:** your actual C++ swerve code connects over `halsim_ws` and drives, strafes, and spins the simulated chassis via USB gamepad with realistic voltage sag and no jitter or energy growth over 60 s.

### Block 3 (Oct 30-Nov 5): Mechanisms, schema, sensors & `SimDevice` Dispatch
- **Mechanisms:**
  - Elevators: slider plus motor, with cascade and continuous rigging coupling. Try Jolt's gear or rack-and-pinion constraints first, and a kinematic coupling if that is less stable. Not yet tested.
  - Arms: single and double-jointed, with gravity torque and hard stops from joint limits.
  - Four-bar linkage under motor load (the bake-off showed 1.26 mm drift with a 15 kg load).
  - Gear and chain coupling between joints.
  - A flywheel as a motor-driven joint, with no piece interaction yet.
- **Robot data schema v1:** Links with mass, center of mass and full inertia tensor about the center of mass (kg and m). Joints with type, axis, origin and limits. Gear ratios. Motor presets keyed by name (Kraken X60, Falcon 500, NEO, NEO Vortex), with constants taken from WPILib's `DCMotor` (verify the values). Intake and shooter contact zones. Material names. The **sidecar format** for what CAD can't hold (motors, CAN IDs, gear ratios, roles).
- **Sensors:** encoders (gear scaling, quantization), 6-axis IMU with bias and noise, limit switches, beam breaks.
- **HALSim `SimDevice` Dispatcher & Lockstep Hook:**
  - Maps CTRE Phoenix 6 (`TalonFXSimState`) and REVLib (`SparkMaxSim`) `SimDevice` messages to the CAN IDs in `sidecar.json`.
  - Test whether Phoenix 6 on-controller closed-loop modes (`PositionVoltage`, `VelocityVoltage`) step cleanly under `frc::sim::PauseTiming()` + `StepTiming(20_ms)`. If Phoenix's background thread drifts at 10x, fall back to outputting motor voltage from robot code or running WPILib-side PID in sim.
- Build the Fusion test assembly (chassis, swerve module, elevator, arm, four-bar) with real joints and physical materials. The exporter needs it by Nov 6.
- **Done when:** a complete robot (swerve, four-bar intake, elevator, arm, flywheel) loads from JSON, all mechanisms run together for 5 simulated minutes driven by your real C++ subsystem code, and **schema v1 is frozen**.
- **Gate:** if the schema isn't frozen by Nov 5, the exporter starts on a versioned v0.9 and the importer absorbs the changes.

### Block 4 (Nov 6-10): Fusion exporter (Python add-in)
- Read the Synthesis exporter (Apache-2.0) for how it walks the joint hierarchy. Do **not** use Mirabuf as the output format, because it has no inertia tensor.
- Group components joined by rigid joints into one link. Export mass, center of mass, inertia tensor (converted from cm to m), material name and a collider proxy.
- Export per joint: type, axis, origin, limits (no limit means continuous), name.
- Join the sidecar by joint and component name, and warn on unmatched names.
- Warn on missing materials, joints without limits, and zero-mass bodies.
- Tessellate visual meshes per link. I haven't verified which API call does this.
- **Done when:** the test assembly exports a manifest and meshes matching the schema v1, with mass, center of mass and inertia agreeing with Fusion's own values.

### Block 5 (Nov 11-15): Importer and mass reconciliation
- C++ importer builds ECS entities, Jolt bodies, joints, colliders, motors and render meshes from the manifest and sidecar.
- Material lookup in `materials.json` for friction and restitution.
- Mass reconciliation tool: compares CAD mass, center of mass and per-subsystem mass to scale readings, and applies per-link scale or an "unmodeled mass" entry.
- Round trip: export, import, drive from C++ robot code, and compare with the FTC robot if its CAD exists.
- **Gate (Nov 15):** if the exporter and importer aren't reliable, use hand-authored JSON for this season. That fallback is cheap, because block 3 already produces working hand-written robots.

### Block 6 (Nov 16-22): Interaction layer
- Piece state machine: `FREE -> INTAKING -> STORED -> ENGAGED -> LAUNCHED -> FREE`, with abort edges.
- Intake: math-injected grip force while the piece is still a solver body. Capture attaches the piece to the robot and trips the simulated beam-break DIO in `SimOutputs`.
- Shooter: math takes over the piece (compression, slip, exit speed, spin, wheel torque). Recoil impulse on the chassis, robot velocity inherited at the shooter point, collision filter against the own robot.
- Flight: drag and Magnus force each step. The solver handles rim hits and landing.
- Promote the bake-off 1D oracles to production models.
- **Done when:** 2,000 randomized shots match the model within 5% exit speed and 10% RPM sag, with no NaNs, and the handoffs conserve momentum.

### Block 7 (Nov 23-27): Parameter store and calibrator
- Layered parameter files (`defaults -> measured -> calibrated`) with units, uncertainty, source and date.
- Experiment definitions, CSV log replay (direct `SimInputs` injection, bypassing `halsim_ws`), Levenberg-Marquardt staged fits with a penalty toward measured priors, 70/30 fit and validation split.
- **Parameter-recovery test:** generate fake data from the sim with known parameters plus noise, and check the calibrator recovers them.
- Fit the FTC drivetrain data from block 2 as the first real-data test.
- **Cut to fit 5 days:** the covariance report and CMA-ES.
- **Done when:** recovery works within tolerance on synthetic data, and the FTC drivetrain fit validates on held-out data.

### Block 8 (Nov 28-Dec 4): Field, Match mode, Option A Fleet Runner & Mock DriverStation
- Field through the same exporter and importer (static bodies), with carpet friction regions and scoring sensors.
- Piece manager (sleeping, CCD above a speed threshold), 6-robot Match mode, bumper collisions.
- **Mock DriverStation Sequencer:** Simulates official FRC match phases (`Autonomous` 15 s -> `Teleop` 135 s -> `Disabled`, alliance station, game-specific message) over the WebSocket bridge.
- **Option A Fleet Runner for Auto Routines:**
  - Process manager launches $N$ headless instances of your compiled C++ robot executable with `HALSIMWS_PORT=3300..3300+N` and `NT4_PORT=5810..5810+N` (so parallel auto instances do not collide on a single NetworkTables server).
  - Lockstep coordinator steps each Jolt world 4 times (20 ms) and triggers `StepTiming(20_ms)` on its paired robot process.
  - Benchmark 4, 8, 12, and 16–20 parallel worker processes across your 8 cores and record total autonomous routines completed per second.
- **Done when:** Match mode runs stably with 6 robots and pieces, and Fleet mode runs parallel C++ autonomous routines deterministically with `reset(seed)`. Record the real throughput number.
- **This is the riskiest block.** v2 gave this content two weeks, and it has one here.

### Block 9 (Dec 5-11): Rendering 1, assets, PBR, lighting (restored)
- glTF 2.0 loader, with meshes coming from the exporter.
- Cook-Torrance PBR, normal mapping, environment reflections so polycarbonate, anodized aluminum, foam pieces and carpet get realistic highlights.
- Overhead arena lights with shadows, alliance-colored emissive panels.
- **Done when:** field, robots and pieces render with realistic materials, shadows and reflections.

### Block 10 (Dec 12-18): Rendering 2, camera optics & NT4 Limelight Server
- Offscreen camera framebuffers at Limelight resolutions.
- Intrinsics and Brown-Conrady distortion in a post-process shader (loads real calibration data).
- Motion blur driven by the physics velocities. Global shutter first, rolling shutter if time allows.
- Exposure, gain, bloom and sensor noise.
- **NetworkTables 4 (NT4) Limelight Server:** Links `ntcore` (or a minimal NT4 server) into `FRCBananaSim` and publishes `/limelight/tv`, `tx`, `ty`, `ta`, `tid`, `/limelight/botpose_wpiblue`, `cl`, and `tl`.
- **Done when:** spinning the chassis at 360 deg/s blurs according to exposure time, distortion matches a calibration matrix, and your C++ robot code reads the `/limelight/*` table over NT4.

### Block 11 (Dec 19-25): Rendering 3, AprilTag validation & Live NT4 Feed
- AprilTag textures with sharp, mipmap-safe edges.
- Run a real AprilTag detector on rendered frames and compare the pose with ground truth across distance, angle, blur and glare.
- Async pixel-buffer readback and a latency ring buffer (camera-to-robot delay and frame-drop jitter) feeding the NT4 `/limelight/botpose_wpiblue` publisher.
- **Done when:** the detector reproduces realistic distance noise and blur dropout without stalling the physics threads, and your C++ robot code's `PoseEstimator` fuses the live NT4 vision measurements during an autonomous routine.

### Block 12 (Dec 26-28): Freeze and handoff
- Minimal ImGui tuning panel with sim and real traces overlaid.
- Recorded datasets and calibrated results saved as regression fixtures.
- **Moved past the cutoff** (Dec 28 - Jan 8 buffer, no AI help needed): soak tests, schema documentation, panel polish.
- **Done when:** you can load a new robot, a new piece, new measurements, and run your C++ auto routines against it without AI help.

## 4. Gates and cut order

| Gate | When | If it fails |
|---|---|---|
| Swerve joint error under 5 mm with margin | Block 1 | Raise iterations, add substeps |
| Phoenix 6 / REV `SimDevice` lockstep at 10x | Nov 5 | Send motor voltages from robot code in sim rather than on-controller closed-loop modes |
| Schema v1 frozen | Nov 5 | Exporter starts on v0.9, importer absorbs changes |
| Exporter and importer reliable | Nov 15 | Hand-authored JSON this season |
| Shooter and intake models | Nov 22 | Simplify the model before touching the engine |
| Calibrator recovers known parameters | Nov 27 | Manual tuning panel plus single-stage fits |
| Option A Fleet IPC throughput | Dec 4 | Cap parallel robot processes at 6–8 (one per core) for highest total auto rollouts/sec |

**Cut order if behind (first to last):**
1. Rolling-shutter distortion, extra arena lights and shadow cascades.
2. Calibrator covariance report and CMA-ES (already cut in block 7).
3. Match-mode bumper compliance (use rigid bumpers).
4. Importer reconciliation tool beyond a CLI.
5. Elevator cascade helpers (use single-stage sliders with gear ratios).
6. Rendering validation depth (block 11), moved past the cutoff.
7. Never cut: swerve and motors, HALSim WS + NT4 bridge, the schema and its mechanisms, the interaction layer, the importer or its hand-authored fallback, the fleet and match runners, camera intrinsics and motion blur.

## 5. Not yet verified

- **Localhost WebSocket IPC latency** with 15–20 parallel robot processes on Windows is unmeasured and is the main bottleneck risk for Option A Fleet mode.
- **CTRE Phoenix 6 background sim threads** under `frc::sim::PauseTiming()` + `StepTiming(20_ms)` need testing in Block 3.
- The exporter, importer and calibrator 2-day cuts are estimates.
- Jolt gear or rack-and-pinion constraints for elevator cascades have not been tested.
- The Fusion API call for visual-mesh tessellation, and the WPILib motor constants, need checking when you get there.
- Full-robot Jolt cost per step is unmeasured.
