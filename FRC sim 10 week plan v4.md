# FRC Simulation: 10-Week Plan, Revision 4 (Oct 21 - Dec 27, 2026)

## What changed from v3

| Change | Detail |
|---|---|
| **Restored: v2 Week 4** (linkages, elevators, arms, robot data schema) | Now a full 7-day block (Oct 30 - Nov 5). It sits **before** the Fusion exporter, because the schema it defines is the contract the exporter and importer both build to. |
| **Restored: v2 Week 7** (glTF, PBR, venue lighting) | Rendering is back to the v2 shape: three full weeks (assets/PBR/lighting, then optics, then AprilTag validation). v3 had squeezed it to 2.5 weeks. |
| **Jolt is the engine** | Bake-off is done. Week 1 shrinks to a 2-day closeout. |
| **Early start on Week 2** | Motors, battery and swerve start Oct 23. |
| **Thanksgiving and Christmas are full-capacity** | No light weeks. Nov 26 and Dec 25 are treated as normal workdays. |
| **Exporter, importer and calibrator trimmed** | 7 days becomes 5 days each (see the accounting below). |

> [!NOTE]
> v3 was not empty on mechanisms. It had schema v0 in Week 1, mechanism *tests* inside the importer week, and sensors in Week 6. What it lacked was a block to *build* elevators, arms, linkages, motor presets and the full schema. That is what is restored.

> [!IMPORTANT]
> I read "v2 week 7 back" as restoring the full three-week rendering block, since v3's Week 8 already held the same assets/PBR/lighting content. If you meant something else, tell me.

## Day accounting (why it fits, barely)

| | Days |
|---|---|
| v3 calendar (Oct 19 - Dec 27) | 70 |
| **Added** | |
| Mechanisms and schema block (v2 Week 4) | +7 |
| Rendering 2.5 weeks to 3 weeks | +3.5 |
| **Freed** | |
| Week 1 closeout (bake-off finished early, Jolt chosen) | -5 |
| Exporter 7 to 5 days | -2 |
| Importer 7 to 5 days | -2 |
| Calibrator 7 to 5 days | -2 |
| Freeze block 3.5 to 3 days | -0.5 |
| **v4 total (Oct 21 - Dec 28)** | **69** |

- Available days from Oct 21 through Dec 27 are 68, so the plan runs **about 1 day over**. The freeze block absorbs it.
- **There is no slack.** The three 2-day cuts are my estimates, not measurements. They are plausible only because the schema is frozen in the mechanisms block, so the exporter and importer start from a fixed contract.
- With the holidays at full capacity, a real day off on Nov 26 or Dec 25 pushes the plan past the cutoff. The cut order (section 4) says what goes first.

## 1. Performance budgets (unchanged)

| Mode | Sims | Robots per sim | Rate / speed | Step budget |
|---|---|---|---|---|
| Fleet | 15-20, one per core across 7 cores | 1 | 200 Hz / 10x | ~175-230 us |
| Match | 1 | 6 | 200 Hz / 1x live or 10x rollout | ~5,000 us (1x), ~500 us (10x) |
| Camera | 1-2 offscreen streams | 1 or 6 | 60-120 FPS | under 2 ms GPU |

Release bake-off numbers for Jolt: 51 us for the 100-body scene, 42 us for the swerve scene, 16 us for the four-bar scene. Full-robot cost is not yet measured.

## 2. Architecture (unchanged)

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
│               2. Control Boundary & Robot Simulation Core                  │
│                                                                            │
│  SimInputs (Volts/PWM) ──► [Implicit DC Motors & Lumped Battery Sag]       │
│                                           │                                │
│       ┌───────────────────────────────────┴────────────────────────┐       │
│       │ Joint Torques & Drive Demands                              │       │
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
│           3. Engine Foundation & Jolt Physics Solver (200 Hz)              │
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
│       4. Execution Runners        │   │    5. GPU Limelight Pipeline       │
│                                   │   │                                    │
│  ┌─────────────────────────────┐  │   │  ┌──────────────────────────────┐  │
│  │ Headless Fleet Runner       │  │   │  │ Offscreen FBOs (960p/480p)   │  │
│  │ - 15-20 Parallel 1-Robot    │  │   │  └──────────────┬───────────────┘  │
│  │ - Single-Threaded @ >= 10x  │  │   │                 ▼                  │
│  └──────────────┬──────────────┘  │   │  ┌──────────────────────────────┐  │
│                 │ Swarm Sweeps    │   │  │ OpenGL PBR Venue Renderer    │  │
│                 ▼                 │   │  │ (Cook-Torrance, Arena Lights)│  │
│  ┌─────────────────────────────┐  │   │  └──────────────┬───────────────┘  │
│  │ Offline Calibrator          │  │   │                 ▼                  │
│  │ - Levenberg-Marquardt Fits  │  │   │  ┌──────────────────────────────┐  │
│  │ - Covariance & Priors Report│  │   │  │ Optics Post-Process Shaders  │  │
│  └─────────────────────────────┘  │   │  │ (Distortion, Motion Blur)    │  │
│                                   │   │  └──────────────┬───────────────┘  │
│  ┌─────────────────────────────┐  │   │                 ▼                  │
│  │ Headless / GUI Match Runner │  │   │  ┌──────────────────────────────┐  │
│  │ - 1x Match (6 Robots + FMS) │  │   │  │ Async PBO Readback & Latency │  │
│  │ - Bumper-to-Bumper Impact   │  │   │  │ AprilTag solvePnP & Detectors│  │
│  └─────────────────────────────┘  │   │  └──────────────────────────────┘  │
└───────────────────────────────────┘   └────────────────────────────────────┘
```

## 3. Schedule

| # | Dates | Block |
|---|---|---|
| 1 | Oct 21-22 | Closeout |
| 2 | Oct 23-29 | Motors, battery, swerve |
| 3 | Oct 30-Nov 5 | **Mechanisms, schema, sensors (restored v2 Week 4)** |
| 4 | Nov 6-10 | Fusion exporter |
| 5 | Nov 11-15 | Importer and mass reconciliation |
| 6 | Nov 16-22 | Interaction layer (shooter and intake math) |
| 7 | Nov 23-27 | Parameter store and calibrator (Thanksgiving, full capacity) |
| 8 | Nov 28-Dec 4 | Field, Match mode, Fleet runner |
| 9 | Dec 5-11 | **Rendering 1: assets, PBR, venue lighting (restored v2 Week 7)** |
| 10 | Dec 12-18 | Rendering 2: camera optics |
| 11 | Dec 19-25 | Rendering 3: AprilTag validation (Christmas, full capacity) |
| 12 | Dec 26-28 | Freeze and handoff |

### Block 1 (Oct 21-22): Closeout
- Project skeleton: CMake, Jolt wrapper, parameter-file loader with units.
- **Done when:** damping and iteration settings are chosen with numbers, and the skeleton builds.

### Block 2 (Oct 23-29): Motors, battery, swerve
- Implicit DC motor and gearbox model (reflected inertia), lumped battery with current limits.
- 4-module swerve chassis from hand-written JSON.
- FTC robot at home, evenings: log commanded voltage, encoder velocity and battery voltage as CSV. This is your first calibration data.
- **Done when:** the chassis drives, strafes and spins with realistic voltage sag, and no jitter or energy growth over 60 s.

### Block 3 (Oct 30-Nov 5): Mechanisms, schema, sensors (restored)
- **Mechanisms:**
  - Elevators: slider plus motor, with cascade and continuous rigging coupling. Try Jolt's gear or rack-and-pinion constraints first, and a kinematic coupling if that is less stable. Not yet tested.
  - Arms: single and double-jointed, with gravity torque and hard stops from joint limits.
  - Four-bar linkage under motor load (the bake-off showed 1.26 mm drift with a 15 kg load).
  - Gear and chain coupling between joints.
  - A flywheel as a motor-driven joint, with no piece interaction yet.
- **Robot data schema v1.** Links with mass, center of mass and full inertia tensor about the center of mass (kg and m). Joints with type, axis, origin and limits. Gear ratios. Motor presets keyed by name (Kraken X60, Falcon 500, NEO, NEO Vortex), with constants taken from WPILib's `DCMotor` (verify the values). Intake and shooter contact zones. Material names. The **sidecar format** for what CAD can't hold (motors, gear ratios, roles).
- **Sensors:** encoders (gear scaling, quantization), 6-axis IMU with bias and noise, limit switches, beam breaks.
- Build the Fusion test assembly (chassis, swerve module, elevator, arm, four-bar) with real joints and physical materials. The exporter needs it by Nov 6.
- **Done when:** a complete robot (swerve, four-bar intake, elevator, arm, flywheel) loads from JSON, all mechanisms run together for 5 simulated minutes without jitter or drift, and **schema v1 is frozen**.
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
- Round trip: export, import, drive, and compare with the FTC robot if its CAD exists.
- **Gate (Nov 15):** if the exporter and importer aren't reliable, use hand-authored JSON for this season. That fallback is cheap, because block 3 already produces working hand-written robots.

### Block 6 (Nov 16-22): Interaction layer
- Piece state machine: `FREE -> INTAKING -> STORED -> ENGAGED -> LAUNCHED -> FREE`, with abort edges.
- Intake: math-injected grip force while the piece is still a solver body. Capture attaches the piece to the robot.
- Shooter: math takes over the piece (compression, slip, exit speed, spin, wheel torque). Recoil impulse on the chassis, robot velocity inherited at the shooter point, collision filter against the own robot.
- Flight: drag and Magnus force each step. The solver handles rim hits and landing.
- Promote the bake-off 1D oracles to production models.
- **Done when:** 2,000 randomized shots match the model within 5% exit speed and 10% RPM sag, with no NaNs, and the handoffs conserve momentum.

### Block 7 (Nov 23-27): Parameter store and calibrator
- Layered parameter files (`defaults -> measured -> calibrated`) with units, uncertainty, source and date.
- Experiment definitions, CSV log replay, Levenberg-Marquardt staged fits with a penalty toward measured priors, 70/30 fit and validation split.
- **Parameter-recovery test:** generate fake data from the sim with known parameters plus noise, and check the calibrator recovers them.
- Fit the FTC drivetrain data from block 2 as the first real-data test.
- **Cut to fit 5 days:** the covariance report and CMA-ES.
- **Done when:** recovery works within tolerance on synthetic data, and the FTC drivetrain fit validates on held-out data.

### Block 8 (Nov 28-Dec 4): Field, Match mode, Fleet runner
- Field through the same exporter and importer (static bodies), with carpet friction regions and scoring sensors.
- Piece manager (sleeping, CCD above a speed threshold), 6-robot Match mode, bumper collisions.
- Fleet runner: 15-20 single-threaded 1-robot instances on 7 cores, fast `reset(seed)`, profiling pass.
- **Done when:** Match mode runs stably with 6 robots and pieces, and Fleet mode sustains 10x headless. Record the real throughput number.
- **This is the riskiest block.** v2 gave this content two weeks, and it has one here.

### Block 9 (Dec 5-11): Rendering 1, assets, PBR, lighting (restored)
- glTF 2.0 loader, with meshes coming from the exporter.
- Cook-Torrance PBR, normal mapping, environment reflections so polycarbonate, anodized aluminum, foam pieces and carpet get realistic highlights.
- Overhead arena lights with shadows, alliance-colored emissive panels.
- **Done when:** field, robots and pieces render with realistic materials, shadows and reflections.

### Block 10 (Dec 12-18): Rendering 2, camera optics
- Offscreen camera framebuffers at Limelight resolutions.
- Intrinsics and Brown-Conrady distortion in a post-process shader (loads real calibration data).
- Motion blur driven by the physics velocities. Global shutter first, rolling shutter if time allows.
- Exposure, gain, bloom and sensor noise.
- **Done when:** spinning the chassis at 360 deg/s blurs according to exposure time, and distortion matches a calibration matrix.

### Block 11 (Dec 19-25): Rendering 3, AprilTag validation
- AprilTag textures with sharp, mipmap-safe edges.
- Run a real AprilTag detector on rendered frames and compare the pose with ground truth across distance, angle, blur and glare.
- Async pixel-buffer readback and a latency ring buffer (camera-to-robot delay and frame-drop jitter).
- **Done when:** the detector reproduces realistic distance noise and blur dropout without stalling the physics threads.

### Block 12 (Dec 26-28): Freeze and handoff
- Control API boundary (`SimInputs`, `SimOutputs`, `CameraFrame`, `reset`, `step`).
- Minimal ImGui tuning panel with sim and real traces overlaid.
- Recorded datasets and calibrated results saved as regression fixtures.
- **Moved past the cutoff** (Dec 28 - Jan 8 buffer, no AI help needed): soak tests, schema documentation, panel polish.
- **Done when:** you can load a new robot, a new piece and new measurements without AI help.

## 4. Gates and cut order

| Gate | When | If it fails |
|---|---|---|
| Swerve joint error under 5 mm with margin | Block 1 | Raise iterations, add substeps |
| Schema v1 frozen | Nov 5 | Exporter starts on v0.9, importer absorbs changes |
| Exporter and importer reliable | Nov 15 | Hand-authored JSON this season |
| Shooter and intake models | Nov 22 | Simplify the model before touching the engine |
| Calibrator recovers known parameters | Nov 27 | Manual tuning panel plus single-stage fits |
| Fleet throughput | Dec 4 | Lower physics rate or speed target, sleep more aggressively |

**Cut order if behind (first to last):**
1. Rolling-shutter distortion, extra arena lights and shadow cascades.
2. Calibrator covariance report and CMA-ES (already cut in block 7).
3. Match-mode bumper compliance (use rigid bumpers).
4. Importer reconciliation tool beyond a CLI.
5. Elevator cascade helpers (use single-stage sliders with gear ratios).
6. Rendering validation depth (block 11), moved past the cutoff.
7. Never cut: swerve and motors, the schema and its mechanisms, the interaction layer, the importer or its hand-authored fallback, the fleet and match runners, camera intrinsics and motion blur.

## 5. Not yet verified

- The exporter, importer and calibrator 2-day cuts are estimates.
- Jolt gear or rack-and-pinion constraints for elevator cascades have not been tested.
- The Fusion API call for visual-mesh tessellation, and the WPILib motor constants, need checking when you get there.
- Full-robot Jolt cost per step is unmeasured. The fleet budget depends on it.
