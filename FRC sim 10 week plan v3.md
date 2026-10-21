# FRC Simulation: 10-Week Plan, Revision 3 (Oct 19 - Dec 27, 2026)

## What changed in this revision

1. **Fusion exporter and importer are now in the plan** (Weeks 3-4). Details in section 3.
2. **Engine: Jolt**, pending three quick checks from the bake-off (damping set to 0, iteration sweep for swerve joint error, PhysX single-thread/dt check). PhysX GPU is dropped.
3. **Hybrid solver/math boundary** replaces the "contact model inside the solver" idea. The shooter pinch and intake pull-in are math models. The solver owns everything geometric.
4. **Parameter store and calibrator** are built before the AI cutoff (Week 6) and validated on synthetic data, then on your FTC robot.
5. **Rendering drops from 3 weeks to about 2.5.** The schedule is full. See the trade-offs in section 6.

> [!WARNING]
> There is no slack inside these 10 weeks. Dec 28 - Jan 8 is buffer, but your AI help ends in Week 10, so anything that slips past Week 10 is done alone. Gates and the cut order (section 5) decide what gives.

## 1. Performance budgets (unchanged)

| Mode | Sims | Robots per sim | Rate / speed | Step budget |
|---|---|---|---|---|
| Fleet | 15-20, one per core across 7 cores | 1 | 200 Hz / 10x | ~175-230 us |
| Match | 1 | 6 | 200 Hz / 1x live or 10x rollout | ~5,000 us (1x), ~500 us (10x) |
| Camera | 1-2 offscreen streams | 1 or 6 | 60-120 FPS | under 2 ms GPU |

Bake-off (release): Jolt 51 us for the 100-body scene and 42 us for the swerve scene, so one chassis uses roughly a quarter of the fleet budget. The rest of the robot, pieces, and models must fit in the remainder. Not yet measured.

## 2. Architecture

```
Fusion 360 add-in (Python)  --> manifest.json + meshes --+
                                                         v
  Parameter store (defaults -> measured -> calibrated) --> Importer (C++) --> ECS + Jolt bodies/joints
                                                                              |
  Sim core: motors/battery (math) + interaction layer (shooter/intake math)  |
            piece state machine: FREE -> INTAKING -> STORED -> ENGAGED -> LAUNCHED
                                                                              |
  Fleet runner (15-20 x 1 robot)   Match runner (6 robots)   Renderer / camera sim
  Calibrator (offline) uses the fleet runner for parameter sweeps
```

- Keep your ECS (benchmarked fast enough).
- The solver owns geometry, joints, contacts, and traction. Models inject forces and impulses.

## 3. Fusion 360 exporter and importer

### Facts checked

| Item | Finding |
|---|---|
| Mass properties | Fusion API `PhysicalProperties` gives mass, density, volume, center of mass, principal and XYZ moments of inertia, with an accuracy setting. Units are cm and kg. Use High accuracy. |
| Joints | `RevoluteJointMotion` (`rotationLimits`), `SliderJointMotion` (`slideLimits`), plus rigid, cylindrical and others. Limits are in radians and cm. |
| Custom data | `Attributes` can store key-value metadata on components, occurrences, joints and bodies, saved inside the `.f3d`. |
| Add-in language | Python add-ins need no compile step. C++ add-ins do. |
| Synthesis exporter | Open source (Apache-2.0). It writes Mirabuf (protobuf, schema repo MPL-2.0, last updated July 2024). |
| **Mirabuf gap** | `PhysicalProperties` has density, mass and center of mass, **but no inertia tensor**. It does have joint limits and dynamics, material friction and restitution, per-part `mass_override`, and a DC motor message. |

### Decision: write your own Python exporter, using Synthesis's as a reference

- Mirabuf lacks the inertia tensor, and you want CAD-derived inertia. Your own manifest gets it directly from `PhysicalProperties`.
- Python avoids the compile-and-reload loop the original plan warned about.
- Read the Synthesis exporter (Apache-2.0) for how it walks the joint hierarchy before writing yours.
- The exporter is a separate tool, so it sits outside the C++ code-style rules for the engine.

### Exporter (Week 3)

- Walks the assembly, groups components connected by rigid joints into one link, and exports per link: mass, center of mass, inertia tensor (about the center of mass, in kg and m, converted from cm), material name, and a simplified collider proxy.
- Exports per joint: type, axis, origin, limits (no limit means continuous), and name.
- Tessellates visual meshes per link through the API's mesh tools (I haven't verified the exact API call) and writes them next to a `manifest.json`.
- Reads a **sidecar config keyed by joint and component name** for what CAD can't hold: motor type, gear ratio, current limits, collider overrides, mechanism roles (intake, shooter). This settles the old open question about motors and gearing. Warn on any name that doesn't match.
- Outputs warnings for missing materials, joints without limits, and zero-mass bodies.
- Test fixture: a small Fusion assembly built in Week 1 (chassis, one swerve module, an arm, a four-bar) with real joints and physical materials.

### Importer (Week 4)

- Reads the manifest and the sidecar. Builds ECS entities, Jolt bodies, joints, colliders, motors, and render meshes.
- Looks up material names in `materials.json` for friction and restitution.
- **Mass reconciliation tool:** compares the CAD total and per-subsystem mass with a scale reading and reports the gap. Handles the gap with a per-link mass scale or an "unmodeled mass" entry at a given location. Inertia scales with the mass correction.
- Round-trip test: export the FTC robot (if you have its CAD) or the test assembly, import it, drop it, drive it, compare with the real robot.

### Process rules for the CAD team (agree now)

- Build the assembly with real **Joints**, not just mates, and set motion limits.
- Assign **physical materials**, not just appearances. Custom materials for foam, rubber, belts.
- Name joints and components with the convention the sidecar uses.
- CAD will undercount mass (wiring, fasteners, motor and electronics internals are usually missing or simplified). Expect a gap and plan to reconcile against the real robot's weight.

### Fallback

If the exporter and importer aren't reliably working by the end of Week 4, timebox the rest: hand-author the JSON manifest this season and keep improving the exporter in the background. The manifest format is the same either way, so nothing downstream changes.

## 4. Weekly schedule

### Week 1 (Oct 19-25): Close the bake-off, skeleton
- Run the 3 checks: set linear and angular damping to 0 on every body, sweep Jolt iteration counts against swerve joint error and step cost, confirm PhysX ran single-threaded at the same dt. Decide the engine.
- Tag the bake-off code in git, delete the losing engine from main.
- Project skeleton: CMake, Jolt wrapper with deterministic seeds, parameter-file loader skeleton with units, robot manifest schema v0.
- Build the Fusion test assembly.
- Done when: engine chosen with evidence, and the schema v0 is agreed.

### Week 2 (Oct 26-Nov 1): Motors, battery, swerve
- Implicit DC motor and gearbox (reflected inertia), lumped battery model with current limits, 4-module swerve chassis from hand-written JSON.
- FTC robot at home (evenings): log voltage steps and ramps, encoder velocity, and battery voltage for your drive motors. These are the first calibration data.
- Done when: the chassis drives, strafes, and spins with realistic voltage sag, and no jitter or energy growth over 60 s.

### Week 3 (Nov 2-8): Fusion exporter v1 (Python)
- Everything in the exporter section above, tested on the test assembly.
- Done when: the manifest and meshes come out of Fusion with correct mass, center of mass, inertia, joints, and limits for the test assembly.

### Week 4 (Nov 9-15): Importer v1 and mass reconciliation
- Importer, material lookup, mass reconciliation tool, mechanism tests on imported assemblies (elevator, arm, four-bar, gear coupling).
- Done when: the test assembly runs in the sim from CAD alone, and the FTC robot round trip works (if its CAD exists).
- Gate: if not reliable by the end of this week, use the fallback above.

### Week 5 (Nov 16-22): Interaction layer (hybrid)
- Piece state machine: `FREE -> INTAKING -> STORED -> ENGAGED -> LAUNCHED -> FREE`, with abort edges.
- Intake: math force injection (grip from roller speed, squeeze, friction) while the piece stays a solver body. Capture attaches it to the robot.
- Shooter: math takeover (compression, slip, exit speed, spin, wheel torque), with recoil impulse on the chassis, robot velocity inherited at the shooter point, and a collision filter against the own robot.
- Flight: drag and Magnus force injection, solver handles rim hits and landing.
- Promote the bake-off 1D oracles to production models.
- Done when: 2,000 randomized shots match the model within 5% exit speed and 10% RPM sag, with no NaNs, and the handoffs conserve momentum.

### Week 6 (Nov 23-29, Thanksgiving, light week): Parameter store and calibrator
- Layered parameter files (`defaults -> measured -> calibrated`), each value with units, uncertainty, source, and date.
- Experiment definitions plus log replay, CSV importer.
- Calibrator tool: staged fits, Levenberg-Marquardt, penalty toward measured priors, parameter covariance report, 70/30 fit/validation split, runs sweeps on the fleet.
- Sensor models (encoders, IMU, limit switches, beam breaks).
- **Parameter-recovery test:** generate fake data from the sim with known parameters plus noise, and check the calibrator recovers them.
- Fit the FTC drivetrain logs from Week 2 as the first real-data test.
- Done when: recovery works within tolerance on synthetic data, and the FTC drivetrain fit validates on held-out data.

### Week 7 (Nov 30-Dec 6): Field, Match mode, Fleet runner
- Field geometry through the same exporter and importer (static bodies), with carpet friction regions and scoring sensors.
- Piece manager (sleep, CCD above a speed threshold), 6-robot Match mode, bumper collisions.
- Fleet runner: 15-20 single-threaded 1-robot instances on 7 cores, fast `reset(seed)`, profiling pass.
- Done when: Match mode runs stably with 6 robots and pieces, and Fleet mode sustains 10x in headless runs. Record the honest throughput number.

### Weeks 8-9 and half of Week 10 (Dec 7-Dec 24): Visual accuracy (about 2.5 weeks)
- **Week 8: assets, PBR, venue lighting.** Meshes from the exporter, glTF loading, Cook-Torrance PBR, polycarbonate reflections, arena lights and shadows.
- **Week 9: camera optics.** Offscreen camera FBO at Limelight resolutions, intrinsics and Brown-Conrady distortion, velocity-driven motion blur, exposure and noise. Calibration data from a real camera loads directly.
- **Week 10 (first half): validation.** AprilTag textures, run an AprilTag detector on the rendered frames and compare pose against ground truth, async readback with a latency buffer.
- Done when: a detector on the sim stream shows realistic distance noise and blur dropout, and the camera costs the rest of the system under its budget.

### Week 10 (Dec 21-27), second half: Freeze and handoff
- Control API boundary (`SimInputs`, `SimOutputs`, `CameraFrame`, `reset`, `step`).
- ImGui tuning panel with sim and real traces overlaid.
- Regression fixtures: recorded datasets plus calibrated results in CTest.
- Soak tests, documentation of every schema, sidecar field, and parameter.
- Done when: you can add the 2027 game and calibrate it without AI help.

## 5. Gates, cuts, and fallbacks

| Gate | When | If it fails |
|---|---|---|
| Engine decision checks | Week 1 | Reconsider PhysX CPU if friction is still over 2% off, or swerve joint error can't get under ~2 mm within ~100 us/step |
| Exporter and importer working | End of Week 4 | Hand-authored manifest fallback, keep improving in the background |
| Shooter and intake models | Week 5 | Simplify the model before changing the engine |
| Calibrator recovers known parameters | Week 6 | Reduce to manual tuning panel plus single-stage fits |
| Fleet throughput | Week 7 | Lower physics rate or speed target, sleep more aggressively |

**Cut order if behind (first to last):**
1. Rolling-shutter distortion (keep global shutter and motion blur).
2. Shadow cascades and extra arena lights.
3. Calibrator covariance report and CMA-ES (keep Levenberg-Marquardt).
4. Cascade rigging helpers.
5. Visual validation depth (Week 10 first half).
6. Never cut: engine decision, swerve and motors, the interaction layer, the exporter/importer or its fallback manifest, fleet and match runners, camera intrinsics and motion blur.

## 6. Trade-offs I made

- **Rendering lost half a week or more.** Fusion export and import (2 weeks) plus the calibrator (1 week) came out of the old rendering time. If you want 3 full weeks back, the cheapest cuts are the mechanism tests in Week 4 and the Match-mode polish in Week 7.
- **Weeks 3 to 6 are the riskiest block.** They hold two new tools (exporter and importer) and the calibrator. Weeks 3 and 4 are the schedule's single point of failure.
- **No data, no accuracy proof.** Everything here is validated against analytic models, synthetic data, and your FTC robot. FRC constants for the new game need real measurement after kickoff.

## 7. FTC robot at home: what it can and can't test

| Can validate | Can't validate |
|---|---|
| Motor model and gearbox constants, battery sag, drivetrain friction and inertia | FRC swerve geometry (unless the FTC robot has swerve) |
| CAD mass, center of mass, and inertia against the real robot (if you have its CAD) | FRC-scale mass and mass ratios |
| Flywheel spin-up and coast, and shot calibration if it has a launcher | Game-piece compression for the 2027 game |
| The whole measure, export, import, calibrate, validate pipeline | Anything specific to the 2027 field |

Write a short opmode that logs timestamp, commanded voltage, encoder velocity, and battery voltage to a CSV. The calibrator's CSV importer reads it directly.

## 8. Measurement with only a 120 fps camera

- At 120 fps one frame is 8.3 ms. A 17 m/s piece moves about 14 cm per frame. Track it over 4-6 frames against a scale grid placed in the shot plane, with the camera square to the plane. That should give exit speed to roughly 2-3%, if the phone records at a constant frame rate (many record variable frame rate, so verify with a timer or flash in frame).
- A 22 ms pinch contact is about 2.7 frames, so it can't be seen directly. Infer contact behavior from the wheel RPM log instead. Raise the controller status-frame rate as high as it allows.
- Static squish test with no load cell: use dead weights (water jugs, plates, books) at 5-10 load steps, measure compression with calipers, then unload to see hysteresis. Wait a fixed time at each step to control creep.
