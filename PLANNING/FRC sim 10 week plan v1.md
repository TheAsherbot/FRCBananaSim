# FRC Simulation: 10-Week Plan (Oct 19 - Dec 27, 2026)

**Scope:** physics and rendering only. Fusion import, HALSim/robot-code link, Driver Station and the RL training code are explicitly deferred. The design leaves hook points for them (section 3).

**Assumptions I'm making. Correct any that are wrong.**
- You can give this project about 20-25 hours a week. If you have less, use the cut order in section 7.
- "15-20 simulations" means each runs at roughly **10x real time**, in parallel, headless.
- 8 *physical* cores. If it is 8 cores / 16 threads, expect +15-30%, not 2x.
- Kickoff is Sat Jan 9, 2027 (from your plan file). I haven't verified it. The two weeks after Week 10 (Dec 28 - Jan 8) are **slip buffer**, not scheduled work, and they contain the winter holidays.
- No shop access until kickoff, so **nothing can be calibrated against real hardware before Week 10 ends**. Everything is validated against analytic and reference models, and every physical constant lives in a data file so it can be tuned after kickoff.

---

## 1. Performance budget (the tightest requirement)

| Assumption | Value |
|---|---|
| Physics rate | 200 Hz (4 steps per 20 ms robot loop) |
| Target speed | 10x per simulation |
| Steps per second per sim | 2,000 |
| Simulations | 20 (worst case) |
| Total steps per second | 40,000 |
| Cores for simulation | 7 (1 reserved for orchestration, rendering and OS) |
| **Budget per step** | **~175 µs for the whole scene** |

Other cases:

| Sims | Physics rate | Budget per step |
|---|---|---|
| 15 | 200 Hz | ~230 µs |
| 20 | 120 Hz | ~290 µs |
| 20 | 200 Hz, speed 5x | ~350 µs |

> [!WARNING]
> 175 µs for 6 robots plus pieces plus a field is aggressive. I can't tell you whether it's reachable until the Week 1 harness gives a real number. The levers, in order: sleep resting pieces aggressively, drop robot self-collision and use simple collision proxies, use a **lower-fidelity model for the 5 non-controlled robots**, lower the physics rate, and lower the speed target. Expect to use several of them.

**How the 3060 and the CPU are used**
- **CPU (7 cores):** all physics. Each sim is a single-threaded instance, shared-nothing.
- **GPU:** camera rendering for 1-2 sims at a time, and RL policy training later. It is **not** used for physics.
- Why not PhysX GPU physics: contact modification and compliant contacts have parity gaps on the GPU path 🔎, GPU runs are less deterministic 🔎, scenes this small underuse a GPU, and the GPU is already busy with rendering and training. It stays an optional post-season experiment.
- The robot code processes (when you add code-in-the-loop later) also use CPU, so the real budget will be tighter than the table.

---

## 2. Architecture

```
 Sim core (backend-agnostic, owns all FRC logic)
   robot model - motors/battery - contact models - game pieces - field - state snapshot/restore
        |
 PhysicsBackend interface  <---  conformance test suite (runs against every backend)
     |                  |
  Jolt backend      PhysX backend
        |
 Sim runner (headless, N instances)     Renderer (offscreen camera + dev viewer)
```

- **Keep RT-PhysicsCore as a library dependency.** Use its math, logging, ECS, telemetry and renderer. Don't fork its physics.
- **Jolt first, PhysX second.** The Jolt path is a complete simulator by Week 8. The PhysX backend is purely additive, so if time runs short it can be cut without losing a working sim.
- **Everything FRC-specific lives above the backend interface.** Motors, battery, intake and shooter contact models, game pieces and sensors never call Jolt or PhysX directly.

### Backend interface: capabilities and how each engine maps

| Capability | Jolt | PhysX 5 |
|---|---|---|
| Rigid bodies, box/sphere/capsule/cylinder/convex/compound | Native | Native (convex hull limits apply 🧠) |
| Revolute, prismatic, fixed | Hinge, slider, fixed | Revolute, prismatic, fixed |
| Loop closure and coupling | Native constraints, gear, rack-and-pinion | D6 joint; no gear in loops |
| Motor torque or velocity target | Constraint motors, or apply torque | Joint drives, or apply torque |
| **Per-contact surface velocity** | `ContactSettings` linear and angular | `PxContactModifyCallback` target velocity |
| **Custom contact force (compression model)** | Sensor contact plus force pass | Compliant contact, or sensor-like filter plus force pass |
| Sensors / trigger volumes | Sensors | Triggers |
| Sleep control | Per body | Per actor |
| Determinism control | Cross-platform deterministic option | `eENABLE_ENHANCED_DETERMINISM` |
| Snapshot / restore (for resets) | State save and restore | Serialize or rebuild scene (**slower**, likely rebuild per reset) 🧪 |

**Honest caution:** supporting two engines is about 1.5 weeks of the 10, and the interface can only expose the common subset. Features unique to one engine (PhysX articulations, Jolt gear constraints) will sit behind capability flags and won't be portable. Each backend gets the same tests, and the **conformance suite is the thing that keeps the abstraction honest.**

---

## 3. Hook points for deferred work

| Deferred item | Hook built now |
|---|---|
| Fusion import | Robot description is a data model (links, joints, colliders, motors, materials). The importer will emit it. Hand-author it in JSON until then. |
| HALSim / robot code | Motor commands in and state readouts out, as a plain struct per tick. |
| RL harness | Runner exposes `reset(seed)` and `step(n)` over shared-nothing instances. |

---

## 4. Weekly plan

### Week 1 (Oct 19-25): Foundations, Jolt v0, first numbers
- New CMake project: Jolt via `FetchContent`, RT-PhysicsCore as a dependency, CTest wired up, headless executable.
- Backend interface v0, designed against *both* engines' capability lists.
- Jolt backend v0: bodies, shapes, hinge, step, deterministic mode on, single-thread job system.
- **Conformance suite v0:** free-fall, hinge-pendulum energy, Coulomb slide distance, and a **45-degree friction test** (finds the cone vs pyramid question). Reuse your RT-PhysicsCore friction test as the oracle.
- **Perf harness:** synthetic scene (~400 bodies, ~100 awake) reporting µs per step.
- **Done when:** suite green on Jolt, and you have a real µs/step number to compare with the 175 µs budget.
- **Gate 1:** if the 45-degree test is badly off, plan a custom friction clamp in the contact listener.

### Week 2 (Oct 26-Nov 1): Motors, swerve module, chassis
- DC motor model: stall torque, free speed, back-EMF, gearbox ratio and efficiency, reflected inertia. **Implicit or semi-implicit coupling**, because high gear ratios make explicit back-EMF unstable.
- Swerve module (steer joint, drive joint, wheel cylinder with material), then a 4-module chassis.
- **Mass-ratio stress test:** ~50 kg chassis on ~1 kg wheel assemblies at full torque for 60 s.
- Done when: chassis drives, strafes and rotates under scripted voltage with no jitter and no energy growth.
- **Gate 2:** if the mass-ratio test fails after substeps and inverse-mass scaling, **pull the PhysX backend forward** (Week 3-4) and evaluate articulations.

### Week 3 (Nov 2-8): Contact-model framework, intake, game pieces
- Engine-independent **pair contact model** interface (inputs: pair, contact data. Outputs: surface velocity, friction scale, extra forces).
- Intake roller via surface velocity, and via a rotating-cylinder variant. Compare the two.
- Generic game-piece system: sphere, cube, cone and ring (compound of convex pieces), with mass, material and sleep policy from config.
- **PhysX build probe (half a day):** run `generate_projects`, get a hello-world scene linking. Catch showstoppers early.
- Done when: a driven roller pulls a ball and a ring into a pocket and holds them, with no escape and no jitter.

### Week 4 (Nov 9-15): Compression and shooter contact model
- Write the engine-free **reference shooter integrator** (about 100 lines).
- Build the shooter contact model on Jolt: nonlinear compression force with damping, regularized Coulomb friction with stick and slip, equal and opposite torque on the wheel, all parameters in a data file.
- Sweep tests vs the reference: exit speed within ~5%, peak RPM sag within ~10%, identical results on repeat, 2,000 randomized shots with no NaNs or tunneling, sphere and ring both pass.
- Done when: those criteria pass. These numbers show the code is internally consistent. They do **not** prove it matches real shots, which needs real data (section 8).

### Week 5 (Nov 16-22): Mechanisms, loops, battery
- Elevator (slider plus motor), arm (hinge plus motor), four-bar linkage, gear or chain coupling, joint limits, brake and neutral modes.
- Lumped battery model: bus voltage = battery voltage minus total current times internal resistance, plus a voltage floor and per-motor current limits.
- State readouts: joint positions and velocities, body poses, body-frame accelerations and gyro rates (raw, no noise models).
- Done when: a robot with chassis, elevator, arm and four-bar runs 5 minutes of simulated time with no drift or blow-up, and drivetrain current visibly sags the voltage.

### Week 6 (Nov 23-29, Thanksgiving, light week): Headless runner and performance pass 1
- Runner: N shared-nothing instances across 7 worker threads or processes, deterministic seeds, `reset(seed)` through state restore, no logging in the hot path.
- Optimize: sleeping, collision filtering layers, no per-step allocation, simplified collision proxies, a lower-fidelity model for non-controlled robots.
- Measure aggregate throughput for 15 and for 20 sims.
- Done when: you have an honest number for "X sims at Y-times real time", and a written decision on which levers you are using.
- Keep this week's feature load small. It is the holiday week.

### Week 7 (Nov 30-Dec 6): Field, multi-robot, assets
- Data-driven field loader (walls, ramps, carpet friction regions), 6-robot scene with real pushing, scoring zones via sensors.
- Spawn 50-100 pieces with a sleep policy.
- Renderer: glTF mesh loading, textures, materials, shared by robots, field and pieces.
- Done when: a match-like scene runs at target speed headless, and renders correctly in the dev viewer.

### Week 8 (Dec 7-13): Camera simulation
- Offscreen framebuffer rendering with no window, pinhole camera with intrinsics, optional lens distortion, AprilTag-textured quads, directional lighting plus shadows, exposure and simple noise, image readback.
- Run 1-2 rendered sims alongside the headless fleet and check what it costs the 3060 and the CPU.
- Done when: a camera frame from a sim looks right to your eye, a tag is detected by an AprilTag library at known distances, and rendering one sim does not break the throughput target for the others.

### Week 9 (Dec 14-20): PhysX backend, core parity
- Implement the backend for bodies, shapes, revolute/prismatic joints (maximal coordinates, **no articulations**), motors, contact-modify surface velocity, and the compression contact model through the same interface.
- Run the whole conformance suite against it.
- Done when: chassis, intake and shooter tests pass on PhysX. Loop closure and `snapshot/restore` can be partial.

### Week 10 (Dec 21-27, winter holidays, ~half capacity): Head-to-head, soak, freeze
- Head-to-head on the same scenes: µs per step, shooter reference match, mass-ratio stability, four-bar drift. Pick the **default backend**.
- Soak tests: long runs checking energy drift, NaNs, determinism.
- Freeze features. Write a short README covering architecture, data files and how to run each suite.
- Done when: default backend chosen with evidence, the suite is green on it, and everything is documented. **Don't schedule new features here.**

---

## 5. Decision gates and contingencies

| Gate | When | If it fails |
|---|---|---|
| 1. Friction accuracy | Week 1 | Add a custom friction clamp; compare PhysX friction |
| 2. Swerve mass ratio | Week 2 | Pull PhysX forward, evaluate articulations |
| 3. Intake behavior | Week 3 | Try the rotating-roller variant and angular surface velocity |
| 4. Shooter vs reference | Week 4 | Fix the contact model first; it's the model, not the engine |
| 5. Throughput | Week 6 | Use more levers; lower the speed target before cutting fidelity of the controlled robot |

---

## 6. Risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| 15-20 sims at 10x doesn't fit on 8 cores | High | Fidelity LOD, lower rate, lower speed target; Week 1 gives an early number |
| Swerve mass-ratio jitter | Medium | Substeps, inverse-mass scaling, PhysX backup |
| Two-backend abstraction leaks | Medium | Conformance suite from Week 1, capability flags |
| Holiday weeks lose capacity | High | Weeks 6 and 10 are planned light |
| Camera work takes longer than one week | Medium | Cut distortion and noise before cutting AprilTag support |
| Subscription / AI help ends mid-plan | Unknown | Front-load the hardest code (contact model, interface) into Weeks 1-4 |

---

## 7. Cut order if behind
1. PhysX loop closure and `snapshot/restore` parity
2. Camera noise and lens distortion
3. The whole PhysX backend (Jolt remains a complete simulator)
4. Lower-priority mechanism types (chain coupling, extra joint modes)
5. Never cut: Weeks 1-4, the conformance suite, and the headless runner.

---

## 8. Post-kickoff measurement checklist (Jan 9 onward)

You can't do these before kickoff, so every item below is a parameter in a data file, not a code change.

| Measure | Used for | How |
|---|---|---|
| Game piece **force vs compression curve** | Compression stiffness and damping | Kitchen scale and a press, caliper for compression |
| Piece mass and dimensions | Piece models | Scale and caliper |
| **Wheel-on-piece friction** | Shooter and intake friction | Pull test on a ramp or with a scale |
| Wheel-on-carpet friction | Chassis grip | Pull test |
| Robot mass and inertia | Chassis model | CAD export, weighed |
| Shooter logs (wheel RPM, exit speed) | Calibrate and validate the shooter model | AdvantageScope logs plus a phone camera at 240 fps |
| Drivetrain voltage steps | Calibrate motor and friction constants | Scripted tests |

---

## 9. What I need from you
- Your real available hours per week, so I can adjust the cut order.
- The date your AI-assist subscription ends, if it ends soon. I'd front-load the code-generation-heavy work.
- Whether to build the lower-fidelity opponent robot model, or require full fidelity for all 6 robots.
