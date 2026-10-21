# FRC Simulation: Revised 10-Week Plan (Oct 19 – Dec 27, 2026)

## What Changed & Why It Makes the Plan Much Stronger

1. **Week 1 is now a true head-to-head Bake-Off — then you pick ONE engine.**
   Instead of maintaining a leaky two-engine abstraction for 10 weeks and spending Week 9 building a second backend right before your AI cutoff, Week 1 pits **Jolt** and **PhysX 5** against the exact same 5 critical tests (including shooter compression and swerve mass ratios). At the end of Week 1, you pick the winner and delete the loser. That frees ~2 full weeks of engineering time and lets you use the winning engine's best native features without compromise.
2. **No cross-platform determinism overhead.**
   Since everything runs on your single Windows PC (8-core CPU + RTX 3060), we drop `CROSS_PLATFORM_DETERMINISTIC` (~8% free speedup in Jolt) and cross-OS build constraints. Same-machine, same-seed repeatability is all we need.
3. **Two distinct workload modes (1-robot fleet vs. 6-robot match) completely solve the CPU bottleneck.**
   - **Fleet Mode (15–20 parallel headless sims at 10x):** Only **1 robot** + field + active game pieces per sim (~15–25 dynamic bodies awake instead of ~200). A 175 µs step budget for a single robot is very comfortable for both Jolt and PhysX.
   - **Match Mode (1 sim at 1x–10x):** All **6 full-fidelity robots** + full field of game pieces in a single simulation instance. Since only 1 instance is running, it has the entire CPU (and multiple threads) to itself—no lower-fidelity "LOD robot" hacks needed.
4. **3 full weeks dedicated to Photorealistic / Limelight-Accurate Rendering (Weeks 7–9).**
   Cutting the second backend in Week 9 frees up time to expand rendering from 1 week to **3 full weeks**. A Limelight (both AprilTag ` MegaTag2` pipelines and neural color/detector pipelines) gets fooled by flat OpenGL Phong shading because real venues have specular carpet/polycarbonate glare, motion blur, rolling/global shutter timing, auto-exposure blowouts, and lens distortion. Your RTX 3060 has plenty of headroom to render a physically based (PBR) camera feed with real optical artifacts.
5. **Engineered around the Week 10 AI Subscription Cutoff.**
   All mathematically dense, boilerplate-heavy, and tricky systems (physics solver glue, implicit motor equations, nonlinear shooter contact math, PBR shaders, lens distortion & motion blur shaders, and CMake/glTF pipelines) are front-loaded into Weeks 1–9 while you have AI assistance. Week 10 is dedicated to clean documentation and config-driven tooling so you can easily add the 2027 game after Jan 9 on your own.

---

## 1. Revised Performance Budgets (8 Cores + RTX 3060)

| Mode | Active Sims | Robots / Sim | Active Bodies / Sim | Physics Rate | Target Speed | Step Budget / Sim | Verdict |
|---|---|---|---|---|---|---|---|
| **Fleet Mode** (Headless RL / Planning) | 15–20 (1 per worker thread across 7 cores) | **1** | ~15–25 awake (rest sleeping) | 200 Hz | **10x** (2,000 steps/s) | **175–230 µs** | **Very achievable** for 1 robot |
| **Match Mode** (Driver Practice / Full Match) | **1** (uses multi-threading across cores) | **6** (all full fidelity) | ~150–250 awake | 200 Hz | **1x** (live) or **10x** (rollout) | **5,000 µs** (1x) / **500 µs** (10x) | **Easy at 1x, comfortable at 10x** |
| **Limelight Sim** (GPU Offscreen FBO) | 1–2 camera streams | 1 or 6 | Full scene | 60–120 FPS | 1x–2x (or 10x at lower res) | **< 2 ms GPU time** on RTX 3060 | **Trivial for RTX 3060** |

---

## 2. Architecture (Post-Week-1 Single Engine)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        FRC Simulation Core                              │
│  Robot Model (JSON) ──► Motors & Battery ──► Custom Contact Models      │
│  (Links, Joints)        (Implicit DC)        (Shooter Squeeze, Intake)  │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                    ┌───────────────▼───────────────┐
                    │  Thin Physics Adapter Layer   │
                    │  (Winner of Week 1 Bake-Off:  │
                    │    Jolt OR PhysX 5 Native)    │
                    └───────┬───────────────┬───────┘
                            │               │
        ┌───────────────────▼───┐       ┌───▼───────────────────────────┐
        │   Headless Sim Runner │       │  Limelight PBR Renderer (GL)  │
        │  - Fleet Mode (15-20x │       │  - glTF + PBR Metallic/Rough  │
        │    1-robot instances) │       │  - Venue Lighting & Reflections│
        │  - Match Mode (1x     │       │  - Motion Blur, Distortion,   │
        │    6-robot instance)  │       │    Auto-Exposure, Sensor Noise│
        └───────────────────────┘       └───────────────────────────────┘
```

---

## 3. Revised 10-Week Schedule

### Week 1 (Oct 19–25): The Jolt vs. PhysX 5 Bake-Off
Build one minimal test harness that links both **Jolt** and **PhysX 5** and runs 5 identical headless tests against a shared 100-line C++ analytical reference:
1. **Build & Ergonomics Test:** Set up both in CMake on your PC. Note compile times, debugging ease, and API friction.
2. **45° Coulomb Friction Test:** Slide a block at 0° and 45° and measure stopping distance error against $s = \frac{v_0^2}{2\mu g}$.
3. **Swerve Mass-Ratio Test:** 50 kg chassis resting on four 1 kg wheel cylinders via steer + drive joints under full stall torque for 60 s simulated time. Measure jitter and energy drift.
4. **Intake Surface-Velocity Test:** Pull a sphere and a compound ring into a hopper using tangential surface velocity at real intake speeds.
5. **Shooter Compression Prototype:** Fire a ball and a ring through a spinning flywheel + hood gap using a nonlinear spring-damper squeeze force and Coulomb slip friction. Compare exit velocity, spin, and wheel RPM sag against the 1D analytical reference, and time the per-step cost for 1 robot + 20 sleeping pieces.

- **End of Week 1 Decision:** Pick the winner, delete the losing engine's code, and commit 100% to the winner's native API for Weeks 2–10.

---

### Week 2 (Oct 26–Nov 1): Motors, Battery, and Full Swerve Drivetrain
- **Implicit DC Motor + Gearbox Model:** Stall torque, free speed, free current, stall current, gear ratio, efficiency, and reflected rotor inertia ($I_{\text{reflected}} = I_{\text{rotor}} \cdot G^2$). Solved implicitly inside the physics step so 100:1 gear ratios never oscillate.
- **Lumped Battery & Brownout Model:** $V_{\text{bus}} = V_{\text{oc}} - I_{\text{total}} R_{\text{int}}$, plus per-motor supply/stator current limits.
- **4-Module Swerve Chassis:** Steer + drive joints, configurable carpet-vs-wheel friction, and validation against expected max speed, acceleration, and rotational rate.
- **Done when:** A 4-module swerve robot drives, strafes, and spins smoothly with realistic voltage sag under hard acceleration.

---

### Week 3 (Nov 2–8): Production Shooter Compression & Intake System
- ** Shooter Contact Model:**
  - Nonlinear compression curve: $F_n = k \cdot \delta^n + c \cdot \dot{\delta}$ (clamped so damping never pulls the piece onto the wheel during separation).
  - Coupled tangential slip/grip friction: $F_t = \min(\mu F_n, F_{\text{stick}})$, applying equal-and-opposite reaction torque $-\mathbf{r} \times \mathbf{F}_t$ to the flywheel so RPM sags realistically.
  - Multi-point support for non-spherical game pieces (donuts/rings/cones built from convex compounds) so squeeze force integrates smoothly across sub-shapes without discrete jumps as piece segments cross the pinch point.
- **Flight Aerodynamics:** Air drag ($\frac{1}{2} \rho v^2 C_d A$) and **Magnus effect lift** ($\mathbf{F}_M \propto \boldsymbol{\omega} \times \mathbf{v}$) so backspin from single-wheel hooded shooters curves the trajectory accurately.
- **Intake Rollers & Indexing:** Surface-velocity and physical roller options with compliant guides.
- **Done when:** 2,000 shots across varying wheel RPMs and hood gaps match the reference integrator within 5% exit velocity and 10% RPM sag with zero NaNs or flyers.

---

### Week 4 (Nov 9–15): Linkages, Elevators, Arms & Robot Data Schema
- **Mechanisms:**
  - Multi-stage elevators (prismatic sliders + cascade/continuous rigging coupling).
  - Single-jointed and double-jointed arms with gravity torques and hard stops.
  - **Closed-loop four-bar linkages** under motor load.
- **JSON Robot & Mechanism Schema:** Clean data format defining links, mass/CoM/inertia, joints, gear ratios, motor presets (Kraken X60, Falcon 500, NEO, NEO Vortex), and intake/shooter contact zones. (This is the exact format your post-Week-10 Fusion script will output).
- **Sensors (Physics Side):** Encoders (with gear scaling and quantization), 6-axis IMU (gyro + accelerometer with configurable bias/noise), limit switches, and beam breaks (raycast/trigger).
- **Done when:** A complete robot with swerve, a four-bar intake, an elevator, and a hooded shooter loads from JSON and operates all mechanisms simultaneously without solver jitter.

---

### Week 5 (Nov 16–22): Field Loader, Game Pieces & 6-Robot Match Mode
- **Field Geometry System:** Data-driven loader for field walls, polycarbonate panels, ramps/stage structures, carpet zones, and scoring sensor volumes.
- **Game Piece Manager:** Configurable pools of balls, rings, or cones with aggressive sleep thresholds and CCD enabled when moving above threshold velocity.
- **Match Mode (6 Full Robots):** 6 complete robots colliding Bumper-to-Bumper (with compliant bumper material parameters) and interacting with 50–100 game pieces in one simulation.
- **Done when:** 6 full-fidelity robots can push each other around a populated field in Match Mode with stable bumper collisions and realistic pushing traction.

---

### Week 6 (Nov 23–29, Thanksgiving — Light Week): Fleet Runner (15–20 Sims @ 10x)
- **Multi-Instance Fleet Runner:** Thread-pool manager running 15–20 isolated, single-threaded 1-robot simulation instances across 7 CPU cores.
- **Fast State Snapshot & Reset:** Instant `reset(seed)` restoring initial binary state buffers instead of rebuilding scenes from scratch.
- **Telemetry & Profiling Pass:** Measure exact throughput on your 8-core CPU using your existing `RT-PhysicsCore` profiler. Tune sleep thresholds, solver iteration counts (e.g., 6 velocity / 2 position steps), and broadphase layers.
- **Done when:** 15–20 single-robot simulations sustain $\ge 10\text{x}$ real-time headless on your CPU with deterministic replay per seed.

---

### Week 7 (Nov 30–Dec 6): Visual Accuracy Part 1 — glTF Assets, PBR & Venue Lighting
*Why this matters for Limelight:* Vision pipelines fail in sim when lighting is too clean. Real FRC fields have shiny polycarbonate shields that reflect overhead lights, textured carpet, and colored alliance station LEDs that tint game pieces.
- **glTF 2.0 Scene & Mesh Loader:** Load textured field, robot, and game-piece meshes with full material properties.
- **PBR Shading Pipeline (OpenGL 4.x on RTX 3060):** Cook-Torrance metallic/roughness BRDF, normal mapping, and environment cubemap reflections (IBL) so polycarbonate, anodized aluminum, foam game pieces, and carpet have realistic specular highlights.
- **Gymnasium / Arena Lighting:** Multiple overhead area/spot lights with shadow mapping + alliance-colored emissive polycarbonate lighting.
- **Done when:** The field, robot, and game pieces render with realistic materials, shadows, and reflections.

---

### Week 8 (Dec 7–13): Visual Accuracy Part 2 — Exact Limelight Optics & Camera Artifacts
*Why this matters for Limelight:* AprilTag pose estimation (`MegaTag` / `MegaTag2` / `solvePnP`) is highly sensitive to lens distortion, shutter blur during fast swerve rotation, and pixel contrast.
- **Offscreen Multi-Camera FBO Pipeline:** Render 1 or more robot-mounted cameras headless directly to GPU framebuffers at exact Limelight resolutions (e.g., 1280×960 or 640×480).
- **Exact Camera Intrinsics & Lens Distortion:** Configurable focal length ($f_x, f_y$), principal point ($c_x, c_y$), and Brown-Conrady radial/tangential distortion ($k_1, k_2, k_3, p_1, p_2$) applied in a post-process shader.
- **Motion Blur & Shutter Model:** Velocity-buffer camera + object motion blur shader driven by the physics engine's exact linear and angular velocities ($\mathbf{v}, \boldsymbol{\omega}$). Supports both global shutter (LL3G) and rolling shutter (LL3/LL2) smear.
- **Sensor Pipeline:** Configurable exposure/gain, bloom/glare around bright LEDs, Bayer/compression artifacts, and shot/read noise.
- **Done when:** Spinning the swerve chassis at 360°/s visibly blurs the image according to exposure time, and lens distortion matches a real Limelight calibration matrix.

---

### Week 9 (Dec 14–20): Visual Accuracy Part 3 — AprilTag & Detector Validation + Async GPU Readback
- **High-Precision AprilTag Field Placement:** Exact 16h5 / 36h11 AprilTag textures rendered with sharp mipmap-clamped edges and retroreflective/matte material response.
- **Vision Ground-Truth vs. Estimated Pose Harness:** Feed the rendered camera frames into an actual AprilTag detector (or WPILib `apriltag` / OpenCV `solvePnP`) and compare the detected 3D pose against the physics engine's ground-truth camera pose across distance, viewing angle, motion blur, and lighting glare.
- **Async PBO Readback & Latency Buffer:** Double-buffered OpenGL Pixel Buffer Objects (`GL_PIXEL_PACK_BUFFER`) so reading frames back to CPU (or feeding a detector) never stalls the GPU or slows down the physics threads, plus a ring buffer to simulate exact camera-to-robot latency (e.g., 20–45 ms) and frame drop jitter.
- **Done when:** An AprilTag detector running on the simulated camera stream reproduces realistic distance noise, angular ambiguity, and motion-blur dropout without stalling the simulation.

---

### Week 10 (Dec 21–27, Winter Holidays & AI Cutoff): Freeze, Tooling & Post-Cutoff Readiness
*Goal: Make sure you never need to touch complex engine or shader internals after your AI subscription ends.*
- **Clean C++ / Python Control API:** Finalize the clean struct/socket boundary (`SimInputs` / `SimOutputs` / `CameraFrame` / `reset()` / `step()`) so wiring up WPILib HALSim, your Fusion exporter, or an RL script later is trivial glue code.
- **Calibration CLI & ImGui Inspector:** Build an interactive tuning panel in the dev viewer where you can tweak shooter stiffness ($k, n, c$), friction ($\mu$), camera exposure/distortion, and motor constants with sliders and immediately see trajectory arcs and RPM curves.
- **Full Documentation & Soak Tests:** 24-hour soak test on Fleet Mode and Match Mode; document every JSON schema and shader uniform.

---

## 4. Updated Cut Order (Protecting the Week 10 Cutoff)

If any week runs long, cut from the bottom up so the core physics and Limelight rendering are 100% finished before Week 10:
1. **Cut first:** Rolling-shutter distortion (keep global shutter + velocity motion blur, since modern Limelight 3G is global shutter anyway).
2. **Cut second:** Complex cascade rigging helpers (keep standard single-stage slider/hinge joints with gear ratios).
3. **Cut third:** Shadow cascades for 15+ overhead lights (keep 2–4 main shadow-casting arena lights + baked ambient cubemap).
4. **Never cut:** Week 1 Bake-Off, Implicit Swerve + Motors (Week 2), Shooter Compression + Magnus Flight (Week 3), Fleet & Match Runners (Weeks 5–6), and PBR + Intrinsics + Motion Blur + AprilTags (Weeks 7–9).
