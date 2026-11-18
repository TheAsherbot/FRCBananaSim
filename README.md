# FRC Banana Simulator 🍌

[![C++20](https://img.shields.io/badge/Language-C%2B%2B20-00599C?logo=c%2B%2B)](https://en.wikipedia.org/wiki/C%2B%2B20)
[![Physics](https://img.shields.io/badge/Physics-Jolt%20Physics-orange)](https://github.com/jrouwe/JoltPhysics)
[![Graphics](https://img.shields.io/badge/Graphics-OpenGL%203.3%20Core-blue?logo=opengl)](https://www.opengl.org/)
[![UI](https://img.shields.io/badge/GUI-Dear%20ImGui-7B68EE)](https://github.com/ocornut/imgui)
[![WPILib](https://img.shields.io/badge/Bridge-WPILib%20HALSim%20WebSocket-red)](https://github.com/wpilibsuite/allwpilib)
[![Tests](https://img.shields.io/badge/Tests-23%2F23%20Passing%20(100%25)-success)](Tests/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20x64-brightgreen?logo=windows)](https://github.com/)

A high-fidelity, high-performance 3D robotics simulation platform and arcade game engine for **FIRST Robotics Competition (FRC)** robots. Built from scratch in modern C++20 on top of **Jolt Physics**, custom Entity-Component-System (ECS) architecture, and the official **WPILib HALSim WebSocket Protocol**.

Whether you want to test and tune autonomous robot code at blinding speeds (up to 100x+ headless) or jump straight into the driver's seat of an FRC robot in the built-in **Robot Comp 2 Shootout Arcade Game**, FRC Banana Simulator gives you instant, plug-and-play simulation.

---

## 📸 Highlights

- **🍌 Built-in Robot Comp 2 Arcade Shootout**: Jump straight into the driver station with an arcade rush timer, combo multipliers, miss penalty tracking, and persistent high score saves.
- **⚡ Deterministic 200 Hz Engine**: Fixed-timestep sub-millisecond physics ticks using Jolt Physics, featuring realistic chassis mass distribution, wheel contact patches, tire saturation slip vectors, and ground traction.
- **🔄 High-Fidelity Swerve Dynamics**: 4-wheel independent steering and driving kinematics, wheel scrub modeling, chassis tilt/roll stabilization, and high-traction carpet physics.
- **🏀 Realistic Game Piece Pipeline**: 6-inch foam Cargo ball dynamics, conveyor translation velocities, non-uniform staggered hopper basket stacking (up to 22 balls capacity), and dual-roller flywheel shooting with aerodynamic drag and lift.
- **🔌 WPILib Driver Station & HALSim Bridge**: Built-in WebSocket server (`:3300/wpilibws`) communicating directly with `./gradlew simulateNative`.
- **🛠️ Schema v1 CAD Ingestion**: Full pipeline from Autodesk Fusion 360 CAD exports into `manifest.json`, `sidecar.json`, and `materials.json`.
- **🚀 100% Portable & Self-Contained**: Fallback robot schemas are embedded directly into the binary—download the standalone `.exe` and play immediately anywhere with zero installation!

---

## 🎮 Arcade Shootout Game

When launched without flags, the simulator boots directly into the **Robot Comp 2 Arcade Shootout**:

```
+------------------------------------------------------------------------------------+
|                                                                                    |
|   [ MATCH TIME: 00:54 ]        [ SCORE: 48,000 PTS ]        [ x12 COMBO STREAK! ]  |
|                                                                                    |
|                                    [ 🎯 TARGET ]                                   |
|                                      ( 4.8 m )                                     |
|                                                                                    |
|                                                                                    |
|                                     🏀 🏀  (Airborne Shots)                        |
|                                                                                    |
|                                   [ ROBOT COMP 2 ]                                 |
|                               +----------------------+                             |
|                               | [Flywheel] 7,640 RPM |                             |
|                               | [Funnel]   Indexing  |                             |
|                               | [Hoop Basket] 8 / 18 |                             |
|                               | [Intake Tray] 3 / 3  |                             |
|                               +----------------------+                             |
|                                                                                    |
|   +----------------------------------------------------------------------------+   |
|   | BALL STORAGE: 11 / 22  |  [SPACE] INTAKE  |  [F/RMB] FUNNEL  |  [LMB] FIRE |   |
|   +----------------------------------------------------------------------------+   |
+------------------------------------------------------------------------------------+
```

### Modes
1. **Arcade Rush (60s Challenge)**: A timed competition round. Score as many balls as possible into the Alliance Upper Hub goal before time runs out. Stack consecutive hits to build high combo multipliers (up to x10+ points per goal!). If you miss a shot or hit the ground, your combo resets with a dynamic `"COMBO LOST!"` indicator.
2. **Free Practice**: Unlimited practice time. Drive freely around the arena, test mechanism speeds, practice bank shots, and reset the field at any time with the `R` key.

### Persistent High Scores
Your record score and best combo streak are automatically saved to disk (`highscore.json`) and displayed on both the title screen and the in-game HUD ribbon.

---

## 🕹️ Controls & Keybindings

| Key / Input | Action | Description |
| :--- | :--- | :--- |
| **`W` / `S`** or **`↑` / `↓`** | Drive Forward / Backward | Field-centric translational drive (+Z / -Z) |
| **`A` / `D`** or **`←` / `→`** | Strafe Left / Right | Field-centric lateral drive (+X / -X) |
| **`Q` / `E`** | Rotate / Yaw Robot | Turn robot chassis heading counter-clockwise / clockwise |
| **`Space`** or **`Left Shift`** | Deploy Intake & Spin Rollers | Lowers the intake pivot and pulls Cargo balls into the front tray |
| **`F`**, **`Right Click`**, or **`V`** | Run Feeder Funnel | Feeds balls from the hopper basket upward into the flywheel feeder |
| **`Left Click`** or **`C`** | Spin Flywheel & Shoot | Spins up dual flywheel rollers and launches balls into the goal |
| **`Z`** | Sprint / Stride Boost | Temporarily boosts chassis drive speed |
| **`Tab`** | Switch Camera Mode | Toggles between **Follow Robot** and **Free-Fly / Orbit Camera** |
| **`Right-Click + Mouse Drag`** | Orbit Camera Look | Swing camera freely around the robot |
| **`Scroll Wheel`** | Zoom Camera | Zoom camera in and out |
| **`Esc`** | Pause / Resume | Opens the in-game pause menu |
| **`R`** | Reset Field | Re-scatters balls around the arena (Free Practice / Menus) |

---

## ⚙️ Robot Storage & Mechanism Pipeline

Robot Comp 2 features a physically modeled 4-stage internal ball transfer system:

```
[ Field Carpet ]
       │
       ▼  (Intake Pivot Down + Intake Rollers Active)
[ 3-Slot Intake Tray ]  (Front lower bumper tray)
       │
       ▼  (Conveyor translation speed: ~2.6 m/s)
[ 18-Slot Basket Hopper ]  (Stacked 3 to 4 wide, 3 layers tall, staggered jitter)
       │
       ▼  (Feeder Funnel active)
[ Funnel Guide Neck ]  (Pre-indexes 1 ball right below flywheel nip)
       │
       ▼  (Flywheel active: ~7,600 RPM)
[ Dual-Roller Flywheel Hood ]  (Launches ball with realistic backspin & trajectory)
```

- **Intake Interlock**: Balls can only be intaken when the intake pivot is deployed down.
- **No Teleporting**: Balls smoothly translate from the carpet into the intake tray, transfer along the indexer rollers into the hopper basket, and feed up the vertical funnel at realistic conveyor speeds.
- **Natural Stacking**: Hopper balls stack naturally across 3 layers with subtle randomized offset jitter, mimicking real FRC foam ball compression.
- **Capacity Overflow Protection**: If the hopper basket is full (18 balls), balls queue up in the intake tray (3 balls). Once the intake tray is full, additional balls bounce off the front rollers.

---

## 🌐 WPILib HALSIM Integration

FRCBananaSim includes a full **WPILib HALSim WebSocket Server** running on port `:3300/wpilibws`.

### Running with Real Robot Code
1. Start the simulator in test-station mode:
   ```powershell
   ./FRCBananaSim.exe --test-station
   ```
2. In your WPILib Java or C++ robot project directory:
   ```bash
   ./gradlew simulateNative
   ```
3. Connect your official WPILib Driver Station. The simulator will automatically detect the connection, exchange CAN motor signals, encoder positions, and limit switch inputs in real-time lockstep!

---

## 🏗️ Architecture & Tech Stack

```
FRCBananaSim/
├── Source/
│   ├── Actuation/          # Electrical system, battery model, terminal voltage sag
│   ├── Bridge/             # WPILib HALSim WebSocket server (:3300/wpilibws)
│   ├── Control/            # Motor boundaries, PWM servos, driver station inputs
│   ├── Core/               # Engine loop, fixed-timestep scheduler, ECS Scene & Systems
│   ├── Game/               # BananaGame arcade state machine, HUD, scoring, audio cues
│   ├── Interaction/        # Cargo ball physics, aerodynamics (Magnus lift), compression
│   ├── Physics/            # Jolt Physics context, rigid bodies, colliders, joints
│   ├── Renderer/           # OpenGL 3.3 Core shaders, camera, procedural primitive meshes
│   ├── Robot/              # Schema v1 loader, robot assembler, swerve drive, mechanisms
│   ├── Telemetry/          # 200 Hz telemetry recorder, profiler window, CSV/FTC export
│   └── Utils/              # DebugDraw immediate mode renderer, logger, math helpers
├── Resources/
│   ├── Config/             # materials.json (friction/restitution), highscore.json
│   └── Robots/             # CAD Schema v1 files (manifest.json, sidecar.json)
├── FusionAddin/            # Autodesk Fusion 360 automated robot exporter script
└── Tests/                  # 23 automated unit and endurance regression test suites
```

---

## 🛠️ Building from Source

### Prerequisites
- **OS**: Windows 10/11 (64-bit)
- **Compiler**: Visual Studio 2022 (MSVC v143) with C++20 support
- **CMake**: Version 3.20 or newer
- **Git**: For cloning submodules / third-party dependencies

### Build Steps

```powershell
# 1. Clone repository
git clone https://github.com/YourUsername/FRCBananaSim.git
cd FRCBananaSim

# 2. Configure with CMake (generates Visual Studio 2022 solution)
cmake -B build -S .

# 3. Build Release configuration
cmake --build build --config Release --target FRCBananaSim

# 4. Run automated test suite (23 tests)
ctest --test-dir build -C Release --output-on-failure
```

### Packaging a Release Bundle
To package a standalone `.zip` distribution containing `FRCBananaSim.exe`, debug symbols (`.pdb`), and the `Resources/` folder:

```powershell
python package_release.py
```
This produces `build/FRCBananaSim-Release-Windows-x64.zip`.

---

## 🧭 Coordinate Frame Convention

Across all 3D viewports, CAD exports, and simulation dynamics:
- **`+X`**: Right side of the robot (`-X` is Left)
- **`+Y`**: Up towards the ceiling (`-Y` is Down towards the carpet floor)
- **`+Z`**: Front / Forward of the robot (`-Z` is Back / Reverse)

*(Converts bidirectionally to/from WPILib's standard North-West-Up [NWU] frame automatically).*

---

## 📚 Documentation & Guides

- [Configuration & Calibration Guide](Configuration%20%26%20Calibration%20Guide.md) — Comprehensive guide to mechanical parameters, electrical models, motor curves, and contact pair friction tables.
- [Sidecar Configurator & CAD Ingestion Guide](SIDECAR_CONFIGURATOR_README.md) — Manual for defining mechanisms, collision hulls, joint limits, and sensor wiring.
- [Fusion 360 Add-in Guide](FusionAddin/README.md) — Script for exporting CAD assemblies into Schema v1 `manifest.json`.

---

## 📄 License & Credits

Distributed under the MIT License. See `LICENSE` for more information.

- **Physics**: [Jolt Physics](https://github.com/jrouwe/JoltPhysics) by Jorrit Rouwe
- **Rendering & Context**: [GLFW](https://www.glfw.org/) & [GLAD](https://glad.dav1d.de/)
- **GUI**: [Dear ImGui](https://github.com/ocornut/imgui) by Omar Cornut
- **JSON**: [nlohmann/json](https://github.com/nlohmann/json) by Niels Lohmann
- **Math**: [GLM (OpenGL Mathematics)](https://github.com/g-truc/glm)
