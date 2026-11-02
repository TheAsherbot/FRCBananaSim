# FRCBananaSim Robot Configuration & Calibration Guide

> [!NOTE]
> **Status:** Block 2 & Block 3 Verified — 6-DOF Analytic Swerve Dynamics, Jolt Rigid Body Chassis, Tire Contact Model, Battery Electromechanics, Robot Data Schema v1.0.0 (`manifest.json`, `sidecar.json`, `materials.json`), Multi-Stage Elevators, Rotational Arms & Wrists, Closed-Loop Four-Bar Linkages, High-RPM Flywheels, Full Sensor Suite (`CANcoder`, 6-Axis `Pigeon 2` IMU, Limit Switches, Beam Breaks, Analog Inputs), and CTRE Phoenix 6 / REVLib HALSim WebSocket Lockstep Bridge.
> This document is the living reference guide for modeling, measuring, and configuring robots in **FRCBananaSim**. It strictly documents features and parameters that are **currently implemented and parsed** by the simulation engine, covering both **Block 2 Standalone Swerve (`robot_swerve.json`)** and **Block 3 Full-Robot Schema v1 (`manifest.json`, `sidecar.json`, `materials.json`)**.

---

## 1. System Overview & The Configuration Files

**FRCBananaSim** supports two configuration workflows:

### A. Block 2 Standalone Swerve Configuration (`robot_swerve.json`)
All standalone swerve drivetrain configuration data resides in:
📁 [`FRCBananaSim/Resources/Robots/robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json)

When launched in standalone swerve mode, [`RobotSpawner::SpawnFromJsonFile`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotSpawner.cpp) and [`SwerveDrivetrainConfig::LoadFromJsonString`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Drivetrain/Model/SwerveParams.cpp) parse this JSON file into physical entities:
1. **Chassis Rigid Body**: A single dynamic Jolt physics body with 6 degrees of freedom (surge, sway, heave, roll, pitch, yaw), custom mass, and custom $3 \times 3$ inertia tensor.
2. **Analytic Swerve Drivetrain**: 4 module contact points evaluating tire brush friction, Stribeck transitions, and normal force load transfers.
3. **Electromechanical Actuation**: DC motor torque curves, gear ratios, drivetrain friction, and buck-converter battery sag models.
4. **HALSim WebSocket Bridge**: Translates CTRE Phoenix 6 network packets to and from your robot code.

```
┌────────────────────────────────────────────────────────┐
│                   robot_swerve.json                    │
└───────────────────────────┬────────────────────────────┘
                            │
            ┌───────────────┴───────────────┐
            ▼                               ▼
  [RobotSpawner.cpp]               [SwerveParams.cpp]
  • Chassis Jolt Rigid Body        • 6-DOF Analytic Dynamics
  • Battery Component              • Motor Torque/Back-EMF
  • Visual Scene Hierarchy         • Tire Brush Friction Model
            │                               │
            └───────────────┬───────────────┘
                            ▼
              [HALSimWebSocketServer.cpp]
              • CAN 0,2,4,6  Drive Motors
              • CAN 1,3,5,7  Steer Motors
              • CAN 20..23   Steer CANcoders
              • CAN 24       Pigeon 2 IMU
```

### B. Block 3 Full-Robot Schema v1.0.0 (`manifest.json`, `sidecar.json`, `materials.json`)
For complete multi-mechanism robots, configuration data is partitioned into three modular JSON files loaded by [`RobotSchemaLoader`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Schema/RobotSchemaLoader.cpp) and assembled by [`RobotAssembler::AssembleFromFiles`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotAssembler.cpp):
* 📁 [`FRCBananaSim/Resources/Robots/Reference2026/manifest.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/Reference2026/manifest.json) — Physical CAD hierarchy (links, masses, CoMs, full $3 \times 3$ inertia tensors, colliders, joints, and sensor contact zones).
* 📁 [`FRCBananaSim/Resources/Robots/Reference2026/sidecar.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/Reference2026/sidecar.json) — Electromechanical actuation, swerve parameters, mechanism controllers, kinematic couplings, and hardware sensors.
* 📁 [`FRCBananaSim/Resources/Config/materials.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Config/materials.json) — Surface friction, restitution, density, and pairwise contact overrides.
* 📁 [`FRCBananaSim/Resources/Config/simdevice_keymap.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Config/simdevice_keymap.json) — Configurable vendor property key aliases for [`SimDeviceDispatcher`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Bridge/SimDeviceDispatcher.cpp).

```
┌───────────────────────────┐   ┌───────────────────────────┐   ┌───────────────────────────┐
│       manifest.json       │   │       sidecar.json        │   │      materials.json       │
└─────────────┬─────────────┘   └─────────────┬─────────────┘   └─────────────┬─────────────┘
              └───────────────────────────────┼───────────────────────────────┘
                                              ▼
                                 [RobotSchemaLoader.cpp]
                          NWU (+X Fwd, +Y Left, +Z Up) ──► Engine (+X Left, +Y Up, +Z Fwd)
                                              │
                                              ▼
                                   [RobotAssembler.cpp]
                                              │
       ┌──────────────────────────────────────┴──────────────────────────────────────┐
       ▼                                                                             ▼
  [8-Stage 200 Hz FixedUpdate Pipeline]                                 [HALSim Bridge & Lockstep]
  1. ElectricalSystem     (Buck-converter supply current & V_bus sag)   • HALSimWebSocketServer (:3300)
  2. ElevatorSystem       (Slider forces, reflected mass, cascade)      • SimDeviceDispatcher
  3. MechanismSystem      (Arms, wrists, 4-bar PD sync, couplings)      • LockstepCoordinator (20 ms)
  4. FlywheelSystem       (800 Hz 1D ODE, parent recoil, wrist mount)   • OnControllerPIDModel (1 kHz)
  5. CompositeMassSystem  (Dynamic CG & Parallel Axis inertia -> Swerve)
  6. SwerveDriveSystem    (6-DOF analytic swerve + Jolt terrain probe)
  7. PhysicsSystem        (Jolt 5 ms step + CoM-to-origin sync)
  8. SensorSystem         (Encoders, 6-axis IMU, DIO limits & beams)
```

### Coordinate System & Unit Conventions
All JSON files (`robot_swerve.json`, `manifest.json`, `sidecar.json`) use **SI Units** ($\text{m}$, $\text{kg}$, $\text{s}$, $\text{rad}$, $\text{V}$, $\text{A}$) and the **WPILib NWU (North-West-Up)** right-handed coordinate frame ($+X$ Forward, $+Y$ Left, $+Z$ Up, CCW-positive yaw). At load time, [`RobotSchemaLoader`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Schema/RobotSchemaLoader.cpp#L24-L60) and [`SwerveDriveSystem`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Drivetrain/Systems/SwerveDriveSystem.cpp#L68-L115) apply a proper right-handed orthogonal rotation ($\det = +1$) into the internal **Engine / OpenGL Y-Up Frame** ($+X$ Left, $+Y$ Up, $+Z$ Forward):
$$\mathbf{v}_{\text{engine}} = \begin{pmatrix} +v_{y, \text{NWU}} \\ v_{z, \text{NWU}} \\ v_{x, \text{NWU}} \end{pmatrix}, \qquad \mathbf{v}_{\text{NWU}} = \begin{pmatrix} v_{z, \text{engine}} \\ +v_{x, \text{engine}} \\ v_{y, \text{engine}} \end{pmatrix}$$
For revolute joints pitching around the lateral axis (NWU `[0, 1, 0]`), [`RobotAssembler`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotAssembler.cpp#L252-L261) negates `axisEngine.x` so positive joint angles pitch the mechanism **upward** ($+X_{\text{NWU}} \to +Z_{\text{NWU}}$).

---

## 2. Parameter Extraction from CAD (Block 2 Swerve & Chassis)

Extract these values directly from your robot CAD assembly (Onshape, SolidWorks, Fusion 360, or Inventor). Every value listed here maps directly into [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json) (and into `"base_link"` / `"swerve"` in Schema v1).

### A. Chassis Frame Geometry
* **Where to find it in CAD:** Measure the outer frame perimeter and module centers with the CAD tape/measure tool.
* **Exact JSON Targets in `robot_swerve.json` (`"chassis"` object):**
  * `"trackwidth_m"`: Distance between the center of the left wheels and right wheels along the robot's lateral axis (e.g. `0.56`).
  * `"wheelbase_m"`: Distance between the center of the front wheels and back wheels along the robot's longitudinal axis (e.g. `0.56`).
  * `"width_m"`: Outer bumper/frame width along the lateral axis (e.g. `0.70`).
  * `"length_m"`: Outer bumper/frame length along the longitudinal axis (e.g. `0.70`).
  * `"height_m"`: Height of the chassis frame box (e.g. `0.15`).
* **Config Snippet:**
  ```json
  "chassis": {
    "width_m": 0.70,
    "height_m": 0.15,
    "length_m": 0.70,
    "wheelbase_m": 0.56,
    "trackwidth_m": 0.56
  }
  ```

### B. Center of Mass & Inertia Tensor
* **Where to find it in CAD:** Select the top-level robot assembly (with battery, electronics, and bumpers modeled or mass-overridden) and open the CAD **Mass Properties** tool.
* **Exact JSON Targets in `robot_swerve.json` (`"chassis"` object):**
  * `"cgHeight_m"`: Vertical distance ($Z$) from the ground plane to the Center of Mass (e.g. `0.20`).
  * `"cgOffset_m"`: `[ dx, dy ]` horizontal offset of the Center of Mass from the geometric center of the drive base (e.g. `[ 0.0, 0.0 ]`).
  * `"inertiaTensor_kgm2"`: The $3 \times 3$ principal moments of inertia taken at the Center of Mass ($I_{xx}, I_{yy}, I_{zz}$).
* **Config Snippet:**
  ```json
  "chassis": {
    "cgHeight_m": 0.20,
    "cgOffset_m": [ 0.0, 0.0 ],
    "inertiaTensor_kgm2": [
      [ 2.1354, 0.0, 0.0 ],
      [ 0.0, 2.1354, 0.0 ],
      [ 0.0, 0.0, 4.0833 ]
    ]
  }
  ```
  *(Note: BananaSim automatically converts standard NWU CAD inertia coordinates to the engine coordinate system in `ChassisParams::GetJoltInertiaTensor()` and `RobotSchemaLoader::NWUToEngineInertiaTensor()`.)*

### C. Swerve Module Locations
* **Where to find it in CAD:** Measure the $(X, Y)$ coordinates of the steering axis of each module relative to the robot's center origin.
* **Exact JSON Targets in `robot_swerve.json` (`"modules"` array):**
  * `modules[i].location_m`: `[ X, Y ]` in meters.
* **Config Snippet:**
  ```json
  "modules": [
    { "index": 0, "name": "FrontLeft",  "location_m": [  0.28,  0.28 ] },
    { "index": 1, "name": "FrontRight", "location_m": [ -0.28,  0.28 ] },
    { "index": 2, "name": "BackLeft",   "location_m": [  0.28, -0.28 ] },
    { "index": 3, "name": "BackRight",  "location_m": [ -0.28, -0.28 ] }
  ]
  ```

### D. Module Gearing & Steer Column Geometry
* **Where to find it in CAD:** Inspect the gear tooth counts or module assembly models:
  * Motor shaft pinion $\to$ intermediate gears $\to$ wheel / azimuth ring gear.
* **Exact JSON Targets in `robot_swerve.json`:**
  * `"motors.drive.gearRatio"`: Total reduction from drive motor to wheel (e.g. `5.27` for MK4n / MK5n L3).
  * `"motors.steer.gearRatio"`: Total reduction from steer motor to azimuth ring (e.g. `26.0909` for MK4n / MK5n).
  * `"steerHousing.mass_kg"`: Mass of the turning steer assembly column (e.g. `0.45`).
  * `"steerHousing.radius_m"`: Radius of the turning steer column (e.g. `0.04`).
* **Config Snippet:**
  ```json
  "motors": {
    "steer": {
      "type": "KrakenX44",
      "gearRatio": 26.0909
    },
    "drive": {
      "type": "KrakenX60",
      "gearRatio": 5.27
    }
  },
  "steerHousing": {
    "mass_kg": 0.45,
    "radius_m": 0.04
  }
  ```

---

## 3. Physical Hardware Measurements & Calibration

Perform these physical checks on the real robot to calibrate parameters that differ from pure theoretical CAD.

### A. Total Mass Calibration
* **Tool:** Scale (or 4 corner scales summed together).
* **Condition:** Robot must be weighed with battery installed, main breaker on, and bumpers mounted.
* **JSON Fields:** `chassis.mass_kg` (in `robot_swerve.json`) or `links[i].mass_kg` and `massSummary.totalMass_kg` (in `manifest.json`).

### B. Wheel Radius (Tread Wear)
* **Tool:** Digital calipers.
* **Procedure:** Measure the outer diameter across the tread on all four wheels while mounted. Standard 4-inch wheels with fresh tread measure $101.6\text{ mm}$ ($r = 0.0508\text{ m}$), but worn nitrile/billet tread frequently wears down to $96 - 98\text{ mm}$ ($r = 0.0480 - 0.0490\text{ m}$).
* **JSON Fields:** `wheel.radius_m` (in `robot_swerve.json`) or `swerve.wheelRadius_m` (in `sidecar.json`).

### C. Battery Internal Resistance & Voltage
* **Tool:** Battery Beak or 12V battery analyzer.
* **Procedure:** Measure a fresh competition battery:
  * Open-circuit voltage ($V_{\text{nominal}}$, typically $12.60\text{ V} - 12.80\text{ V}$).
  * Internal resistance ($R_{\text{int}}$, typically $0.018 - 0.024\ \Omega$).
* **JSON Fields:**
  * `battery.nominalVoltage_V` / `electrical.openCircuitVoltage_V`
  * `battery.internalResistance_ohm` / `electrical.internalResistance_Ohm`

### D. Drivetrain & Mechanism Mechanical Friction Calibration
* **Setup:** Place the robot on blocks with wheels (or mechanism) unloaded in the air.
* **Procedure:** Command the motors to full speed ($12\text{ V}$) via Phoenix Tuner X or manual test code.
  * Measure steady-state velocity ($\text{RPS}$).
  * A Kraken X60 has a theoretical free speed of $\sim 6000\text{ RPM}$ ($100\text{ RPS}$). If the rotor reaches $94\text{ RPS}$ in the air, the drivetrain consumes $\sim 6\%$ in friction.
  * Measure the minimum breakaway voltage $V_{\text{break}}$ required to initiate motion from rest: $\tau_{\text{coulomb}} \approx G \eta \tau_{\text{stall}} (V_{\text{break}} / 12.0)$.
* **JSON Fields:**
  * `motors.drive.efficiency` / `mechanisms[i].efficiency` (typically $0.90 - 0.97$)
  * `driveModule.coulombFriction_Nm` / `mechanisms[i].coulombFriction` (typically $1.5 - 2.0\text{ Nm}$ for drive, $0.3 - 0.8\text{ Nm}$ for arms, $10 - 15\text{ N}$ for elevators)
  * `mechanisms[i].springAssist.force_N`: Measure constant-force spring pull on the elevator carriage with a fish/luggage scale (e.g. $45.0\text{ N}$).

---

## 4. Electronics, CAN Bus & Sensor Configuration

FRCBananaSim's network bridge communicates directly with CTRE Phoenix 6 and REVLib devices over the WPILib HALSim WebSocket protocol.

### A. Swerve & Reference 2026 CAN Bus ID Mapping
Verify that the CAN IDs in [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json) or [`sidecar.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/Reference2026/sidecar.json) match your physical robot:

| Component | Physical Device | `robot_swerve.json` Path | `sidecar.json` Path | Default ID | `SimInputs` Slot |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Front Left Steer** | Talon FX (v6) | `modules[0].steerCAN` | `swerve.modules[0].steerCanId` | `1` | `0` |
| **Front Right Steer** | Talon FX (v6) | `modules[1].steerCAN` | `swerve.modules[1].steerCanId` | `3` | `1` |
| **Back Left Steer** | Talon FX (v6) | `modules[2].steerCAN` | `swerve.modules[2].steerCanId` | `5` | `2` |
| **Back Right Steer** | Talon FX (v6) | `modules[3].steerCAN` | `swerve.modules[3].steerCanId` | `7` | `3` |
| **Front Left Drive** | Talon FX (v6) | `modules[0].driveCAN` | `swerve.modules[0].driveCanId` | `0` | `4` |
| **Front Right Drive** | Talon FX (v6) | `modules[1].driveCAN` | `swerve.modules[1].driveCanId` | `2` | `5` |
| **Back Left Drive** | Talon FX (v6) | `modules[2].driveCAN` | `swerve.modules[2].driveCanId` | `4` | `6` |
| **Back Right Drive** | Talon FX (v6) | `modules[3].driveCAN` | `swerve.modules[3].driveCanId` | `6` | `7` |
| **Elevator Leader / Follower**| Talon FX (v6) | — | `mechanisms[0].controllers` | `10`, `11` | `8` |
| **Arm Shoulder** | Talon FX (v6) | — | `mechanisms[1].controllers` | `12` | `9` |
| **Arm Wrist** | SPARK MAX | — | `mechanisms[2].controllers` | `13` | `10` |
| **Intake Four-Bar Deploy** | Talon FX (v6) | — | `mechanisms[3].controllers` | `14` | `11` |
| **Shooter Flywheel L / F** | Talon FX (v6) | — | `mechanisms[4].controllers` | `15`, `16` | `12` |
| **Swerve Steer CANcoders** | CANcoder (v6) | `modules[0..3].canCoder` | `swerve.modules[0..3].cancoderId` | `20..23` | `CANcoder 0..3` |
| **Shoulder CANcoder** | CANcoder (v6) | — | `sensors[1].canId` | `12` | `CANcoder 4` |
| **IMU Gyro** | Pigeon 2 (v6) | Handled in Bridge | `sensors[0].canId` | `24` | IMU Output |

### B. Current Limits & Buck-Converter Supply Current
* **Where to find it:** Check your robot code's `CurrentLimitsConfigs` on the Talons and SPARKs.
* **How BananaSim models it:** Both [`ElectricalSystem`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Actuation/Systems/ElectricalSystem.cpp) and [`SwerveDrivetrainModel`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Drivetrain/Model/SwerveDrivetrainModel.cpp) model motor controllers as PWM buck converters where battery supply current scales with duty cycle ($I_{\text{supply}} = \min(I_{\text{supply\_limit}}, |V_{\text{applied}} / V_{\text{bus}}| \cdot I_{\text{stator}})$) filtered through a $5\text{ ms}$ low-pass filter.
* **JSON Fields:**
  * `motors.drive.statorCurrentLimit_A`, `motors.steer.statorCurrentLimit_A`, `battery.statorCurrentLimit_A`, `battery.supplyCurrentLimit_A` (in `robot_swerve.json`)
  * `swerve.driveStatorLimit_A`, `swerve.driveSupplyLimit_A`, `swerve.steerStatorLimit_A`, `swerve.steerSupplyLimit_A`, `mechanisms[i].statorCurrentLimit_A`, `mechanisms[i].supplyCurrentLimit_A` (in `sidecar.json`)

### C. Vendor `SimDevice` Mapping Table (`SimDeviceDispatcher`)
[`SimDeviceDispatcher`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Bridge/SimDeviceDispatcher.cpp) translates WPILib HALSim WebSocket `SimDevice` frames using [`simdevice_keymap.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Config/simdevice_keymap.json):

| Vendor Device | `SimDevice` Pattern | Robot $\to$ Sim Keys (`>`) | Sim $\to$ Robot Keys (`<`) |
| :--- | :--- | :--- | :--- |
| **CTRE `TalonFX`** | `"TalonFX[<id>]"`, `"Talon FX[<id>]"`, `"TalonFX:<bus>[<id>]"` | `">motorVoltage"`, `">motorOutputLead"`, `">BridgeOutput"` | `"<supplyVoltage"`, `"<rawRotorPosition"`, `"<rotorVelocity"`, `"<rotorAcceleration"`, `"<torqueCurrent"`, `"<supplyCurrent"`, `"<forwardLimit"`, `"<reverseLimit"` |
| **CTRE `CANcoder`** | `"CANcoder[<id>]"`, `"CANcoder:<bus>[<id>]"` | *(none)* | `"<supplyVoltage"`, `"<rawPosition"` (rot), `"<velocity"` (RPS) |
| **CTRE `Pigeon2`** | `"Pigeon2[<id>]"`, `"Pigeon 2[<id>]"`, `"Pigeon2:<bus>[<id>]"` | *(none)* | `"<supplyVoltage"`, `"<rawYaw"` (deg), `"<pitch"` (deg), `"<roll"` (deg), `"<angularVelocityX/Y/Z"` (deg/s), `"<accelX/Y/Z"` ($g$) |
| **REV `SparkMax` / `SparkFlex`** | `"SPARK MAX [<id>]"`, `"SPARK Flex [<id>]"` | `">Applied Output"` | `"<Bus Voltage"`, `"<Motor Current"`, `"<Position"` (rot), `"<Velocity"` (RPM) |
| **Kauai Labs `navX`** | `"navX-Sensor[0]"` | *(none)* | `"<yaw"` (deg, CW+), `"<pitch"`, `"<roll"`, `"<LinearWorldAccelX/Y/Z"` |

---

## 5. Configuring Block 3 Mechanisms & Couplings (`manifest.json` & `sidecar.json`)

### A. Links, Colliders & Visual Mesh Generation
* **Procedural Visual Meshes from Colliders:** When a link in `manifest.json` does not load an external `.glb` mesh, [`RobotAssembler`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotAssembler.cpp#L200-L239) generates procedural visual box/sphere/cylinder meshes from `colliders[0].origin_m` and `colliders[0].dimensions_m`. Always define a collider on every visible link (including four-bar crank and follower links).
* **Center-of-Mass Offset Alignment:** When `centerOfMass_m` is non-zero, [`PhysicsSystem`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Physics/Systems/PhysicsSystem.cpp#L485-L505) automatically converts Jolt's CoM world position back to the link origin (`comPos - rot * localOffset`) so visual meshes and collider wireframes remain 100% aligned.

### B. Elevators (`SingleStage`, `Cascade`, `Continuous`)
* Configured via `"role": "elevator"` in `sidecar.json` (`"drumRadius_m"` converts motor torque into linear force and computes reflected linear rotor mass $m_{\text{reflected}} = N J_{\text{rotor}}(G/r_{\text{drum}})^2$).
* **Cascade Rigging (`"type": "elevator_cascade"`, `"solverMode": "analytic_coupled"`):** Stage 1 carries the full lumped mass and gravity load while upper stages track $x_k = k \cdot x_1$ analytically with zero drift.

### C. Arms, Wrists & Power Couplings
* Configured via `"role": "arm"` or `"role": "wrist"` in `sidecar.json`.
* **Active Motor Priority over Passive Coupling:** If a slave joint in a `"power_coupling"` entry (such as `shoulder_to_wrist_coupling`) also has its own motorized mechanism entry (such as `arm_wrist`), [`MechanismSystem::UpdateCoupling`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Mechanisms/Systems/MechanismSystem.cpp#L357-L380) automatically yields to the independent motor controller so the wrist can articulate freely at any shoulder angle.

### D. Closed-Loop Four-Bar Linkages
* **Avoid Collinear Singularity:** Place the ground pivots (`fourbar_crank_hinge` and `fourbar_follower_hinge`) with both a longitudinal ($X$) and vertical ($Z$) offset above the bumper (e.g., crank at `[0.40, 0.0, 0.19]` and follower at `[0.28, 0.0, 0.25]`, delta `[-0.12, 0.0, +0.06]`), matching `"closureParentAnchor_m": [0.24, 0.0, 0.0]` and `"closureChildAnchor_m": [-0.12, 0.0, 0.06]`.
* **Follower Synchronization:** [`MechanismSystem::UpdateFourBar`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Mechanisms/Systems/MechanismSystem.cpp#L218-L355) applies feed-forward + PD tracking torque to the follower bar during motion and locks both hinges in Jolt `Position` mode when stopped so the back bar never lags during folding.

### E. High-RPM Flywheels
* Configured via `"role": "flywheel"` in `sidecar.json`. Sub-stepped at $800\text{ Hz}$ ($\Delta t = 1.25\text{ ms}$) with equal-and-opposite Newton's 3rd Law recoil torque applied to `"parentLink"`, and rendered at `mountOffset` (`(0.0f, 0.06f, 0.06f)` on the wrist) via [`FlywheelSystem::RenderUpdate`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Mechanisms/Systems/FlywheelSystem.cpp#L118-L148).

---

## 6. Master Configuration Dictionary for [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json) (Block 2)

This table lists every single property read by [`RobotSpawner`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotSpawner.cpp) and [`SwerveDrivetrainConfig`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Drivetrain/Model/SwerveParams.cpp):

| JSON Key | C++ Struct Target | Unit | Description & Calibration Source |
| :--- | :--- | :--- | :--- |
| **`chassis.mass_kg`** | `ChassisParams::mass` | $\text{kg}$ | Total robot mass with battery & bumpers (from scales). |
| **`chassis.width_m`** | `ChassisParams::dimensions.y` | $\text{m}$ | Frame width (lateral span). |
| **`chassis.length_m`** | `ChassisParams::dimensions.x` | $\text{m}$ | Frame length (longitudinal span). |
| **`chassis.height_m`** | `ChassisParams::dimensions.z` | $\text{m}$ | Chassis physical collision box height. |
| **`chassis.trackwidth_m`** | `SwerveChassisComponent::trackWidth` | $\text{m}$ | Lateral distance between left & right wheel centers. |
| **`chassis.wheelbase_m`** | `SwerveChassisComponent::wheelBase` | $\text{m}$ | Longitudinal distance between front & rear wheel centers. |
| **`chassis.cgHeight_m`** | `ChassisParams::centerOfMassHeight` | $\text{m}$ | Height of Center of Mass above carpet (from CAD). |
| **`chassis.cgOffset_m`** | `ChassisParams::centerOfMassOffset` | $\text{m}$ | `[dx, dy]` offset of Center of Mass from frame center. |
| **`chassis.inertiaTensor_kgm2`**| `ChassisParams::inertiaTensor` | $\text{kg}\cdot\text{m}^2$| $3\times3$ principal inertia matrix ($I_{xx}, I_{yy}, I_{zz}$). |
| **`chassis.friction`** | `RigidBodyComponent::friction` | — | Bumper/chassis contact friction against field obstacles. |
| **`battery.nominalVoltage_V`** | `BatteryModel::nominalVoltage` | $\text{V}$ | Open-circuit battery voltage (from Battery Beak). |
| **`battery.internalResistance_ohm`**| `BatteryModel::internalResistance` | $\Omega$ | Battery and power harness resistance (from Battery Beak). |
| **`battery.quiescentCurrent_A`**| `BatteryModel::quiescentCurrent` | $\text{A}$ | Robot idle current draw (electronics, radios, coprocessors). |
| **`battery.statorCurrentLimit_A`**| `BatteryModel::statorCurrentLimit`| $\text{A}$ | Motor controller stator current limit. |
| **`battery.supplyCurrentLimit_A`**| `BatteryModel::supplyCurrentLimit`| $\text{A}$ | Motor controller supply current limit. |
| **`battery.brownoutWarning_V`** | `BatteryModel::brownoutWarning` | $\text{V}$ | Voltage drop threshold triggering warning telemetry. |
| **`battery.brownoutCutoff_V`** | `BatteryModel::brownoutCutoff` | $\text{V}$ | roboRIO brownout voltage ($6.3\text{ V}$). |
| **`motors.drive.type`** | `DCMotorModel` factory | string | `"KrakenX60"`, `"KrakenX60FOC"`, `"Falcon500"`, `"NEO"`, etc. |
| **`motors.drive.gearRatio`** | `DCMotorModel::gearRatio` | ratio | Total drive reduction (motor rotations per wheel turn). |
| **`motors.drive.efficiency`** | `DCMotorModel::efficiency` | $0-1$ | Mechanical drivetrain transmission efficiency. |
| **`motors.steer.type`** | `DCMotorModel` factory | string | `"KrakenX44"`, `"KrakenX44FOC"`, `"Falcon500"`, `"NEO"`, etc. |
| **`motors.steer.gearRatio`** | `DCMotorModel::gearRatio` | ratio | Total steer reduction (motor rotations per azimuth turn). |
| **`motors.steer.efficiency`** | `DCMotorModel::efficiency` | $0-1$ | Steer module mechanical efficiency. |
| **`steerHousing.mass_kg`** | Housing inertia formula | $\text{kg}$ | Mass of rotating steer module column. |
| **`steerHousing.radius_m`** | Housing inertia formula | $\text{m}$ | Radius of rotating steer module column. |
| **`steerModule.steerHousingInertia_kgm2`**| `SteerParams::steerHousingInertia`| $\text{kg}\cdot\text{m}^2$| Direct override of steer rotational inertia. |
| **`steerModule.coulombFriction_Nm`** | `SteerParams::coulombFrictionTorque` | $\text{N}\cdot\text{m}$ | Coulomb friction torque opposing steer rotation. |
| **`steerModule.viscousDamping_Nms`** | `SteerParams::viscousDamping` | $\text{N}\cdot\text{m}\cdot\text{s}$ | Viscous damping opposing steer velocity. |
| **`driveModule.coulombFriction_Nm`** | `DriveParams::coulombFrictionTorque` | $\text{N}\cdot\text{m}$ | Coulomb friction torque in drive geartrain. |
| **`wheel.radius_m`** | `TireParams::wheelRadius` | $\text{m}$ | Radius of wheels accounting for tread wear. |
| **`wheel.width_m`** | `TireParams::wheelWidth` | $\text{m}$ | Tread contact width. |
| **`wheel.mass_kg`** | `TireParams::wheelMass` | $\text{kg}$ | Mass of one wheel and tread. |
| **`wheel.axleInertia_kgm2`** | `TireParams::wheelAxleInertia` | $\text{kg}\cdot\text{m}^2$| Rotational inertia of wheel around drive axle. |
| **`tire.mu_static`** | `TireParams::muStatic` | ratio | Static coefficient of friction on FRC carpet. |
| **`tire.mu_kinetic`** | `TireParams::muKinetic` | ratio | Kinetic/sliding coefficient of friction on FRC carpet. |
| **`tire.radialStiffness_Npm`** | `TireParams::radialStiffness` | $\text{N}/\text{m}$ | Vertical compliance rate of tire/tread ($120000\text{ N/m}$). |
| **`tire.radialDamping_Nspm`** | `TireParams::radialDamping` | $\text{N}\cdot\text{s}/\text{m}$ | Vertical damping rate of tire/tread ($1200\text{ N}\cdot\text{s/m}$). |
| **`tire.preSlide_m`** | `TireParams::preSlideDisplacement` | $\text{m}$ | Elastic deformation before full sliding slip ($0.001\text{ m}$). |
| **`tire.stribeckVelocity_mps`**| `TireParams::stribeckVelocity` | $\text{m}/\text{s}$| Velocity threshold transition between static & kinetic grip. |
| **`tire.loadSensitivityBeta`**| `TireParams::loadSensitivityBeta` | $1/\text{N}$ | Normal load sensitivity coefficient ($0.08$). |
| **`tire.rollingResistance`** | `TireParams::rollingResistance` | ratio | Dimensionless rolling resistance factor ($0.015$). |
| **`tire.lateralFrictionRatio`**| `TireParams::lateralFrictionRatio`| ratio | Lateral grip scale factor relative to longitudinal ($1.0$). |
| **`modules[i].name`** | `SwerveModuleConfig::name` | string | Identifier (`"FrontLeft"`, `"FrontRight"`, etc.). |
| **`modules[i].location_m`** | `SwerveModuleConfig::locationOnChassis` | $\text{m}$ | `[X, Y]` coordinates relative to chassis center. |
| **`modules[i].driveCAN`** | `SwerveModuleConfig::driveCANId` | int | CAN ID of Talon FX drive motor. |
| **`modules[i].steerCAN`** | `SwerveModuleConfig::steerCANId` | int | CAN ID of Talon FX steer motor. |
| **`modules[i].canCoder`** | `SwerveModuleConfig::canCoderId` | int | CAN ID of steer absolute CANcoder. |

---

## 7. Master Configuration Dictionary for Schema v1 (`manifest.json` & `sidecar.json`) (Block 3)

### A. [`manifest.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/Reference2026/manifest.json) Keys

| JSON Key | C++ Struct Target | Unit | Description |
| :--- | :--- | :--- | :--- |
| **`schemaVersion`** | `RobotManifest::schemaVersion` | string | Must be `"1.0.0"`. |
| **`coordinateSystem`** | `RobotManifest::coordinateSystem` | string | Must be `"NWU"` ($+X$ Fwd, $+Y$ Left, $+Z$ Up). |
| **`links[i].name`** | `LinkDef::name` | string | Unique link identifier referenced by joints and sidecar. |
| **`links[i].isRoot`** | `LinkDef::isRoot` | bool | `true` for `"base_link"` (chassis root). |
| **`links[i].mass_kg`** | `LinkDef::mass_kg` | $\text{kg}$ | Link mass ($> 0$). |
| **`links[i].centerOfMass_m`** | `LinkDef::centerOfMass_m` | $\text{m}$ | Local `[x, y, z]` CoM in NWU; converted to Engine via `NWUToEnginePos`. |
| **`links[i].inertiaTensor_kgm2`** | `LinkDef::inertiaTensor` | $\text{kg}\cdot\text{m}^2$ | Full $3\times 3$ inertia tensor (`ixx, iyy, izz, ixy, ixz, iyz`) about CoM. |
| **`links[i].colliders[j]`** | `ColliderDef` | $\text{m}$ | Collision shape (`box`, `sphere`, `cylinder`, `capsule`), `origin_m`, `dimensions_m`, and `material`. Also drives procedural visual mesh generation. |
| **`joints[i].type`** | `JointDef::type` | string | `"revolute"`, `"prismatic"`, `"continuous"`, or `"fixed"`. |
| **`joints[i].origin_m`** | `JointDef::origin_m` | $\text{m}$ | Joint anchor in parent link frame. |
| **`joints[i].axis`** | `JointDef::axis` | unit vec | Joint axis in NWU (`[0,0,1]` vertical slider, `[0,1,0]` pitching hinge). |
| **`joints[i].limits`** | `JointLimitsDef` | $\text{m}$ or $\text{rad}$ | `lower`, `upper`, `springFrequency_Hz` ($40\text{ Hz}$), `springDampingRatio` ($1.0$). |
| **`contactZones[i]`** | `ContactZoneDef` | $\text{m}$ | Sensor trigger volume (`ObjectLayers::SENSOR`) for beam breaks, intakes, and shooters. |

### B. [`sidecar.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/Reference2026/sidecar.json) Keys

| JSON Key | C++ Struct Target | Unit | Description |
| :--- | :--- | :--- | :--- |
| **`electrical.openCircuitVoltage_V`** | `ElectricalConfig::openCircuitVoltage_V` | $\text{V}$ | Nominal open-circuit battery voltage ($12.6\text{ V}$). |
| **`electrical.internalResistance_Ohm`** | `ElectricalConfig::internalResistance_Ohm` | $\Omega$ | Battery + main harness resistance ($0.023\ \Omega$). |
| **`swerve.*`** | `SwerveSidecarConfig` | SI | Motor presets, gear ratios, efficiencies, wheel radius, current limits, and 4 module `[x, y, z]` locations & CAN IDs. |
| **`mechanisms[i].role`** | `MechanismConfig::role` | string | `"elevator"`, `"arm"`, `"wrist"`, `"four_bar"`, or `"flywheel"`. |
| **`mechanisms[i].targetJoint`** | `MechanismConfig::targetJoint` | string | Target joint name in `manifest.json` (or `"parentLink"` for flywheels). |
| **`mechanisms[i].motorPreset`** | `MechanismConfig::motorPreset` | string | Motor type preset (`"KrakenX60"`, `"KrakenX44"`, `"NEO"`, etc.). |
| **`mechanisms[i].gearRatio`** | `MechanismConfig::gearRatio` | ratio | Gearbox reduction $G$ (motor turns per mechanism turn). |
| **`mechanisms[i].drumRadius_m`** | `MechanismConfig::drumRadius_m` | $\text{m}$ | Sprocket/drum pitch radius for linear elevators ($0.01905\text{ m}$). |
| **`mechanisms[i].springAssist`** | `SpringAssistConfig` | $\text{N}$ or $\text{N}\cdot\text{m}$ | `"constant_force"` (`force_N`), `"torsion"`, or `"cosine_gas"`. |
| **`mechanisms[i].controllers`** | `ControllerConfig` | — | Array of `vendor`, `canId`, `canBus`, `inverted`, and `role` (`"leader"` / `"follower"`). Number of controllers sets `motorCount`. |
| **`couplings[i].type`** | `CouplingConfig::type` | string | `"elevator_cascade"`, `"elevator_continuous"`, `"four_bar_loop"`, or `"power_coupling"`. |
| **`sensors[i].type`** | `SensorConfig::type` | string | `"imu_pigeon2"`, `"cancoder"`, `"limit_switch"`, `"beam_break"`, `"analog_input"`, or `"laser_can"`. |
