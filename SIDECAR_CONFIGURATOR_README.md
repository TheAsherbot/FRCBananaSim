# FRC Banana Simulator — Sidecar Configurator & CAD Ingestion Suite

The **Sidecar Configurator** is a dedicated graphical application designed for FRC robot teams to ingest exported CAD robot assemblies, inspect 3D kinematics, configure actuator and motor controllers, tune electrical and mechanical physics, and validate simulator schemas in real time.

---

## Table of Contents
1. [Core Architectural Concept: Manifest vs Sidecar](#1-core-architectural-concept-manifest-vs-sidecar)
2. [Building and Launching](#2-building-and-launching)
3. [Quickstart Guide](#3-quickstart-guide)
4. [Robot Project Management](#4-robot-project-management)
   - [Opening Robot Folders](#opening-robot-folders)
   - [Recent Robots History (Last 3 MRU)](#recent-robots-history-last-3-mru)
   - [Sample Robots](#sample-robots)
   - [One-Click Starter Sidecar Bootstrap](#one-click-starter-sidecar-bootstrap)
   - [Saving and Exporting](#saving-and-exporting)
5. [3D CAD Viewport & Camera Navigation](#5-3d-cad-viewport--camera-navigation)
   - [Viewport Controls & Navigation Presets](#viewport-controls--navigation-presets)
   - [Camera Settings Modal](#camera-settings-modal)
   - [Gold Selection Highlighting & HUD Overlay](#gold-selection-highlighting--hud-overlay)
6. [CAD Hierarchy Panel](#6-cad-hierarchy-panel)
7. [Subsystem Inspector Panels](#7-subsystem-inspector-panels)
   - [Two-Line Segmented Navigation Bar](#two-line-segmented-navigation-bar)
   - [Swerve Drivetrain Panel](#swerve-drivetrain-panel)
   - [Mechanisms Panel (1-DOF Mechanisms & 2-DOF Differentials)](#mechanisms-panel-1-dof-mechanisms--2-dof-differential-wrists)
   - [Mechanical Couplings & Linkages Panel](#mechanical-couplings--linkages-panel)
   - [Sensor Suite Panel (IMUs, CANcoders, Beam Breaks, Limit Switches, Analog)](#sensor-suite-panel-imus-cancoders-beam-breaks-limit-switches-analog)
   - [Mass Reconciliation & Ballast Calibration](#mass-reconciliation--ballast-calibration)
8. [Real-Time Diagnostic Schema Linter](#8-real-time-diagnostic-schema-linter)
9. [Engineering Reference & FAQs](#9-engineering-reference--faqs)

---

## 1. Core Architectural Concept: Manifest vs Sidecar

FRC Banana Simulator enforces a strict architectural boundary between CAD mechanical structure and simulation physics:

```
[CAD Software (Fusion 360 / Onshape)]
                  │
                  ▼ Export via Banana Fusion Add-in
         ┌───────────────────┐
         │   manifest.json   │  <-- Mechanical Geometry & Kinematics
         └───────────────────┘
                  │
                  ▼ Ingested by SidecarConfigurator
         ┌───────────────────┐
         │   sidecar.json    │  <-- Physics, Motors, CAN IDs, Gear Ratios
         └───────────────────┘
                  │
                  ▼ Loaded by FRCBananaSim Engine
        [High-Speed Physics & HAL Simulation]
```

- **`manifest.json` (Read-Only CAD Data)**: Produced directly by your CAD exporter (e.g., Fusion 360 add-in). Defines the assembly link tree, mesh geometry files, collision primitives, physical joint constraints (revolute, prismatic, continuous, fixed), mass, center of mass, and 3x3 inertia tensors.
- **`sidecar.json` (Simulation & Electrical Sidecar)**: Defines everything that does not live in CAD: CAN bus IDs, motor controller models (`TalonFX`, `SPARK MAX`), gear reductions, mechanical efficiencies, Coulomb and viscous friction, battery/stator current limits, wheel radius calibration, and sensor mappings.

> [!NOTE]
> By separating the two, you can iterate on your CAD models (changing pocketing, lightening holes, or bracket dimensions) and re-export without ever losing your tuned motor PID constants, CAN bus mappings, or gear ratios!

---

## 2. Building and Launching

### Prerequisites
- Windows 10/11 64-bit
- CMake 3.20+
- Visual Studio 2022 (MSVC with C++20 support)
- OpenGL 3.3 compatible graphics card

### Build Commands
From the repository root:
```powershell
# 1. Generate CMake build tree
cmake -B build -S .

# 2. Build the Configurator in Release mode
cmake --build build --config Release --target SidecarConfigurator

# 3. Launch the application
.\build\Release\SidecarConfigurator.exe
```

---

## 3. Quickstart Guide

Getting your robot configured and ready for simulation takes just a few steps:

1. **Launch the Application**:
   ```powershell
   .\build\Release\SidecarConfigurator.exe
   ```
2. **Open Your Robot**:
   - Select **`Reference2026`** from the **Samples:** dropdown (or click **Browse...** to choose an exported CAD directory containing `manifest.json`).
3. **Generate Starter Sidecar**:
   - Click **Auto-Generate Starter** on the top toolbar. This automatically provisions a 4-module swerve drive, all 1-DOF mechanisms (elevators/arms), and an IMU.
4. **Tune Subsystems**:
   - Use the 2-row navigation bar on the right (`Swerve Drive`, `Mechanisms`, `Couplings`, `Sensors`, `Mass & Scale`) to set motor types, gear ratios, CAN IDs, and sensors.
5. **Check Linter & Save**:
   - Ensure the bottom **Diagnostic Schema Linter** dock shows **0 Errors**.
   - Click **Save sidecar.json** on the top toolbar.
6. **Verify in Physics**:
   - Run the automated test suite to confirm your configuration loads and runs cleanly:
     ```powershell
     .\build\Release\FRCBananaSim_Tests.exe --round-trip
     ```

---

## 4. Robot Project Management

The top toolbar provides complete project loading, discovery, and saving operations:

```
[ CAD Folder: C:/Game Dev/.../Reference2026 ] [Open] [Browse...] | Recent: [ 1. Reference2026 v ] | Samples: [ Reference2026 v ] | [Camera Settings] [Auto-Generate Starter] [Save sidecar.json] [Clear]
```

### Opening Robot Folders
- **Manual Path Entry**: Type or paste any relative or absolute path into the `CAD Folder:` box and click **Open**.
- **Native Windows Folder Browser**: Click **Browse...** to open a native Windows folder dialog and select the folder containing your robot's `manifest.json`.

### Recent Robots History (Last 3 MRU)
- The application automatically tracks the **last 3 opened robots**.
- The **Recent:** dropdown lets you switch between your recent robots in **one click** without re-typing long paths.
- Hovering over any item in the Recent dropdown displays its full disk path in a tooltip.
- The history list is persisted in `FRCBananaSim/Resources/Config/configurator_settings.json` under `"recentRobots"`.
- If a robot is already in your recent history, opening it re-promotes it to position 1 without duplicating entries.

### Sample Robots
- The **Samples:** dropdown lists ready-to-test robot assemblies packaged in `FRCBananaSim/Resources/Robots`:
  - **`Reference2026`**: Complete reference robot featuring a 4-module swerve drivetrain, 2-stage elevator, shoulder arm, intake, and IMU.
  - **`TestAssembly2026`**: CAD assembly used for stress-testing small component inertia tensors, wheel colliders, and contact zones.

### One-Click Starter Sidecar Bootstrap
- Clicking **Auto-Generate Starter** analyzes the loaded CAD `manifest.json` and automatically provisions:
  1. A complete 4-module Swerve Drivetrain binding front-left, front-right, back-left, and back-right modules based on coordinate geometry.
  2. 1-DOF Elevator mechanisms for all `prismatic` joints.
  3. 1-DOF Arm or Wrist mechanisms for all `revolute` / `continuous` joints (detecting turret/azimuth joints automatically).
  4. An IMU sensor bound to the base chassis link.
- Provides an immediate, valid baseline with zero linter errors.

### Saving and Exporting
- Clicking **Save sidecar.json** validates the configuration against the schema linter and formats/writes `sidecar.json` directly into the open robot directory.

---

## 5. 3D CAD Viewport & Camera Navigation

The center panel renders an interactive OpenGL 3D preview of your robot assembly, with real-time lighting, grid floor, coordinate axes, and selection highlighting.

### Viewport Controls & Navigation Presets
Navigate the 3D scene using mouse controls:

| Navigation Preset | Orbit | Pan | Zoom |
| :--- | :--- | :--- | :--- |
| **Fusion 360 (Default)** | <kbd>Shift</kbd> + Middle Mouse Button | Middle Mouse Button Drag | Mouse Scroll Wheel |
| **Blender** | Middle Mouse Button Drag | <kbd>Shift</kbd> + Middle Mouse Button | Mouse Scroll Wheel |
| **CAD Standard** | Right Mouse Button Drag | Middle Mouse Button Drag | Mouse Scroll Wheel |

### Camera Settings Modal
Click **Camera Settings** in the top toolbar to adjust:
- **Navigation Preset**: Switch between Fusion 360, Blender, or CAD Standard.
- **Invert Scroll Zoom**: Toggle zoom direction to match personal preference.
- **Invert Orbit Direction (Horizontal X & Vertical Y)**: Customize horizontal turntable and vertical tilt orbit directions.
- **Field of View (FOV)**: Adjust between 20° and 90° (default 45°).
- **Orbit, Pan, and Zoom Sensitivities**: Fine-tune mouse tracking speed.
- All camera settings are automatically saved across sessions in `configurator_settings.json`.

### Gold Selection Highlighting & HUD Overlay
- When you click any **Link**, **Joint**, or **Mechanism** in the hierarchy panel or the inspector, the viewport instantly draws that component highlighted in **Gold Wireframe & Silhouette**.
- The bottom-left of the 3D viewport displays a live **HUD Overlay** detailing:
  - Active Mechanism role and name.
  - Bound CAD Target Joint and kinematic type.
  - Bound Parent Link.

---

## 6. CAD Hierarchy Panel

The left panel displays the structural tree ingested from `manifest.json`:
- **Links Tree**: Displays all rigid bodies with their mass (kg), collision shapes (boxes, cylinders, spheres, convex hulls), and visual meshes. Clicking any link highlights it in the 3D viewport and populates the CAD Selection tab.
- **Joints Tree**: Shows all kinematic joints, their parent/child link relationships, joint types (`revolute`, `prismatic`, `continuous`, `fixed`), and physical limits ($rad$ or $m$).
- **Subsystems Quick Jump**: Direct shortcuts to inspect the **Swerve Drivetrain**, **Mechanisms**, **Sensors**, or **Couplings**.

---

## 7. Subsystem Inspector Panels

The right panel features a clean **two-line segmented navigation bar** giving rapid access to all robot subsystems without text truncation or cramped single-line clipping:
- **Line 1 (Kinematics & Actuation)**: `[ CAD Selection ]` `[ Swerve Drive ]` `[ Mechanisms ]`
- **Line 2 (Linkages, Sensors & Calibrations)**: `[ Couplings ]` `[ Sensors ]` `[ Mass & Scale ]`

### Unified Single-Control Numeric Sliders & Automatic Unit Conversion
Every numeric parameter across all inspector panels uses a **single unified control widget** (no redundant side-by-side duplicate boxes):
- **Click & Drag**: Drag horizontally with the left mouse button to smoothly scrub/slide values.
- **In-Place Direct Typing**: **Double-click**, **Ctrl+Click**, or **Right-click** on the box to enter text-edit mode in place.
- **CAD-Style Intelligent Unit Entry**: You can type values in whichever unit you are measuring or thinking in, and pressing <kbd>Enter</kbd> (or clicking away) will automatically convert the value into the field's target unit:

- **Distance / Dimension Fields (e.g. `Wheelbase (m)`, `Trackwidth (m)`, `Pos X (m)`, `Offset X (m)`)**:
  - Type `24"` or `24 in` $\implies$ automatically converts to `0.6096 m`.
  - Type `2.5 ft` or `2.5'` $\implies$ automatically converts to `0.7620 m`.
  - Type `600 mm` $\implies$ automatically converts to `0.6000 m`.
  - Type `60 cm` $\implies$ automatically converts to `0.6000 m`.
  - Type `0.884` or `0.884 m` $\implies$ stays `0.8840 m`.
- **Caliper Diameter Inputs**:
  - In `Caliper Diam (in)`: type `100 mm` $\implies$ converts to `3.937 in`.
  - In `Caliper Diam (mm)`: type `4"` or `4 in` $\implies$ converts to `101.6 mm`.
- **Mass Fields (e.g. `Bench Scale Mass (kg)`, `Ballast Mass (kg)`, individual component overrides)**:
  - Type `120 lbs` or `120#` $\implies$ converts to `54.431 kg`.
  - Type `54.4 kg` $\implies$ stays `54.400 kg`.
- **Angle Fields (e.g. `Magnet Offset (rad)`, `Travel Range (deg)`)**:
  - In `(rad)` fields: type `90 deg` or `90°` $\implies$ converts to `1.5708 rad`.
  - In `(deg)` fields: type `3.14159 rad` $\implies$ converts to `180.0 deg`.
- **Torque Fields (e.g. `Stall Torque (N*m)`)**:
  - Type `20 kg*cm` $\implies$ converts to `1.961 N*m`.
  - Type `15 in-lb` $\implies$ converts to `1.695 N*m`.
  - Type `10 ft-lb` $\implies$ converts to `13.558 N*m`.
- **Speed Fields (e.g. `Speed (s/60 deg)` vs `Free Speed (RPM)`)**:
  - In `(s/60 deg)`: type `100 rpm` $\implies$ converts to `0.100 s/60`.
  - In `(RPM)`: type `0.100 s/60` $\implies$ converts to `100 RPM`.
- **Mathematical Fractions & Gear Reductions**:
  - Type `1/9` into velocity ratio $\implies$ calculates `0.111111` (9:1 reduction).
  - Type `12 / 60` into gear reductions $\implies$ calculates `0.200`.
  - Type `24/2"` into wheelbase/trackwidth $\implies$ evaluates `12"` and converts to `0.3048 m`.

### Swerve Drivetrain Panel
- **Chassis Link Binding**: Dropdown selecting the main robot frame/chassis link from the CAD manifest.
- **COTS Module Presets**:
  - Swerve Drive Specialties: **MK4**, **MK4i**, **MK4n**, **MK5i**, **MK5n** (supporting **L1**, **L2**, and **L3** / **R1**, **R2**, **R3** gear ratios).
  - WestCoast Products: **SwerveX**, **SwerveX2** (supporting Standard, High-Speed, and Flipped configurations).
  - REV Robotics: **MAXSwerve** (supporting 12T, 13T, 14T pinion variations).
  - **Automatic Preset Recognition on Load**: When loading a sidecar, the configurator inspects the drive and steer ratios and wheel radius against all COTS presets. If a match is found (e.g. Reference2026 matches SDS MK5n R3), the module family and ratio dropdowns automatically default to that preset, displaying green `[COTS Preset Active]` confirmation (or orange `[Custom / Modified]` if custom ratios are used).
  - **Custom**: Full parametric override of drive and steer gear ratios.
- **Actuators (CAN DC Motor vs PWM Servo)**:
  - Toggle between industrial brushless CAN motors (Kraken X60, Falcon 500, NEO, NEO Vortex, CIM) and PWM Servos.
  - **GoBILDA & Custom Servo Presets**: Pre-configured specs for GoBILDA Super Speed (300 RPM), Speed (115 RPM), Torque (60 RPM), Swyft Speed (200 RPM), Balanced (115 RPM), Swyft Torque (60 RPM), or **Custom**.
  - **Custom Servo Specs**: Direct input for Operating Voltage ($V$), Stall Torque ($kg\cdot cm$), Speed ($s/60^\circ$ and automatic RPM conversion), Stall/Free Currents ($A$), and Travel Range ($^\circ$).
- **Wheel Caliper Wear Calculator**:
  - Enter physical caliper measurements (diameter in $mm$ or radius in $m$) of worn or brand-new tread wheels (e.g. 98.5 mm).
  - Automatically calculates the exact effective radius ($m$) and applies it to all modules.
- **4-Module Matrix (Front-Left, Front-Right, Back-Left, Back-Right)**:
  - $X, Y$ chassis offset coordinates ($m$).
  - Drive Motor CAN ID & Steer Motor CAN ID.
  - CAN Bus Assignment: **`roboRIO (rio)`** vs **`CANivore (canivore)`**.
  - Inversion toggles for drive and steer motors.
  - Absolute steering encoder offset ($rad$).

### Mechanisms Panel (1-DOF Mechanisms & 2-DOF Differential Wrists)
Configure arms, elevators, flywheels, intakes, wrists, turrets, and differentials:
- **Mechanism Roles**:
  - `elevator`: Prismatic motion along an axis.
  - `arm`: Revolute joint operating against gravity.
  - `wrist`: Revolute joint mounted on an arm or carriage.
  - `turret`: Azimuth yaw rotation perpendicular to gravity.
  - `flywheel`: High-speed spinning wheel for shooting game pieces.
  - `intake`: Roller or deployable intake joint.
  - `differential`: 2-DOF differential wrist driving coupled pitch and roll motions.
- **Mechanism Nickname**: Custom code/telemetry identifier (e.g., `shooter_hood`, `coral_elevator`, `diff_wrist`).
- **Dynamic CAD Joint Filtering**:
  - Choosing `elevator` restricts the Target Joint dropdown to only **prismatic** joints.
  - Choosing `arm`, `wrist`, `turret`, `flywheel`, or `intake` restricts the dropdown to **revolute** and **continuous** joints.
  - Choosing `differential` presents dual joint pickers (`Pitch Joint` and `Roll Joint`) strictly populated with **revolute** joints.
- **2-DOF Differential Mechanism Topology (2 Outputs & 2 Inputs)**:
  - **2 Kinematic Outputs (Joint Kinematics)**:
    - Pitch Joint and Roll Joint dynamic pickers populated strictly with CAD revolute joints.
    - Differential Types: `symmetric_bevel`, `custom_matrix`, `core_xy`, `coupled_arm`.
    - Independent Pitch ($G_p$) and Roll ($G_r$) gear reduction inputs, or full 2x2 coupling matrix ($M_{00}, M_{01}, M_{10}, M_{11}$).
    - Independent Coulomb friction and viscous damping parameters for each output joint.
  - **2 Actuation Inputs (Input Power Sources)**:
    - **Input Power Source 1 (Motor / Actuator A)**:
      - Independent Actuator Type: Industrial CAN DC Motor vs PWM Servo.
      - Motor / Servo Preset: Kraken X60, Falcon 500, NEO, NEO Vortex, CIM, GoBILDA presets, or Custom specs.
      - Primary Hardware Controller: Model (`CTRE_TalonFX`, `CTRE_TalonSRX`, `REV_SPARK_MAX`, `REV_SPARK_Flex`), CAN ID, CAN Bus (`rio` vs `canivore`), Invert flag.
      - Electrical Limits: Independent Stator Current Limit ($A$) and Supply Current Limit ($A$).
    - **Input Power Source 2 (Motor / Actuator B)**:
      - Independent Actuator Type: Industrial CAN DC Motor vs PWM Servo.
      - Motor / Servo Preset: Kraken X60, Falcon 500, NEO, NEO Vortex, CIM, GoBILDA presets, or Custom specs.
      - Primary Hardware Controller: Model, CAN ID, CAN Bus, Invert flag.
      - Electrical Limits: Independent Stator Current Limit ($A$) and Supply Current Limit ($A$).
    - **Follower Controllers Table**:
      - For heavy differential mechanisms (e.g. dual-motor-per-side differential wrists), add arbitrary follower motor controllers mapped to either Input A or Input B.
- **Multi-Controller Hardware Matrix**:
  - ImGui table managing the primary **Leader** controller and arbitrary **Follower** controllers.
  - Controller Model: `CTRE_TalonFX`, `CTRE_TalonSRX`, `REV_SPARK_MAX`, `REV_SPARK_Flex`.
  - CAN ID and CAN Bus (`roboRIO (rio)` vs `CANivore (canivore)`).
  - Individual controller Invert (`Inv`) flags.
- **Electrical Current Limits**:
  - Stator Current Limit ($A$) and Supply Current Limit ($A$).
- **Transmission Dynamics**:
  - Gear Reduction Ratio ($G:1$).
  - Mechanical Efficiency ($\eta \in [0.5, 1.0]$).
  - Coulomb Friction ($N\cdot m$) and Viscous Damping ($N\cdot m \cdot s / rad$).
- **Role-Specific Parameters**:
  - Flywheel: Rotor moment of inertia ($kg\cdot m^2$), shooter wheel mount offset ($X, Y, Z$ in meters).
  - Elevator: Drum radius ($m$), carriage mass ($kg$), constant spring assist force ($N$).

### Mechanical Couplings & Linkages Panel
Dedicated subsystem inspector for mechanical transmissions between joints and kinematic loop closures:
- **Power Transmission Couplings (`power_coupling`)**:
  - Connect a driver (Master) joint to a driven (Slave) joint.
  - Transmission Type: `gear`, `belt`, `chain`, `cable`, or `virtual_fourbar`.
  - Velocity ratio ($\omega_{\text{slave}} / \omega_{\text{master}}$) with direct input and slider.
  - Inverted direction checkbox.
  - Mechanical efficiency ($\eta \in [0.5, 1.0]$) and backlash/deadband ($rad$).
- **Multi-Stage Elevator Cascades (`elevator_cascade`)**:
  - Couples multi-stage linear lift travel ($x_k = k \cdot x_1$).
  - Stage table with `+ Add Cascade Stage`, dynamic joint dropdowns filtered strictly to `prismatic` joints, and individual ratio multipliers ($k$).
- **Four-Bar Kinematic Loops (`four_bar_loop`)**:
  - Models closed planar four-bar linkages without expensive physics solver iterations.
  - Joint Chain: Crank joint (driven), Coupler joint, and Follower joint (all from CAD).
  - Loop Closures: Closure parent link, closure child link, 3D parent anchor point ($X, Y, Z$), 3D child anchor point ($X, Y, Z$), and closure rotation axis vector.
- **Solver Modes**:
  - `analytic_coupled`: Direct high-speed closed-form constraint solver running at 100x real-time speed.
  - `jolt_constraint`: Jolt multi-body constraint fallback.

### Sensor Suite Panel (IMUs, CANcoders, Beam Breaks, Limit Switches, Analog)
Author and calibrate all sensory instrumentation for autonomous navigation, mechanism feedback, and game piece detection:
- **CTRE Pigeon 2.0 (`imu_pigeon2`)**:
  - 6-axis IMU reporting yaw, pitch, roll, angular rates, and 3-axis linear accelerations.
  - CAN ID and CAN Bus assignment (`roboRIO (rio)` vs `CANivore (canivore)`).
  - Rigid mounting to any CAD parent link with 3D offset coordinates ($X, Y, Z$ in meters).
  - Real-world stochastic noise modeling: Gyro noise standard deviation ($rad/s$), gyro bias random walk ($rad/s^2$), and accelerometer noise ($m/s^2$).
- **Kauai Labs navX2 (`imu_navx`)**:
  - 6-axis IMU supporting `SPI (Onboard)`, `I2C (Onboard)`, or `I2C (MXP Extension)` bus interfaces.
  - Mount parent link and 3D offset coordinates ($X, Y, Z$ in meters).
  - Drift and noise parameters.
- **CTRE CANcoder (`cancoder`)**:
  - Absolute magnetic angular encoder over CAN (`rio` vs `canivore`).
  - Dynamic CAD target joint binding filtered strictly to revolute and continuous joints.
  - Sensor-to-mechanism gear ratio, absolute magnet offset calibration in radians and degrees, inversion flag, and CPR (4096).
- **Optical Beam Break (`beam_break`)**:
  - roboRIO Digital I/O (DIO) channel assignment $[0..31]$.
  - Associative CAD contact zone binding (e.g. `intake_capture_zone`, indexer, hopper) that detects real-time game piece presence.
  - Normally Closed (NC) vs Normally Open (NO) switch logic and debounce filter duration in milliseconds.
- **Limit Switch (`limit_switch`)**:
  - Hardware connection routing: **roboRIO DIO Channel** $[0..31]$ or **Hard-Wired to Motor Controller CAN ID**.
  - Limit direction: `forward` vs `reverse`.
  - Dynamic target joint binding with physical trigger range band (`Trigger Min` and `Trigger Max` in radians or meters).
  - Normally Closed (NC) logic and debounce filter duration in milliseconds.
- **Analog Input (`analog_input`)**:
  - roboRIO Analog Input channel assignment $[0..7]$.
  - Dynamic joint binding for continuous analog potentiometers and pressure transducers.
  - Direct 0V to 5V linear span calibration (`Min Value @ 0V` to `Max Value @ 5V`).
- **LaserCAN (`laser_can`) & Color Sensor V3 (`color_sensor`)**:
  - Grapple LaserCAN TOF distance sensing with CAN ID, CAN bus, and 3D link offset.
  - REV Color Sensor V3 over I2C (`Onboard` vs `MXP`) with 7-bit device address.
- **Diagnostic Sensor Linter Rules**:
  - Real-time detection of duplicate CAN IDs across swerve, mechanisms, and sensors.
  - Real-time detection of duplicate DIO channels across beam breaks and limit switches.
  - Real-time detection of duplicate Analog channels across analog transducers.
  - Instant validation of target joints, parent links, contact zones, and attached motor controller CAN IDs.

### Mass Reconciliation & Ballast Calibration
Reconcile the difference between your theoretical CAD model weight and the real-world scale weight without modifying CAD files:
- **Scale Weight Input**: Input the robot's physical weight as measured on inspection scales ($kg$ or $lbs$) and see real-time weight deficit or surplus.
- **Mode A (Proportional Robot Scaling)**: Uniformly scales all link masses and rotational inertia tensors by a global multiplier ($M_{\text{effective}} = M_{\text{cad}} \times \text{multiplier}$).
- **Mode B (Unmodeled Ballast Offset)**: Injects concentrated point masses (e.g. 12V battery, radio, wire harness, brass ballast weights) onto a specific target link at a 3D coordinate $[X, Y, Z]$, updating center of mass and parallel-axis inertia without phantom collider shapes.
- **Mode C (Individual Component Mass Overrides)**: Directly override the mass of specific individual links (e.g. fabricated parts differing from CAD). Individual overrides scale that link's rotational inertia proportionally ($I_{\text{new}} = I_{\text{cad}} \times (m_{\text{new}} / m_{\text{cad}})$) while leaving all other links governed by Mode A or raw CAD.

---

## 8. Real-Time Diagnostic Schema Linter

The bottom dock hosts the continuous schema linter. As you edit parameters, the linter validates the entire robot topology on every frame:

| Severity | Diagnostic Check | Description |
| :--- | :--- | :--- |
| 🛑 **Error** | **CAN ID Collision** | Two controllers share the same CAN ID on the same bus (`rio` or `canivore`). |
| 🛑 **Error** | **PWM Channel Collision** | Two servos share the same PWM channel. |
| 🛑 **Error** | **Orphan Joint Reference** | Mechanism references a target joint that does not exist in `manifest.json`. |
| 🛑 **Error** | **Orphan Link Reference** | Chassis or ballast references a link missing from `manifest.json`. |
| 🛑 **Error** | **Differential Joint Invalid** | Differential coupling targets a joint that is not revolute/continuous. |
| ⚠️ **Warning** | **High Stator Current** | Aggregate stator limits exceed 400A (potential brownout risk). |
| ⚠️ **Warning** | **Missing Joint Limits** | Joint targeted by a mechanism has no physical stop limits defined in CAD. |

The status bar and tab badge display error counts. If errors exist, the linter details the exact subsystem, controller, and conflicting ID with actionable instructions.

---

## 9. Engineering Reference & FAQs

### Q: Why is a Turret its own role rather than an Arm?
**A:** Both are 1-DOF rotational mechanisms driven through gear reductions, but their **gravity dynamics are fundamentally different**:
- An **Arm** rotates about a horizontal pitch axis and fights gravity ($\tau_g = mgr\cos\theta$). Holding an arm at $0^\circ$ requires continuous holding torque from the motor.
- A **Turret** rotates about a vertical yaw axis ($Z$ in NWU / $Y$ in simulation). Because the gravity vector is parallel to the rotation axis, gravitational torque is **zero** ($\tau_g = 0$). The turret motor works solely against rotational inertia ($J\alpha$), friction, and damping. Modeling a turret as an arm would inject artificial gravitational droop.

### Q: How do Servo Speed specifications ($s/60^\circ$) convert to RPM?
**A:** High-torque servos (like GoBILDA Dual-Mode Servos) specify rotational speed in seconds per 60 degrees. To convert to standard revolutions per minute:
$$\text{RPM} = \frac{1 \text{ rev}}{\Delta t} = \frac{360^\circ}{6 \times 60^\circ \times (s/60^\circ)} = \frac{60 \text{ s}}{6 \times (s/60^\circ)} = \frac{10}{s/60^\circ}$$

*Examples:*
- $0.100\text{ s}/60^\circ \implies 10 / 0.100 = \mathbf{100\text{ RPM}}$
- $0.050\text{ s}/60^\circ \implies 10 / 0.050 = \mathbf{200\text{ RPM}}$
- $0.170\text{ s}/60^\circ \implies 10 / 0.170 = \mathbf{58.8\text{ RPM}}$

The Configurator automatically computes and displays this equivalent RPM value in real time.

### Q: What is the difference between `roboRIO (rio)` and `CANivore (canivore)`?
- **`roboRIO (rio)`**: Standard 1 Mbps CAN 2.0B bus routed through the roboRIO CAN terminals. Supports Talon FX, Talon SRX, SPARK MAX, SPARK Flex, CANcoders, and Pigeon 2.
- **`CANivore (canivore)`**: High-speed CAN FD bus (up to 5-8 Mbps) using a dedicated USB-to-CANivore adapter. Allows offloading high-bandwidth CTRE devices to prevent bus utilization saturation.
- Device CAN IDs must be unique **per bus** (CAN ID 1 on `rio` and CAN ID 1 on `canivore` do not conflict).

### Q: What are Stator Current vs Supply Current, and where do I find them?
**A:** Modern brushless motor controllers (like Talon FX and SPARK MAX) regulate two distinct electrical currents:

1. **Supply Current ($I_{\text{supply}}$) — The Battery & Breaker Limit**:
   - This is the current drawn from the 12V battery through your PDP / PDH breaker.
   - **Purpose**: Prevents tripping the 40A circuit breakers and prevents battery voltage sag (brownouts).
   - **Where to find it in your robot code**:
     - CTRE Phoenix 6: `CurrentLimitsConfigs.SupplyCurrentLimit` (usually set to `35.0`–`40.0` A).
     - REVLib: `spark.setSecondaryCurrentLimit(...)`.
   - **Standard Default**: **35A – 40A** (matches standard continuous PDP/PDH breaker ratings).

2. **Stator Current ($I_{\text{stator}}$) — The Motor Winding & Torque Limit**:
   - Inside the motor controller, the H-bridge switching acts like a DC-to-DC buck converter. At low speeds and high loads, the current through the motor windings (stator current) can be **much higher** than the supply current drawn from the battery (e.g. 70A stator while only drawing 20A supply!).
   - **Purpose**: Stator current directly sets **motor output torque** ($\tau = K_t \cdot I_{\text{stator}}$). Clamping stator current limits peak torque to protect gears, belts, and tread from shredding, prevents wheel slip, and keeps motors from overheating under stall.
   - **Where to find it in your robot code**:
     - CTRE Phoenix 6: `CurrentLimitsConfigs.StatorCurrentLimit` (typically `60.0`–`80.0` A for drive/arm, `40.0` A for steer).
     - REVLib: `spark.setSmartCurrentLimit(...)` (typically `40`–`60` A).
   - **Standard Defaults**:
     - **Swerve Drive**: 60A – 80A
     - **Swerve Steer**: 30A – 40A
     - **Elevator / Heavy Arm**: 60A – 80A
     - **Intake / Rollers**: 30A – 40A
     - **Shooter Flywheels**: 60A – 80A

#### Where to find them in the Sidecar Configurator:
- **Swerve Modules**: Click **`[ Swerve Drive ]`** in the inspector $\to$ scroll down to **Electrical Current Limits**:
  - `Drive Stator Limit (A)` & `Drive Supply Limit (A)`
  - `Steer Stator Limit (A)` & `Steer Supply Limit (A)`
- **Mechanisms (Arms, Elevators, Intakes, Flywheels)**: Click **`[ Mechanisms ]`** in the inspector $\to$ expand any mechanism $\to$ see **Electrical Current Limits**:
  - `Stator Limit (A)`
  - `Supply Limit (A)`
- **Differential Wrists**: Click **`[ Mechanisms ]`** in the inspector $\to$ expand your differential mechanism $\to$ see independent `Stator Limit (A)` and `Supply Limit (A)` inputs under **Input Power Source 1** and **Input Power Source 2**.

> [!TIP]
> If you don't know your robot's exact software settings yet, clicking **Auto-Generate Starter** automatically fills in standard, safe FRC defaults (60A Stator / 40A Supply for heavy loads, 40A Stator for steer/intakes). You can safely leave those defaults as-is!

### Q: What does Inverted (`Inv`) mean for CTRE (Kraken/Falcon) vs. REV (NEO/Vortex) motors?

**A:** CTRE and REV define motor rotation conventions from **different physical viewpoints**, which means their default directions are opposite when viewing the shaft face:

1. **CTRE (Kraken X60, Falcon 500, Talon FX — Phoenix 6)**:
   - **Point of View**: Looking directly at the **front face** of the motor (with the shaft tip pointing towards you / closer to you).
   - **Normal (`Inv = false` / `CounterClockwise_Positive`)**: Positive voltage/command rotates the shaft **Counter-Clockwise (CCW)** when looking directly at the shaft tip.
   - **Inverted (`Inv = true` / `Clockwise_Positive`)**: Positive voltage/command rotates the shaft **Clockwise (CW)** when looking directly at the shaft tip.

2. **REV Robotics (NEO, NEO Vortex, SPARK MAX, SPARK Flex — REVLib)**:
   - **Point of View**: Looking from the **back / rear** of the motor (looking forward along the motor body, so the shaft tip is farthest away from you).
   - **Normal (`Inv = false`)**: Positive voltage/command rotates **Counter-Clockwise (CCW)** looking from the rear. When viewed from the **front shaft face**, this is **Clockwise (CW)**!
   - **Inverted (`Inv = true`)**: Positive voltage/command rotates **Clockwise (CW)** looking from the rear. When viewed from the **front shaft face**, this is **Counter-Clockwise (CCW)**!

| Motor Family | Controller / Library | Viewpoint | Normal (`Inv = false`) at Shaft Face | Inverted (`Inv = true`) at Shaft Face |
| :--- | :--- | :--- | :--- | :--- |
| **CTRE Kraken X60 / Falcon 500** | Talon FX (Phoenix 6) | Looking at **front face** (shaft closer to you) | **Counter-Clockwise (CCW)** | **Clockwise (CW)** |
| **REV NEO / NEO Vortex** | SPARK MAX / Flex (REVLib) | Looking from **rear back** (shaft farthest from you) | **Clockwise (CW)** | **Counter-Clockwise (CCW)** |

#### How Gearboxes Affect Direction:
- **Single Spur Gear Stage**: Inverts rotation (motor CCW $\to$ driven gear CW).
- **Two Spur Gear Stages / Idler**: Preserves motor rotation (motor CCW $\to$ driven gear CCW).
- **Planetary Gearboxes (MAXPlanetary / VersaPlanetary)**: Preserves motor rotation.
- **Timing Belts & Roller Chains**: Preserves motor rotation.

> [!TIP]
> In the Sidecar Configurator, whenever you hover over the **`Inv`** checkbox or the **`(?)`** help marker in the Hardware Controllers table, the tooltip dynamically reminds you of the exact rotation direction for your chosen motor vendor!

### Q: How do I enter Gear Reductions into the Sidecar? If my gearbox is 9:1, do I enter 9 or 1/9?

**A:** You enter **`9` (or `9.0`)!** Do **NOT** enter `1/9` or `0.111111`.

In mechanical engineering and `FRCBananaSim`, **Gear Reduction** is defined as the ratio $G:1$, which is the number of motor shaft rotations needed to produce 1 rotation of the output mechanism:
$$G = \frac{\omega_{\text{motor}}}{\omega_{\text{joint}}} = \frac{\tau_{\text{joint}}}{\tau_{\text{motor}}}$$

- **Entering a number $G > 1$ (e.g. `9.0`, `25.0`, `64.0`)**:
  - **Multiplies motor torque** by $G$ ($\tau_{\text{delivered}} = G \cdot \eta \cdot \tau_{\text{motor}}$). A 9:1 gearbox delivers **$9\times$ more torque**!
  - **Divides mechanism speed** by $G$ ($\omega_{\text{joint}} = \omega_{\text{motor}} / G$). The mechanism spins at $1/9^{\text{th}}$ of motor speed.
- **What happens if you enter `1/9` (0.111111)?**:
  - The simulator will treat the gearbox as an **overdrive / step-up** (spinning the output 9 times faster than the motor, with only $1/9^{\text{th}}$ of the motor's torque). Your mechanism would have almost zero lifting torque and would stall immediately!

#### Quick Reference Table:
| Mechanism Type | Typical Gearbox | Value to Enter in Sidecar | Torque Multiplier | Speed Multiplier |
| :--- | :--- | :--- | :--- | :--- |
| **Intake Rollers** | 4:1 to 9:1 planetary | **`4.0` to `9.0`** | $4\times$ to $9\times$ | $1/4$ to $1/9$ |
| **Elevator Lift** | 12:1 to 36:1 reduction | **`12.0` to `36.0`** | $12\times$ to $36\times$ | $1/12$ to $1/36$ |
| **Shoulder Arm** | 60:1 to 150:1 reduction | **`60.0` to `150.0`** | $60\times$ to $150\times$ | $1/60$ to $1/150$ |
| **Swerve Drive (L2)** | 6.75:1 reduction | **`6.75`** | $6.75\times$ | $1/6.75$ |
| **Swerve Steer** | 21.43:1 reduction | **`21.43`** | $21.43\times$ | $1/21.43$ |
| **Shooter Flywheel (Step-Up)** | 1:1.5 Overdrive (spins faster) | **`0.667`** ($1/1.5$) | $0.667\times$ | $1.5\times$ |

### Q: How does Transmission Efficiency affect Couplings? If I set Efficiency to 0.5, does that make both the master and slave go 50% slower?

**A: NO!** Transmission efficiency ($\eta$) does **NOT** alter the kinematic speed ratio of either joint.

1. **Kinematics (Speed Ratio)**:
   The relative speed between the master and slave joint is strictly governed by the **Velocity Ratio** ($R$):
   $$\omega_{\text{slave}} = \pm R \cdot \omega_{\text{master}}$$
   Because gears, belts, and chains are mechanically meshed (teeth engaging teeth), 1 tooth rotation on the driver forces exactly 1 tooth rotation on the driven element. Speed is kinematically locked — setting efficiency to 0.5 cannot cause the gears to slip or spin at half speed.

2. **Dynamics (Power Loss & Reaction Torque)**:
   Transmission efficiency ($\eta \in [0.0, 1.0]$) governs **power and torque conservation**:
   $$P_{\text{out}} = \eta \cdot P_{\text{in}}$$
   When mechanical power is transmitted through the coupling, a fraction ($1 - \eta$) is lost as friction and heat. To deliver a given torque $\tau_{\text{couple}}$ to the slave joint, the master joint experiences a reaction torque:
   $$\tau_{\text{reaction}} = -\frac{\tau_{\text{couple}}}{R \cdot \eta}$$
   - **At $\eta = 1.0$ (100% efficient)**: Ideal lossless transmission.
   - **At $\eta = 0.5$ (50% efficient)**: Half of the transferred mechanical energy is lost to friction. The master motor must exert **$2\times$ more reaction torque** to drive the slave!

> [!NOTE]
> If that extra friction load causes your master motor to hit its stator current limit or stall torque limit, the motor will decelerate (or stall) under the heavier load. But the slave joint will still spin at **exact lockstep ratio $R$** relative to the master joint!

### Q: On Mass & Scale, can I redefine the mass of individual components, or is it only for the whole robot?

**A: YES, you can do both!** The Mass & Scale inspector panel provides three complementary modes to calibrate physics without ever altering your original CAD manifest files:

1. **Whole-Robot Scaling (Mode A — Proportional Link Scaling)**:
   - Scales the mass and inertia tensors of **all links** uniformly by a single multiplier ($M_{\text{effective}} = M_{\text{cad}} \times \text{multiplier}$).
   - Perfect when the overall physical robot is heavier than CAD (e.g., due to fasteners, wiring, and grease distributed throughout the machine).
   - Clicking **`Compute Multiplier from Bench Scale`** calculates the exact multiplier ($M_{\text{bench}} / M_{\text{cad}}$) automatically.

2. **Targeted Lumped Mass (Mode B — Unmodeled Ballast)**:
   - Injects a concentrated point-mass onto an **individual target link** (such as `base_link` or an arm stage) at a specific 3D coordinate $[X, Y, Z]$.
   - Ideal for heavy discrete components that were not modeled in CAD, like the 12V SLA battery (~6.0 kg), radio, main breaker, or lead ballast plates.
   - Automatically computes composite center of mass and injects parallel-axis rotational inertia ($I_{\text{ballast}} = m ((\mathbf{r}\cdot\mathbf{r})\mathbf{I} - \mathbf{r}\mathbf{r}^T)$) without creating physical collision shapes.

3. **Individual Component Overrides (Mode C — Link Mass Overrides)**:
   - Allows directly overriding the exact mass of **specific individual components / links** (e.g. fabricated carbon fiber plates, weighted rollers, or modified brackets).
   - Select any link from the dropdown, input its measured weight in kilograms or pounds (e.g. `4.2kg` or `9.25lbs`), and click **`Set Override`**.
   - That link's mass is locked to your override, and its rotational inertia tensor scales proportionally ($I_{\text{new}} = I_{\text{cad}} \times (m_{\text{new}} / m_{\text{cad}})$).
   - Any links without an individual override continue to follow Mode A or raw CAD mass. When inspecting links in the CAD Selection tab, overridden components clearly display their active override mass.
