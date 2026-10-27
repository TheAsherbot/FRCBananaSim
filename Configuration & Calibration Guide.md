# FRCBananaSim Robot Configuration & Calibration Guide

> [!NOTE]
> **Status:** Block 2 Verified (6-DOF Analytic Swerve Dynamics, Jolt Rigid Body Chassis, Tire Contact Model, Battery Electromechanics, and CTRE Phoenix 6 HALSim WebSocket Bridge).
> This document is the living reference guide for modeling, measuring, and configuring robots in FRCBananaSim. It strictly documents features and parameters that are **currently implemented and parsed** by the simulation engine. As mechanisms, sensors, and new subsystems are added in subsequent blocks, this guide will expand.

---

## 1. System Overview & The Configuration File

All robot configuration data currently resides in:
📁 [`FRCBananaSim/Resources/Robots/robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json)

When the simulator launches, [`RobotSpawner::SpawnFromJsonFile`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/RobotSpawner.cpp) and [`SwerveDrivetrainConfig::LoadFromJsonString`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Source/Robot/Drivetrain/Model/SwerveParams.cpp) parse this JSON file into physical entities:
1. **Chassis Rigid Body**: A single dynamic Jolt physics body with 6 degrees of freedom (surge, sway, heave, roll, pitch, yaw), custom mass, and custom $3 \times 3$ inertia tensor.
2. **Analytic Swerve Drivetrain**: 4 module contact points evaluating tire brush friction, Stribeck transitions, and normal force load transfers.
3. **Electromechanical Actuation**: DC motor torque curves, gear ratios, drivetrain friction, and battery sag models.
4. **HALSim WebSocket Bridge**: Translates CTRE Phoenix 6 network packets to and from your robot code at 100 Hz.

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

---

## 2. Parameter Extraction from CAD

Extract these values directly from your robot CAD assembly (Onshape, SolidWorks, or Inventor). Every value listed here maps directly into [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json).

### A. Chassis Frame Geometry
* **Where to find it in CAD:** Measure the outer frame perimeter and module centers with the CAD tape/measure tool.
* **Exact JSON Targets in `robot_swerve.json` (`"chassis"` object):**
  * `"trackwidth_m"`: Distance between the center of the left wheels and right wheels along the robot's lateral axis (e.g. `0.56`).
  * `"wheelbase_m"`: Distance between the center of the front wheels and back wheels along the robot's longitudinal axis (e.g. `0.56`).
  * `"width_m"`: Outer bumper/frame width along the X axis (e.g. `0.70`).
  * `"length_m"`: Outer bumper/frame length along the Z axis (e.g. `0.70`).
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
  *(Note: BananaSim automatically converts standard NWU CAD inertia coordinates to the engine coordinate system in `ChassisParams::GetJoltInertiaTensor()`.)*

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
  * `"motors.drive.gearRatio"`: Total reduction from drive motor to wheel (e.g. `5.27` for MK4n L3).
  * `"motors.steer.gearRatio"`: Total reduction from steer motor to azimuth ring (e.g. `26.0909` for MK4n).
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

## 3. Physical Hardware Measurements

Perform these physical checks on the real robot to calibrate parameters that differ from pure theoretical CAD.

### A. Total Mass Calibration
* **Tool:** Scale (or 4 corner scales summed together).
* **Condition:** Robot must be weighed with battery installed, main breaker on, and bumpers mounted.
* **JSON Field:** `chassis.mass_kg`

### B. Wheel Radius (Tread Wear)
* **Tool:** Digital calipers.
* **Procedure:** Measure the outer diameter across the tread on all four wheels while mounted. Standard 4-inch wheels with fresh tread measure $101.6\text{ mm}$ ($r = 0.0508\text{ m}$), but worn nitrile/billet tread frequently wears down to $96 - 98\text{ mm}$ ($r = 0.0480 - 0.0490\text{ m}$).
* **JSON Field:** `wheel.radius_m`

### C. Battery Internal Resistance & Voltage
* **Tool:** Battery Beak or 12V battery analyzer.
* **Procedure:** Measure a fresh competition battery:
  * Open-circuit voltage ($V_{nominal}$, typically $12.60\text{ V} - 12.80\text{ V}$).
  * Internal resistance ($R_{int}$, typically $0.018 - 0.024\ \Omega$).
* **JSON Fields:**
  * `battery.nominalVoltage_V`
  * `battery.internalResistance_ohm`

### D. Drivetrain Mechanical Friction Calibration
* **Setup:** Place the robot on blocks with wheels suspended in the air.
* **Procedure:** Command the drive motors to full speed ($12\text{ V}$) via Phoenix Tuner X or manual test code.
  * Measure steady-state wheel velocity ($\text{RPS}$).
  * A Kraken X60 has a theoretical free speed of $\sim 6000\text{ RPM}$ ($100\text{ RPS}$). If the rotor reaches $94\text{ RPS}$ in the air, the drivetrain consumes $\sim 6\%$ in friction.
* **JSON Fields:**
  * `motors.drive.efficiency` (typically $0.94 - 0.96$)
  * `driveModule.coulombFriction_Nm` (typically $1.5 - 2.0\text{ Nm}$)

---

## 4. Electronics, CAN Bus & Sensor Configuration

FRCBananaSim's network bridge communicates directly with CTRE Phoenix 6 native devices over the WPILib HALSim WebSocket protocol.

### A. CAN Bus ID Mapping
Verify that the CAN IDs in [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json) match your physical robot:

| Component | Physical Device | JSON Path | Default / Competition ID |
| :--- | :--- | :--- | :--- |
| **Front Left Drive** | Talon FX (v6) | `modules[0].driveCAN` | `0` |
| **Front Left Steer** | Talon FX (v6) | `modules[0].steerCAN` | `1` |
| **Front Left CANcoder** | CANcoder (v6) | `modules[0].canCoder` | `20` |
| **Front Right Drive** | Talon FX (v6) | `modules[1].driveCAN` | `2` |
| **Front Right Steer** | Talon FX (v6) | `modules[1].steerCAN` | `3` |
| **Front Right CANcoder** | CANcoder (v6) | `modules[1].canCoder` | `21` |
| **Back Left Drive** | Talon FX (v6) | `modules[2].driveCAN` | `4` |
| **Back Left Steer** | Talon FX (v6) | `modules[2].steerCAN` | `5` |
| **Back Left CANcoder** | CANcoder (v6) | `modules[2].canCoder` | `22` |
| **Back Right Drive** | Talon FX (v6) | `modules[3].driveCAN` | `6` |
| **Back Right Steer** | Talon FX (v6) | `modules[3].steerCAN` | `7` |
| **Back Right CANcoder** | CANcoder (v6) | `modules[3].canCoder` | `23` |
| **IMU Gyro** | Pigeon 2 (v6) | Handled in Bridge | `24` |

### B. Current Limits
* **Where to find it:** Check your robot code's `CurrentLimitsConfigs` on the drive and steer Talons.
* **JSON Fields:**
  * `motors.drive.statorCurrentLimit_A`
  * `motors.steer.statorCurrentLimit_A`
  * `battery.statorCurrentLimit_A`
  * `battery.supplyCurrentLimit_A`

---

## 5. Master Configuration Dictionary for [`robot_swerve.json`](file:///C:/Game%20Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots/robot_swerve.json)

This table lists every single property currently read by the C++ engine:

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
