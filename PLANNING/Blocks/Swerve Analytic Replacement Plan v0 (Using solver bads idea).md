# FRCBananaSim — Swerve Physics & Coordinate Standardization Plan

**Scope:** Standardize the global and local coordinate frame across the entire physical simulation pipeline ($+X$ Right, $+Y$ Up, $+Z$ Forward), eliminate motor torque attenuation bugs, enforce stator current clamping, resolve continuous azimuth wrapping, implement battery brownout hysteresis, and establish the decoupled control bridge to CTRE Phoenix 6 / WPILib HALSim.

---

## 1. Standardized Coordinate Architecture

FRCBananaSim locks in a right-handed 3D Cartesian coordinate convention matching standard graphics/game physics while establishing an explicit transformation bridge to WPILib NWU:

```
          +Y (Vertical Up)
           ▲
           │
           │   ▲ +Z (Horizontal Forward)
           │  /
           │ /
           └──────────► +X (Horizontal Right)
```

### 1.1 Vector & Axis Semantics

| Coordinate Axis | World Semantic | Robot Direction | Local Joint / Collider Usage |
|---|---|---|---|
| **$+X$** | Horizontal Right | Starboard / Right side | **Wheel Axle Hinge Axis** $\hat{\mathbf{a}}_{\text{axle}} = (1, 0, 0)$. Positive torque rolls the wheel forward in $+Z$. |
| **$-X$** | Horizontal Left | Port / Left side | Left swerve module placement offset. |
| **$+Y$** | Vertical Up | Sky / Top of robot | **Steer Hinge Axis** $\hat{\mathbf{a}}_{\text{steer}} = (0, 1, 0)$. Module swivels around vertical axis. |
| **$-Y$** | Vertical Down | Ground / Carpet | Gravity vector: $\mathbf{g} = (0, -9.80665, 0)\text{ m/s}^2$. |
| **$+Z$** | Horizontal Forward | Front of robot | Direction of forward translational drive. |
| **$-Z$** | Horizontal Backward | Rear of robot | Direction of reverse translational drive. |

**Right-Handed Basis Verification:**
$$\hat{\mathbf{X}} \times \hat{\mathbf{Y}} = (1, 0, 0) \times (0, 1, 0) = (0, 0, 1) = \hat{\mathbf{Z}}$$

### 1.2 Swerve Module Spatial Layout

For a square $0.56\text{ m} \times 0.56\text{ m}$ wheelbase ($d = 0.28\text{ m}$ half-extent):

* **Module 0 — Front-Left (FL):** $(X = -0.28\text{ m}, Z = +0.28\text{ m})$
* **Module 1 — Front-Right (FR):** $(X = +0.28\text{ m}, Z = +0.28\text{ m})$
* **Module 2 — Back-Left (BL):** $(X = -0.28\text{ m}, Z = -0.28\text{ m})$
* **Module 3 — Back-Right (BR):** $(X = +0.28\text{ m}, Z = -0.28\text{ m})$

```
                  FRONT (+Z)
          FL [-0.28, +0.28]    FR [+0.28, +0.28]
                 ┌──────────────┐
                 │      ▲       │
                 │      │ +Z    │
   LEFT (-X)     │  ◄───┼       │     RIGHT (+X)
                 │  -X  │       │
                 │              │
                 └──────────────┘
          BL [-0.28, -0.28]    BR [+0.28, -0.28]
                   BACK (-Z)
```

### 1.3 Kinematic Alignment & Rolling Physics

1. **Wheel Axle Joint:**
   * Anchor: Center of wheel hub $(X_{\text{mod}}, Y_{\text{wheel}}, Z_{\text{mod}})$.
   * Rotation Axis: $\hat{\mathbf{a}}_{\text{axle}} = (1, 0, 0)$.
   * At the ground contact patch $\mathbf{r}_{\text{contact}} = (0, -R, 0)$, positive angular velocity $\vec{\omega} = (\omega_x, 0, 0)$ with $\omega_x > 0$ yields linear velocity:
     $$\mathbf{v}_{\text{contact}} = \vec{\omega} \times \mathbf{r} = (\omega_x, 0, 0) \times (0, -R, 0) = (0, 0, -\omega_x R)$$
   * The contact surface pushes backward in $-Z$, producing an equal-and-opposite carpet traction force propelling the robot forward in $+Z$:
     $$\mathbf{F}_{\text{carpet}} = (0, 0, +\mu N)$$

2. **Steer Azimuth Joint:**
   * Anchor: Steer housing center $(X_{\text{mod}}, Y_{\text{steer}}, Z_{\text{mod}})$.
   * Rotation Axis: $\hat{\mathbf{a}}_{\text{steer}} = (0, 1, 0)$ ($+Y$).
   * Positive torque swivels the steer assembly around $+Y$.

3. **Jolt Wheel Collider (`CylinderShape`):**
   * Jolt cylinders are oriented along the local $Y$-axis by default.
   * To align the cylinder disc with the horizontal axle ($X$), the collider must be rotated $90^\circ$ around $Z$:
     ```cpp
     wheelCollider.localRotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
     ```

### 1.4 CTRE Pigeon 2 IMU & WPILib NWU Mapping

WPILib and CTRE Pigeon 2 standard is North-West-Up (NWU) with counter-clockwise (CCW) positive yaw:

```
FRCBananaSim (Right-Handed):   +X = Right, +Y = Up,      +Z = Forward
WPILib NWU:                    +X = Forward, +Y = Left,  +Z = Up
```

Euler angle extraction from chassis orientation quaternion $\mathbf{q}$:
* Forward unit vector: $\mathbf{f} = \mathbf{q} \cdot (0, 0, 1)$
* Up unit vector: $\mathbf{u} = \mathbf{q} \cdot (0, 1, 0)$
* Right unit vector: $\mathbf{r} = \mathbf{q} \cdot (1, 0, 0)$
* **Yaw (CCW Positive):**
  $$\text{Yaw} = \text{atan2}(-f.x, f.z)$$
  * Facing $+Z$ (Forward): $\text{Yaw} = 0\text{ rad}$.
  * Turning Left (towards $-X$): $\text{Yaw} = +\frac{\pi}{2}\text{ rad}$ ($+90^\circ$, positive).
  * Turning Right (towards $+X$): $\text{Yaw} = -\frac{\pi}{2}\text{ rad}$ ($-90^\circ$, negative).
* **Yaw Rate (CCW Positive):**
  $$\text{YawRate} = -\omega_y$$
* **Pitch (Nose Up Positive):**
  $$\text{Pitch} = \text{asin}(f.y)$$
* **Roll (Right Bank Positive):**
  $$\text{Roll} = \text{atan2}(r.y, u.y)$$

---

## 2. Drivetrain Physics & Electrical Fixes

### Fix 1: Eliminate Output Torque Attenuation
* **Root Cause:** In `SwerveDriveSystem.cpp`, motor output torque was multiplied by $(J_{\text{load}} / J_{\text{effective}})$ ($\sim 0.29$) before being applied to Jolt, while `DCMotorModel::CalculateTorque` already accounts for semi-implicit back-EMF stability. This cut applied wheel torque to $\sim 17\%$ of actual output.
* **Correction:**
  1. Remove load-ratio scaling: apply full gearbox torque directly to Jolt hinge constraints via `JointComponent::appliedTorque`.
  2. In `CalculateTorque`, pass the chassis reflected load inertia for grounded drive wheels:
     $$J_{\text{load, drive}} = J_{\text{wheel}} + \frac{1}{4} M_{\text{chassis}} r_{\text{wheel}}^2 \approx 0.00084 + 12.5 \times (0.0508)^2 \approx 0.0331\text{ kg}\cdot\text{m}^2$$
  3. Day 4 `RobotSpawner` will directly inject reflected rotor inertia ($J_{\text{rotor}} G^2$) into Jolt body mass matrices via `BodyCreationSettings::mOverrideMassProperties`.

### Fix 2: Stator Current Limiting Clamping
* **Root Cause:** Stator current limits existed in `MotorCommand` but never clamped torque or current telemetry.
* **Correction:**
  Compute maximum gearbox torque allowed by the stator current limit and clamp:
  $$\tau_{\text{stator\_max}} = G \cdot \eta \cdot k_t \cdot I_{\text{stator\_limit}}$$
  $$\tau_{\text{applied}} = \text{clamp}(\tau_{\text{motor}}, -\tau_{\text{stator\_max}}, \tau_{\text{stator\_max}})$$
  $$I_{\text{stator, clamped}} = \min(I_{\text{stator}}, I_{\text{stator\_limit}})$$

### Fix 3: Continuous Steer Angle Tracking (Anti-Wrapping)
* **Root Cause:** `JPH::HingeConstraint::GetCurrentAngle()` wraps at $\pm\pi$. When passed directly to `SimOutputs::encoderPositions`, Phoenix 6 receives discontinuous $2\pi$ jumps.
* **Correction:**
  Accumulate unwrapped continuous angle in `SwerveModuleComponent`:
  ```cpp
  double currentHingeAngle = steerJoint->currentAngle;
  double delta = currentHingeAngle - module->lastHingeAngle;
  while (delta > 3.14159265358979323846)  delta -= 2.0 * 3.14159265358979323846;
  while (delta < -3.14159265358979323846) delta += 2.0 * 3.14159265358979323846;
  module->unwrappedSteerAngle += delta;
  module->lastHingeAngle = currentHingeAngle;
  module->currentSteerAngle = module->unwrappedSteerAngle;
  currentOutputs.encoderPositions[i] = module->unwrappedSteerAngle;
  ```

### Fix 4: Battery Brownout Schmidt Trigger Hysteresis
* **Root Cause:** A hard cutoff at $6.3\text{V}$ instantly drops bus current, causing terminal voltage to bounce back to $12.56\text{V}$, creating 200 Hz on/off chattering.
* **Correction:**
  Implement stateful hysteresis in `BatteryModel`:
  * Trip critical brownout when $V_{\text{terminal}} < 6.3\text{ V}$.
  * Only clear brownout state when $V_{\text{terminal}} > 7.5\text{ V}$.

### Fix 5: Neutral Mode Handling (Brake vs. Coast)
* **Root Cause:** Disabled robot forced zero torque (free coast), while $0\text{V}$ demand in Brake mode should actively damp angular motion.
* **Correction:**
  * Add `NeutralMode` enum (`Brake`, `Coast`) to `SimControlBoundary.h`.
  * When $V_{\text{cmd}} = 0$ or robot is disabled:
    * `Brake`: apply back-EMF damping torque $\tau = -D \cdot \omega$.
    * `Coast`: apply $0\text{ N}\cdot\text{m}$.

---

## 3. Phased Execution Roadmap

```mermaid
flowchart TD
    subgraph Phase 1: Electrical & Boundary Models
        B1["BatteryModel.h / .cpp<br>Add hysteresis state (6.3V trip / 7.5V clear)"]
        B2["SimControlBoundary.h<br>Add NeutralMode enum & struct members"]
        B3["SwerveModuleComponent.h<br>Add continuous unwrapped angle & update docs"]
    end

    subgraph Phase 2: Swerve Drive System Refactor
        S1["SwerveDriveSystem.cpp<br>Apply full gearbox torque (no load-ratio decimation)"]
        S2["SwerveDriveSystem.cpp<br>Enforce stator current clamp on torque & telemetry"]
        S3["SwerveDriveSystem.cpp<br>Continuous angle unwrapping for steer encoders"]
        S4["SwerveDriveSystem.cpp<br>CCW-positive 3D IMU Euler decomposition"]
    end

    subgraph Phase 3: Regression Suite Realignment
        T1["SwervePhysicalDriveTest.cpp<br>Standardize module coordinates (+X Right, +Z Forward)"]
        T2["SwervePhysicalDriveTest.cpp<br>Rotate wheel cylinder collider 90 deg around Z"]
        T3["SwervePhysicalDriveTest.cpp<br>Verify stall torque, stator clamp, steer unwrapping & IMU"]
        T4["CTest Execution<br>Verify 100% test pass in Release mode"]
    end

    Phase 1 --> Phase 2
    Phase 2 --> Phase 3
```

---

## 4. Verification & Gate Criteria

| Verification Metric | Target Value / Behavior | Status |
|---|---|---|
| **Drive Stall Torque** | Full Kraken X60 stall torque ($41.65\text{ N}\cdot\text{m}$ at wheel axle) applied without artificial reduction. | Pending Phase 2 |
| **Stator Current Clamp** | Locked-rotor torque strictly clamped to $G \cdot \eta \cdot k_t \cdot I_{\text{stator\_limit}}$. | Pending Phase 2 |
| **Continuous Steer Angle** | Steer encoder increments continuously across $\pm\pi$ without $2\pi$ wraps. | Pending Phase 2 |
| **Wheel Rolling Direction** | Positive torque on horizontal $+X$ axle accelerates wheel and chassis forward along $+Z$. | Pending Phase 3 |
| **Gyro IMU Alignment** | $0$ facing $+Z$, $+\frac{\pi}{2}$ facing Left ($-X$), $-\frac{\pi}{2}$ facing Right ($+X$). | Pending Phase 3 |
| **Brownout Stability** | No 200 Hz chattering under sustained heavy stall loads. | Pending Phase 1 |
| **CTest Suite** | All tests pass cleanly in Release build (`ctest -C Release`). | Pending Phase 3 |
