# FRCBananaExporter: Autodesk Fusion 360 Robot Exporter Add-In

`FRCBananaExporter` is an Autodesk Fusion 360 Python add-in that exports jointed FRC/FTC robot CAD assemblies directly into **FRCBananaSim Robot Data Schema v1.0.0** (`manifest.json`, binary `.glb` visual meshes, and diagnostic report).

---

## 1. Quick Start: How to Install in Fusion 360

1. Open **Autodesk Fusion 360**.
2. Press **`Shift + S`** (or navigate to **Utilities $\to$ ADD-INS $\to$ Scripts and Add-Ins**).
3. Select the **Add-Ins** tab.
4. Click the green **`+`** (Add) icon next to "My Add-Ins".
5. Browse to and select this repository's folder:
   ```
   C:\Game Dev\C++\Projects\FRCBananaSim\FusionAddin
   ```
6. Select **`FRCBananaExporter`** in the list and click **Run**.
   *(Optional: Check **"Run on Startup"** to keep the tool ready whenever Fusion opens).*
7. You will now see the **"Export Robot to BananaSim"** button in Fusion's **Utilities $\to$ Add-Ins** toolbar panel!

---

## 2. Running Diagnostic API Probes (P1 – P5)

Before exporting a full robot for the first time, you can verify Fusion's internal API properties:
1. Open any model or test part in Fusion.
2. Press **`Shift + S`** $\to$ select the **Scripts** tab.
3. Click the green **`+`** icon and select:
   ```
   C:\Game Dev\C++\Projects\FRCBananaSim\FusionAddin\probes
   ```
4. Select **`run_probes`** and click **Run**.
5. Fusion will display a popup verifying:
   * **P1 (Units):** Length in cm, mass in kg, angles in radians.
   * **P2 (Proxy Coordinate Space):** Occurrence world transforms and native-to-proxy mapping.
   * **P3 (Inertia Product Sign):** Determines `FUSION_PRODUCT_SIGN`.
   * **P4 (Revolute Semantics):** Rotation axes and motion limits.
   * **P5 (Mesh Calculator):** TriangleMesh coordinates align with bounding boxes.

---

## 3. The CAD Authoring Contract

BananaSim calculates weight, CoM ($X, Y, Z$), and tip resistance directly from CAD. Follow these standards:

### A. Coordinate System Alignment
* **Front / Intake:** Fusion **$+Z$** (Forward in WPILib NWU)
* **Left Side:** Fusion **$+X$** (Left in WPILib NWU)
* **Top / Up:** Fusion **$+Y$** (Up in WPILib NWU)
* *Tip:* Our team's Fusion frame ($+X$ Left, $+Y$ Up, $+Z$ Forward) is **identical to BananaSim's Engine OpenGL frame**, so exported GLB meshes load with zero axis distortion!

### B. Robot Origin & Grounding
* By default, position the design origin `(0,0,0)` at **frame center on the floor plane ($Y = 0$)**.
* If your origin is elsewhere, create a Construction Point named **`ROBOT_ORIGIN`** at frame center on the floor.
* **Ground the chassis:** Exactly **one** base component must be grounded (Right-click chassis $\to$ **Ground**). This marks the root `base_link`.

### C. Physical Materials (Avoid 350 lb Robots!)
* Always assign **Modify $\to$ Physical Material** (e.g. `Aluminum 6061`, `Polycarbonate`, `Steel 4140`).
* Real motors (Kraken X60, Falcon 500, NEO) are dense: assign motor bodies **Steel 4140** (should weigh $\sim 1.1\text{ lbs}$ / $0.50\text{ kg}$).

### D. Moving Mechanisms & Rigid Groups
* Elevators: **Slider Joint** with physical limits.
* Arms & Wrists: **Revolute Joint** with physical limits.
* Bolted/Rigid parts: Assemble using **Rigid Groups** (the exporter merges them into single links).

### E. Branch Pruning & Visibility (Hiding Components)
* The exporter uses **hierarchical recursive branch pruning**:
  * If a parent component is hidden (eye/lightbulb toggled **OFF** in Fusion's browser tree), the exporter **skips that entire branch** and will not inspect any of its children, bodies, or joints.
  * Use this to keep test fixtures, prototypes, or field obstacles in your CAD without having them leak into the sim.

### F. Colliders & Sensor Zones
* **`COL_<name>`:** Solid bodies for collision geometry. Excluded from mass and visuals.
* **`ZONE_<role>_<name>`:** Non-colliding trigger volumes (beam breaks, intakes, shooters).
* **Visibility Rule for Proxies:**
  * If visible (eye **ON**): Exported immediately without question.
  * If hidden (eye **OFF**): The exporter will prompt you during export to confirm whether each hidden proxy should be included!

---

## 4. Configuration (`config/exporter_config.example.json`)

To customize settings for your robot, create `config/exporter_config.json`:
```json
{
  "configVersion": "1.0.0",
  "robotName": "TestAssembly2026",
  "outputDir": "C:/Game Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots",
  "sidecarPath": "./sidecar.json",
  "materialMapPath": "./material_map.json",
  "fusionUp": "+Y",
  "fusionForward": "+Z",
  "robotOrigin": "ROBOT_ORIGIN",
  "revoluteAxisSignShim": true,
  "includeHidden": false,
  "hiddenProxyAction": "prompt",
  "foldJoints": ["fl_*", "fr_*", "bl_*", "br_*"],
  "detachedVisualJoints": ["shooter_flywheel_axle"],
  "noCollider": ["shooter_flywheel_visual"],
  "meshQuality": "normal",
  "meshExclude": ["*Screw*", "*Nut*", "*Rivet*", "*Washer*"],
  "floatSignificantDigits": 9
}
```

---

## 5. Exporting Your Robot

1. Open your robot assembly in Fusion 360.
2. Ensure mechanisms are at or near their home/resting position.
3. Click **Export Robot to BananaSim** in the **Utilities** tab.
4. The add-in will:
   * Safely normalize joints to $0.0$ via `ZeroPoseGuard`.
   * Prune hidden subassembly branches.
   * Prompt for any hidden `COL_*` / `ZONE_*` proxies.
   * Cluster rigid links, construct the BFS kinematic tree, and fold swerve modules into `base_link`.
   * Cross-check `sidecar.json` with difflib "did-you-mean" suggestions.
   * Tessellate visible link meshes into binary `.glb` files.
   * Compute Gate 4 validation checks against Fusion's own whole-assembly mass properties.
   * Restore all joints back to their original positions.
5. Review the resulting **`export_report.md`** and launch `FRCBananaSim`!
