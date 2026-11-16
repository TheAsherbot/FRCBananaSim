# FRCBananaSim
Trying to make a FRC sim that can run at 100x speed. IDk if it is possible, but worth looking into.

## Coordinate Frame Convention
Across the Sidecar Configurator GUI, 3D viewports, and simulation dynamics:
- **`+X`**: Right side of the robot (`-X` is Left)
- **`+Y`**: Up towards the ceiling (`-Y` is Down towards the carpet floor)
- **`+Z`**: Front / Forward of the robot (`-Z` is Back / Reverse)

## Tools & Documentation
- [Sidecar Configurator & CAD Ingestion Guide](SIDECAR_CONFIGURATOR_README.md) — Comprehensive user manual for ingesting CAD assemblies, configuring swerve drivetrains, 1-DOF mechanisms, 2-DOF differential wrists, parametric FRC bumpers, sensor suites, and real-time schema validation.
- [Fusion 360 Add-in Guide](FusionAddin/README.md) — Exporter script for generating `manifest.json` and visual meshes directly from Autodesk Fusion 360.
- [Robot Configuration & Calibration Guide](Configuration%20%26%20Calibration%20Guide.md) — Living reference for mechanical and electrical parameters, workbench calibrations, materials, and master dictionary keys.

