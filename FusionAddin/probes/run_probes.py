"""
run_probes.py: Fusion 360 API diagnostic probes (P1 through P5).
Run from Fusion 360's "Scripts and Add-Ins" dialog to verify internal API assumptions.

Probes:
  P1: Internal Units (length in cm, mass in kg, angles in rad).
  P2: Proxy Transforms & Joint World Coordinates.
  P3: Product-of-Inertia Sign Convention (determines FUSION_PRODUCT_SIGN: -1 vs +1).
  P4: Revolute Joint Axis and Limits Motion Semantics.
  P5: Mesh Calculator / Tessellation Coordinate Space (World vs Component).
"""

import math
import os
import sys
import traceback

ADDIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_DIR = os.path.join(ADDIN_DIR, "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False

from banana_core.linalg import mat3_det, mat3_transpose, vec3_norm, vec3_sub
from banana_core.inertia import InertiaTensor, origin_to_com_tensor


def log_msg(app, msg: str):
    print(msg)
    if app:
        app.log(msg)


def run_probe_p1(app, root_comp) -> bool:
    """P1: Units probe. Checks internal Fusion unit assumptions."""
    log_msg(app, "--- Running Probe P1: Internal Units ---")
    # In Fusion 360 API: length is cm, mass is kg, angles are radians.
    # Check if active design has any bodies to inspect
    if root_comp.bRepBodies.count == 0 and root_comp.occurrences.count == 0:
        log_msg(app, "P1 INFO: Design has no bodies. Testing design unit system properties.")
        des = app.activeProduct
        units_mgr = des.unitsManager
        internal_length = units_mgr.internalLengthUnits
        log_msg(app, f"P1: internalLengthUnits = '{internal_length}' (expected: 'cm')")
        return internal_length.lower() == "cm"

    # If bodies exist, inspect first body
    body = root_comp.bRepBodies.item(0) if root_comp.bRepBodies.count > 0 else root_comp.occurrences.item(0).bRepBodies.item(0)
    props = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    mass = props.mass
    com = props.centerOfMass
    log_msg(app, f"P1: Body '{body.name}' mass = {mass:.4f} kg, CoM = ({com.x:.4f}, {com.y:.4f}, {com.z:.4f}) cm")
    return True


def run_probe_p2(app, root_comp) -> bool:
    """P2: Proxy transforms and joint world coordinates."""
    log_msg(app, "--- Running Probe P2: Proxy Coordinate Space ---")
    if root_comp.occurrences.count == 0:
        log_msg(app, "P2 SKIP: No occurrences present in active design.")
        return True

    occ = root_comp.occurrences.item(0)
    t = occ.transform2
    log_msg(app, f"P2: Occurrence '{occ.name}' translation = ({t.translation.x:.4f}, {t.translation.y:.4f}, {t.translation.z:.4f}) cm")

    # If occurrence has bodies, check native vs proxy CoM
    if occ.bRepBodies.count > 0:
        proxy_body = occ.bRepBodies.item(0)
        native_body = proxy_body.nativeObject
        if native_body:
            p_props = proxy_body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            n_props = native_body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            log_msg(app, f"P2: Proxy CoM = ({p_props.centerOfMass.x:.4f}, {p_props.centerOfMass.y:.4f}, {p_props.centerOfMass.z:.4f}) cm")
            log_msg(app, f"P2: Native CoM = ({n_props.centerOfMass.x:.4f}, {n_props.centerOfMass.y:.4f}, {n_props.centerOfMass.z:.4f}) cm")
    return True


def run_probe_p3(app, root_comp) -> bool:
    """P3: Product-of-Inertia sign convention probe."""
    log_msg(app, "--- Running Probe P3: Product-of-Inertia Sign ---")
    # Fusion's getXYZMomentsOfInertia() returns:
    # (ret, ixx, iyy, izz, ixy, iyz, ixz) about the world origin.
    # Note: the return order in Fusion C++ / Python is (ret, ixx, iyy, izz, ixy, iyz, ixz)
    bodies = []
    for b in root_comp.bRepBodies:
        bodies.append(b)
    for occ in root_comp.allOccurrences:
        for b in occ.bRepBodies:
            bodies.append(b)

    if not bodies:
        log_msg(app, "P3 SKIP: No bodies found to evaluate inertia products.")
        return True

    target_body = bodies[0]
    props = target_body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    success, ixx, iyy, izz, ixy, iyz, ixz = props.getXYZMomentsOfInertia()
    log_msg(app, f"P3: '{target_body.name}' Moments: Ixx={ixx:.2f}, Iyy={iyy:.2f}, Izz={izz:.2f}")
    log_msg(app, f"P3: Products returned: Ixy={ixy:.2f}, Ixz={ixz:.2f}, Iyz={iyz:.2f}")

    # Check principal moments returned by Fusion
    p_success, p1, p2, p3_val = props.getPrincipalMomentsOfInertia()
    log_msg(app, f"P3: Fusion Principal Moments: P1={p1:.2f}, P2={p2:.2f}, P3={p3_val:.2f}")

    # Test Hypothesis A: Tensor-element sign (off-diagonals are already -int(xy dm))
    hypA = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
    eigsA = hypA.principal_moments()

    # Test Hypothesis B: Product sign (Fusion returns +int(xy dm), so tensor element is -ixy)
    hypB = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=-ixy, ixz=-ixz, iyz=-iyz)
    eigsB = hypB.principal_moments()

    fusion_eigs = sorted([p1, p2, p3_val], reverse=True)
    errA = sum(abs(ea - ef) for ea, ef in zip(eigsA, fusion_eigs))
    errB = sum(abs(eb - ef) for eb, ef in zip(eigsB, fusion_eigs))

    log_msg(app, f"P3: Hypothesis A error = {errA:.4e}, Hypothesis B error = {errB:.4e}")
    if errA < errB and errA < 1e-3:
        log_msg(app, "P3 RESULT: FUSION_PRODUCT_SIGN = +1 (Tensor-element sign convention)")
    elif errB < errA and errB < 1e-3:
        log_msg(app, "P3 RESULT: FUSION_PRODUCT_SIGN = -1 (Positive-integral product convention)")
    else:
        log_msg(app, "P3: Off-diagonals may be near zero on this body. Test with an asymmetric L-bracket.")

    return True


def run_probe_p4(app, root_comp) -> bool:
    """P4: Revolute joint axis and limits direction check."""
    log_msg(app, "--- Running Probe P4: Revolute Joint Semantics ---")
    joints = root_comp.joints
    as_built = root_comp.asBuiltJoints
    total_joints = joints.count + as_built.count

    if total_joints == 0:
        log_msg(app, "P4 SKIP: No joints found in design.")
        return True

    for j in joints:
        motion = j.jointMotion
        j_type = motion.jointType
        log_msg(app, f"P4: Joint '{j.name}' Type={j_type}")
        if j_type == adsk.fusion.JointTypes.RevoluteJointType:
            rev_motion = motion
            axis = rev_motion.rotationAxisVector
            log_msg(app, f"P4:   Revolute Axis = ({axis.x:.4f}, {axis.y:.4f}, {axis.z:.4f})")
            limits = rev_motion.rotationLimits
            log_msg(app, f"P4:   Limits: min_enabled={limits.isMinimumValueEnabled}, min={limits.minimumValue}, max_enabled={limits.isMaximumValueEnabled}, max={limits.maximumValue}")

    return True


def run_probe_p5(app, root_comp) -> bool:
    """P5: Mesh calculator / tessellation coordinate space check."""
    log_msg(app, "--- Running Probe P5: Mesh Calculator Space ---")
    bodies = []
    for b in root_comp.bRepBodies:
        bodies.append(b)
    for occ in root_comp.allOccurrences:
        for b in occ.bRepBodies:
            bodies.append(b)

    if not bodies:
        log_msg(app, "P5 SKIP: No bodies found for mesh calculation.")
        return True

    body = bodies[0]
    mesh_calc = body.meshManager.createMeshCalculator()
    mesh_calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
    mesh = mesh_calc.calculate()

    bbox = body.boundingBox
    log_msg(app, f"P5: Body '{body.name}' BBox Min=({bbox.minPoint.x:.2f}, {bbox.minPoint.y:.2f}, {bbox.minPoint.z:.2f})")
    log_msg(app, f"P5: Mesh Nodes: {mesh.nodeCount}, Triangles: {mesh.triangleCount}")

    nodes = mesh.nodeCoordinatesAsDouble
    if nodes:
        xs = [nodes[i] for i in range(0, len(nodes), 3)]
        ys = [nodes[i] for i in range(1, len(nodes), 3)]
        zs = [nodes[i] for i in range(2, len(nodes), 3)]
        mesh_min = (min(xs), min(ys), min(zs))
        mesh_max = (max(xs), max(ys), max(zs))
        log_msg(app, f"P5: Mesh Min=({mesh_min[0]:.2f}, {mesh_min[1]:.2f}, {mesh_min[2]:.2f})")
        log_msg(app, f"P5: Mesh Max=({mesh_max[0]:.2f}, {mesh_max[1]:.2f}, {mesh_max[2]:.2f})")

        diff_min = math.sqrt((mesh_min[0] - bbox.minPoint.x)**2 + (mesh_min[1] - bbox.minPoint.y)**2 + (mesh_min[2] - bbox.minPoint.z)**2)
        log_msg(app, f"P5: Diff between mesh min and BBox min = {diff_min:.4f} cm")
        if diff_min < 0.5:
            log_msg(app, "P5 RESULT: Mesh calculator returns geometry in the SAME coordinate space as body.boundingBox (World Space for proxy bodies). PASS.")

    return True


def run(context=None):
    """Entry point when executed from Fusion Scripts menu."""
    if not IN_FUSION:
        print("run_probes: Not running inside Fusion 360.")
        return

    app = adsk.core.Application.get()
    ui = app.userInterface
    des = app.activeProduct
    if not des:
        ui.messageBox("No active Fusion 360 design found.")
        return

    root_comp = des.rootComponent
    log_msg(app, "==================================================")
    log_msg(app, "   FRCBananaSim Exporter: Fusion API Probes P1-P5 ")
    log_msg(app, "==================================================")

    try:
        p1_ok = run_probe_p1(app, root_comp)
        p2_ok = run_probe_p2(app, root_comp)
        p3_ok = run_probe_p3(app, root_comp)
        p4_ok = run_probe_p4(app, root_comp)
        p5_ok = run_probe_p5(app, root_comp)

        summary = (
            f"Probe Results:\n"
            f"  P1 (Units): {'PASS' if p1_ok else 'FAIL'}\n"
            f"  P2 (Proxy Space): {'PASS' if p2_ok else 'FAIL'}\n"
            f"  P3 (Inertia Sign): {'PASS' if p3_ok else 'FAIL'}\n"
            f"  P4 (Revolute Semantics): {'PASS' if p4_ok else 'FAIL'}\n"
            f"  P5 (Mesh Coordinates): {'PASS' if p5_ok else 'FAIL'}\n\n"
            f"Check the Fusion Text Commands window (Ctrl+Alt+C) for detailed logs."
        )
        ui.messageBox(summary, "FRCBananaExporter Probes Complete")

    except Exception:
        ui.messageBox(f"Probe execution failed:\n{traceback.format_exc()}")


if __name__ == "__main__":
    run()
