"""
sidecar_check.py: Validates sidecar.json against exported kinematic links, tree joints, and contact zones.
Implements §6.9 Sidecar cross-check and 'did-you-mean' suggestions via difflib.
"""

import difflib
from typing import Any, Dict, List, Optional, Set
from .diagnostics import DiagnosticCollector

VALID_SERVO_PRESETS = {
    "GoBildaTorque",
    "GoBildaSpeed",
    "GenericStandard",
    "GenericMicro",
    "GenericContinuous",
    "Custom",
}



VALID_ACTUATOR_TYPES = {"dc_motor", "servo"}


def find_close_match(name: str, candidates: Set[str]) -> Optional[str]:
    matches = difflib.get_close_matches(name, list(candidates), n=1, cutoff=0.6)
    return matches[0] if matches else None


def cross_check_sidecar(
    sidecar: Dict[str, Any],
    exported_link_names: Set[str],
    exported_tree_joint_names: Set[str],
    exported_zone_names: Set[str],
    root_link_name: str,
    collector: DiagnosticCollector,
) -> None:
    """
    Checks all references in sidecar.json against the exported robot structure.
    Emits W501 with close-match suggestions on misses, and I502 on passive joints.
    """
    referenced_joints: Set[str] = set()

    def check_link(lname: str, field_path: str):
        if lname not in exported_link_names:
            suggestion = find_close_match(lname, exported_link_names)
            s_msg = f" Did you mean '{suggestion}'?" if suggestion else ""
            collector.warn(
                "W501",
                f"Link '{lname}' referenced in {field_path} was not found in exported links.{s_msg}",
                target=lname,
                suggestion=suggestion,
            )

    def check_joint(jname: str, field_path: str):
        referenced_joints.add(jname)
        if jname not in exported_tree_joint_names:
            suggestion = find_close_match(jname, exported_tree_joint_names)
            s_msg = f" Did you mean '{suggestion}'?" if suggestion else ""
            collector.warn(
                "W501",
                f"Joint '{jname}' referenced in {field_path} was not found in exported tree joints.{s_msg}",
                target=jname,
                suggestion=suggestion,
            )

    def check_zone(zname: str, field_path: str):
        if zname not in exported_zone_names:
            suggestion = find_close_match(zname, exported_zone_names)
            s_msg = f" Did you mean '{suggestion}'?" if suggestion else ""
            collector.warn(
                "W501",
                f"Contact zone '{zname}' referenced in {field_path} was not found in exported contact zones.{s_msg}",
                target=zname,
                suggestion=suggestion,
            )

    def check_servo_block(s_block: Dict[str, Any], field_path: str):
        preset = s_block.get("preset")
        if preset and preset not in VALID_SERVO_PRESETS:
            collector.warn("W501", f"Unknown servo preset '{preset}' in {field_path}. Valid presets: {sorted(list(VALID_SERVO_PRESETS))}")
        ch = s_block.get("pwmChannel")
        if ch is not None:
            if not isinstance(ch, int) or ch < 0 or ch > 15:
                collector.warn("W501", f"Invalid PWM channel {ch} in {field_path}. Must be integer in range [0, 15].")

    # 1. Swerve
    if "swerve" in sidecar:
        sw = sidecar["swerve"]
        c_link = sw.get("chassisLink")
        if c_link:
            check_link(c_link, "swerve.chassisLink")
            if c_link != root_link_name:
                collector.warn("W501", f"swerve.chassisLink '{c_link}' does not match root link '{root_link_name}'.")

        steer_act = sw.get("steerActuatorType", "dc_motor")
        if steer_act not in VALID_ACTUATOR_TYPES:
            collector.warn("W501", f"Invalid swerve.steerActuatorType '{steer_act}'.")
        if steer_act == "servo":
            preset = sw.get("steerServoPreset")
            if preset and preset not in VALID_SERVO_PRESETS:
                collector.warn("W501", f"Invalid swerve.steerServoPreset '{preset}'.")

        drive_act = sw.get("driveActuatorType", "dc_motor")
        if drive_act not in VALID_ACTUATOR_TYPES:
            collector.warn("W501", f"Invalid swerve.driveActuatorType '{drive_act}'.")
        if drive_act == "servo":
            preset = sw.get("driveServoPreset")
            if preset and preset not in VALID_SERVO_PRESETS:
                collector.warn("W501", f"Invalid swerve.driveServoPreset '{preset}'.")

        for i, mod in enumerate(sw.get("modules", [])):
            if steer_act == "servo" and "steerPwmChannel" in mod:
                ch = mod["steerPwmChannel"]
                if not isinstance(ch, int) or ch < 0 or ch > 15:
                    collector.warn("W501", f"Invalid steerPwmChannel {ch} in swerve.modules[{i}].")
            if drive_act == "servo" and "drivePwmChannel" in mod:
                ch = mod["drivePwmChannel"]
                if not isinstance(ch, int) or ch < 0 or ch > 15:
                    collector.warn("W501", f"Invalid drivePwmChannel {ch} in swerve.modules[{i}].")

    # 2. Mechanisms
    for i, mech in enumerate(sidecar.get("mechanisms", [])):
        path = f"mechanisms[{i}] ('{mech.get('name', 'unnamed')}')"
        tj = mech.get("targetJoint")
        if tj:
            check_joint(tj, f"{path}.targetJoint")

        tjs = mech.get("targetJoints")
        if tjs:
            if not isinstance(tjs, list) or len(tjs) != 2:
                collector.warn("W501", f"{path}.targetJoints must be an array of exactly 2 joints.")
            else:
                for j_idx, jn in enumerate(tjs):
                    check_joint(jn, f"{path}.targetJoints[{j_idx}]")

        pl = mech.get("parentLink")
        if pl:
            check_link(pl, f"{path}.parentLink")

        vl = mech.get("visualLink")
        if vl:
            check_link(vl, f"{path}.visualLink")

        act_type = mech.get("actuatorType", "dc_motor")
        if act_type not in VALID_ACTUATOR_TYPES:
            collector.warn("W501", f"Invalid actuatorType '{act_type}' in {path}.")
        if act_type == "servo":
            if "servo" in mech:
                check_servo_block(mech["servo"], f"{path}.servo")
            if "servoA" in mech:
                check_servo_block(mech["servoA"], f"{path}.servoA")
            if "servoB" in mech:
                check_servo_block(mech["servoB"], f"{path}.servoB")

    # 3. Couplings
    for i, coup in enumerate(sidecar.get("couplings", [])):
        path = f"couplings[{i}] ('{coup.get('name', 'unnamed')}')"
        for st in coup.get("cascadeStages", []):
            if "joint" in st:
                check_joint(st["joint"], f"{path}.cascadeStages[].joint")

        for key in ("crankJoint", "couplerJoint", "followerJoint"):
            if key in coup:
                check_joint(coup[key], f"{path}.{key}")

        if "closureParentLink" in coup:
            check_link(coup["closureParentLink"], f"{path}.closureParentLink")
        if "closureChildLink" in coup:
            check_link(coup["closureChildLink"], f"{path}.closureChildLink")

        pc = coup.get("powerCoupling")
        if pc:
            if "masterJoint" in pc:
                check_joint(pc["masterJoint"], f"{path}.powerCoupling.masterJoint")
            if "slaveJoint" in pc:
                check_joint(pc["slaveJoint"], f"{path}.powerCoupling.slaveJoint")

    # 4. Sensors
    for i, sens in enumerate(sidecar.get("sensors", [])):
        path = f"sensors[{i}] ('{sens.get('name', 'unnamed')}')"
        if "parentLink" in sens:
            check_link(sens["parentLink"], f"{path}.parentLink")
        if "targetJoint" in sens:
            check_joint(sens["targetJoint"], f"{path}.targetJoint")
        if "contactZone" in sens:
            check_zone(sens["contactZone"], f"{path}.contactZone")

    # Check for passive joints
    for tj in exported_tree_joint_names:
        if tj not in referenced_joints:
            collector.info("I502", f"Passive joint '{tj}' is not actuated by any mechanism or coupling.", target=tj)
