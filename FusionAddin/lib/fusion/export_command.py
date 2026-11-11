"""
export_command.py: Complete exporter workflow orchestrator for Autodesk Fusion 360.
Coordinates zero-pose capture, assembly traversal, graph construction, manifest generation,
sidecar verification, GLB tessellation, Gate-4 validation, and report writing.
"""

import json
import os
import shutil
import sys
import traceback
from typing import Any, Dict

ADDIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LIB_DIR = os.path.join(ADDIN_DIR, "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from banana_core.diagnostics import DiagnosticCollector
from banana_core.frames import FrameConverter
from banana_core.graph import build_kinematic_graph
from banana_core.inertia import InertiaTensor, compute_reference_properties
from banana_core.linalg import mat3_frobenius_norm, mat3_sub, vec3_norm, vec3_sub
from banana_core.manifest_builder import build_manifest_data
from banana_core.sidecar_check import cross_check_sidecar
from writers.manifest_writer import write_manifest_json
from writers.report_writer import write_export_reports
from .zero_pose import ExportAborted, ZeroPoseGuard
from .fusion_reader import read_assembly
from .mesh_reader import export_link_meshes

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False


def _load_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def run_export_dialog(app: Any, ui: Any):
    """Executes the export pipeline with user interaction."""
    if not IN_FUSION:
        print("Cannot run export outside Autodesk Fusion 360.")
        return

    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        ui.messageBox(
            "The active document is not a Fusion Design.\n"
            "Switch to the Design workspace with your robot assembly open, then export again."
        )
        return

    # 1. Config
    config_dir = os.path.join(ADDIN_DIR, "config")
    config_path = os.path.join(config_dir, "exporter_config.json")
    if not os.path.exists(config_path):
        config_path = os.path.join(config_dir, "exporter_config.example.json")
    if not os.path.exists(config_path):
        ui.messageBox(f"Exporter config file not found at:\n{config_path}")
        return
    try:
        config = _load_json(config_path)
    except Exception as e:
        ui.messageBox(f"Failed to parse config file:\n{config_path}\n\n{e}")
        return

    # 2. Material map
    material_map: Dict[str, str] = {}
    material_map_path = os.path.join(config_dir, "material_map.json")
    if os.path.exists(material_map_path):
        try:
            material_map = _load_json(material_map_path).get("mappings", {})
        except Exception:
            pass

    # 3. Output directory
    robot_name = config.get("robotName", "TestAssembly2026")
    base_output_dir = config.get("outputDir", "C:/Game Dev/C++/Projects/FRCBananaSim/FRCBananaSim/Resources/Robots")
    output_dir = os.path.join(base_output_dir, robot_name)
    os.makedirs(output_dir, exist_ok=True)

    # 4. Sidecar (optional input; copied byte-for-byte, never rewritten)
    sidecar_cfg = config.get("sidecarPath", "./sidecar.json")
    candidates = [
        sidecar_cfg if os.path.isabs(sidecar_cfg) else os.path.join(config_dir, sidecar_cfg),
        os.path.join(output_dir, "sidecar.json"),
        os.path.join(config_dir, "sidecar.json"),
    ]
    sidecar_path = None
    sidecar_data = None
    for cand in candidates:
        if cand and os.path.exists(cand):
            try:
                sidecar_data = _load_json(cand)
                sidecar_path = cand
                break
            except Exception:
                continue

    collector = DiagnosticCollector()

    def hidden_proxy_callback(body_name: str, occ_name: str) -> bool:
        msg = (
            f"Hidden proxy body '{body_name}' was found in '{occ_name}'.\n\n"
            "Export it as an active collision / sensor-zone proxy?"
        )
        res = ui.messageBox(msg, "Hidden Proxy Detected", adsk.core.MessageBoxButtonTypes.YesNoButtonType)
        return res == adsk.core.DialogResults.DialogYes

    try:
        frame_conv = FrameConverter(
            fusion_up=config.get("fusionUp", "+Y"),
            fusion_forward=config.get("fusionForward", "+Z"),
        )
    except Exception as e:
        ui.messageBox(f"Invalid frame configuration:\n{e}")
        return

    app.log(f"FRCBananaExporter: starting export of '{robot_name}'...")

    try:
        with ZeroPoseGuard(design, collector):
            # --- Read CAD ---
            assembly = read_assembly(app, design, config, collector, hidden_proxy_prompt_cb=hidden_proxy_callback)

            # --- Kinematic graph ---
            graph_res = build_kinematic_graph(
                assembly.occurrences, assembly.joints, assembly.rigid_groups,
                config, collector, sidecar=sidecar_data,
            )

            # --- Manifest ---
            built = build_manifest_data(
                graph_res,
                {
                    "structural": assembly.structural_bodies,
                    "collider": assembly.collider_bodies,
                    "zone": assembly.zone_bodies,
                },
                frame_conv, material_map, config, collector,
                origin_point_m=assembly.origin_point_m,
            )
            manifest = built["manifest"]

            # --- Sidecar cross-check (needs the final exported names) ---
            if sidecar_data:
                cross_check_sidecar(
                    sidecar_data,
                    {l["name"] for l in manifest["links"]},
                    {j["name"] for j in manifest["joints"]},
                    {z["name"] for z in manifest["contactZones"]},
                    graph_res.root_link,
                    collector,
                )
            else:
                collector.warn("W500", "No sidecar.json found; sidecar cross-check skipped.")

            write_manifest_json(manifest, os.path.join(output_dir, "manifest.json"),
                                digits=config.get("floatSignificantDigits", 9))

            if sidecar_path and os.path.exists(sidecar_path):
                dest = os.path.join(output_dir, "sidecar.json")
                try:
                    if not (os.path.exists(dest) and os.path.samefile(sidecar_path, dest)):
                        shutil.copyfile(sidecar_path, dest)
                except Exception:
                    pass

            # --- Meshes (link-local frame origins, NOT CoM) ---
            link_structural = {name: [] for name in graph_res.links}
            for b in assembly.structural_bodies:
                lname = graph_res.occ_to_link.get(b.occ_name)
                if lname in link_structural:
                    link_structural[lname].append(b)

            mesh_counts = export_link_meshes(
                manifest["links"], link_structural, built["link_frame_origins"],
                frame_conv, config, collector, output_dir,
                origin_point_m=assembly.origin_point_m,
            )

            # --- Gate 4 validation (§5.4) ---
            excluded = [
                (b.mass_kg, b.com_cm, InertiaTensor(*b.inertia_origin_kgcm2))
                for b in assembly.excluded_visible_proxies
            ]
            ref_mass, ref_com_cm, ref_I_com = compute_reference_properties(
                assembly.whole_design_mass_kg,
                assembly.whole_design_com_cm,
                InertiaTensor(*assembly.whole_design_inertia_kgcm2),
                excluded,
            )

            exp_mass = built["total_mass"]
            mass_err_pct = abs(exp_mass - ref_mass) / ref_mass * 100.0 if ref_mass > 0 else 0.0

            ref_com_m = frame_conv.fusion_to_nwu_point(ref_com_cm, origin_fusion_m=assembly.origin_point_m)
            com_dist_mm = vec3_norm(vec3_sub(built["total_com"], ref_com_m)) * 1000.0

            ref_I_nwu = frame_conv.fusion_to_nwu_inertia(ref_I_com.to_matrix())
            exp_I_nwu = built["total_inertia"].to_matrix()
            ref_norm = mat3_frobenius_norm(ref_I_nwu)
            inertia_err_pct = (mat3_frobenius_norm(mat3_sub(exp_I_nwu, ref_I_nwu)) / ref_norm * 100.0) if ref_norm > 0 else 0.0

            if assembly.pruned_occurrences or assembly.hidden_bodies_skipped:
                collector.warn(
                    "W292",
                    f"{assembly.pruned_occurrences} hidden occurrence branch(es) and {assembly.hidden_bodies_skipped} "
                    "hidden body(ies) were excluded. If Fusion's whole-design properties still count hidden geometry, "
                    "the Gate-4 numbers below include that mass. For a strict check, delete or unhide them temporarily.",
                )

            validation = {
                "exported_mass_kg": exp_mass,
                "cad_mass_kg": ref_mass,
                "mass_error_pct": mass_err_pct,
                "com_distance_mm": com_dist_mm,
                "inertia_error_pct": inertia_err_pct,
                "excluded_proxy_mass_kg": sum(e[0] for e in excluded),
                "product_sign": assembly.product_sign,
            }

            summary = {
                "robotName": robot_name,
                "totalMass_kg": exp_mass,
                "linkCount": len(manifest["links"]),
                "jointCount": len(graph_res.all_joints),
                "treeJointCount": len(manifest["joints"]),
                "closureCount": sum(1 for j in graph_res.all_joints if j.disposition.value == "closure"),
                "foldedCount": sum(1 for j in graph_res.all_joints if j.disposition.value == "folded"),
                "zoneCount": len(manifest["contactZones"]),
                "totalTriangles": sum(mesh_counts.values()),
            }

            links_report = [{
                "name": l["name"],
                "mass_kg": l["mass_kg"],
                "com": l["centerOfMass_m"],
                "collider_offset_mm": built["link_com_offsets"].get(l["name"], 0.0) * 1000.0,
                "triangles": mesh_counts.get(l["name"], 0),
                "material": l["material"],
                "occurrences": graph_res.links.get(l["name"], []),
            } for l in manifest["links"]]

            joints_report = [{
                "name": pj.name,
                "type": pj.joint_type,
                "parent_link": pj.parent_link,
                "child_link": pj.child_link,
                "disposition": pj.disposition.value,
                "occ1": pj.raw_joint.occ1_name,
                "occ2": pj.raw_joint.occ2_name,
                "has_limits": pj.has_limits,
                "lower_limit": pj.lower_limit,
                "upper_limit": pj.upper_limit,
            } for pj in graph_res.all_joints]

            motion_links_report = [{
                "name": ml.name,
                "joint_one": ml.joint_one_name,
                "joint_two": ml.joint_two_name,
                "ratio": ml.ratio,
                "is_reversed": ml.is_reversed,
            } for ml in assembly.motion_links]

            mass_by_mat: Dict[str, float] = {}
            for l in manifest["links"]:
                mass_by_mat[l["material"]] = mass_by_mat.get(l["material"], 0.0) + l["mass_kg"]

            _, md_path = write_export_reports(
                {
                    "summary": summary,
                    "validation": validation,
                    "links": links_report,
                    "joints": joints_report,
                    "motionLinks": motion_links_report,
                    "massByMaterial": mass_by_mat,
                },
                collector, output_dir,
            )

        # (joints are restored here, after the `with` block, even if anything above threw)
        status = "FAILED (errors)" if collector.has_errors else ("OK with warnings" if collector.has_warnings else "OK")
        prioritized_diags = sorted(
            [d for d in collector.diagnostics if d.code != "I295"],
            key=lambda d: (0 if d.severity == "ERROR" else (1 if d.severity == "WARN" else 2)),
        )
        top = prioritized_diags[:8]
        diag_lines = "\n".join(f"  [{d.code}] {d.message[:140]}" for d in top)
        more = f"\n  ... and {len(collector.diagnostics) - len(top)} more (see report)" if len(collector.diagnostics) > len(top) else ""
        ui.messageBox(
            f"FRCBananaSim export: {status}\n\n"
            f"Robot: {robot_name}\n"
            f"Links: {summary['linkCount']}   Joints: {summary['treeJointCount']}   Zones: {summary['zoneCount']}\n"
            f"Mass: {exp_mass:.3f} kg   (vs Fusion {ref_mass:.3f} kg, error {mass_err_pct:.3f}%)\n"
            f"CoM error: {com_dist_mm:.3f} mm   Inertia error: {inertia_err_pct:.3f}%\n"
            f"Triangles: {summary['totalTriangles']:,}\n"
            f"Errors: {collector.error_count}   Warnings: {collector.warning_count}\n\n"
            f"{diag_lines}{more}\n\n"
            f"Output: {output_dir}\nReport: {md_path}",
            "FRCBananaSim Exporter",
        )

    except ExportAborted as e:
        ui.messageBox(f"Export aborted (nothing was written):\n\n{e}", "FRCBananaSim Exporter")
    except Exception:
        ui.messageBox(f"Export failed with unhandled exception:\n{traceback.format_exc()}")
