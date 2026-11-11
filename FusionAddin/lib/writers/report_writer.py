"""
report_writer.py: Generates machine-readable export_report.json and human-readable export_report.md.
Includes Gate 4 validation metrics (§5.4), mass breakdown by material, per-link stats,
and diagnostic code catalog.
"""

from datetime import datetime
import json
import os
from typing import Any, Dict, List, Optional, Tuple
from banana_core.diagnostics import DiagnosticCollector
from banana_core.linalg import mat3_frobenius_norm, vec3_norm, vec3_sub


def write_export_reports(
    report_data: Dict[str, Any],
    collector: DiagnosticCollector,
    output_dir: str,
) -> Tuple[str, str]:
    """
    Writes export_report.json and export_report.md to output_dir.
    Returns (json_path, md_path).
    """
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "export_report.json")
    md_path = os.path.join(output_dir, "export_report.md")

    status = "success"
    if collector.has_errors:
        status = "failed"
    elif collector.has_warnings:
        status = "warnings"

    report_obj = {
        "exportStatus": status,
        "timestamp": datetime.now().isoformat(),
        "summary": report_data.get("summary", {}),
        "validation": report_data.get("validation", {}),
        "links": report_data.get("links", []),
        "joints": report_data.get("joints", []),
        "motionLinks": report_data.get("motionLinks", []),
        "massByMaterial": report_data.get("massByMaterial", {}),
        "diagnostics": collector.to_list(),
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report_obj, f, indent=2)
        f.write("\n")

    # Generate Markdown Report
    summary = report_data.get("summary", {})
    val = report_data.get("validation", {})
    links = report_data.get("links", [])
    joints = report_data.get("joints", [])
    motion_links = report_data.get("motionLinks", [])
    mat_map = report_data.get("massByMaterial", {})

    status_badge = "✅ SUCCESS"
    if status == "failed":
        status_badge = "❌ FAILED (Errors detected)"
    elif status == "warnings":
        status_badge = "⚠️ WARNINGS (Review suggested)"

    lines = [
        f"# FRCBananaSim Robot Export Report",
        f"**Status:** {status_badge}  ",
        f"**Robot Name:** `{summary.get('robotName', 'Robot')}`  ",
        f"**Timestamp:** {report_obj['timestamp']}  ",
        "",
        "---",
        "",
        "## 1. Executive Summary",
        "",
        f"- **Total Mass:** {summary.get('totalMass_kg', 0.0):.4f} kg ({summary.get('totalMass_kg', 0.0)*2.20462:.2f} lbs)",
        f"- **Links Exported:** {summary.get('linkCount', 0)}",
        f"- **Kinematic Joints:** {summary.get('jointCount', 0)} ({summary.get('treeJointCount', 0)} tree, {summary.get('closureCount', 0)} loop closures, {summary.get('foldedCount', 0)} folded swerve)",
        f"- **Contact Zones:** {summary.get('zoneCount', 0)}",
        f"- **Total Triangles:** {summary.get('totalTriangles', 0):,}",
        "",
        "---",
        "",
        "## 2. Gate 4 Validation vs Fusion Ground Truth",
        "",
        "| Quantity | Exported Value | Fusion CAD Value | Error | Gate 4 Criteria | Pass? |",
        "|---|---|---|---|---|---|",
    ]

    m_exp = val.get("exported_mass_kg", 0.0)
    m_cad = val.get("cad_mass_kg", 0.0)
    m_err = val.get("mass_error_pct", 0.0)
    m_pass = "✅ PASS" if m_err <= 0.1 else "❌ FAIL"
    lines.append(f"| **Total Mass** | {m_exp:.4f} kg | {m_cad:.4f} kg | {m_err:.3f}% | $\\le 0.1\\%$ | {m_pass} |")

    com_dist = val.get("com_distance_mm", 0.0)
    com_pass = "✅ PASS" if com_dist <= 0.5 else "❌ FAIL"
    lines.append(f"| **Composite CoM** | — | — | {com_dist:.3f} mm | $\\le 0.5\\text{{ mm}}$ | {com_pass} |")

    i_err = val.get("inertia_error_pct", 0.0)
    i_pass = "✅ PASS" if i_err <= 0.5 else "❌ FAIL"
    lines.append(f"| **Inertia Tensor** | — | — | {i_err:.3f}% | $\\le 0.5\\%$ | {i_pass} |")

    lines.extend([
        "",
        "---",
        "",
        "## 3. Kinematic Joints & Motion Tree",
        "",
        "| Joint Name | Type | Parent Link | Child Link | Disposition | Limits |",
        "|---|---|---|---|---|---|",
    ])

    if not joints:
        lines.append("| *(none)* | — | — | — | — | — |")
    else:
        for j in joints:
            lim_str = "None"
            if j.get("has_limits"):
                unit = "m" if j.get("type") == "prismatic" else "rad"
                lim_str = f"[{j.get('lower_limit', 0.0):.2f}, {j.get('upper_limit', 0.0):.2f}] {unit}"
            lines.append(
                f"| `{j.get('name')}` | `{j.get('type')}` | `{j.get('parent_link')}` | `{j.get('child_link')}` | `{j.get('disposition')}` | {lim_str} |"
            )

    if motion_links:
        lines.extend([
            "",
            "---",
            "",
            "## 4. Fusion Motion Links",
            "",
            "| Motion Link | Joint 1 | Joint 2 | Ratio | Reversed |",
            "|---|---|---|---|---|",
        ])
        for ml in motion_links:
            rev_str = "Yes" if ml.get("is_reversed") else "No"
            lines.append(
                f"| `{ml.get('name')}` | `{ml.get('joint_one')}` | `{ml.get('joint_two')}` | {ml.get('ratio', 1.0):.4f} | {rev_str} |"
            )

    sec_links_num = "5" if motion_links else "4"
    sec_mat_num = "6" if motion_links else "5"
    sec_diag_num = "7" if motion_links else "6"

    lines.extend([
        "",
        "---",
        "",
        f"## {sec_links_num}. Link Breakdown Table",
        "",
        "| Link Name | Mass (kg) | CoM (X, Y, Z) [m] | Collider $\\Delta$CoM [mm] | Triangles | Primary Material |",
        "|---|---|---|---|---|---|",
    ])

    for l in links:
        com_str = f"[{l.get('com')[0]:.3f}, {l.get('com')[1]:.3f}, {l.get('com')[2]:.3f}]"
        lines.append(f"| `{l.get('name')}` | {l.get('mass_kg'):.3f} | {com_str} | {l.get('collider_offset_mm', 0.0):.1f} | {l.get('triangles', 0):,} | `{l.get('material')}` |")

    lines.extend([
        "",
        "---",
        "",
        f"## {sec_mat_num}. Mass by Material",
        "",
        "| Material | Mass (kg) | Percentage |",
        "|---|---|---|",
    ])

    tot_m = summary.get("totalMass_kg", 1.0)
    for mat, m in sorted(mat_map.items(), key=lambda x: x[1], reverse=True):
        pct = (m / tot_m) * 100.0 if tot_m > 0 else 0.0
        lines.append(f"| `{mat}` | {m:.3f} kg | {pct:.1f}% |")

    lines.extend([
        "",
        "---",
        "",
        f"## {sec_diag_num}. Diagnostics",
        "",
    ])

    if not collector.diagnostics:
        lines.append("*Zero errors or warnings.*")
    else:
        lines.append("| Severity | Code | Target | Message | Suggestion |")
        lines.append("|---|---|---|---|---|")
        for d in collector.diagnostics:
            sugg = d.suggestion if d.suggestion else "—"
            tgt = f"`{d.target}`" if d.target else "—"
            lines.append(f"| {d.severity.value} | `{d.code}` | {tgt} | {d.message} | {sugg} |")

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return json_path, md_path
