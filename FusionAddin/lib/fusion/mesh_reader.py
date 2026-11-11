"""
mesh_reader.py: Tessellates CAD geometry using Fusion's MeshCalculator API,
converts vertices to link-local glTF/Engine coordinates, and writes binary GLB meshes.
Matches Block 4 Plan §6.8.
"""

import fnmatch
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

ADDIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LIB_DIR = os.path.join(ADDIN_DIR, "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from banana_core.diagnostics import DiagnosticCollector
from banana_core.frames import FrameConverter
from writers.glb_writer import write_glb

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False


def export_link_meshes(
    links_data: List[Dict[str, Any]],
    link_structural_bodies: Dict[str, List[Any]],
    link_frame_origins: Dict[str, Tuple[float, float, float]],
    frame_conv: FrameConverter,
    config: Dict[str, Any],
    collector: DiagnosticCollector,
    output_dir: str,
    origin_point_m: Optional[Tuple[float, float, float]] = None,
    progress_dialog: Optional[Any] = None,
) -> Dict[str, int]:
    """
    Tessellates visible structural bodies per link and writes binary GLBs.
    Returns mapping of link_name -> triangle_count.
    """
    mesh_counts: Dict[str, int] = {}
    if not IN_FUSION:
        return mesh_counts

    mesh_dir = os.path.join(output_dir, "Meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    mesh_quality_str = config.get("meshQuality", "normal").lower()
    mesh_exclude_patterns = config.get("meshExclude", ["*Screw*", "*Nut*", "*Rivet*", "*Washer*"])

    quality_map = {
        "low": adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh,
        "normal": adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh,
        "high": adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh,
    }
    selected_quality = quality_map.get(mesh_quality_str, adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)

    total_robot_triangles = 0

    for link_entry in links_data:
        lname = link_entry["name"]
        p_L = link_frame_origins.get(lname, (0.0, 0.0, 0.0))
        bodies = link_structural_bodies.get(lname, [])

        link_vertices: List[Tuple[float, float, float]] = []
        link_normals: List[Tuple[float, float, float]] = []
        link_indices: List[int] = []

        for b in bodies:
            fusion_body = getattr(b, "fusion_body", None)
            if not fusion_body:
                continue

            # Hardware exclude check (§6.8): match the body name OR its occurrence path, since FRC
            # hardware is usually a component named 'Screw ...' holding a body with a generic name.
            b_name = b.name
            occ_leaf = b.occ_name.replace("+", "/").split("/")[-1]
            if any(
                fnmatch.fnmatch(b_name, pat) or fnmatch.fnmatch(occ_leaf, pat)
                for pat in mesh_exclude_patterns
            ):
                continue

            try:
                mesh_mgr = getattr(fusion_body, "meshManager", None)
                if not mesh_mgr:
                    continue
                calc = mesh_mgr.createMeshCalculator()
                calc.setQuality(selected_quality)
                tri_mesh = calc.calculate()
                if not tri_mesh:
                    continue

                coords = getattr(tri_mesh, "nodeCoordinatesAsDouble", None)
                if not coords:
                    continue
                norms = getattr(tri_mesh, "normalVectorsAsDouble", None)
                raw_idxs = getattr(tri_mesh, "nodeIndices", None) or getattr(tri_mesh, "triangleNodeIndices", None)
                if not raw_idxs:
                    continue

                base_vertex_idx = len(link_vertices)
                has_norms = bool(norms and len(norms) == len(coords))

                # Process vertices and normals
                for i in range(0, len(coords), 3):
                    # Vertex in Fusion cm
                    pF_cm = (coords[i], coords[i + 1], coords[i + 2])
                    # NWU in meters
                    pN_m = frame_conv.fusion_to_nwu_point(pF_cm, origin_fusion_m=origin_point_m)
                    # Link-local NWU: pN_m - p_L
                    pLocal_nwu = (pN_m[0] - p_L[0], pN_m[1] - p_L[1], pN_m[2] - p_L[2])
                    # Engine / glTF convention: (y_N, z_N, x_N)
                    p_engine = (pLocal_nwu[1], pLocal_nwu[2], pLocal_nwu[0])
                    link_vertices.append(p_engine)

                    # Normal vector
                    if has_norms:
                        nF = (norms[i], norms[i + 1], norms[i + 2])
                        nN = frame_conv.fusion_to_nwu_vector(nF)
                        n_engine = (nN[1], nN[2], nN[0])
                        link_normals.append(n_engine)
                    else:
                        link_normals.append((0.0, 1.0, 0.0))

                for idx in raw_idxs:
                    link_indices.append(base_vertex_idx + int(idx))

            except Exception as e:
                collector.warn("W281", f"Failed to tessellate body '{b_name}' for link '{lname}': {e}", target=b_name)

        # Write GLB file for link
        glb_path = os.path.join(mesh_dir, f"{lname}.glb")
        tris = write_glb(link_vertices, link_normals, link_indices, glb_path, mesh_name=lname)
        mesh_counts[lname] = tris
        total_robot_triangles += tris

        if tris > 150000:
            collector.warn("W280", f"Link '{lname}' triangle budget exceeded ({tris:,} > 150,000 triangles).", target=lname)

    if total_robot_triangles > 500000:
        collector.warn("W280", f"Robot total triangle budget exceeded ({total_robot_triangles:,} > 500,000 triangles).")

    return mesh_counts
