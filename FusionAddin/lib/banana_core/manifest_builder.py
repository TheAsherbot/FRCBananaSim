"""
manifest_builder.py: Builds Schema v1.0.0 manifest.json data structure from extracted CAD data,
kinematic graph results, and configuration.
Matches Block 4 Plan §5 and §6, and the key names parsed by RobotSchemaLoader.
"""

from typing import Any, Dict, List, Optional, Tuple

from .diagnostics import DiagnosticCollector
from .frames import FrameConverter
from .graph import JointDisposition, KinematicGraphResult
from .inertia import (
    BodyMassProps,
    InertiaTensor,
    aggregate_link_mass_props,
    origin_to_com_tensor,
)
from .linalg import vec3_norm, vec3_sub

VALID_ZONE_ROLES = ("intake", "shooter", "indexer")


def _r6(v: Tuple[float, float, float]) -> List[float]:
    return [round(v[0], 6), round(v[1], 6), round(v[2], 6)]


def split_zone_name(body_name: str) -> Tuple[str, str]:
    """
    'ZONE_intake_capture_zone' -> ('intake_capture_zone', 'intake').
    The exported zone name has no 'ZONE_' prefix (matches Reference2026 and sidecar `contactZone`),
    and the role is the first token after the prefix.
    """
    rest = body_name[5:] if body_name.upper().startswith("ZONE_") else body_name
    role = rest.split("_", 1)[0].lower()
    return rest, role


def split_collider_name(body_name: str) -> str:
    """'COL_chassis_box' -> 'chassis_box'."""
    return body_name[4:] if body_name.upper().startswith("COL_") else body_name


def _aabb_nwu(
    frame_conv: FrameConverter,
    bbox_min_cm: Tuple[float, float, float],
    bbox_max_cm: Tuple[float, float, float],
    origin_point_m: Optional[Tuple[float, float, float]],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Returns (center, dimensions) of a Fusion world AABB, expressed in NWU meters.
    The Fusion->NWU map is a pure axis permutation (possibly with sign flips), so the
    AABB maps to an AABB: center = midpoint of the converted corners, dims = abs(diff)."""
    pmin = frame_conv.fusion_to_nwu_point(bbox_min_cm, origin_fusion_m=origin_point_m)
    pmax = frame_conv.fusion_to_nwu_point(bbox_max_cm, origin_fusion_m=origin_point_m)
    center = ((pmin[0] + pmax[0]) * 0.5, (pmin[1] + pmax[1]) * 0.5, (pmin[2] + pmax[2]) * 0.5)
    dims = (abs(pmax[0] - pmin[0]), abs(pmax[1] - pmin[1]), abs(pmax[2] - pmin[2]))
    return center, dims


def build_manifest_data(
    graph_res: KinematicGraphResult,
    assembly_bodies: Dict[str, Any],  # 'structural', 'collider', 'zone'
    frame_conv: FrameConverter,
    material_map: Dict[str, str],
    config: Dict[str, Any],
    collector: DiagnosticCollector,
    origin_point_m: Optional[Tuple[float, float, float]] = None,
) -> Dict[str, Any]:
    """Constructs the complete Schema v1 manifest dictionary plus report side-data."""
    robot_name = config.get("robotName", "Robot")
    revolute_shim = config.get("revoluteAxisSignShim", True)
    no_collider_links = set(config.get("noCollider", []))
    joint_overrides = config.get("jointOverrides", {})
    zone_configs = config.get("contactZones", {})

    structural_bodies = assembly_bodies.get("structural", [])
    collider_bodies = assembly_bodies.get("collider", [])
    zone_bodies = assembly_bodies.get("zone", [])

    link_structural: Dict[str, List[Any]] = {lname: [] for lname in graph_res.links}
    link_colliders: Dict[str, List[Any]] = {lname: [] for lname in graph_res.links}

    for b in structural_bodies:
        lname = graph_res.occ_to_link.get(b.occ_name)
        if lname in link_structural:
            link_structural[lname].append(b)
    for b in collider_bodies:
        lname = graph_res.occ_to_link.get(b.occ_name)
        if lname in link_colliders:
            link_colliders[lname].append(b)

    # ------------------------------------------------------------------
    # 1. Link frame origins p_L in NWU meters (§2.1, §5.5)
    #    Root = (0,0,0). Child link frame sits at its parent joint's origin.
    # ------------------------------------------------------------------
    link_frame_origins: Dict[str, Tuple[float, float, float]] = {graph_res.root_link: (0.0, 0.0, 0.0)}

    def joint_world_nwu(pj) -> Tuple[float, float, float]:
        o = pj.raw_joint.origin_m  # Fusion axes, meters (already scaled from cm)
        return frame_conv.fusion_to_nwu_point((o[0] * 100.0, o[1] * 100.0, o[2] * 100.0), origin_fusion_m=origin_point_m)

    joint_world: Dict[str, Tuple[float, float, float]] = {}
    for pj in graph_res.tree_joints:
        j_nwu = joint_world_nwu(pj)
        joint_world[pj.name] = j_nwu
        link_frame_origins[pj.child_link] = j_nwu

    # Detached visual links (flywheel, rollers): frame sits at their (dropped) axle joint.
    for pj in graph_res.all_joints:
        if pj.disposition == JointDisposition.DETACHED:
            j_nwu = joint_world_nwu(pj)
            for ln in (pj.child_link, pj.parent_link):
                if ln not in link_frame_origins:
                    link_frame_origins[ln] = j_nwu

    for lname in graph_res.links:
        link_frame_origins.setdefault(lname, (0.0, 0.0, 0.0))

    # ------------------------------------------------------------------
    # 2. Links (BFS order)
    # ------------------------------------------------------------------
    order = list(graph_res.link_order) if graph_res.link_order else list(graph_res.links.keys())
    for lname in graph_res.links:
        if lname not in order:
            order.append(lname)

    manifest_links: List[Dict[str, Any]] = []
    link_com_offsets: Dict[str, float] = {}
    all_body_props: List[BodyMassProps] = []
    min_z_nwu = None
    unmapped_materials_warned: set = set()

    for lname in order:
        is_root = (lname == graph_res.root_link)
        p_L = link_frame_origins[lname]
        bodies = link_structural[lname]
        body_props_list: List[BodyMassProps] = []
        material_weights: Dict[str, float] = {}

        for b in bodies:
            m_b = b.mass_kg
            com_N = frame_conv.fusion_to_nwu_point(b.com_cm, origin_fusion_m=origin_point_m)

            ixx, iyy, izz, ixy, ixz, iyz = b.inertia_origin_kgcm2
            I_orig = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
            I_com_F = origin_to_com_tensor(I_orig, m_b, b.com_cm)
            I_com_N = InertiaTensor.from_matrix(frame_conv.fusion_to_nwu_inertia(I_com_F.to_matrix()))

            b_prop = BodyMassProps(mass_kg=m_b, com=com_N, inertia_at_com=I_com_N)
            body_props_list.append(b_prop)
            all_body_props.append(b_prop)

            mapped_mat = material_map.get(b.material_name)
            if not mapped_mat:
                mapped_mat = b.material_name.lower().replace(" ", "_")
                if b.material_name not in unmapped_materials_warned:
                    unmapped_materials_warned.add(b.material_name)
                    collector.warn(
                        "W270",
                        f"Unmapped Fusion material '{b.material_name}' (first seen on '{b.name}'). Falling back to '{mapped_mat}'.",
                        target=b.name,
                    )
            material_weights[mapped_mat] = material_weights.get(mapped_mat, 0.0) + m_b

            for z in (frame_conv.fusion_to_nwu_point(b.bbox_min_cm, origin_fusion_m=origin_point_m)[2],
                      frame_conv.fusion_to_nwu_point(b.bbox_max_cm, origin_fusion_m=origin_point_m)[2]):
                min_z_nwu = z if min_z_nwu is None else min(min_z_nwu, z)

        if body_props_list:
            m_L, c_L_N, I_L_N = aggregate_link_mass_props(body_props_list)
        else:
            collector.error(
                "E301",
                f"Link '{lname}' has no mass-bearing bodies (zero mass). Occurrences: {graph_res.links.get(lname, [])}",
                target=lname,
            )
            m_L, c_L_N, I_L_N = 0.0, p_L, InertiaTensor()

        local_com = (c_L_N[0] - p_L[0], c_L_N[1] - p_L[1], c_L_N[2] - p_L[2])

        if body_props_list:
            if m_L <= 1e-6:
                collector.error("E301", f"Link '{lname}' has zero mass.", target=lname)
            if not I_L_N.is_positive_definite():
                collector.error("E302", f"Link '{lname}' inertia tensor is not positive-definite.", target=lname)
            elif not I_L_N.passes_loader_check():
                collector.warn(
                    "W304",
                    f"Link '{lname}' inertia is physically valid but too small for the C++ loader's absolute "
                    f"1e-9 positive-definite test (RobotSchemaLoader::Validate will report an error). "
                    f"Merge it into a neighbouring link, or relax InertiaTensor::IsPositiveDefinite().",
                    target=lname,
                )
            tri_ok, _ = I_L_N.check_triangle_inequality()
            if not tri_ok:
                collector.warn(
                    "W303",
                    f"Link '{lname}' has nonphysical principal moments (violates triangle inequality).",
                    target=lname,
                )

        chosen_material = "aluminum_6061"
        if material_weights:
            chosen_material = max(material_weights.items(), key=lambda item: item[1])[0]

        # Colliders (§6.5)
        colliders_json: List[Dict[str, Any]] = []
        if lname not in no_collider_links:
            col_proxies = link_colliders[lname]
            if col_proxies:
                for cp in col_proxies:
                    center, dims = _aabb_nwu(frame_conv, cp.bbox_min_cm, cp.bbox_max_cm, origin_point_m)
                    local_center = (center[0] - p_L[0], center[1] - p_L[1], center[2] - p_L[2])
                    colliders_json.append({
                        "name": split_collider_name(cp.name),
                        "type": "box",
                        "origin_m": _r6(local_center),
                        "rotation_quat": [1.0, 0.0, 0.0, 0.0],
                        "dimensions_m": _r6(dims),
                        "material": material_map.get(cp.material_name, chosen_material),
                    })
            elif bodies:
                corners = []
                for b in bodies:
                    corners.append(frame_conv.fusion_to_nwu_point(b.bbox_min_cm, origin_fusion_m=origin_point_m))
                    corners.append(frame_conv.fusion_to_nwu_point(b.bbox_max_cm, origin_fusion_m=origin_point_m))
                lo = (min(c[0] for c in corners), min(c[1] for c in corners), min(c[2] for c in corners))
                hi = (max(c[0] for c in corners), max(c[1] for c in corners), max(c[2] for c in corners))
                center = ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)
                dims = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
                local_center = (center[0] - p_L[0], center[1] - p_L[1], center[2] - p_L[2])
                colliders_json.append({
                    "name": f"{lname}_auto_box",
                    "type": "box",
                    "origin_m": _r6(local_center),
                    "rotation_quat": [1.0, 0.0, 0.0, 0.0],
                    "dimensions_m": _r6(dims),
                    "material": chosen_material,
                })

        if colliders_json:
            link_com_offsets[lname] = vec3_norm(vec3_sub(colliders_json[0]["origin_m"], local_com))
        else:
            link_com_offsets[lname] = 0.0

        manifest_links.append({
            "name": lname,
            "isRoot": is_root,
            "mass_kg": round(m_L, 6),
            "centerOfMass_m": _r6(local_com),
            "inertiaTensor_kgm2": I_L_N.to_dict(digits=9),
            "material": chosen_material,
            "visualMesh": f"Meshes/{lname}.glb",
            "colliders": colliders_json,
        })

    # ------------------------------------------------------------------
    # 3. Joints (BFS edge order)
    # ------------------------------------------------------------------
    manifest_joints: List[Dict[str, Any]] = []
    for pj in graph_res.tree_joints:
        p_P = link_frame_origins[pj.parent_link]
        j_nwu = joint_world[pj.name]
        local_orig = (j_nwu[0] - p_P[0], j_nwu[1] - p_P[1], j_nwu[2] - p_P[2])

        axis_nwu = frame_conv.fusion_to_nwu_vector(pj.axis)
        if revolute_shim and pj.joint_type in ("revolute", "continuous"):
            # D1: RobotAssembler negates the NWU-Y axis component for revolute joints.
            axis_nwu = (axis_nwu[0], -axis_nwu[1], axis_nwu[2])

        spring_freq = 40.0
        spring_damp = 1.0
        if pj.name in joint_overrides:
            ov = joint_overrides[pj.name]
            spring_freq = ov.get("springFrequency_Hz", spring_freq)
            spring_damp = ov.get("springDampingRatio", spring_damp)

        manifest_joints.append({
            "name": pj.name,
            "type": pj.joint_type,
            "parentLink": pj.parent_link,
            "childLink": pj.child_link,
            "origin_m": _r6(local_orig),
            "axis": _r6(axis_nwu),
            "limits": {
                "hasLimits": bool(pj.has_limits),
                "lower": round(pj.lower_limit, 6) if pj.has_limits else 0.0,
                "upper": round(pj.upper_limit, 6) if pj.has_limits else 0.0,
                "unit": "m" if pj.joint_type == "prismatic" else "rad",
                "springFrequency_Hz": float(spring_freq),
                "springDampingRatio": float(spring_damp),
            },
        })

    # ------------------------------------------------------------------
    # 4. Contact zones (§6.6)
    # ------------------------------------------------------------------
    manifest_zones: List[Dict[str, Any]] = []
    for zb in sorted(zone_bodies, key=lambda z: z.name):
        lname = graph_res.occ_to_link.get(zb.occ_name)
        if not lname:
            continue
        p_L = link_frame_origins[lname]
        zone_name, role = split_zone_name(zb.name)
        if role not in VALID_ZONE_ROLES:
            collector.warn(
                "W261",
                f"Zone '{zb.name}' has unknown role '{role}' (expected one of {VALID_ZONE_ROLES}); using 'intake'.",
                target=zb.name,
            )
            role = "intake"

        center, dims = _aabb_nwu(frame_conv, zb.bbox_min_cm, zb.bbox_max_cm, origin_point_m)
        local_center = (center[0] - p_L[0], center[1] - p_L[1], center[2] - p_L[2])

        z_conf = zone_configs.get(zone_name) or zone_configs.get(zb.name) or {}
        if not z_conf:
            collector.warn(
                "W260",
                f"Contact zone '{zone_name}' has no metadata in exporter_config 'contactZones'; using default axes.",
                target=zone_name,
            )

        manifest_zones.append({
            "name": zone_name,
            "role": role,
            "parentLink": lname,
            "shape": "box",
            "origin_m": _r6(local_center),
            "rotation_quat": [1.0, 0.0, 0.0, 0.0],
            "dimensions_m": _r6(dims),
            "tangentAxis": z_conf.get("tangentAxis", [1.0, 0.0, 0.0]),
            "compressionAxis": z_conf.get("compressionAxis", [0.0, 0.0, 1.0]),
            "nominalGap_m": z_conf.get("nominalGap_m", round(min(dims), 4)),
        })

    # ------------------------------------------------------------------
    # 5. Whole-robot summary + frame sanity (§5.1)
    # ------------------------------------------------------------------
    total_mass = 0.0
    total_com = (0.0, 0.0, 0.0)
    total_inertia = InertiaTensor()
    if all_body_props:
        total_mass, total_com, total_inertia = aggregate_link_mass_props(all_body_props)

        if total_com[2] < 0.0:
            collector.warn("W210", f"Robot CoM has negative Z ({total_com[2]:.4f} m). Check the CAD floor/origin and up axis.")
        if min_z_nwu is not None and abs(min_z_nwu) > 0.02:
            collector.warn(
                "W210",
                f"Lowest point of the robot is {min_z_nwu:.3f} m from the floor plane (expected within 2 cm of 0). "
                "Check the CAD origin / ROBOT_ORIGIN point and the up axis.",
            )

    manifest_dict = {
        "schemaVersion": "1.0.0",
        "robotName": robot_name,
        "units": "SI",
        "coordinateSystem": "NWU",
        "massSummary": {
            "totalMass_kg": round(total_mass, 6),
            "centerOfMass_m": _r6(total_com),
        },
        "links": manifest_links,
        "joints": manifest_joints,
        "contactZones": manifest_zones,
    }

    return {
        "manifest": manifest_dict,
        "link_com_offsets": link_com_offsets,
        "link_frame_origins": link_frame_origins,
        "total_mass": total_mass,
        "total_com": total_com,
        "total_inertia": total_inertia,
    }
