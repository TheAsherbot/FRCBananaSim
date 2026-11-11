"""
fusion_reader.py: Reads CAD geometry, joints, mass properties, and proxy bodies from Autodesk Fusion 360.
Strictly implements recursive assembly traversal with branch-level visibility pruning (§6.2),
proxy body user prompting (§6.2), per-occurrence joint proxying (§4), and rigid group collection.

NOTE: adsk objects are unhashable: never use them as set members / dict keys.
"""

from dataclasses import dataclass, field
import os
import sys
from typing import Any, Callable, Dict, List, Optional, Tuple, Set

ADDIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LIB_DIR = os.path.join(ADDIN_DIR, "lib")
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from banana_core.diagnostics import DiagnosticCollector
from banana_core.graph import RawJoint, RawOccurrence, sanitize_name, sanitize_instance_name, join_occ_paths
from banana_core.inertia import detect_product_sign, flip_products

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False

Tensor6 = Tuple[float, float, float, float, float, float]  # ixx, iyy, izz, ixy, ixz, iyz


@dataclass
class ExtractedBody:
    name: str
    occ_name: str
    proxy_type: str  # 'structural', 'collider', 'zone'
    mass_kg: float = 0.0
    com_cm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia_origin_kgcm2: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    principal_kgcm2: Optional[Tuple[float, float, float]] = None
    material_name: str = "aluminum_6061"
    bbox_min_cm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    bbox_max_cm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    was_visible: bool = True
    fusion_body: Any = None


@dataclass
class ExtractedMotionLink:
    name: str
    joint_one_name: str
    joint_two_name: str
    ratio: float = 1.0
    is_reversed: bool = False


@dataclass
class AssemblyData:
    occurrences: List[RawOccurrence]
    structural_bodies: List[ExtractedBody]
    collider_bodies: List[ExtractedBody]
    zone_bodies: List[ExtractedBody]
    joints: List[RawJoint]
    rigid_groups: List[List[str]]
    motion_links: List[ExtractedMotionLink] = field(default_factory=list)
    whole_design_mass_kg: float = 0.0
    whole_design_com_cm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    whole_design_inertia_kgcm2: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    origin_point_m: Optional[Tuple[float, float, float]] = None
    excluded_visible_proxies: List[ExtractedBody] = field(default_factory=list)
    pruned_occurrences: int = 0
    hidden_bodies_skipped: int = 0
    product_sign: int = 1


def _safe_attr(obj: Any, attr: str, default: Any = None) -> Any:
    """Safely get an attribute, catching both AttributeError AND RuntimeError (e.g. InternalValidationError)."""
    if obj is None:
        return default
    try:
        val = getattr(obj, attr, default)
        return val if val is not None else default
    except Exception:
        return default


def _safe_iter(collection: Any) -> List[Any]:
    """Safely returns a Python list of items from any Fusion collection or iterable."""
    if collection is None:
        return []
    if isinstance(collection, (list, tuple)):
        return list(collection)
    cnt = _safe_attr(collection, "count")
    if cnt is not None and isinstance(cnt, int):
        items = []
        for i in range(cnt):
            try:
                it = collection.item(i)
                if it is not None:
                    items.append(it)
            except Exception:
                pass
        return items
    try:
        return list(collection)
    except Exception:
        return []


def _occ_path(occ: Any) -> str:
    if occ is None:
        return ""
    try:
        path = occ.fullPathName
        if path:
            return path
    except Exception:
        pass
    try:
        return occ.name
    except Exception:
        return str(occ)


def _is_occ_visible(occ: Any, include_hidden: bool = False) -> bool:
    """Safe check for occurrence canvas visibility without raising exceptions."""
    if include_hidden:
        return True
    try:
        # isVisible indicates actual computed visibility in Fusion graphics window
        v = _safe_attr(occ, "isVisible")
        if v is not None:
            return bool(v)
        lb = _safe_attr(occ, "isLightBulbOn")
        if lb is not None:
            return bool(lb)
        return True
    except Exception:
        return True


def _extract_occ(entity: Any) -> Optional[Any]:
    """Tries to extract an Occurrence proxy from an entity, geometry, or proxy context."""
    if entity is None:
        return None
    # If entity is already an Occurrence
    if _safe_attr(entity, "fullPathName") is not None and _safe_attr(entity, "bRepBodies") is not None:
        return entity
    # Direct assemblyContext
    occ = _safe_attr(entity, "assemblyContext")
    if occ is not None and _safe_attr(occ, "fullPathName") is not None:
        return occ
    # Sub-properties
    for sub_attr in ("entityOne", "entity", "parent", "nativeObject", "body"):
        sub = _safe_attr(entity, sub_attr)
        if sub is not None:
            occ = _safe_attr(sub, "assemblyContext")
            if occ is not None and _safe_attr(occ, "fullPathName") is not None:
                return occ
            if _safe_attr(sub, "fullPathName") is not None and _safe_attr(sub, "bRepBodies") is not None:
                return sub
    return None


def _extract_joint_entity_occ(geom_or_origin: Any, parent_occ: Optional[Any] = None) -> Optional[Any]:
    """
    Extracts the Occurrence associated with a joint's geometry or origin.
    Works for both assembly context proxies and native component definitions.
    """
    if geom_or_origin is None:
        return None

    # 1. Direct occurrence / assemblyContext
    occ = _safe_attr(geom_or_origin, "assemblyContext")
    if occ is not None and _safe_attr(occ, "fullPathName") is not None:
        return occ
    if _safe_attr(geom_or_origin, "fullPathName") is not None and _safe_attr(geom_or_origin, "bRepBodies") is not None:
        return geom_or_origin

    # 2. Inspect inner geometry entity (including secondary entities on JointGeometry)
    candidates = [
        _safe_attr(geom_or_origin, "entityOne"),
        _safe_attr(geom_or_origin, "entityTwo"),
        _safe_attr(geom_or_origin, "planeOne"),
        _safe_attr(geom_or_origin, "planeTwo"),
        _safe_attr(geom_or_origin, "geometry"),
        _safe_attr(geom_or_origin, "entity"),
        geom_or_origin,
    ]
    for ent in candidates:
        if ent is None:
            continue
        occ = _safe_attr(ent, "assemblyContext")
        if occ is not None and _safe_attr(occ, "fullPathName") is not None:
            return occ
        # Sketch entities (SketchPoint, SketchCurve) have parentSketch.parentComponent / assemblyContext
        sketch = _safe_attr(ent, "parentSketch")
        if sketch is not None:
            s_occ = _safe_attr(sketch, "assemblyContext")
            if s_occ is not None and _safe_attr(s_occ, "fullPathName") is not None:
                return s_occ

    # 3. Check BRep body / sketch / construction entity parentComponent matching within parent_occ
    for ent in candidates:
        if ent is None:
            continue
        body = (
            _safe_attr(ent, "body")
            or (ent if _safe_attr(ent, "parentComponent") is not None else None)
            or _safe_attr(_safe_attr(ent, "parentSketch"), "parentComponent")
        )
        body_comp = None
        if body is not None:
            body_comp = _safe_attr(body, "parentComponent") or (body if _safe_attr(body, "bRepBodies") is not None else None)
        if body_comp is not None and parent_occ is not None:
            parent_comp = _safe_attr(parent_occ, "component")
            if parent_comp is not None:
                if body_comp == parent_comp or _safe_attr(body_comp, "name") == _safe_attr(parent_comp, "name"):
                    return parent_occ
                for child in _safe_iter(_safe_attr(parent_occ, "childOccurrences")):
                    c_comp = _safe_attr(child, "component")
                    if c_comp == body_comp or (_safe_attr(c_comp, "name") and _safe_attr(c_comp, "name") == _safe_attr(body_comp, "name")):
                        return child
                for child in _safe_iter(_safe_attr(parent_occ, "allOccurrences")):
                    c_comp = _safe_attr(child, "component")
                    if c_comp == body_comp or (_safe_attr(c_comp, "name") and _safe_attr(c_comp, "name") == _safe_attr(body_comp, "name")):
                        return child

    # 4. JointOrigin parent / parentComponent
    for p_attr in ("parent", "parentComponent"):
        parent_obj = _safe_attr(geom_or_origin, p_attr)
        if parent_obj is not None:
            if _safe_attr(parent_obj, "fullPathName") is not None and _safe_attr(parent_obj, "bRepBodies") is not None:
                return parent_obj
            occ = _safe_attr(parent_obj, "assemblyContext")
            if occ is not None and _safe_attr(occ, "fullPathName") is not None:
                return occ
            if parent_occ is not None:
                parent_comp = _safe_attr(parent_occ, "component")
                if parent_comp == parent_obj or _safe_attr(parent_comp, "name") == _safe_attr(parent_obj, "name"):
                    return parent_occ
                for child in _safe_iter(_safe_attr(parent_occ, "childOccurrences")):
                    c_comp = _safe_attr(child, "component")
                    if c_comp == parent_obj or (_safe_attr(c_comp, "name") and _safe_attr(c_comp, "name") == _safe_attr(parent_obj, "name")):
                        return child
                for child in _safe_iter(_safe_attr(parent_occ, "allOccurrences")):
                    c_comp = _safe_attr(child, "component")
                    if c_comp == parent_obj or (_safe_attr(c_comp, "name") and _safe_attr(c_comp, "name") == _safe_attr(parent_obj, "name")):
                        return child

    return None


def _resolve_occurrence_path(
    occ: Any,
    parent_occ_path: Optional[str],
    other_occ_path: Optional[str],
    root_occ_name: str,
    visible_occ_names: Optional[Set[str]] = None,
) -> str:
    """
    Resolves an occurrence entity to a fully qualified path matching visible_occ_names.
    Handles:
    1. Null occ in subassembly context -> parent_occ_path; at root -> root_occ_name
    2. Direct match in visible_occ_names
    3. Prefixing with parent_occ_path via join_occ_paths
    4. Context sharing with the other side of the joint (other_occ_path)
    5. Disambiguated suffix candidate matching across visible_occ_names
    6. Sub-path suffix matching as ultimate fallback
    """
    if occ is None:
        if parent_occ_path and other_occ_path != parent_occ_path:
            return parent_occ_path
        return root_occ_name

    raw_path = _occ_path(occ)
    if not raw_path:
        if parent_occ_path and other_occ_path != parent_occ_path:
            return parent_occ_path
        return root_occ_name

    if raw_path == root_occ_name:
        return root_occ_name

    if visible_occ_names is None or len(visible_occ_names) == 0:
        if parent_occ_path:
            return join_occ_paths(parent_occ_path, raw_path)
        return raw_path

    # Direct exact match
    if raw_path in visible_occ_names:
        return raw_path

    # Try parent_occ_path qualification
    if parent_occ_path:
        joined = join_occ_paths(parent_occ_path, raw_path)
        if joined in visible_occ_names:
            return joined

    # Try qualifying via other_occ_path context
    if other_occ_path and other_occ_path in visible_occ_names:
        joined = join_occ_paths(other_occ_path, raw_path)
        if joined in visible_occ_names:
            return joined
        other_segs = other_occ_path.split("+")
        for i in range(len(other_segs) - 1, 0, -1):
            pfx = "+".join(other_segs[:i])
            joined = join_occ_paths(pfx, raw_path)
            if joined in visible_occ_names:
                return joined

    # Candidate suffix match in visible_occ_names
    candidates = [c for c in visible_occ_names if c == raw_path or c.endswith("+" + raw_path)]
    if len(candidates) == 1:
        return candidates[0]
    elif len(candidates) > 1:
        ctx = parent_occ_path or other_occ_path
        if ctx:
            ctx_segs = ctx.split("+")
            best_cand = candidates[0]
            best_score = -1
            for cand in candidates:
                cand_segs = cand.split("+")
                score = 0
                for s1, s2 in zip(ctx_segs, cand_segs):
                    if s1 == s2:
                        score += 1
                    else:
                        break
                if score > best_score:
                    best_score = score
                    best_cand = cand
            return best_cand
        return candidates[0]

    # Sub-path suffix search (strip leading segments of raw_path)
    raw_segs = raw_path.split("+")
    for k in range(len(raw_segs) - 1, 0, -1):
        sub_raw = "+".join(raw_segs[-k:])
        sub_cands = [c for c in visible_occ_names if c == sub_raw or c.endswith("+" + sub_raw)]
        if len(sub_cands) == 1:
            return sub_cands[0]
        elif len(sub_cands) > 1:
            ctx = parent_occ_path or other_occ_path
            if ctx:
                ctx_segs = ctx.split("+")
                best_cand = sub_cands[0]
                best_score = -1
                for cand in sub_cands:
                    cand_segs = cand.split("+")
                    score = sum(1 for s1, s2 in zip(ctx_segs, cand_segs) if s1 == s2)
                    if score > best_score:
                        best_score = score
                        best_cand = cand
                return best_cand
            return sub_cands[0]

    if parent_occ_path:
        return join_occ_paths(parent_occ_path, raw_path)
    return raw_path


def _resolve_joint_occurrences(
    j: Any,
    root_occ_name: str,
    parent_occ_name: Optional[str] = None,
    visible_occ_names: Optional[Set[str]] = None,
    parent_occ: Optional[Any] = None,
) -> Tuple[str, str]:
    """
    Resolves (occ1_name, occ2_name) for any Joint or AsBuiltJoint.
    If j is in root assembly context, j.occurrenceOne/Two give the exact root paths.
    If j is a native joint in a subassembly, qualifies relative occurrences with parent_occ_name.
    """
    if parent_occ is None:
        parent_occ = _safe_attr(j, "assemblyContext")

    if not parent_occ_name and parent_occ is not None:
        parent_occ_name = _occ_path(parent_occ)

    # 1. Query direct occurrenceOne / occurrenceTwo on j
    occ1 = _safe_attr(j, "occurrenceOne")
    occ2 = _safe_attr(j, "occurrenceTwo")

    # 2. Try nativeObject fallback if direct was None
    if occ1 is None or occ2 is None:
        nat = _safe_attr(j, "nativeObject")
        if nat is not None:
            if occ1 is None:
                occ1 = _safe_attr(nat, "occurrenceOne")
            if occ2 is None:
                occ2 = _safe_attr(nat, "occurrenceTwo")

    # 3. Geometry fallback with parent_occ context (including AsBuiltJoint.geometry / jointOrigin)
    if occ1 is None:
        for attr in ("geometryOrOriginOne", "geometry", "jointOrigin"):
            occ1 = (
                _extract_joint_entity_occ(_safe_attr(j, attr), parent_occ)
                or _extract_joint_entity_occ(_safe_attr(_safe_attr(j, "nativeObject"), attr), parent_occ)
            )
            if occ1 is not None:
                break
    if occ2 is None:
        for attr in ("geometryOrOriginTwo", "geometry", "jointOrigin"):
            cand = (
                _extract_joint_entity_occ(_safe_attr(j, attr), parent_occ)
                or _extract_joint_entity_occ(_safe_attr(_safe_attr(j, "nativeObject"), attr), parent_occ)
            )
            if cand is not None and (occ1 is None or _occ_path(cand) != _occ_path(occ1)):
                occ2 = cand
                break

    # 4. Referenced entities fallback (for AsBuiltJoint)
    if occ1 is None or occ2 is None:
        ref_ents = _safe_attr(j, "referencedEntities") or _safe_attr(_safe_attr(j, "nativeObject"), "referencedEntities")
        if ref_ents:
            try:
                for ent in _safe_iter(ref_ents):
                    candidate = _extract_joint_entity_occ(ent, parent_occ) or _extract_occ(ent)
                    if candidate is not None:
                        if occ1 is None:
                            occ1 = candidate
                        elif occ2 is None and candidate != occ1:
                            occ2 = candidate
            except Exception:
                pass

    # Resolve name1 with parent context
    name1 = _resolve_occurrence_path(occ1, parent_occ_name, None, root_occ_name, visible_occ_names)
    # Resolve name2 with parent context AND name1 as other-side context
    name2 = _resolve_occurrence_path(occ2, parent_occ_name, name1, root_occ_name, visible_occ_names)
    # If name1 wasn't in visible_occ_names on first pass, retry with name2 as other-side context
    if (visible_occ_names and name1 not in visible_occ_names and name2 in visible_occ_names) or occ1 is None:
        name1 = _resolve_occurrence_path(occ1, parent_occ_name, name2, root_occ_name, visible_occ_names)

    return name1, name2


def _transform_point_to_world(pt_cm: Tuple[float, float, float], occ: Any) -> Tuple[float, float, float]:
    """Transforms a 3D point in component local cm into root assembly world cm using occurrence transforms."""
    if occ is None:
        return pt_cm
    tr = _safe_attr(occ, "transform2") or _safe_attr(occ, "transform")
    if tr is None:
        return pt_cm
    try:
        x, y, z = pt_cm
        xw = tr.getCell(0, 0)*x + tr.getCell(0, 1)*y + tr.getCell(0, 2)*z + tr.getCell(0, 3)
        yw = tr.getCell(1, 0)*x + tr.getCell(1, 1)*y + tr.getCell(1, 2)*z + tr.getCell(1, 3)
        zw = tr.getCell(2, 0)*x + tr.getCell(2, 1)*y + tr.getCell(2, 2)*z + tr.getCell(2, 3)
        return (float(xw), float(yw), float(zw))
    except Exception:
        return pt_cm


def _transform_vector_to_world(vec: Tuple[float, float, float], occ: Any) -> Tuple[float, float, float]:
    """Transforms and normalizes a 3D direction vector from component space into world space."""
    if occ is None:
        return vec
    tr = _safe_attr(occ, "transform2") or _safe_attr(occ, "transform")
    if tr is None:
        return vec
    try:
        x, y, z = vec
        xw = tr.getCell(0, 0)*x + tr.getCell(0, 1)*y + tr.getCell(0, 2)*z
        yw = tr.getCell(1, 0)*x + tr.getCell(1, 1)*y + tr.getCell(1, 2)*z
        zw = tr.getCell(2, 0)*x + tr.getCell(2, 1)*y + tr.getCell(2, 2)*z
        length = (xw*xw + yw*yw + zw*zw)**0.5
        if length > 1e-9:
            return (float(xw/length), float(yw/length), float(zw/length))
        return (float(xw), float(yw), float(zw))
    except Exception:
        return vec


def _read_body_mass(body: Any) -> Tuple[float, Tuple[float, float, float], Tuple[float, ...], Optional[Tuple[float, float, float]]]:
    """Returns (mass_kg, com_cm, (ixx,iyy,izz,ixy,ixz,iyz) about WORLD ORIGIN in kg*cm^2, principal moments)."""
    props = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    com = (props.centerOfMass.x, props.centerOfMass.y, props.centerOfMass.z)
    res = props.getXYZMomentsOfInertia()
    # Fusion returns (ok, ixx, iyy, izz, ixy, iyz, ixz)
    ixx, iyy, izz, ixy, iyz, ixz = res[1], res[2], res[3], res[4], res[5], res[6]
    principal = None
    try:
        pr = props.getPrincipalMomentsOfInertia()
        if pr[0]:
            principal = (float(pr[1]), float(pr[2]), float(pr[3]))
    except Exception:
        principal = None
    return props.mass, com, (ixx, iyy, izz, ixy, ixz, iyz), principal


def _joint_origin_cm(j: Any) -> Optional[Tuple[float, float, float]]:
    """
    Joint origin in cm (world cm for proxy joints, local cm for native joints).
    Checks geometryOrOriginOne, geometryOrOriginTwo, geometry, jointOrigin, jointMotion (rotationAxis/slideDirection/pitchPoint),
    nativeObject fallbacks, and entity bounding boxes safely without raising internal validation errors.
    """
    if j is None:
        return None

    def _extract_pt(cand: Any) -> Optional[Tuple[float, float, float]]:
        if cand is None:
            return None
        if hasattr(cand, "x") and hasattr(cand, "y") and hasattr(cand, "z"):
            try:
                return (float(cand.x), float(cand.y), float(cand.z))
            except Exception:
                pass
        for pt_attr in ("origin", "point", "pitchPoint", "center", "position"):
            pt = _safe_attr(cand, pt_attr)
            if pt is not None and hasattr(pt, "x"):
                try:
                    return (float(pt.x), float(pt.y), float(pt.z))
                except Exception:
                    pass
        tr = _safe_attr(cand, "transform") or _safe_attr(cand, "transform2")
        if tr is not None:
            trans = _safe_attr(tr, "translation")
            if trans is not None and hasattr(trans, "x"):
                try:
                    return (float(trans.x), float(trans.y), float(trans.z))
                except Exception:
                    pass
            try:
                return (float(tr.getCell(0, 3)), float(tr.getCell(1, 3)), float(tr.getCell(2, 3)))
            except Exception:
                pass
        return None

    targets = [j]
    nat = _safe_attr(j, "nativeObject")
    if nat is not None:
        targets.append(nat)

    # 1. Try geometry / origin properties on j and nativeObject
    for tgt in targets:
        for attr in ("geometryOrOriginOne", "geometryOrOriginTwo", "geometry", "jointOrigin"):
            obj = _safe_attr(tgt, attr)
            if obj is None:
                continue
            for cand in (
                obj,
                _safe_attr(obj, "geometry"),
                _safe_attr(obj, "entityOne"),
                _safe_attr(_safe_attr(obj, "entityOne"), "geometry"),
                _safe_attr(obj, "entity"),
            ):
                res = _extract_pt(cand)
                if res is not None:
                    return res

        # 2. Try jointMotion axis/origin (RevoluteJointMotion.rotationAxis, CylindricalJointMotion.rotationAxis, BallJointMotion.pitchPoint, etc.)
        motion = _safe_attr(tgt, "jointMotion")
        if motion is not None:
            for m_attr in ("rotationAxis", "slideDirection", "pitchPoint", "customRotationAxisEntity", "customSlideDirectionEntity"):
                m_obj = _safe_attr(motion, m_attr)
                if m_obj is not None:
                    res = _extract_pt(m_obj) or _extract_pt(_safe_attr(m_obj, "geometry"))
                    if res is not None:
                        return res

        # 3. Try joint's own transform or origin
        res = _extract_pt(tgt)
        if res is not None:
            return res

        # 4. BoundingBox center of geometry entity or occurrenceOne as last resort
        for attr in ("geometryOrOriginOne", "geometryOrOriginTwo", "geometry", "occurrenceOne", "occurrenceTwo"):
            obj = _safe_attr(tgt, attr)
            if obj is None:
                continue
            for sub in (obj, _safe_attr(obj, "entityOne"), _safe_attr(obj, "body")):
                bbox = _safe_attr(sub, "boundingBox")
                if bbox is not None:
                    min_pt = _safe_attr(bbox, "minPoint")
                    max_pt = _safe_attr(bbox, "maxPoint")
                    if min_pt is not None and max_pt is not None and hasattr(min_pt, "x"):
                        try:
                            return (
                                0.5 * (float(min_pt.x) + float(max_pt.x)),
                                0.5 * (float(min_pt.y) + float(max_pt.y)),
                                0.5 * (float(min_pt.z) + float(max_pt.z)),
                            )
                        except Exception:
                            pass

    return None


def _flip_body(b: ExtractedBody) -> None:
    b.inertia_origin_kgcm2 = flip_products(tuple(b.inertia_origin_kgcm2))  # type: ignore[arg-type]


def read_assembly(
    app: Any,
    design: Any,
    config: Dict[str, Any],
    collector: DiagnosticCollector,
    hidden_proxy_prompt_cb: Optional[Callable[[str, str], bool]] = None,
    progress_dialog: Optional[Any] = None,
) -> AssemblyData:
    """Traverses the active Fusion 360 design and extracts assembly data for export."""
    if not IN_FUSION or not design:
        return AssemblyData([], [], [], [], [], [])

    root_comp = design.rootComponent
    hidden_proxy_action = config.get("hiddenProxyAction", "prompt").lower()
    stats = {"pruned": 0, "hidden_bodies": 0}

    visible_occ_names: set = set()  # strings only
    raw_occurrences: List[RawOccurrence] = []
    structural_bodies: List[ExtractedBody] = []
    collider_bodies: List[ExtractedBody] = []
    zone_bodies: List[ExtractedBody] = []

    # 1. Custom robot origin construction point (§8)
    origin_point_name = config.get("robotOrigin", "ROBOT_ORIGIN")
    origin_m: Optional[Tuple[float, float, float]] = None
    for pt in root_comp.constructionPoints:
        if pt.name == origin_point_name:
            g = pt.geometry  # cm
            origin_m = (g.x * 0.01, g.y * 0.01, g.z * 0.01)
            collector.info(
                "I120",
                f"Using construction point '{pt.name}' as robot origin at ({origin_m[0]:.4f}, {origin_m[1]:.4f}, {origin_m[2]:.4f}) m.",
            )
            break

    # Root component stands for "the ground": its bodies and any unjointed top-level parts merge into base_link.
    root_occ_name = root_comp.name
    visible_occ_names.add(root_occ_name)
    raw_occurrences.append(RawOccurrence(
        name=root_occ_name, parent_name=None, is_grounded=True, is_visible=True,
        body_count=root_comp.bRepBodies.count,
    ))

    for body in root_comp.bRepBodies:
        _process_body_entry(body, root_occ_name, hidden_proxy_action, structural_bodies,
                            collider_bodies, zone_bodies, collector, hidden_proxy_prompt_cb, stats)

    # 2. Recursive traversal with branch pruning (§6.2)
    grounded_seen = 0
    include_hidden = bool(config.get("includeHidden", False))
    traversed_occurrences: List[Any] = []

    def traverse(occs, parent_name: Optional[str]):
        nonlocal grounded_seen
        for occ in _safe_iter(occs):
            if getattr(occ, "isSuppressed", False):
                continue
            if not _is_occ_visible(occ, include_hidden=include_hidden):
                stats["pruned"] += 1
                pruned_path = _occ_path(occ)
                stats.setdefault("pruned_paths", set()).add(pruned_path)
                collector.info("I295", f"Pruned hidden occurrence branch '{pruned_path}'.")
                continue  # prunes this occurrence AND everything below it

            occ_name = _occ_path(occ)
            visible_occ_names.add(occ_name)
            traversed_occurrences.append(occ)

            try:
                is_grounded = bool(occ.isGrounded)
            except Exception:
                is_grounded = False
            if is_grounded:
                grounded_seen += 1

            b_count = 0
            try:
                b_count = occ.bRepBodies.count
            except Exception:
                b_count = 0

            raw_occurrences.append(RawOccurrence(
                name=occ_name, parent_name=parent_name, is_grounded=is_grounded,
                is_visible=True, body_count=b_count,
            ))

            try:
                for body in _safe_iter(occ.bRepBodies):
                    _process_body_entry(body, occ_name, hidden_proxy_action, structural_bodies,
                                        collider_bodies, zone_bodies, collector, hidden_proxy_prompt_cb, stats)
            except Exception:
                pass

            try:
                children = occ.childOccurrences
                if children and _safe_attr(children, "count", 0) > 0:
                    traverse(children, occ_name)
            except Exception:
                pass

    traverse(root_comp.occurrences, root_occ_name)

    if grounded_seen == 0:
        collector.warn(
            "W230",
            "No occurrence is marked Grounded. Everything not connected by a movable joint is treated as the chassis "
            "(base_link). Ground your chassis component to remove this ambiguity.",
        )

    # 3. Joints (assembly-context proxies per occurrence, §4)
    extracted_joints: List[RawJoint] = []
    seen_joint_tokens: Set[str] = set()
    seen_joint_signatures: Set[Tuple[str, str, str, float, float, float]] = set()
    seen_joint_names: Dict[str, int] = {}

    def process_joint(j: Any, name_prefix: str = "", parent_occ: Optional[Any] = None):
        try:
            if j is None or _safe_attr(j, "isSuppressed", False):
                return

            if parent_occ is None:
                parent_occ = _safe_attr(j, "assemblyContext")
            parent_occ_path = _occ_path(parent_occ) if parent_occ else None

            if not name_prefix and parent_occ:
                comp = _safe_attr(parent_occ, "component")
                reused = False
                try:
                    reused = root_comp.allOccurrencesByComponent(comp).count > 1
                except Exception:
                    pass
                name_prefix = f"{sanitize_instance_name(parent_occ.name)}_" if reused else f"{sanitize_name(parent_occ.name)}_"

            token = _safe_attr(j, "entityToken")
            nat_token = _safe_attr(_safe_attr(j, "nativeObject"), "entityToken")

            tok_key = f"{parent_occ_path}::{token}" if (parent_occ_path and token) else token
            nat_key = f"{parent_occ_path}::{nat_token}" if (parent_occ_path and nat_token) else None

            if tok_key and tok_key in seen_joint_tokens:
                return
            if nat_key and nat_key in seen_joint_tokens:
                return

            name1, name2 = _resolve_joint_occurrences(
                j, root_occ_name, parent_occ_path, visible_occ_names, parent_occ=parent_occ
            )

            # Internal constraints within the same occurrence do not link separate components
            if name1 == name2:
                collector.info("I241", f"Joint '{_safe_attr(j, 'name')}' connects occurrence '{name1}' to itself; skipped as internal constraint.")
                return

            # Joints that touch a pruned (hidden) branch are ignored per §6.2.
            if name1 not in visible_occ_names or name2 not in visible_occ_names:
                pruned_set = stats.get("pruned_paths", set())
                is_pruned = any(p in name1 or p in name2 for p in pruned_set)
                if is_pruned:
                    collector.info("I242", f"Joint '{_safe_attr(j, 'name')}' between '{name1}' and '{name2}' was skipped because an occurrence was pruned by visibility setting.")
                else:
                    collector.warn("W242", f"Joint '{_safe_attr(j, 'name')}' between '{name1}' and '{name2}' was skipped because an occurrence is not in visible assembly tree.")
                return

            motion = _safe_attr(j, "jointMotion")
            if not motion:
                return
            jt = _safe_attr(motion, "jointType")
            j_type = "fixed"
            has_limits = False
            lower = 0.0
            upper = 0.0
            axis = (0.0, 0.0, 1.0)
            raw_j_name = _safe_attr(j, "name", "joint")
            base_j_name = f"{name_prefix}{raw_j_name}" if name_prefix else raw_j_name

            if jt == adsk.fusion.JointTypes.RigidJointType:
                j_type = "rigid"
            elif jt == adsk.fusion.JointTypes.RevoluteJointType:
                j_type = "revolute"
            elif jt == adsk.fusion.JointTypes.SliderJointType:
                j_type = "slider"
            elif jt == adsk.fusion.JointTypes.CylindricalJointType:
                j_type = "cylindrical"
            elif jt == adsk.fusion.JointTypes.BallJointType:
                j_type = "ball"
            elif jt == adsk.fusion.JointTypes.PlanarJointType:
                j_type = "planar"
            elif jt == adsk.fusion.JointTypes.PinSlotJointType:
                j_type = "pinslot"

            orig_cm = _joint_origin_cm(j)
            origin_missing = orig_cm is None
            if orig_cm is None:
                orig_cm = (0.0, 0.0, 0.0)

            # Transform origin and axis from component local space to world space ONLY if native object
            # Must happen BEFORE structural deduplication so world-space coordinates are compared!
            is_proxy = _safe_attr(j, "nativeObject") is not None
            if not is_proxy and parent_occ is not None:
                orig_cm = _transform_point_to_world(orig_cm, parent_occ)

            # Structural deduplication (in world space; include raw_j_name when origin is missing or rigid so distinct joints aren't collapsed)
            sig = (
                min(name1, name2),
                max(name1, name2),
                j_type,
                raw_j_name if (origin_missing or j_type == "rigid") else "",
                round(orig_cm[0], 2),
                round(orig_cm[1], 2),
                round(orig_cm[2], 2),
            )
            if sig in seen_joint_signatures:
                return

            name_count = seen_joint_names.get(base_j_name, 0)
            seen_joint_names[base_j_name] = name_count + 1
            j_name = base_j_name if name_count == 0 else f"{base_j_name}_{name_count + 1}"

            if jt == adsk.fusion.JointTypes.RevoluteJointType:
                rev = adsk.fusion.RevoluteJointMotion.cast(motion) or motion
                try:
                    av = _safe_attr(rev, "rotationAxisVector")
                    if av is not None:
                        axis = (float(av.x), float(av.y), float(av.z))
                    else:
                        axis = (0.0, 0.0, 1.0)
                except Exception:
                    axis = (0.0, 0.0, 1.0)
                try:
                    lim = _safe_attr(rev, "rotationLimits")
                    if lim and _safe_attr(lim, "isMinimumValueEnabled") and _safe_attr(lim, "isMaximumValueEnabled"):
                        has_limits = True
                        lower = float(_safe_attr(lim, "minimumValue", 0.0))
                        upper = float(_safe_attr(lim, "maximumValue", 0.0))
                    elif lim and (_safe_attr(lim, "isMinimumValueEnabled") or _safe_attr(lim, "isMaximumValueEnabled")):
                        collector.warn("W403", f"Joint '{j_name}' has a one-sided limit; schema v1 cannot express it, so it is exported without limits.", target=j_name)
                    else:
                        collector.warn("W402", f"Revolute joint '{j_name}' has no limits.", target=j_name)
                except Exception:
                    pass
            elif jt == adsk.fusion.JointTypes.SliderJointType:
                slider = adsk.fusion.SliderJointMotion.cast(motion) or motion
                try:
                    av = _safe_attr(slider, "slideDirectionVector")
                    if av is not None:
                        axis = (float(av.x), float(av.y), float(av.z))
                    else:
                        axis = (0.0, 0.0, 1.0)
                except Exception:
                    axis = (0.0, 0.0, 1.0)
                try:
                    lim = _safe_attr(slider, "slideLimits")
                    if lim and _safe_attr(lim, "isMinimumValueEnabled") and _safe_attr(lim, "isMaximumValueEnabled"):
                        has_limits = True
                        lower = float(_safe_attr(lim, "minimumValue", 0.0)) * 0.01  # cm -> m
                        upper = float(_safe_attr(lim, "maximumValue", 0.0)) * 0.01  # cm -> m
                    elif lim and (_safe_attr(lim, "isMinimumValueEnabled") or _safe_attr(lim, "isMaximumValueEnabled")):
                        collector.warn("W403", f"Slider '{j_name}' has a one-sided limit; schema v1 cannot express it, so it is exported without limits.", target=j_name)
                    else:
                        collector.warn("W402", f"Slider joint '{j_name}' has no limits.", target=j_name)
                except Exception:
                    pass
            elif jt == adsk.fusion.JointTypes.CylindricalJointType:
                cyl = adsk.fusion.CylindricalJointMotion.cast(motion) or motion
                try:
                    av = _safe_attr(cyl, "rotationAxisVector")
                    if av is not None:
                        axis = (float(av.x), float(av.y), float(av.z))
                    else:
                        axis = (0.0, 0.0, 1.0)
                except Exception:
                    axis = (0.0, 0.0, 1.0)

            if not is_proxy and parent_occ is not None:
                axis = _transform_vector_to_world(axis, parent_occ)

            if origin_missing and j_type not in ("rigid",):
                collector.error(
                    "E243",
                    f"Could not read the origin of joint '{j_name}'. Its link frame would be wrong.",
                    target=j_name,
                )

            extracted_joints.append(RawJoint(
                name=j_name,
                joint_type=j_type,
                occ1_name=name1,
                occ2_name=name2,
                origin_m=(orig_cm[0] * 0.01, orig_cm[1] * 0.01, orig_cm[2] * 0.01),
                axis=axis,
                has_limits=has_limits,
                lower_limit=lower,
                upper_limit=upper,
                is_suppressed=False,
                raw_joint=j,
            ))

            seen_joint_signatures.add(sig)
            if tok_key:
                seen_joint_tokens.add(tok_key)
            elif token:
                seen_joint_tokens.add(token)
            if nat_key:
                seen_joint_tokens.add(nat_key)

        except Exception as e:
            collector.warn("W290", f"Skipped a joint that could not be read: {e}")

    # 4. Motion links helper (§6.9, couplings)
    extracted_motion_links: List[ExtractedMotionLink] = []
    seen_ml_tokens: set = set()
    seen_ml_signatures: set = set()

    def process_motion_link(ml: Any, name_prefix: str = ""):
        if ml is None or _safe_attr(ml, "isSuppressed", False):
            return
        token = _safe_attr(ml, "entityToken")
        nat_token = _safe_attr(_safe_attr(ml, "nativeObject"), "entityToken")
        ml_key = f"{name_prefix}::{token}" if (name_prefix and token) else token
        nat_ml_key = f"{name_prefix}::{nat_token}" if (name_prefix and nat_token) else nat_token
        if (ml_key and ml_key in seen_ml_tokens) or (nat_ml_key and nat_ml_key in seen_ml_tokens):
            return

        raw_name = _safe_attr(ml, "name", "motion_link")
        ml_name = f"{name_prefix}{raw_name}" if name_prefix else raw_name

        ml_obj = _safe_attr(ml, "motionLink") or _safe_attr(ml, "nativeObject") or ml

        # Try multiple attribute names for joint 1 and joint 2
        j1 = (
            _safe_attr(ml, "jointOne")
            or _safe_attr(ml_obj, "jointOne")
            or _safe_attr(ml_obj, "joint1")
            or _safe_attr(ml_obj, "entityOne")
            or _safe_attr(ml_obj, "firstJoint")
            or _safe_attr(ml_obj, "primaryJoint")
            or _safe_attr(ml, "joint1")
            or _safe_attr(ml, "entityOne")
        )
        j2 = (
            _safe_attr(ml, "jointTwo")
            or _safe_attr(ml_obj, "jointTwo")
            or _safe_attr(ml_obj, "joint2")
            or _safe_attr(ml_obj, "entityTwo")
            or _safe_attr(ml_obj, "secondJoint")
            or _safe_attr(ml_obj, "secondaryJoint")
            or _safe_attr(ml, "joint2")
            or _safe_attr(ml, "entityTwo")
        )

        # Try joints collection
        if not j1 or not j2:
            joints_col = _safe_attr(ml_obj, "joints") or _safe_attr(ml, "joints")
            if joints_col:
                items = _safe_iter(joints_col)
                if len(items) >= 2:
                    j1 = j1 or items[0]
                    j2 = j2 or items[1]

        # Try referencedEntities
        if not j1 or not j2:
            refs = _safe_attr(ml_obj, "referencedEntities") or _safe_attr(ml, "referencedEntities")
            if refs:
                cand = [e for e in _safe_iter(refs) if "joint" in (_safe_attr(e, "objectType", "").lower())]
                if len(cand) >= 2:
                    j1 = j1 or cand[0]
                    j2 = j2 or cand[1]

        if not j1 or not j2:
            collector.warn("W512", f"Motion Link '{ml_name}' could not resolve its connected joints.", target=ml_name)
            return

        j1_raw = _safe_attr(j1, "name") or _safe_attr(_safe_attr(j1, "nativeObject"), "name") or "unknown"
        j2_raw = _safe_attr(j2, "name") or _safe_attr(_safe_attr(j2, "nativeObject"), "name") or "unknown"

        def find_matching_joint_name(joint_obj: Any, raw_name: str, pfx: str) -> str:
            tok = _safe_attr(joint_obj, "entityToken")
            nat_tok = _safe_attr(_safe_attr(joint_obj, "nativeObject"), "entityToken")
            for ej in extracted_joints:
                ej_j = getattr(ej, "raw_joint", None)
                if ej_j:
                    ej_tok = _safe_attr(ej_j, "entityToken")
                    ej_nat = _safe_attr(_safe_attr(ej_j, "nativeObject"), "entityToken")
                    if tok and (tok == ej_tok or tok == ej_nat):
                        if not pfx or ej.name.startswith(pfx):
                            return ej.name
                    if nat_tok and (nat_tok == ej_tok or nat_tok == ej_nat):
                        if not pfx or ej.name.startswith(pfx):
                            return ej.name
            if pfx:
                cand = [ej.name for ej in extracted_joints if ej.name == f"{pfx}{raw_name}"]
                if cand:
                    return cand[0]
            cand = [ej.name for ej in extracted_joints if ej.name == raw_name]
            if cand:
                return cand[0]
            cand = [ej.name for ej in extracted_joints if ej.name.endswith(f"_{raw_name}")]
            if cand:
                return cand[0]
            return f"{pfx}{raw_name}" if pfx else raw_name

        j1_name = find_matching_joint_name(j1, j1_raw, name_prefix)
        j2_name = find_matching_joint_name(j2, j2_raw, name_prefix)

        ml_sig = (min(j1_name, j2_name), max(j1_name, j2_name), raw_name)
        if ml_sig in seen_ml_signatures:
            return
        seen_ml_signatures.add(ml_sig)
        if ml_key:
            seen_ml_tokens.add(ml_key)
        if nat_ml_key:
            seen_ml_tokens.add(nat_ml_key)

        is_reversed = bool(_safe_attr(ml_obj, "isReversed", False) or _safe_attr(ml, "isReversed", False))
        ratio = 1.0
        # Fusion MotionLink uses distanceOne/distanceTwo/angleOne/angleTwo (ModelParameter with .value in cm or rad)
        val1 = None
        val2 = None
        for a1, a2 in (("angleOne", "angleTwo"), ("distanceOne", "distanceTwo"), ("valueOne", "valueTwo")):
            p1 = _safe_attr(ml_obj, a1) or _safe_attr(ml, a1)
            p2 = _safe_attr(ml_obj, a2) or _safe_attr(ml, a2)
            if p1 is not None and p2 is not None:
                v1 = _safe_attr(p1, "value", p1 if isinstance(p1, (int, float)) else None)
                v2 = _safe_attr(p2, "value", p2 if isinstance(p2, (int, float)) else None)
                if v1 is not None and v2 is not None:
                    try:
                        val1 = float(v1)
                        val2 = float(v2)
                        break
                    except Exception:
                        pass
        if val1 is not None and val2 is not None and abs(val1) > 1e-9:
            ratio = val2 / val1
        else:
            for rat_attr in ("ratio", "multiplier", "relationshipValue", "value"):
                val = _safe_attr(ml_obj, rat_attr) or _safe_attr(ml, rat_attr)
                if val is not None:
                    v_num = _safe_attr(val, "value", val if isinstance(val, (int, float)) else None)
                    if v_num is not None:
                        try:
                            ratio = float(v_num)
                            break
                        except Exception:
                            pass
            if ratio == 1.0:
                exp = _safe_attr(ml_obj, "expression") or _safe_attr(ml, "expression")
                if exp:
                    try:
                        ratio = float(exp)
                    except Exception:
                        pass

        extracted_motion_links.append(ExtractedMotionLink(
            name=ml_name,
            joint_one_name=j1_name,
            joint_two_name=j2_name,
            ratio=ratio,
            is_reversed=is_reversed,
        ))
        collector.info(
            "I510",
            f"Detected Fusion Motion Link '{ml_name}' linking '{j1_name}' to '{j2_name}' (ratio: {ratio:.4f}, reversed: {is_reversed}).",
            target=ml_name,
        )

    def _scan_timeline_for_motion_links(tl: Any, pfx: str = ""):
        for item in _safe_iter(tl):
            try:
                if _safe_attr(item, "isSuppressed", False):
                    continue
                ent = _safe_attr(item, "entity")
                item_name = (_safe_attr(item, "name", "") or "")
                obj_type = (_safe_attr(ent, "objectType", "") or "").lower()
                if (
                    "motionlink" in obj_type
                    or "motion link" in item_name.lower()
                    or (ent is not None and (hasattr(ent, "jointOne") or hasattr(ent, "joint1")))
                ):
                    process_motion_link(ent or item, name_prefix=pfx)
                # Timeline groups (e.g. subcomponent feature groups) can contain nested timeline items
                child_tl = _safe_attr(item, "timeline")
                if child_tl:
                    _scan_timeline_for_motion_links(child_tl, pfx=pfx)
            except Exception:
                pass

    # 1. Direct joints on root_comp and assembly-wide proxies
    for j in _safe_iter(_safe_attr(root_comp, "allJoints")):
        process_joint(j)
    for j in _safe_iter(_safe_attr(root_comp, "allAsBuiltJoints")):
        process_joint(j)
    for j in _safe_iter(_safe_attr(root_comp, "joints")):
        process_joint(j)
    for j in _safe_iter(_safe_attr(root_comp, "asBuiltJoints")):
        process_joint(j)

    # 2. Timeline items (joints in top-level design history)
    for item in _safe_iter(_safe_attr(design, "timeline")):
        ent = _safe_attr(item, "entity")
        if ent is not None and _safe_attr(ent, "jointMotion") is not None:
            process_joint(ent)

    # 3. Traversed visible occurrences (subassemblies and external components)
    # Process all joints FIRST before resolving motion links so extracted_joints is fully populated!
    seen_ext_designs: Set[str] = set()
    for occ in traversed_occurrences:
        comp = _safe_attr(occ, "component")
        if not comp:
            continue
        reused = False
        try:
            reused = root_comp.allOccurrencesByComponent(comp).count > 1
        except Exception:
            pass
        prefix = f"{sanitize_instance_name(occ.name)}_" if reused else f"{sanitize_name(occ.name)}_"

        for j in _safe_iter(_safe_attr(comp, "joints")):
            proxy = None
            try:
                proxy = j.createForAssemblyContext(occ)
            except Exception:
                proxy = None
            process_joint(proxy or j, prefix, occ)

        for j in _safe_iter(_safe_attr(comp, "asBuiltJoints")):
            proxy = None
            try:
                proxy = j.createForAssemblyContext(occ)
            except Exception:
                proxy = None
            process_joint(proxy or j, prefix, occ)

    # 4. Now collect MotionLinks across root design, all traversed occurrences, and external designs
    _scan_timeline_for_motion_links(_safe_attr(design, "timeline"))

    for ml in _safe_iter(_safe_attr(root_comp, "allMotionLinks")):
        process_motion_link(ml)
    for ml in _safe_iter(_safe_attr(root_comp, "motionLinks")):
        process_motion_link(ml)
    for ml in _safe_iter(_safe_attr(design, "motionLinks")):
        process_motion_link(ml)

    for occ in traversed_occurrences:
        comp = _safe_attr(occ, "component")
        if not comp:
            continue
        reused = False
        try:
            reused = root_comp.allOccurrencesByComponent(comp).count > 1
        except Exception:
            pass
        prefix = f"{sanitize_instance_name(occ.name)}_" if reused else f"{sanitize_name(occ.name)}_"

        for ml_attr in ("motionLinks", "allMotionLinks"):
            for ml in _safe_iter(_safe_attr(comp, ml_attr)):
                proxy = None
                try:
                    proxy = ml.createForAssemblyContext(occ)
                except Exception:
                    proxy = None
                process_motion_link(proxy or ml, name_prefix=prefix)

        p_design = _safe_attr(comp, "parentDesign")
        if p_design is not None and p_design != design:
            ext_root = _safe_attr(p_design, "rootComponent")
            # Only scan an external design's root timeline/motionLinks when occ IS the root of that external design
            if ext_root is not None and (comp == ext_root or _safe_attr(comp, "name") == _safe_attr(ext_root, "name")):
                d_key = f"{prefix}::{_safe_attr(ext_root, 'name', '')}"
                if d_key not in seen_ext_designs:
                    seen_ext_designs.add(d_key)
                    _scan_timeline_for_motion_links(_safe_attr(p_design, "timeline"), pfx=prefix)
                    for ml_attr in ("motionLinks", "allMotionLinks"):
                        for ml in _safe_iter(_safe_attr(ext_root, ml_attr)):
                            process_motion_link(ml, name_prefix=prefix)
                        for ml in _safe_iter(_safe_attr(p_design, ml_attr)):
                            process_motion_link(ml, name_prefix=prefix)

    for comp in _safe_iter(_safe_attr(design, "allComponents")):
        for ml in _safe_iter(_safe_attr(comp, "motionLinks")):
            process_motion_link(ml)

    # 5. Rigid groups
    rigid_groups: List[List[str]] = []
    seen_rg_ids = set()

    def process_rigid_group(rg: Any):
        if rg is None or _safe_attr(rg, "isSuppressed", False):
            return
        rg_id = str(id(rg))
        if rg_id in seen_rg_ids:
            return
        seen_rg_ids.add(rg_id)
        members = []
        for occ in _safe_iter(_safe_attr(rg, "occurrences")):
            n = _occ_path(occ)
            if n in visible_occ_names:
                members.append(n)
        if len(members) > 1:
            rigid_groups.append(members)

    for rg in _safe_iter(_safe_attr(root_comp, "allRigidGroups") or _safe_attr(root_comp, "rigidGroups")):
        process_rigid_group(rg)

    for comp in _safe_iter(_safe_attr(design, "allComponents")):
        for rg in _safe_iter(_safe_attr(comp, "allRigidGroups") or _safe_attr(comp, "rigidGroups")):
            process_rigid_group(rg)

    # 5. Whole-design reference properties (Gate 4)
    whole_mass = 0.0
    whole_com = (0.0, 0.0, 0.0)
    whole_inertia: Tuple[float, ...] = (0.0,) * 6
    try:
        ref = root_comp.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        whole_mass = ref.mass
        whole_com = (ref.centerOfMass.x, ref.centerOfMass.y, ref.centerOfMass.z)
        res = ref.getXYZMomentsOfInertia()
        whole_inertia = (res[1], res[2], res[3], res[4], res[6], res[5])  # ixx, iyy, izz, ixy, ixz, iyz
    except Exception as e:
        collector.warn("W291", f"Could not read Fusion whole-design physical properties; Gate-4 validation is unavailable ({e}).")

    # 6. Inertia product sign (FUSION_PRODUCT_SIGN): auto-detected from Fusion's own principal moments.
    samples = [
        (b.mass_kg, b.com_cm, b.inertia_origin_kgcm2, b.principal_kgcm2)
        for b in structural_bodies if b.principal_kgcm2 is not None
    ]
    sign, confident = detect_product_sign(samples)
    if sign < 0:
        for b in structural_bodies + collider_bodies + zone_bodies:
            _flip_body(b)
        whole_inertia = flip_products(tuple(whole_inertia))  # type: ignore[arg-type]
    collector.info(
        "I130",
        "Inertia product sign: "
        + ("Fusion returns tensor-element products (sign +1)." if sign > 0 else "Fusion returns positive-integral products; negated once at read (sign -1).")
        + ("" if confident else " LOW CONFIDENCE: no body had significant products; used the default.")
    )

    excluded_visible = [b for b in collider_bodies + zone_bodies if b.was_visible and b.mass_kg > 0.0]

    return AssemblyData(
        occurrences=raw_occurrences,
        structural_bodies=structural_bodies,
        collider_bodies=collider_bodies,
        zone_bodies=zone_bodies,
        joints=extracted_joints,
        rigid_groups=rigid_groups,
        motion_links=extracted_motion_links,
        whole_design_mass_kg=whole_mass,
        whole_design_com_cm=whole_com,
        whole_design_inertia_kgcm2=whole_inertia,
        origin_point_m=origin_m,
        excluded_visible_proxies=excluded_visible,
        pruned_occurrences=stats["pruned"],
        hidden_bodies_skipped=stats["hidden_bodies"],
        product_sign=sign,
    )


def _process_body_entry(
    body: Any,
    occ_name: str,
    hidden_proxy_action: str,
    structural_bodies: List[ExtractedBody],
    collider_bodies: List[ExtractedBody],
    zone_bodies: List[ExtractedBody],
    collector: DiagnosticCollector,
    hidden_proxy_prompt_cb: Optional[Callable[[str, str], bool]],
    stats: Dict[str, int],
):
    try:
        b_name = body.name
    except Exception:
        return

    upper = b_name.upper()
    is_col = upper.startswith("COL_")
    is_zone = upper.startswith("ZONE_")
    is_proxy = is_col or is_zone

    try:
        is_visible = bool(body.isLightBulbOn and body.isVisible)
    except Exception:
        is_visible = True

    if not is_visible:
        if is_proxy:
            if hidden_proxy_action == "include":
                should_export = True
            elif hidden_proxy_action == "exclude":
                should_export = False
            else:  # 'prompt'
                should_export = hidden_proxy_prompt_cb(b_name, occ_name) if hidden_proxy_prompt_cb else True
            if not should_export:
                return
        else:
            stats["hidden_bodies"] += 1
            return  # hidden structural bodies are skipped silently

    bbox_min = (0.0, 0.0, 0.0)
    bbox_max = (0.0, 0.0, 0.0)
    try:
        bbox = body.boundingBox
        bbox_min = (bbox.minPoint.x, bbox.minPoint.y, bbox.minPoint.z)
        bbox_max = (bbox.maxPoint.x, bbox.maxPoint.y, bbox.maxPoint.z)
    except Exception:
        pass

    mat_name = "aluminum_6061"
    try:
        if body.material and body.material.name:
            mat_name = body.material.name
    except Exception:
        mat_name = "aluminum_6061"

    if is_proxy:
        eb = ExtractedBody(
            name=b_name, occ_name=occ_name, proxy_type="collider" if is_col else "zone",
            material_name=mat_name, bbox_min_cm=bbox_min, bbox_max_cm=bbox_max,
            was_visible=is_visible, fusion_body=body,
        )
        if is_visible:
            # Visible proxies carry real (default-material) mass inside Fusion's whole-design total.
            # Their properties are needed to subtract them from the Gate-4 reference.
            try:
                eb.mass_kg, eb.com_cm, eb.inertia_origin_kgcm2, eb.principal_kgcm2 = _read_body_mass(body)
            except Exception:
                pass
        (collider_bodies if is_col else zone_bodies).append(eb)
        return

    try:
        mass, com, inertia_origin, principal = _read_body_mass(body)
    except Exception as e:
        collector.error("E300", f"Could not read mass properties of body '{b_name}' in '{occ_name}': {e}", target=b_name)
        return

    if mass <= 1e-6:
        collector.warn("W301", f"Body '{b_name}' in '{occ_name}' has zero or negligible mass ({mass:.6f} kg).", target=b_name)

    structural_bodies.append(ExtractedBody(
        name=b_name, occ_name=occ_name, proxy_type="structural",
        mass_kg=mass, com_cm=com, inertia_origin_kgcm2=inertia_origin, principal_kgcm2=principal,
        material_name=mat_name, bbox_min_cm=bbox_min, bbox_max_cm=bbox_max,
        was_visible=True, fusion_body=body,
    ))
