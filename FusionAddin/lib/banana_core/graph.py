"""
graph.py: Union-find occurrence clustering into kinematic links,
BFS kinematic tree construction, loop detection, and joint dispositions.
Matches Block 4 Plan §6.3 and §6.4.
"""

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import fnmatch
import re
from typing import Any, Dict, List, Optional, Set, Tuple

from .diagnostics import DiagnosticCollector


class JointDisposition(str, Enum):
    MERGED = "merged"
    FOLDED = "folded"
    TREE = "tree"
    CLOSURE = "closure"
    DETACHED = "detached"
    IGNORED = "ignored"


def sanitize_name(name: str) -> str:
    """
    Sanitizes an occurrence or joint name:
    - Handles Fusion full paths ('Sub:1+Arm:1' or 'Sub:1/Arm:1') segment by segment
    - Strips instance notation like ':1', ':2', ' (1)' from every segment
    - Converts to lower snake_case [a-z0-9_] and collapses consecutive underscores
    """
    segments = []
    for seg in re.split(r"[+/]", name.strip()):
        seg = re.sub(r":\d+$", "", seg.strip())
        seg = re.sub(r"\s*\(\d+\)$", "", seg)
        segments.append(seg)
    s = "_".join(segments)
    s = re.sub(r"[^a-zA-Z0-9_]", "_", s)
    s = re.sub(r"_+", "_", s)
    s = s.strip("_").lower()
    return s if s else "unnamed"


def sanitize_instance_name(name: str) -> str:
    """
    Sanitizes an occurrence name preserving instance numbers (e.g. 'Omni Wheel:2' -> 'omni_wheel_2').
    """
    segments = []
    for seg in re.split(r"[+/]", name.strip()):
        seg = re.sub(r":(\d+)$", r"_\1", seg.strip())
        seg = re.sub(r"\s*\((\d+)\)$", r"_\1", seg)
        segments.append(seg)
    s = "_".join(segments)
    s = re.sub(r"[^a-zA-Z0-9_]", "_", s)
    s = re.sub(r"_+", "_", s)
    s = s.strip("_").lower()
    return s if s else "unnamed"


def join_occ_paths(parent_path: Optional[str], child_path: Optional[str]) -> str:
    """
    Combines parent occurrence path and child occurrence path, deduplicating any overlapping segments.
    Examples:
      join_occ_paths("Intake:1", "Mount:1") -> "Intake:1+Mount:1"
      join_occ_paths("Intake:1", "HexShafts:1+Shaft:1") -> "Intake:1+HexShafts:1+Shaft:1"
      join_occ_paths("RObot Chassis:1", "Frame:1+MK5n:1") -> "RObot Chassis:1+Frame:1+MK5n:1"
      join_occ_paths("RObot Chassis:1+Frame:1", "Frame:1+MK5n:1") -> "RObot Chassis:1+Frame:1+MK5n:1"
      join_occ_paths("RObot Chassis:1+Omni Wheels:1", "Omni Wheels:1+Omni Wheel:1") -> "RObot Chassis:1+Omni Wheels:1+Omni Wheel:1"
    """
    if not parent_path:
        return child_path or ""
    if not child_path:
        return parent_path
    if child_path == parent_path or child_path.startswith(parent_path + "+"):
        return child_path

    p_segs = parent_path.split("+")
    c_segs = child_path.split("+")

    max_overlap = min(len(p_segs), len(c_segs))
    for k in range(max_overlap, 0, -1):
        if p_segs[-k:] == c_segs[:k]:
            return "+".join(p_segs + c_segs[k:])

    return f"{parent_path}+{child_path}"


class DisjointSet:
    """Union-Find data structure with path compression and union by rank."""

    def __init__(self):
        self.parent: Dict[str, str] = {}
        self.rank: Dict[str, int] = {}

    def add(self, x: str) -> None:
        if x not in self.parent:
            self.parent[x] = x
            self.rank[x] = 0

    def find(self, x: str) -> str:
        if x not in self.parent:
            self.add(x)
            return x
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, x: str, y: str) -> None:
        rx = self.find(x)
        ry = self.find(y)
        if rx == ry:
            return
        if self.rank[rx] < self.rank[ry]:
            self.parent[rx] = ry
        elif self.rank[rx] > self.rank[ry]:
            self.parent[ry] = rx
        else:
            self.parent[ry] = rx
            self.rank[rx] += 1


@dataclass
class RawJoint:
    name: str
    joint_type: str  # 'revolute', 'slider', 'cylindrical', 'rigid', 'ball', etc.
    occ1_name: str
    occ2_name: str
    origin_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    has_limits: bool = False
    lower_limit: float = 0.0
    upper_limit: float = 0.0
    is_suppressed: bool = False
    raw_joint: Any = None


@dataclass
class RawOccurrence:
    name: str
    parent_name: Optional[str] = None
    is_grounded: bool = False
    is_visible: bool = True
    body_count: int = 0


@dataclass
class ProcessedJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    disposition: JointDisposition
    origin_m: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    has_limits: bool
    lower_limit: float
    upper_limit: float
    raw_joint: RawJoint


@dataclass
class KinematicGraphResult:
    links: Dict[str, List[str]]  # link_name -> list of occurrence names
    occ_to_link: Dict[str, str]  # occ_name -> link_name
    root_link: str
    tree_joints: List[ProcessedJoint]
    all_joints: List[ProcessedJoint]
    link_order: List[str] = field(default_factory=list)  # root first, then BFS visit order


def build_kinematic_graph(
    occurrences: List[RawOccurrence],
    joints: List[RawJoint],
    rigid_groups: List[List[str]],
    config: Dict[str, Any],
    collector: DiagnosticCollector,
    sidecar: Optional[Dict[str, Any]] = None,
) -> KinematicGraphResult:
    """
    Builds the kinematic tree from raw occurrences, joints, and rigid groups.
    Implements §6.3 Link grouping (union-find) and §6.4 Kinematic tree & dispositions.
    """
    fold_patterns: List[str] = config.get("foldJoints", [])
    detached_patterns: List[str] = config.get("detachedVisualJoints", [])
    link_name_overrides: Dict[str, str] = config.get("linkNames", {})
    keep_separate_pairs: List[Tuple[str, str]] = [tuple(p) for p in config.get("keepSeparate", [])]

    # Set of occurrences involved in movable joints
    movable_joint_occs: Set[str] = set()
    for j in joints:
        if j.is_suppressed:
            continue
        is_folded = any(fnmatch.fnmatch(j.name, pat) for pat in fold_patterns)
        if j.joint_type.lower() not in ("rigid", "rigidjointtype") and not is_folded:
            movable_joint_occs.add(j.occ1_name)
            movable_joint_occs.add(j.occ2_name)

    # 1. Initialize Union-Find
    uf = DisjointSet()
    for occ in occurrences:
        uf.add(occ.name)

    # Helper to check if union is forbidden by keepSeparate
    def can_union(a: str, b: str) -> bool:
        ra = uf.find(a)
        rb = uf.find(b)
        for p1, p2 in keep_separate_pairs:
            rp1 = uf.find(p1)
            rp2 = uf.find(p2)
            if (ra == rp1 and rb == rp2) or (ra == rp2 and rb == rp1):
                return False
        return True

    # Rule 1: Rigid joints and rigid as-built joints
    for j in joints:
        if j.is_suppressed:
            continue
        if j.joint_type.lower() in ("rigid", "rigidjointtype"):
            if can_union(j.occ1_name, j.occ2_name):
                uf.union(j.occ1_name, j.occ2_name)

    # Rule 2: Rigid groups
    for group in rigid_groups:
        if len(group) > 1:
            first = group[0]
            for other in group[1:]:
                if can_union(first, other):
                    uf.union(first, other)

    # Rule 3: Implicit parent rigidity
    # A sub-occurrence moves with its parent occurrence UNLESS:
    # - there is a movable joint between its cluster and its parent's cluster, OR
    # - its cluster is already the moving side (or a distinct movable cluster) of a movable joint.
    movable_pairs: Set[Tuple[str, str]] = set()
    moving_child_clusters: Set[str] = set()
    for j in joints:
        if j.is_suppressed:
            continue
        is_folded = any(fnmatch.fnmatch(j.name, pat) for pat in fold_patterns)
        if j.joint_type.lower() not in ("rigid", "rigidjointtype") and not is_folded:
            r1 = uf.find(j.occ1_name)
            r2 = uf.find(j.occ2_name)
            movable_pairs.add((r1, r2))
            movable_pairs.add((r2, r1))
            moving_child_clusters.add(r1)

    grounded_names = {occ.name for occ in occurrences if occ.is_grounded}
    occ_by_name: Dict[str, RawOccurrence] = {occ.name: occ for occ in occurrences}

    # Build initial cluster members after Rules 1 & 2 (explicit rigid joints & rigid groups)
    explicit_cluster_members: Dict[str, Set[str]] = {}
    for occ in occurrences:
        rep = uf.find(occ.name)
        explicit_cluster_members.setdefault(rep, set()).add(occ.name)

    def _has_ancestor_in_set(occ_name: str, target_set: Set[str]) -> bool:
        curr = occ_by_name.get(occ_name)
        while curr and curr.parent_name:
            if curr.parent_name in target_set:
                return True
            curr = occ_by_name.get(curr.parent_name)
        return False

    # Multi-pass bottom-up / iterative parent merging so deep subtrees bubble up cleanly
    # while respecting movable joints between any descendant cluster and ancestor cluster.
    changed = True
    while changed:
        changed = False
        for occ in reversed(occurrences):
            if not occ.parent_name:
                continue
            c_rep = uf.find(occ.name)
            p_rep = uf.find(occ.parent_name)
            if c_rep == p_rep:
                continue
            # Never merge two clusters that have a movable joint between them!
            if (c_rep, p_rep) in movable_pairs:
                continue
            # If this occurrence's explicit rigid cluster contains a movable-joint participant
            # AND none of the members of this explicit cluster are the parent/ancestor,
            # check whether this cluster is a moving child relative to an ancestor.
            orig_members = explicit_cluster_members.get(c_rep, {occ.name})
            if c_rep in moving_child_clusters and occ.parent_name not in orig_members:
                # If the movable joint is against an ancestor (or something outside this subtree),
                # do not implicitly weld the moving child cluster to its parent container!
                continue
            # Also if this occurrence was explicitly rigidly joined to a movable-joint participant
            # (e.g. Mount bolted to Gear, where Gear revolves on Base, and both Mount and Gear are children of Base),
            # do not implicitly weld Mount to Base!
            if any(m in movable_joint_occs for m in orig_members) and occ.parent_name not in orig_members:
                # Wait: what if Side Plates in Intake is the stationary base of Revolute 21 (HexShafts <-> Mount),
                # or what if Mount is the stationary base of Revolute 21 (HexShafts <-> Mount) and Mount is bolted to Chassis?
                # If Mount is ALREADY bolted to Chassis, c_rep already equals Chassis's rep!
                # What if Mount in Bottom Basket is bolted to Chassis (so c_rep == Chassis), and Bottom Basket:1 is a free container?
                # Then when occ is Bottom Basket:1 (parent Robot Comp 2), Bottom Basket:1 is NOT in movable_joint_occs, so it merges with Robot Comp 2!
                # And when occ is Bottom Basket:1+Mounts:1 (parent Bottom Basket:1), both are in base_link, so they merge!
                # Only skip if merging c_rep and p_rep would collapse a movable pair or merge a moving child into its stationary parent!
                will_collapse_movable = any(
                    (a == c_rep and b == p_rep) or (a == p_rep and b == c_rep)
                    for a, b in movable_pairs
                )
                if will_collapse_movable:
                    continue
                # What if p_rep hasn't merged with the other side of the movable joint YET,
                # e.g. occ is Base:1+Mount:1 (bolted to Base:1+Gear:1), and parent is Base:1,
                # where Base:1+Gear:1 has a revolute joint with Base:1?
                # In that case, (c_rep, p_rep) IS ALREADY in movable_pairs because c_rep == find(Gear) and p_rep == find(Base)!
                # Wait: what if occ is New FRC Turret:1+Static Hood:1 (parent New FRC Turret:1), and Static Hood has a revolute joint with Moving Hood?
                # Then (find(Static Hood), find(Moving Hood)) is in movable_pairs.
                # Merging Static Hood with New FRC Turret:1 does NOT collapse (Static Hood, Moving Hood) as long as Moving Hood doesn't ALSO merge with New FRC Turret:1!
                # And once Static Hood merges with New FRC Turret:1, (find(New FRC Turret:1), find(Moving Hood)) becomes in movable_pairs,
                # which automatically prevents Moving Hood from merging with New FRC Turret:1!
                pass

            if can_union(occ.name, occ.parent_name):
                uf.union(occ.name, occ.parent_name)
                new_rep = uf.find(occ.name)
                if new_rep != c_rep or new_rep != p_rep:
                    updated_pairs = set()
                    for a, b in movable_pairs:
                        ua = new_rep if (a == c_rep or a == p_rep) else a
                        ub = new_rep if (b == c_rep or b == p_rep) else b
                        updated_pairs.add((ua, ub))
                    movable_pairs = updated_pairs
                    if c_rep in moving_child_clusters or p_rep in moving_child_clusters:
                        moving_child_clusters.add(new_rep)
                    changed = True

    # Rule 4: Grounded occurrences union together
    grounded_occs = [occ.name for occ in occurrences if occ.is_grounded]
    if len(grounded_occs) > 1:
        first_g = grounded_occs[0]
        for other_g in grounded_occs[1:]:
            if can_union(first_g, other_g):
                uf.union(first_g, other_g)

    # Rule 5: Fold joints (e.g. swerve module steer/drive axles merge into chassis)
    for j in joints:
        if j.is_suppressed:
            continue
        if any(fnmatch.fnmatch(j.name, pat) for pat in fold_patterns):
            if can_union(j.occ1_name, j.occ2_name):
                uf.union(j.occ1_name, j.occ2_name)

    # Collect grouped occurrences
    groups: Dict[str, List[str]] = {}
    for occ in occurrences:
        rep = uf.find(occ.name)
        groups.setdefault(rep, []).append(occ.name)

    # Determine root link (contains grounded occurrence)
    root_reps = set()
    for occ_name in grounded_occs:
        root_reps.add(uf.find(occ_name))

    if not root_reps:
        collector.error("E230", "No grounded occurrence found in assembly. Exactly one required.", target="Chassis")
        # Fallback root: representative of the first occurrence
        root_rep = uf.find(occurrences[0].name) if occurrences else ""
    elif len(root_reps) > 1:
        collector.error("E230", f"Multiple distinct grounded links found: {root_reps}. Exactly one required.")
        root_rep = sorted(list(root_reps))[0]
    else:
        root_rep = list(root_reps)[0]

    # Name the links
    default_chassis_name = "base_link"
    if sidecar and "swerve" in sidecar and "chassisLink" in sidecar["swerve"]:
        default_chassis_name = sidecar["swerve"]["chassisLink"]

    link_names: Dict[str, str] = {}  # rep -> link_name
    used_names: Dict[str, int] = {}

    # Root link always gets chassis link name
    if root_rep:
        link_names[root_rep] = default_chassis_name
        used_names[default_chassis_name] = 1

    for rep, members in groups.items():
        if rep in link_names:
            continue
        # Check override on rep or any member
        chosen_name = None
        for m in members:
            if m in link_name_overrides:
                chosen_name = link_name_overrides[m]
                break

        if not chosen_name:
            # Top-most (shallowest) occurrence in the group; name it by its own leaf segment.
            top_occ = sorted(members, key=lambda x: (x.count("+") + x.count("/"), len(x)))[0]
            leaf = re.split(r"[+/]", top_occ)[-1]
            chosen_name = sanitize_name(leaf)

        # Handle collisions
        if chosen_name in used_names:
            used_names[chosen_name] += 1
            collided_name = f"{chosen_name}_{used_names[chosen_name]}"
            collector.warn("W220", f"Link name collision on '{chosen_name}'. Renamed to '{collided_name}'.", target=collided_name)
            chosen_name = collided_name
        else:
            used_names[chosen_name] = 1

        link_names[rep] = chosen_name

    # Build final link dictionary and occ_to_link mapping
    final_links: Dict[str, List[str]] = {}
    occ_to_link: Dict[str, str] = {}
    for rep, members in groups.items():
        lname = link_names[rep]
        final_links[lname] = members
        for m in members:
            occ_to_link[m] = lname

    final_root_link = link_names.get(root_rep, default_chassis_name)

    # 2. Process Joints & Kinematic Graph
    # Build multigraph between links
    adj: Dict[str, List[RawJoint]] = {lname: [] for lname in final_links}
    all_processed_joints: List[ProcessedJoint] = []

    for j in joints:
        if j.is_suppressed:
            continue

        l1 = occ_to_link.get(j.occ1_name)
        l2 = occ_to_link.get(j.occ2_name)

        if not l1 or not l2:
            continue

        is_folded = any(fnmatch.fnmatch(j.name, pat) for pat in fold_patterns)
        is_detached = any(fnmatch.fnmatch(j.name, pat) for pat in detached_patterns)
        j_type = j.joint_type.lower()

        # Check disposition
        if l1 == l2:
            disp = JointDisposition.FOLDED if is_folded else JointDisposition.MERGED
            all_processed_joints.append(ProcessedJoint(
                name=j.name,
                joint_type="fixed" if j_type in ("rigid", "rigidjointtype") else j_type,
                parent_link=l1,
                child_link=l2,
                disposition=disp,
                origin_m=j.origin_m,
                axis=j.axis,
                has_limits=j.has_limits,
                lower_limit=j.lower_limit,
                upper_limit=j.upper_limit,
                raw_joint=j,
            ))
            continue

        if is_detached:
            all_processed_joints.append(ProcessedJoint(
                name=j.name,
                joint_type=j_type,
                parent_link=l1,
                child_link=l2,
                disposition=JointDisposition.DETACHED,
                origin_m=j.origin_m,
                axis=j.axis,
                has_limits=j.has_limits,
                lower_limit=j.lower_limit,
                upper_limit=j.upper_limit,
                raw_joint=j,
            ))
            continue

        if j_type in ("ball", "balljointtype", "planar", "planarjointtype", "pinslot", "pinslotjointtype"):
            collector.warn("W250", f"Unsupported joint type '{j.joint_type}' on joint '{j.name}'. Treated as rigid.", target=j.name)
            all_processed_joints.append(ProcessedJoint(
                name=j.name,
                joint_type=j_type,
                parent_link=l1,
                child_link=l2,
                disposition=JointDisposition.IGNORED,
                origin_m=j.origin_m,
                axis=j.axis,
                has_limits=False,
                lower_limit=0.0,
                upper_limit=0.0,
                raw_joint=j,
            ))
            continue

        adj[l1].append(j)
        adj[l2].append(j)

    # 3. BFS Tree Traversal from Root Link
    visited_links: Set[str] = {final_root_link}
    link_order: List[str] = [final_root_link]
    tree_joints: List[ProcessedJoint] = []
    queue = deque([final_root_link])

    # Sidecar four-bar loop names for loop resolution (§6.4 rule 4)
    sidecar_fourbar_joints: Set[str] = set()
    sidecar_fourbar_pairs: Set[Tuple[str, str]] = set()
    if sidecar and "couplings" in sidecar:
        for c in sidecar["couplings"]:
            if c.get("type") == "four_bar_loop":
                if "crankJoint" in c: sidecar_fourbar_joints.add(c["crankJoint"])
                if "couplerJoint" in c: sidecar_fourbar_joints.add(c["couplerJoint"])
                if "followerJoint" in c: sidecar_fourbar_joints.add(c["followerJoint"])
                if "closureParentLink" in c and "closureChildLink" in c:
                    sidecar_fourbar_pairs.add((c["closureParentLink"], c["closureChildLink"]))
                    sidecar_fourbar_pairs.add((c["closureChildLink"], c["closureParentLink"]))

    processed_raw_joints: Set[int] = set()

    while queue:
        curr_link = queue.popleft()
        # Sort candidate edges deterministically by sanitized joint name
        candidate_joints = sorted(adj[curr_link], key=lambda x: sanitize_name(x.name))

        for j in candidate_joints:
            if id(j) in processed_raw_joints:
                continue

            l1 = occ_to_link[j.occ1_name]
            l2 = occ_to_link[j.occ2_name]
            other_link = l2 if l1 == curr_link else l1

            j_type = j.joint_type.lower()
            mapped_type = "revolute"
            if "slider" in j_type or "prismatic" in j_type:
                mapped_type = "prismatic"
            elif "cylindrical" in j_type:
                mapped_type = "revolute"
                collector.warn("W251", f"Cylindrical joint '{j.name}' translation dropped; emitted as revolute.", target=j.name)
            elif "fixed" in j_type or "rigid" in j_type:
                # e.g. a rigid joint between groups forced apart by keepSeparate
                mapped_type = "fixed"

            if other_link not in visited_links:
                # Tree edge
                visited_links.add(other_link)
                link_order.append(other_link)
                queue.append(other_link)
                processed_raw_joints.add(id(j))

                # Axis direction:
                # In Fusion, rotation theta moves occ1 relative to occ2.
                # If parent == occ1 (curr_link == l1), child (l2) rotates by -theta.
                # To maintain right-hand rule, invert axis so joint value means the same thing (§5.5).
                axis = j.axis
                if curr_link == l1:
                    axis = (-axis[0], -axis[1], -axis[2])

                pj = ProcessedJoint(
                    name=j.name,
                    joint_type=mapped_type,
                    parent_link=curr_link,
                    child_link=other_link,
                    disposition=JointDisposition.TREE,
                    origin_m=j.origin_m,
                    axis=axis,
                    has_limits=j.has_limits,
                    lower_limit=j.lower_limit,
                    upper_limit=j.upper_limit,
                    raw_joint=j,
                )
                tree_joints.append(pj)
                all_processed_joints.append(pj)
            else:
                # Non-tree edge (loop closure)
                processed_raw_joints.add(id(j))
                is_sidecar_match = (
                    j.name in sidecar_fourbar_joints
                    or (curr_link, other_link) in sidecar_fourbar_pairs
                )

                if is_sidecar_match:
                    disp = JointDisposition.CLOSURE
                else:
                    disp = JointDisposition.CLOSURE
                    collector.warn("W240", f"Loop-closing joint '{j.name}' between '{curr_link}' and '{other_link}' not matched to a sidecar four_bar_loop.", target=j.name)

                all_processed_joints.append(ProcessedJoint(
                    name=j.name,
                    joint_type=mapped_type,
                    parent_link=curr_link,
                    child_link=other_link,
                    disposition=disp,
                    origin_m=j.origin_m,
                    axis=j.axis,
                    has_limits=j.has_limits,
                    lower_limit=j.lower_limit,
                    upper_limit=j.upper_limit,
                    raw_joint=j,
                ))

    # 4. Check Reachability (Disconnected Links)
    for lname, occs in final_links.items():
        if lname not in visited_links:
            # Check if it was intentionally detached
            is_detached_link = any(
                pj.disposition == JointDisposition.DETACHED and (pj.parent_link == lname or pj.child_link == lname)
                for pj in all_processed_joints
            )
            if not is_detached_link:
                collector.error("E241", f"Link '{lname}' is disconnected from root '{final_root_link}'.", target=lname)

    # Detached / unreachable links go last in a deterministic order.
    for lname in sorted(final_links.keys()):
        if lname not in visited_links:
            link_order.append(lname)

    return KinematicGraphResult(
        links=final_links,
        occ_to_link=occ_to_link,
        root_link=final_root_link,
        tree_joints=tree_joints,
        all_joints=all_processed_joints,
        link_order=link_order,
    )
