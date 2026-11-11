"""
zero_pose.py: Context manager to drive joints to their resting zero pose for export,
and safely restore original poses even if an error occurs.
Matches Block 4 Plan §6.1.

NOTE: adsk objects (Joint, JointMotion, ...) are NOT hashable, so they must never be used
as set elements or dict keys. We dedupe on `entityToken` strings and store plain lists.
"""

from typing import Any, List, Tuple
from banana_core.diagnostics import DiagnosticCollector

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False


class ExportAborted(Exception):
    """Raised when the design is not in a state that can be exported safely (E111)."""


def _safe_attr(obj: Any, attr: str, default: Any = None) -> Any:
    """Safely get an attribute, catching both AttributeError AND RuntimeError (e.g. InternalValidationError)."""
    if obj is None:
        return default
    try:
        val = getattr(obj, attr, default)
        return val if val is not None else default
    except Exception:
        return default


def _safe_iter(collection: Any) -> list:
    """Safely converts a Fusion collection into a Python list without throwing TypeError or RuntimeError."""
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


def _entity_key(entity: Any) -> str:
    try:
        token = _safe_attr(entity, "entityToken")
        if token:
            return token
    except Exception:
        pass
    return str(id(entity))


class ZeroPoseGuard:
    """
    Context manager that verifies the design is clean, snapshots joint positions,
    resets them to zero, and restores them on exit (including on exceptions).
    """

    def __init__(self, design: Any, collector: DiagnosticCollector):
        self.design = design
        self.collector = collector
        # list of (motion, kind, original_value) where kind is 'revolute' or 'slider'
        self._saved: List[Tuple[Any, str, float]] = []
        self.normalized_joints: List[str] = []

    # ------------------------------------------------------------------
    # Design cleanliness (E111)
    # ------------------------------------------------------------------
    def _check_design_clean(self) -> None:
        design = self.design
        try:
            is_parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        except Exception:
            is_parametric = False
        if not is_parametric:
            return  # Direct-modeling designs have no timeline.

        timeline = design.timeline
        if not timeline:
            return

        try:
            if timeline.markerPosition != timeline.count:
                raise ExportAborted(
                    "E111: The timeline is rolled back (marker is not at the end). "
                    "Move the timeline marker to the end and export again."
                )
        except ExportAborted:
            raise
        except Exception:
            pass

        error_names: List[str] = []
        for item in _safe_iter(timeline):
            try:
                state = item.healthState
            except Exception:
                continue

            name = ""
            try:
                name = item.name
            except Exception:
                try:
                    name = item.entity.name
                except Exception:
                    name = f"timeline item {_safe_attr(item, 'index', 0)}"

            if state == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
                error_names.append(name)
            elif state == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState:
                self.collector.warn(
                    "W111",
                    f"Timeline feature '{name}' is in a warning state. Export continues, but check this feature.",
                    target=name,
                )

        if error_names:
            shown = ", ".join(error_names[:8]) + (" ..." if len(error_names) > 8 else "")
            self.collector.error(
                "E111",
                f"Timeline has {len(error_names)} feature(s) in an error state: {shown}",
                target=error_names[0],
            )
            raise ExportAborted(
                f"E111: Timeline has {len(error_names)} feature(s) in an error state: {shown}\n\n"
                "Fix the red timeline items (or Revert pending changes) and export again."
            )

    # ------------------------------------------------------------------
    # Joint collection (assembly-context proxies per occurrence)
    # ------------------------------------------------------------------
    def _collect_joints(self) -> List[Any]:
        root = self.design.rootComponent
        joints: List[Any] = []
        seen = set()

        def add(j: Any) -> None:
            if j is None:
                return
            key = _entity_key(j)
            if key in seen:
                return
            seen.add(key)
            joints.append(j)

        # 1. allJoints and allAsBuiltJoints from root (all assembly context proxies)
        try:
            for j in _safe_iter(_safe_attr(root, "allJoints")):
                add(j)
        except Exception:
            pass
        try:
            for j in _safe_iter(_safe_attr(root, "allAsBuiltJoints")):
                add(j)
        except Exception:
            pass

        # 2. direct joints on root
        try:
            for j in _safe_iter(_safe_attr(root, "joints")):
                add(j)
        except Exception:
            pass
        try:
            for j in _safe_iter(_safe_attr(root, "asBuiltJoints")):
                add(j)
        except Exception:
            pass

        # 3. occurrences fallback
        try:
            for occ in _safe_iter(_safe_attr(root, "allOccurrences")):
                try:
                    if _safe_attr(occ, "isSuppressed", False) or not _safe_attr(occ, "isLightBulbOn", True) or not _safe_attr(occ, "isVisible", True):
                        continue
                    comp = _safe_attr(occ, "component")
                    if not comp:
                        continue

                    try:
                        for j in _safe_iter(_safe_attr(comp, "joints")):
                            proxy = None
                            try:
                                proxy = j.createForAssemblyContext(occ)
                            except Exception:
                                proxy = None
                            add(proxy or j)
                    except Exception:
                        pass
                    try:
                        for j in _safe_iter(_safe_attr(comp, "asBuiltJoints")):
                            proxy = None
                            try:
                                proxy = j.createForAssemblyContext(occ)
                            except Exception:
                                proxy = None
                            add(proxy or j)
                    except Exception:
                        pass

                    # External referenced design check
                    try:
                        p_design = _safe_attr(comp, "parentDesign")
                        if p_design is not None and p_design != self.design:
                            ext_root = _safe_attr(p_design, "rootComponent")
                            if ext_root:
                                for j in _safe_iter(_safe_attr(ext_root, "allJoints")):
                                    add(j)
                                for j in _safe_iter(_safe_attr(ext_root, "allAsBuiltJoints")):
                                    add(j)
                                for j in _safe_iter(_safe_attr(ext_root, "joints")):
                                    add(j)
                                for j in _safe_iter(_safe_attr(ext_root, "asBuiltJoints")):
                                    add(j)
                    except Exception:
                        pass
                except Exception:
                    pass
        except Exception:
            pass
        return joints

    # ------------------------------------------------------------------
    # Context manager protocol
    # ------------------------------------------------------------------
    def __enter__(self):
        if not IN_FUSION or not self.design:
            return self
        # Raises ExportAborted before touching any joint, so nothing needs restoring.
        self._check_design_clean()





        failed: List[str] = []
        for j in self._collect_joints():
            try:
                if _safe_attr(j, "isSuppressed", False):
                    continue
                motion = _safe_attr(j, "jointMotion")
                if not motion:
                    continue
                j_type = _safe_attr(motion, "jointType")
                j_name = _safe_attr(j, "name", "joint")
                if j_type == adsk.fusion.JointTypes.RevoluteJointType:
                    rev = adsk.fusion.RevoluteJointMotion.cast(motion) or motion
                    val = _safe_attr(rev, "rotationValue")
                    if val is not None and abs(val) > 1e-6:
                        try:
                            rev.rotationValue = 0.0
                            self._saved.append((rev, "revolute", val))
                            self.normalized_joints.append(f"{j_name} ({val:.4f} rad -> 0)")
                        except Exception:
                            failed.append(j_name)
                elif j_type == adsk.fusion.JointTypes.SliderJointType:
                    slider = adsk.fusion.SliderJointMotion.cast(motion) or motion
                    val = _safe_attr(slider, "slideValue")
                    if val is not None and abs(val) > 1e-6:
                        try:
                            slider.slideValue = 0.0
                            self._saved.append((slider, "slider", val))
                            self.normalized_joints.append(f"{j_name} ({val:.4f} cm -> 0)")
                        except Exception:
                            failed.append(j_name)
            except Exception:
                continue


        if self._saved:
            try:
                self.design.computeAll()
            except Exception:
                pass
            shown = ", ".join(self.normalized_joints[:5])
            more = f" and {len(self.normalized_joints) - 5} more" if len(self.normalized_joints) > 5 else ""
            self.collector.info(
                "I110",
                f"Drove {len(self._saved)} joint(s) to zero pose for capture: {shown}{more}. They will be restored.",
            )

        for name in failed:
            self.collector.warn(
                "W112",
                f"Joint '{name}' could not be driven to zero (limits exclude 0, or it is locked/driven). "
                "Exported geometry for it is at its current pose.",
                target=name,
            )

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not IN_FUSION or not self._saved:
            return False

        # Restore in reverse order, even if the export threw.
        for motion, kind, val in reversed(self._saved):
            try:
                if kind == "revolute":
                    motion.rotationValue = val
                else:
                    motion.slideValue = val
            except Exception:
                pass

        try:
            self.design.computeAll()
        except Exception:
            pass

        return False
