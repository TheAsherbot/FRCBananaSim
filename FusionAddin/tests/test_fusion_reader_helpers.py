import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from fusion.fusion_reader import (
    _safe_attr,
    _extract_occ,
    _resolve_joint_occurrences,
    _joint_origin_cm,
    ExtractedMotionLink,
)


class DummyEntity:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class ErrorEntity:
    @property
    def occurrenceOne(self):
        raise RuntimeError("2 : InternalValidationError : jointOcc")

    @property
    def isSuppressed(self):
        raise RuntimeError("2 : InternalValidationError : jointOcc")


class TestFusionReaderHelpers(unittest.TestCase):
    def test_safe_attr_catches_runtime_error(self):
        err_obj = ErrorEntity()
        # Standard getattr would raise RuntimeError!
        val = _safe_attr(err_obj, "occurrenceOne", default=None)
        self.assertIsNone(val)

        sup = _safe_attr(err_obj, "isSuppressed", default=False)
        self.assertFalse(sup)

    def test_extract_occ_direct_and_assembly_context(self):
        dummy_occ = DummyEntity(fullPathName="Root+Sub:1", bRepBodies=[1])
        self.assertEqual(_extract_occ(dummy_occ), dummy_occ)

        geom = DummyEntity(assemblyContext=dummy_occ)
        self.assertEqual(_extract_occ(geom), dummy_occ)

        sub_geom = DummyEntity(entityOne=geom)
        self.assertEqual(_extract_occ(sub_geom), dummy_occ)

    def test_resolve_joint_occurrences_with_fallbacks(self):
        root_name = "RobotChassis"
        occ_a = DummyEntity(fullPathName="RobotChassis+SubA:1", bRepBodies=[1])
        occ_b = DummyEntity(fullPathName="RobotChassis+SubB:1", bRepBodies=[1])

        # Normal joint
        j1 = DummyEntity(occurrenceOne=occ_a, occurrenceTwo=occ_b)
        n1, n2 = _resolve_joint_occurrences(j1, root_name)
        self.assertEqual(n1, "RobotChassis+SubA:1")
        self.assertEqual(n2, "RobotChassis+SubB:1")

        # As-built joint with referencedEntities fallback
        j2 = DummyEntity(
            occurrenceOne=None,
            occurrenceTwo=None,
            geometryOrOriginOne=None,
            geometryOrOriginTwo=None,
            referencedEntities=[occ_a, occ_b],
        )
        n1, n2 = _resolve_joint_occurrences(j2, root_name)
        self.assertEqual(n1, "RobotChassis+SubA:1")
        self.assertEqual(n2, "RobotChassis+SubB:1")

        # Grounded joint (one side None -> root)
        j3 = DummyEntity(occurrenceOne=occ_a, occurrenceTwo=None, geometryOrOriginTwo=None, referencedEntities=[])
        n1, n2 = _resolve_joint_occurrences(j3, root_name)
        self.assertEqual(n1, "RobotChassis+SubA:1")
        self.assertEqual(n2, root_name)

    def test_resolve_joint_occurrences_nested_subassembly_suffix_match(self):
        root_name = "TestAssembly2026"
        visible = {
            "TestAssembly2026",
            "RObot Chassis:1",
            "RObot Chassis:1+Frame:1",
            "RObot Chassis:1+Frame:1+MK5n, B v3:1+Kraken X44:2",
            "RObot Chassis:1+Frame:1+MK5n, B v3:1+Component14:1",
            "Bottom Basket:1",
            "Bottom Basket:1+Shafts:1+1/2\" Hex Shaft 36\" length v2:5",
            "Intake:1",
            "Intake:1+Mount:1",
            "Intake:1+HexShafts:1+1/2\" Hex Shaft 36\" length v2 (1):1",
        }

        # Subassembly joint where occurrence names are relative to subassembly
        occ1 = DummyEntity(fullPathName="Frame:1+MK5n, B v3:1+Kraken X44:2", bRepBodies=[1])
        occ2 = DummyEntity(fullPathName="Frame:1+MK5n, B v3:1+Component14:1", bRepBodies=[1])
        j = DummyEntity(occurrenceOne=occ1, occurrenceTwo=occ2)

        # 1. With parent_occ_name provided
        n1, n2 = _resolve_joint_occurrences(j, root_name, parent_occ_name="RObot Chassis:1", visible_occ_names=visible)
        self.assertEqual(n1, "RObot Chassis:1+Frame:1+MK5n, B v3:1+Kraken X44:2")
        self.assertEqual(n2, "RObot Chassis:1+Frame:1+MK5n, B v3:1+Component14:1")

        # 2. Even without parent_occ_name (suffix matching from visible_occ_names)
        n1_auto, n2_auto = _resolve_joint_occurrences(j, root_name, parent_occ_name=None, visible_occ_names=visible)
        self.assertEqual(n1_auto, "RObot Chassis:1+Frame:1+MK5n, B v3:1+Kraken X44:2")
        self.assertEqual(n2_auto, "RObot Chassis:1+Frame:1+MK5n, B v3:1+Component14:1")

        # 3. One side top-level, other side subassembly (like line 508 in report)
        occ_basket = DummyEntity(fullPathName="Bottom Basket:1", bRepBodies=[1])
        occ_shaft = DummyEntity(fullPathName="Shafts:1+1/2\" Hex Shaft 36\" length v2:5", bRepBodies=[1])
        j_basket = DummyEntity(occurrenceOne=occ_basket, occurrenceTwo=occ_shaft)
        nb1, nb2 = _resolve_joint_occurrences(j_basket, root_name, parent_occ_name=None, visible_occ_names=visible)
        self.assertEqual(nb1, "Bottom Basket:1")
        self.assertEqual(nb2, "Bottom Basket:1+Shafts:1+1/2\" Hex Shaft 36\" length v2:5")

        # 4. Context sharing (Intake mount and relative shaft)
        occ_intake_shaft = DummyEntity(fullPathName="HexShafts:1+1/2\" Hex Shaft 36\" length v2 (1):1", bRepBodies=[1])
        occ_intake_mount = DummyEntity(fullPathName="Intake:1+Mount:1", bRepBodies=[1])
        j_intake = DummyEntity(occurrenceOne=occ_intake_shaft, occurrenceTwo=occ_intake_mount)
        ni1, ni2 = _resolve_joint_occurrences(j_intake, root_name, parent_occ_name=None, visible_occ_names=visible)
        self.assertEqual(ni1, "Intake:1+HexShafts:1+1/2\" Hex Shaft 36\" length v2 (1):1")
        self.assertEqual(ni2, "Intake:1+Mount:1")

        # 5. Joint connecting to root component directly while in subassembly context
        occ_root = DummyEntity(fullPathName="", name="TestAssembly2026")
        j_to_root = DummyEntity(occurrenceOne=occ_intake_mount, occurrenceTwo=occ_root)
        nr1, nr2 = _resolve_joint_occurrences(
            j_to_root, root_name, parent_occ_name="Intake:1", visible_occ_names=visible
        )
        self.assertEqual(nr1, "Intake:1+Mount:1")
        self.assertEqual(nr2, root_name)  # Must NOT collapse to "Intake:1"!

    def test_joint_origin_cm_safely_extracts(self):
        class Point:
            x = 10.0
            y = 20.0
            z = 30.0

        geom = DummyEntity(origin=Point())
        j = DummyEntity(geometryOrOriginOne=geom)
        orig = _joint_origin_cm(j)
        self.assertEqual(orig, (10.0, 20.0, 30.0))

        # JointOrigin where geometry IS a Point3D directly (has .x, .y, .z without .origin)
        joint_origin = DummyEntity(geometry=Point())
        j_jo = DummyEntity(geometryOrOriginOne=joint_origin)
        self.assertEqual(_joint_origin_cm(j_jo), (10.0, 20.0, 30.0))

        # Joint where geometryOrOriginOne is None but jointMotion.rotationAxis.origin has the point
        rot_axis = DummyEntity(origin=Point())
        motion = DummyEntity(rotationAxis=rot_axis)
        j_motion = DummyEntity(geometryOrOriginOne=None, jointMotion=motion)
        self.assertEqual(_joint_origin_cm(j_motion), (10.0, 20.0, 30.0))

        # Faulty joint that raises on attributes
        err_j = ErrorEntity()
        self.assertIsNone(_joint_origin_cm(err_j))

    def test_motion_link_dataclass(self):
        ml = ExtractedMotionLink(
            name="MotionLink1",
            joint_one_name="joint_a",
            joint_two_name="joint_b",
            ratio=2.5,
            is_reversed=True,
        )
        self.assertEqual(ml.name, "MotionLink1")
        self.assertEqual(ml.joint_one_name, "joint_a")
        self.assertEqual(ml.joint_two_name, "joint_b")
        self.assertEqual(ml.ratio, 2.5)
        self.assertTrue(ml.is_reversed)


if __name__ == "__main__":
    unittest.main()
