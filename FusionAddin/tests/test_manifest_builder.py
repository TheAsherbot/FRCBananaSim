from dataclasses import dataclass
import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.diagnostics import DiagnosticCollector
from banana_core.frames import FrameConverter
from banana_core.graph import KinematicGraphResult, ProcessedJoint, JointDisposition, RawJoint
from banana_core.manifest_builder import build_manifest_data


@dataclass
class MockBody:
    name: str
    occ_name: str
    mass_kg: float
    com_cm: tuple
    inertia_origin_kgcm2: tuple
    material_name: str
    bbox_min_cm: tuple
    bbox_max_cm: tuple


class TestManifestBuilder(unittest.TestCase):
    def test_synthetic_arm_manifest(self):
        # 2 links: base_link and arm_link
        # Joint: arm_shoulder_hinge at Fusion (0, 50, 0) cm [Up = +Y, Fwd = +Z] -> NWU (0, 0, 0.5) m
        frame_conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        collector = DiagnosticCollector()

        raw_j = RawJoint(
            name="arm_shoulder_hinge",
            joint_type="revolute",
            occ1_name="Chassis:1",
            occ2_name="Arm:1",
            origin_m=(0.0, 0.5, 0.0),  # Fusion cm -> 0, 50cm, 0
            axis=(1.0, 0.0, 0.0),       # Fusion +X (Left) -> NWU +Y (Left)
            has_limits=True,
            lower_limit=-0.5,
            upper_limit=1.5,
        )

        tree_joints = [
            ProcessedJoint(
                name="arm_shoulder_hinge",
                joint_type="revolute",
                parent_link="base_link",
                child_link="arm_link",
                disposition=JointDisposition.TREE,
                origin_m=(0.0, 0.5, 0.0),
                axis=(1.0, 0.0, 0.0),
                has_limits=True,
                lower_limit=-0.5,
                upper_limit=1.5,
                raw_joint=raw_j,
            )
        ]

        graph_res = KinematicGraphResult(
            links={
                "base_link": ["Chassis:1"],
                "arm_link": ["Arm:1"],
            },
            occ_to_link={
                "Chassis:1": "base_link",
                "Arm:1": "arm_link",
            },
            root_link="base_link",
            tree_joints=tree_joints,
            all_joints=tree_joints,
        )

        # Bodies
        # Chassis body: 25 kg, center at (0, 10, 0) cm
        chassis_b = MockBody(
            name="Chassis_Plate",
            occ_name="Chassis:1",
            mass_kg=25.0,
            com_cm=(0.0, 10.0, 0.0),
            inertia_origin_kgcm2=(50000.0, 50000.0, 50000.0, 0.0, 0.0, 0.0),
            material_name="Aluminum 6061",
            bbox_min_cm=(-30.0, 0.0, -30.0),
            bbox_max_cm=(30.0, 20.0, 30.0),
        )

        # Arm body: 4 kg, center at (0, 50, 25) cm [i.e. +Z 25cm in Fusion = +X 0.25m in NWU]
        arm_b = MockBody(
            name="Arm_Tube",
            occ_name="Arm:1",
            mass_kg=4.0,
            com_cm=(0.0, 50.0, 25.0),
            inertia_origin_kgcm2=(20000.0, 20000.0, 20000.0, 0.0, 0.0, 0.0),
            material_name="Aluminum 6061",
            bbox_min_cm=(-5.0, 45.0, 0.0),
            bbox_max_cm=(5.0, 55.0, 50.0),
        )

        # Contact zone on arm: ZONE_intake_capture_zone
        zone_b = MockBody(
            name="ZONE_intake_capture_zone",
            occ_name="Arm:1",
            mass_kg=0.0,
            com_cm=(0.0, 50.0, 50.0),
            inertia_origin_kgcm2=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            material_name="zone_material",
            bbox_min_cm=(-10.0, 40.0, 45.0),
            bbox_max_cm=(10.0, 60.0, 55.0),
        )

        assembly_bodies = {
            "structural": [chassis_b, arm_b],
            "collider": [],
            "zone": [zone_b],
        }

        material_map = {"Aluminum 6061": "aluminum_6061"}
        config = {
            "robotName": "SyntheticArm",
            "revoluteAxisSignShim": True,
            "noCollider": [],
            "contactZones": {
                "ZONE_intake_capture_zone": {
                    "tangentAxis": [-1.0, 0.0, 0.0],
                    "compressionAxis": [0.0, 0.0, 1.0],
                    "nominalGap_m": 0.10,
                }
            }
        }

        res = build_manifest_data(
            graph_res,
            assembly_bodies,
            frame_conv,
            material_map,
            config,
            collector,
        )

        self.assertFalse(collector.has_errors)
        manifest = res["manifest"]

        # 1. Check Schema headers
        self.assertEqual(manifest["schemaVersion"], "1.0.0")
        self.assertEqual(manifest["coordinateSystem"], "NWU")
        self.assertEqual(manifest["robotName"], "SyntheticArm")

        # 2. Check Links
        self.assertEqual(len(manifest["links"]), 2)
        base_link = next(l for l in manifest["links"] if l["name"] == "base_link")
        arm_link = next(l for l in manifest["links"] if l["name"] == "arm_link")

        self.assertTrue(base_link["isRoot"])
        self.assertFalse(arm_link["isRoot"])
        self.assertAlmostEqual(base_link["mass_kg"], 25.0)
        self.assertAlmostEqual(arm_link["mass_kg"], 4.0)

        # Arm local CoM check:
        # Arm CoM in Fusion is at (0, 50cm, 25cm) -> NWU (0.25, 0.0, 0.50) m.
        # Joint is at (0, 50cm, 0) -> NWU (0.0, 0.0, 0.50) m = link frame origin p_L.
        # Local CoM = c_L - p_L = (0.25, 0.0, 0.50) - (0.0, 0.0, 0.50) = (0.25, 0.0, 0.0) m!
        self.assertAlmostEqual(arm_link["centerOfMass_m"][0], 0.25, places=5)
        self.assertAlmostEqual(arm_link["centerOfMass_m"][1], 0.0, places=5)
        self.assertAlmostEqual(arm_link["centerOfMass_m"][2], 0.0, places=5)

        # 3. Check Joint
        self.assertEqual(len(manifest["joints"]), 1)
        j = manifest["joints"][0]
        self.assertEqual(j["name"], "arm_shoulder_hinge")
        self.assertEqual(j["parentLink"], "base_link")
        self.assertEqual(j["childLink"], "arm_link")
        # Joint origin in parent frame (base_link is at 0,0,0) -> (0.0, 0.0, 0.50)
        self.assertAlmostEqual(j["origin_m"][2], 0.50, places=5)

        # Revolute axis with sign shim:
        # Fusion +X -> NWU +Y (0, 1, 0).
        # With shim (negate Y): (0, -1, 0)!
        self.assertAlmostEqual(j["axis"][0], 0.0, places=5)
        self.assertAlmostEqual(j["axis"][1], -1.0, places=5)
        self.assertAlmostEqual(j["axis"][2], 0.0, places=5)

        # 4. Check Zone: exported name has no ZONE_ prefix, and carries a role (loader defaults to 'intake').
        self.assertEqual(manifest["units"], "SI")
        self.assertEqual(j["limits"]["unit"], "rad")
        self.assertEqual(len(manifest["contactZones"]), 1)
        zone = manifest["contactZones"][0]
        self.assertEqual(zone["name"], "intake_capture_zone")
        self.assertEqual(zone["role"], "intake")
        self.assertEqual(zone["parentLink"], "arm_link")
        self.assertEqual(zone["nominalGap_m"], 0.10)

        # 5. Links are written root first
        self.assertEqual([l["name"] for l in manifest["links"]], ["base_link", "arm_link"])

        # 6. Frame origins are exposed for mesh export (child frame sits at the joint origin)
        origins = res["link_frame_origins"]
        self.assertAlmostEqual(origins["arm_link"][2], 0.50, places=5)
        self.assertEqual(origins["base_link"], (0.0, 0.0, 0.0))

    def test_shooter_zone_role_and_prismatic_unit(self):
        frame_conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        collector = DiagnosticCollector()
        raw_j = RawJoint(
            name="lift", joint_type="slider", occ1_name="Chassis:1", occ2_name="Lift:1",
            origin_m=(0.0, 0.1, 0.0), axis=(0.0, 1.0, 0.0), has_limits=True, lower_limit=0.0, upper_limit=0.6,
        )
        pj = ProcessedJoint(
            name="lift", joint_type="prismatic", parent_link="base_link", child_link="lift_link",
            disposition=JointDisposition.TREE, origin_m=(0.0, 0.1, 0.0), axis=(0.0, 1.0, 0.0),
            has_limits=True, lower_limit=0.0, upper_limit=0.6, raw_joint=raw_j,
        )
        graph_res = KinematicGraphResult(
            links={"base_link": ["Chassis:1"], "lift_link": ["Lift:1"]},
            occ_to_link={"Chassis:1": "base_link", "Lift:1": "lift_link"},
            root_link="base_link", tree_joints=[pj], all_joints=[pj],
        )

        def body(name, occ, mass, com):
            return MockBody(name=name, occ_name=occ, mass_kg=mass, com_cm=com,
                            inertia_origin_kgcm2=(50000.0, 50000.0, 50000.0, 0.0, 0.0, 0.0),
                            material_name="Aluminum 6061",
                            bbox_min_cm=(-5.0, 0.0, -5.0), bbox_max_cm=(5.0, 10.0, 5.0))

        zone = body("ZONE_shooter_engagement_zone", "Lift:1", 0.0, (0.0, 20.0, 0.0))
        bodies = {
            "structural": [body("Plate", "Chassis:1", 20.0, (0.0, 5.0, 0.0)), body("Rail", "Lift:1", 3.0, (0.0, 15.0, 0.0))],
            "collider": [], "zone": [zone],
        }
        res = build_manifest_data(graph_res, bodies, frame_conv, {"Aluminum 6061": "aluminum_6061"},
                                  {"revoluteAxisSignShim": True}, collector)
        m = res["manifest"]
        self.assertEqual(m["joints"][0]["limits"]["unit"], "m")
        self.assertEqual(m["joints"][0]["type"], "prismatic")
        self.assertEqual(m["contactZones"][0]["name"], "shooter_engagement_zone")
        self.assertEqual(m["contactZones"][0]["role"], "shooter")

    def test_empty_link_is_zero_mass_error(self):
        frame_conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        collector = DiagnosticCollector()
        graph_res = KinematicGraphResult(
            links={"base_link": ["Chassis:1"], "ghost": ["Ghost:1"]},
            occ_to_link={"Chassis:1": "base_link", "Ghost:1": "ghost"},
            root_link="base_link", tree_joints=[], all_joints=[],
        )
        chassis = MockBody(name="Plate", occ_name="Chassis:1", mass_kg=10.0, com_cm=(0.0, 5.0, 0.0),
                           inertia_origin_kgcm2=(50000.0, 50000.0, 50000.0, 0.0, 0.0, 0.0),
                           material_name="Aluminum 6061",
                           bbox_min_cm=(-5.0, 0.0, -5.0), bbox_max_cm=(5.0, 10.0, 5.0))
        build_manifest_data(graph_res, {"structural": [chassis], "collider": [], "zone": []},
                            frame_conv, {"Aluminum 6061": "aluminum_6061"}, {}, collector)
        self.assertTrue(any(d.code == "E301" and d.target == "ghost" for d in collector.diagnostics))


if __name__ == "__main__":
    unittest.main()
