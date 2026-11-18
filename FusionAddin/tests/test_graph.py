import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.diagnostics import DiagnosticCollector
from banana_core.graph import (
    RawOccurrence,
    RawJoint,
    JointDisposition,
    build_kinematic_graph,
    sanitize_name,
    sanitize_instance_name,
    join_occ_paths,
)


class TestGraph(unittest.TestCase):
    def test_sanitize_name(self):
        self.assertEqual(sanitize_name("Chassis:1"), "chassis")
        self.assertEqual(sanitize_name("Arm Shoulder Hinge (1)"), "arm_shoulder_hinge")
        self.assertEqual(sanitize_name("SubAssembly:2/Link:1"), "subassembly_link")
        self.assertEqual(sanitize_name("Sub:1+Arm:1"), "sub_arm")
        self.assertEqual(sanitize_name("---Elevator_Stage1---"), "elevator_stage1")

    def test_sanitize_instance_name(self):
        self.assertEqual(sanitize_instance_name("Omni Wheel:2"), "omni_wheel_2")
        self.assertEqual(sanitize_instance_name("MK5n, A v4:1"), "mk5n_a_v4_1")
        self.assertEqual(sanitize_instance_name("Arm Shoulder Hinge (1)"), "arm_shoulder_hinge_1")
        self.assertEqual(sanitize_instance_name("SubAssembly:2/Link:1"), "subassembly_2_link_1")
        self.assertEqual(sanitize_instance_name("Chassis"), "chassis")

    def test_join_occ_paths(self):
        self.assertEqual(join_occ_paths("Intake:1", "Mount:1"), "Intake:1+Mount:1")
        self.assertEqual(join_occ_paths("Intake:1", "Intake:1+Mount:1"), "Intake:1+Mount:1")
        self.assertEqual(join_occ_paths("Intake:1+Wheels:1", "Wheels:1+Front:1"), "Intake:1+Wheels:1+Front:1")
        self.assertEqual(join_occ_paths("RObot Chassis:1+Omni Wheels:1", "Omni Wheels:1+Omni Wheel:1+Part:1"), "RObot Chassis:1+Omni Wheels:1+Omni Wheel:1+Part:1")
        self.assertEqual(join_occ_paths("RObot Chassis:1+MAXTube:1", "MAXTube:1+REV:1"), "RObot Chassis:1+MAXTube:1+REV:1")
        self.assertEqual(join_occ_paths(None, "Part:1"), "Part:1")
        self.assertEqual(join_occ_paths("Root:1", None), "Root:1")

    def test_nested_occurrence_link_naming_uses_leaf(self):
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="Elevator:1+Stage:1", parent_name="Elevator:1"),
        ]
        joints = [
            RawJoint(name="stage_slider", joint_type="slider",
                     occ1_name="Elevator:1+Stage:1", occ2_name="Chassis:1"),
        ]
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], {}, collector)
        self.assertFalse(collector.has_errors)
        self.assertIn("stage", res.links)
        self.assertEqual(res.tree_joints[0].joint_type, "prismatic")
        self.assertEqual(res.tree_joints[0].parent_link, "base_link")

    def test_top_level_occurrence_without_grounding_merges_with_root(self):
        occs = [
            RawOccurrence(name="Robot Comp 2", is_grounded=True),
            RawOccurrence(name="RObot Chassis:1", parent_name="Robot Comp 2"),
            RawOccurrence(name="RObot Chassis:1+Frame:1", parent_name="RObot Chassis:1"),
            RawOccurrence(name="Turret:1", parent_name="Robot Comp 2"),
        ]
        joints = [
            RawJoint(name="turret_joint", joint_type="revolute", occ1_name="Turret:1", occ2_name="RObot Chassis:1"),
        ]
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], {}, collector)
        self.assertFalse(collector.has_errors)
        self.assertIn("base_link", res.links)
        self.assertIn("turret", res.links)
        self.assertIn("Robot Comp 2", res.links["base_link"])
        self.assertIn("RObot Chassis:1", res.links["base_link"])
        self.assertIn("RObot Chassis:1+Frame:1", res.links["base_link"])
        self.assertEqual(len(res.tree_joints), 1)
        self.assertEqual(res.tree_joints[0].parent_link, "base_link")
        self.assertEqual(res.tree_joints[0].child_link, "turret")

    def test_keep_separate_rigid_joint_becomes_fixed(self):
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="Plate:1"),
        ]
        joints = [
            RawJoint(name="weld", joint_type="rigid", occ1_name="Chassis:1", occ2_name="Plate:1"),
        ]
        config = {"keepSeparate": [["Chassis:1", "Plate:1"]]}
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], config, collector)
        self.assertEqual(len(res.tree_joints), 1)
        self.assertEqual(res.tree_joints[0].joint_type, "fixed")

    def test_link_order_is_root_then_bfs(self):
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="B:1"),
            RawOccurrence(name="A:1"),
            RawOccurrence(name="C:1"),
        ]
        joints = [
            RawJoint(name="j_chassis_b", joint_type="revolute", occ1_name="Chassis:1", occ2_name="B:1"),
            RawJoint(name="j_chassis_a", joint_type="revolute", occ1_name="Chassis:1", occ2_name="A:1"),
            RawJoint(name="j_b_c", joint_type="revolute", occ1_name="B:1", occ2_name="C:1"),
        ]
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], {}, collector)
        self.assertEqual(res.link_order[0], "base_link")
        # Edges from the root are visited in sanitized joint-name order: j_chassis_a before j_chassis_b.
        self.assertEqual(res.link_order, ["base_link", "a", "b", "c"])

    def test_four_bar_loop(self):
        # 4 links: chassis (grounded), crank, coupler, follower
        # 4 revolute joints forming a loop:
        # J1: chassis -> crank (crank_pivot)
        # J2: crank -> coupler (coupler_hinge)
        # J3: follower -> coupler (closure_hinge)
        # J4: chassis -> follower (follower_pivot)
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="Crank:1"),
            RawOccurrence(name="Coupler:1"),
            RawOccurrence(name="Follower:1"),
        ]
        joints = [
            RawJoint(name="crank_pivot", joint_type="revolute", occ1_name="Chassis:1", occ2_name="Crank:1"),
            RawJoint(name="coupler_hinge", joint_type="revolute", occ1_name="Crank:1", occ2_name="Coupler:1"),
            RawJoint(name="follower_pivot", joint_type="revolute", occ1_name="Chassis:1", occ2_name="Follower:1"),
            RawJoint(name="closure_hinge", joint_type="revolute", occ1_name="Follower:1", occ2_name="Coupler:1"),
        ]
        config = {
            "foldJoints": [],
            "detachedVisualJoints": [],
        }
        collector = DiagnosticCollector()
        sidecar = {
            "couplings": [
                {
                    "name": "four_bar",
                    "type": "four_bar_loop",
                    "crankJoint": "crank_pivot",
                    "couplerJoint": "coupler_hinge",
                    "followerJoint": "follower_pivot",
                }
            ]
        }

        res = build_kinematic_graph(occs, joints, [], config, collector, sidecar)

        self.assertFalse(collector.has_errors)
        self.assertEqual(res.root_link, "base_link")
        # Exactly 3 tree joints and 1 closure joint
        self.assertEqual(len(res.tree_joints), 3)
        closure_joints = [j for j in res.all_joints if j.disposition == JointDisposition.CLOSURE]
        self.assertEqual(len(closure_joints), 1)
        self.assertEqual(closure_joints[0].name, "closure_hinge")

    def test_swerve_folding(self):
        # Chassis + 4 swerve modules whose joints match "fl_*", "fr_*", "bl_*", "br_*"
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="FL_Steer:1"),
            RawOccurrence(name="FR_Steer:1"),
        ]
        joints = [
            RawJoint(name="fl_steer_joint", joint_type="revolute", occ1_name="Chassis:1", occ2_name="FL_Steer:1"),
            RawJoint(name="fr_steer_joint", joint_type="revolute", occ1_name="Chassis:1", occ2_name="FR_Steer:1"),
        ]
        config = {
            "foldJoints": ["fl_*", "fr_*", "bl_*", "br_*"],
        }
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], config, collector)

        self.assertFalse(collector.has_errors)
        # FL_Steer and FR_Steer should have merged into base_link via foldJoints
        self.assertEqual(res.occ_to_link["FL_Steer:1"], "base_link")
        self.assertEqual(res.occ_to_link["FR_Steer:1"], "base_link")
        self.assertEqual(len(res.tree_joints), 0)

    def test_disconnected_link_error(self):
        # Arm is disconnected with no joint to chassis
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=True),
            RawOccurrence(name="FloatingArm:1"),
        ]
        joints = []
        config = {}
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], config, collector)

        self.assertTrue(collector.has_errors)
        self.assertTrue(any(d.code == "E241" for d in collector.diagnostics))

    def test_no_grounded_occurrence_error(self):
        occs = [
            RawOccurrence(name="Chassis:1", is_grounded=False),
        ]
        joints = []
        config = {}
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], config, collector)

        self.assertTrue(collector.has_errors)
        self.assertTrue(any(d.code == "E230" for d in collector.diagnostics))

    def test_child_rigidly_attached_to_moving_cluster_does_not_merge_to_parent(self):
        # Scenario: Base is grounded. Gear revolves on Base. Mount is child of Base in tree,
        # but Mount is rigidly bolted to Gear. Mount should move with Gear, NOT merge into Base!
        occs = [
            RawOccurrence(name="Base:1", is_grounded=True),
            RawOccurrence(name="Base:1+Gear:1", parent_name="Base:1"),
            RawOccurrence(name="Base:1+Mount:1", parent_name="Base:1"),
        ]
        joints = [
            RawJoint(name="rev_gear", joint_type="revolute", occ1_name="Base:1+Gear:1", occ2_name="Base:1"),
            RawJoint(name="rigid_mount_to_gear", joint_type="rigid", occ1_name="Base:1+Mount:1", occ2_name="Base:1+Gear:1"),
        ]
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], {}, collector)
        self.assertFalse(collector.has_errors)
        self.assertEqual(len(res.tree_joints), 1)
        self.assertEqual(res.tree_joints[0].name, "rev_gear")
        self.assertEqual(res.occ_to_link["Base:1+Gear:1"], res.occ_to_link["Base:1+Mount:1"])
        self.assertNotEqual(res.occ_to_link["Base:1+Mount:1"], "base_link")

    def test_multi_subassembly_mechanisms_bubble_to_root(self):
        # Scenario matching competition robot:
        # Root 'Robot Comp 2' contains unjointed subassembly containers ('New FRC Turret:1', 'Intake:1', 'Bottom Basket:1'),
        # where stationary internal parts ('Static Hood:1', 'Mount:1', 'Mounts:1') are either unjointed to root or bolted to chassis,
        # and moving parts ('Moving Hood:1', 'HexShafts:1+Shaft:1') are connected via revolute joints to the stationary parts.
        occs = [
            RawOccurrence(name="Robot Comp 2", is_grounded=True),
            RawOccurrence(name="RObot Chassis:1", parent_name="Robot Comp 2"),
            RawOccurrence(name="New FRC Turret:1", parent_name="Robot Comp 2"),
            RawOccurrence(name="New FRC Turret:1+Static Hood:1", parent_name="New FRC Turret:1"),
            RawOccurrence(name="New FRC Turret:1+Moving Hood:1", parent_name="New FRC Turret:1"),
            RawOccurrence(name="Intake:1", parent_name="Robot Comp 2"),
            RawOccurrence(name="Intake:1+Mount:1", parent_name="Intake:1"),
            RawOccurrence(name="Intake:1+HexShafts:1", parent_name="Intake:1"),
            RawOccurrence(name="Intake:1+HexShafts:1+Shaft:1", parent_name="Intake:1+HexShafts:1"),
        ]
        joints = [
            RawJoint(name="hood_revolute", joint_type="revolute",
                     occ1_name="New FRC Turret:1+Moving Hood:1", occ2_name="New FRC Turret:1+Static Hood:1"),
            RawJoint(name="intake_mount_weld", joint_type="rigid",
                     occ1_name="Intake:1+Mount:1", occ2_name="RObot Chassis:1"),
            RawJoint(name="intake_pivot", joint_type="revolute",
                     occ1_name="Intake:1+HexShafts:1+Shaft:1", occ2_name="Intake:1+Mount:1"),
        ]
        collector = DiagnosticCollector()
        res = build_kinematic_graph(occs, joints, [], {}, collector)
        self.assertFalse(collector.has_errors)
        self.assertEqual(len(res.tree_joints), 2)
        self.assertEqual(res.occ_to_link["New FRC Turret:1+Static Hood:1"], "base_link")
        self.assertEqual(res.occ_to_link["Intake:1+Mount:1"], "base_link")
        self.assertNotEqual(res.occ_to_link["New FRC Turret:1+Moving Hood:1"], "base_link")
        self.assertNotEqual(res.occ_to_link["Intake:1+HexShafts:1+Shaft:1"], "base_link")


if __name__ == "__main__":
    unittest.main()
