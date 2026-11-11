import json
import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.diagnostics import DiagnosticCollector
from banana_core.sidecar_check import cross_check_sidecar


class TestSidecarCheck(unittest.TestCase):
    def setUp(self):
        sidecar_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "FRCBananaSim", "Resources", "Robots", "TestAssembly2026", "sidecar.json")
        )
        with open(sidecar_path, "r", encoding="utf-8") as f:
            self.sidecar = json.load(f)

        self.valid_links = {
            "base_link",
            "elevator_stage1_link",
            "elevator_carriage_link",
            "arm_shoulder_link",
            "arm_wrist_link",
            "shooter_flywheel_visual",
            "intake_crank_link",
            "intake_coupler_link",
            "intake_follower_link",
        }
        self.valid_joints = {
            "elevator_stage1_slider",
            "elevator_carriage_slider",
            "arm_shoulder_hinge",
            "arm_wrist_hinge",
            "fourbar_crank_hinge",
            "fourbar_coupler_hinge",
            "fourbar_follower_hinge",
        }
        self.valid_zones = {
            "intake_capture_zone",
        }

    def test_clean_sidecar_cross_check(self):
        collector = DiagnosticCollector()
        cross_check_sidecar(
            self.sidecar,
            self.valid_links,
            self.valid_joints,
            self.valid_zones,
            "base_link",
            collector,
        )
        # Should have zero warnings / errors
        self.assertFalse(collector.has_errors)
        warnings = [d for d in collector.diagnostics if d.code == "W501"]
        self.assertEqual(len(warnings), 0)

    def test_sidecar_typo_with_suggestion(self):
        collector = DiagnosticCollector()
        # Introduce a typo in the sidecar target joint
        corrupted_sidecar = json.loads(json.dumps(self.sidecar))
        corrupted_sidecar["mechanisms"][1]["targetJoint"] = "arm_shoulder_hingee"

        cross_check_sidecar(
            corrupted_sidecar,
            self.valid_links,
            self.valid_joints,
            self.valid_zones,
            "base_link",
            collector,
        )
        warnings = [d for d in collector.diagnostics if d.code == "W501"]
        self.assertEqual(len(warnings), 1)
        self.assertEqual(warnings[0].target, "arm_shoulder_hingee")
        self.assertEqual(warnings[0].suggestion, "arm_shoulder_hinge")

    def test_servo_validation(self):
        collector = DiagnosticCollector()
        servo_sidecar = {
            "swerve": {
                "chassisLink": "base_link",
                "steerActuatorType": "servo",
                "steerServoPreset": "InvalidPresetName",
                "modules": [
                    {"steerPwmChannel": 20}  # Channel > 15
                ]
            }
        }
        cross_check_sidecar(
            servo_sidecar,
            {"base_link"},
            set(),
            set(),
            "base_link",
            collector,
        )
        warnings = [d for d in collector.diagnostics if d.code == "W501"]
        self.assertTrue(any("InvalidPresetName" in w.message for w in warnings))
        self.assertTrue(any("20" in w.message for w in warnings))

    def test_passive_joint_info(self):
        collector = DiagnosticCollector()
        joints_with_extra = set(self.valid_joints)
        joints_with_extra.add("extra_unpowered_hinge")

        cross_check_sidecar(
            self.sidecar,
            self.valid_links,
            joints_with_extra,
            self.valid_zones,
            "base_link",
            collector,
        )
        infos = [d for d in collector.diagnostics if d.code == "I502"]
        self.assertEqual(len(infos), 1)
        self.assertEqual(infos[0].target, "extra_unpowered_hinge")


if __name__ == "__main__":
    unittest.main()
