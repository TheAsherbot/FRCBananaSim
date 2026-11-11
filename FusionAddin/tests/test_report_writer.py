import json
import os
import shutil
import tempfile
import unittest

from banana_core.diagnostics import DiagnosticCollector
from writers.report_writer import write_export_reports


class TestReportWriter(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_report_writer_full(self):
        collector = DiagnosticCollector()
        collector.info("I110", "Drove 15 joint(s) to zero pose.")
        collector.warn("W402", "Revolute joint 'ArmHinge' has no limits.", target="ArmHinge")

        report_data = {
            "summary": {
                "robotName": "TestBot",
                "totalMass_kg": 55.0,
                "linkCount": 4,
                "jointCount": 3,
                "treeJointCount": 3,
                "closureCount": 0,
                "foldedCount": 0,
                "zoneCount": 1,
                "totalTriangles": 12000,
            },
            "validation": {
                "exported_mass_kg": 55.0,
                "cad_mass_kg": 55.0,
                "mass_error_pct": 0.0,
                "com_distance_mm": 0.1,
                "inertia_error_pct": 0.2,
            },
            "links": [
                {
                    "name": "base_link",
                    "mass_kg": 40.0,
                    "com": [0.0, 0.0, 0.1],
                    "collider_offset_mm": 2.0,
                    "triangles": 8000,
                    "material": "aluminum_6061",
                },
                {
                    "name": "arm_link",
                    "mass_kg": 15.0,
                    "com": [0.2, 0.0, 0.4],
                    "collider_offset_mm": 1.5,
                    "triangles": 4000,
                    "material": "aluminum_6061",
                },
            ],
            "joints": [
                {
                    "name": "ArmHinge",
                    "type": "revolute",
                    "parent_link": "base_link",
                    "child_link": "arm_link",
                    "disposition": "tree",
                    "has_limits": True,
                    "lower_limit": -1.57,
                    "upper_limit": 1.57,
                }
            ],
            "motionLinks": [
                {
                    "name": "MotionLink1",
                    "joint_one": "ArmHinge",
                    "joint_two": "WristHinge",
                    "ratio": 2.0,
                    "is_reversed": True,
                }
            ],
            "massByMaterial": {
                "aluminum_6061": 55.0,
            },
        }

        json_path, md_path = write_export_reports(report_data, collector, self.test_dir)

        self.assertTrue(os.path.exists(json_path))
        self.assertTrue(os.path.exists(md_path))

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            self.assertEqual(data["exportStatus"], "warnings")
            self.assertEqual(len(data["joints"]), 1)
            self.assertEqual(len(data["motionLinks"]), 1)

        with open(md_path, "r", encoding="utf-8") as f:
            content = f.read()
            self.assertIn("## 3. Kinematic Joints & Motion Tree", content)
            self.assertIn("ArmHinge", content)
            self.assertIn("## 4. Fusion Motion Links", content)
            self.assertIn("MotionLink1", content)
            self.assertIn("WristHinge", content)


if __name__ == "__main__":
    unittest.main()
