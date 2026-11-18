import json
import os
import shutil
import tempfile
import unittest
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from fusion.export_command import resolve_sidecar, get_cad_robot_name


class DummyEntity:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class TestResolveSidecar(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.config_dir = os.path.join(self.test_dir, "config")
        self.output_dir = os.path.join(self.test_dir, "output")
        os.makedirs(self.config_dir, exist_ok=True)
        os.makedirs(self.output_dir, exist_ok=True)

        # Create template in config_dir
        self.template_sidecar = os.path.join(self.config_dir, "sidecar.json")
        with open(self.template_sidecar, "w", encoding="utf-8") as f:
            json.dump({"source": "template", "mechanisms": []}, f)

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_prefers_output_dir_when_present(self):
        # User has already configured a sidecar in output_dir
        robot_sidecar = os.path.join(self.output_dir, "sidecar.json")
        with open(robot_sidecar, "w", encoding="utf-8") as f:
            json.dump({"source": "user_robot", "mechanisms": [{"name": "intake"}]}, f)

        config = {"sidecarPath": "./sidecar.json"}
        path, data = resolve_sidecar(config, self.config_dir, self.output_dir)

        self.assertEqual(os.path.abspath(path), os.path.abspath(robot_sidecar))
        self.assertIsNotNone(data)
        self.assertEqual(data.get("source"), "user_robot")

    def test_falls_back_to_template_when_output_dir_has_no_sidecar(self):
        # Fresh robot export with no sidecar in output_dir
        config = {"sidecarPath": "./sidecar.json"}
        path, data = resolve_sidecar(config, self.config_dir, self.output_dir)

        self.assertEqual(os.path.abspath(path), os.path.abspath(self.template_sidecar))
        self.assertIsNotNone(data)
        self.assertEqual(data.get("source"), "template")

    def test_falls_back_to_explicit_sidecar_path(self):
        custom_sidecar = os.path.join(self.test_dir, "custom_sidecar.json")
        with open(custom_sidecar, "w", encoding="utf-8") as f:
            json.dump({"source": "custom"}, f)

        config = {"sidecarPath": custom_sidecar}
        path, data = resolve_sidecar(config, self.config_dir, self.output_dir)

        self.assertEqual(os.path.abspath(path), os.path.abspath(custom_sidecar))
        self.assertEqual(data.get("source"), "custom")

    def test_cad_robot_name_from_root_component(self):
        root = DummyEntity(name="FRC2026_BananaBot")
        design = DummyEntity(rootComponent=root)
        name = get_cad_robot_name(None, design, {"robotName": "auto"})
        self.assertEqual(name, "FRC2026_BananaBot")

    def test_cad_robot_name_strips_fusion_version_suffix(self):
        doc = DummyEntity(name="Competition_Robot v12")
        app = DummyEntity(activeDocument=doc)
        name = get_cad_robot_name(app, None, {"robotName": "auto"})
        self.assertEqual(name, "Competition_Robot")

    def test_cad_robot_name_sanitizes_invalid_characters(self):
        doc = DummyEntity(name="Robot: Arm/Intake? v2")
        app = DummyEntity(activeDocument=doc)
        name = get_cad_robot_name(app, None, {})
        self.assertEqual(name, "Robot_ Arm_Intake_")

    def test_cad_robot_name_explicit_override_takes_precedence(self):
        doc = DummyEntity(name="CAD_Model_Name")
        app = DummyEntity(activeDocument=doc)
        name = get_cad_robot_name(app, None, {"robotName": "CustomFolderOverride"})
        self.assertEqual(name, "CustomFolderOverride")


if __name__ == "__main__":
    unittest.main()
