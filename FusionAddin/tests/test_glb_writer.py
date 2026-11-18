import json
import os
import struct
import tempfile
import unittest
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from writers.glb_writer import write_glb


class TestGlbWriter(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        for f in os.listdir(self.temp_dir):
            os.remove(os.path.join(self.temp_dir, f))
        os.rmdir(self.temp_dir)

    def test_single_triangle_glb(self):
        out_path = os.path.join(self.temp_dir, "triangle.glb")
        verts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        norms = [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)]
        idxs = [0, 1, 2]

        tri_count = write_glb(verts, norms, idxs, out_path, mesh_name="TestTri")
        self.assertEqual(tri_count, 1)
        self.assertTrue(os.path.exists(out_path))

        # Inspect binary structure
        file_size = os.path.getsize(out_path)
        with open(out_path, "rb") as f:
            magic, version, total_length = struct.unpack("<4sII", f.read(12))
            self.assertEqual(magic, b"glTF")
            self.assertEqual(version, 2)
            self.assertEqual(total_length, file_size)

            # Chunk 0: JSON
            json_len, json_type = struct.unpack("<I4s", f.read(8))
            self.assertEqual(json_type, b"JSON")
            self.assertEqual(json_len % 4, 0)  # 4-byte aligned
            json_bytes = f.read(json_len)
            gltf_json = json.loads(json_bytes.decode("utf-8"))

            # Chunk 1: BIN
            bin_len, bin_type = struct.unpack("<I4s", f.read(8))
            self.assertEqual(bin_type, b"BIN\x00")
            self.assertEqual(bin_len % 4, 0)  # 4-byte aligned

        # Verify accessors
        pos_acc = gltf_json["accessors"][0]
        self.assertEqual(pos_acc["min"], [0.0, 0.0, 0.0])
        self.assertEqual(pos_acc["max"], [1.0, 1.0, 0.0])
        self.assertEqual(pos_acc["count"], 3)
        self.assertEqual(pos_acc["type"], "VEC3")

        # Verify index accessor uses UNSIGNED_SHORT (5123) for small mesh
        idx_acc = gltf_json["accessors"][2]
        self.assertEqual(idx_acc["componentType"], 5123)
        self.assertEqual(idx_acc["count"], 3)

    def test_uint32_indices_for_large_mesh(self):
        out_path = os.path.join(self.temp_dir, "large_mesh.glb")
        # Generate 70,000 vertices to trigger uint32 index path (>= 65,536)
        vert_count = 70000
        verts = [(float(i), 0.0, 0.0) for i in range(vert_count)]
        norms = [(0.0, 1.0, 0.0)] * vert_count
        idxs = [0, 1, 2, 65535, 65536, 69999]

        tri_count = write_glb(verts, norms, idxs, out_path, mesh_name="LargeMesh")
        self.assertEqual(tri_count, 2)

        with open(out_path, "rb") as f:
            f.seek(12)  # Skip header
            json_len, _ = struct.unpack("<I4s", f.read(8))
            json_bytes = f.read(json_len)
            gltf_json = json.loads(json_bytes.decode("utf-8"))

        # Verify index accessor uses UNSIGNED_INT (5125)
        idx_acc = gltf_json["accessors"][2]
        self.assertEqual(idx_acc["componentType"], 5125)
        self.assertEqual(idx_acc["count"], len(idxs))


if __name__ == "__main__":
    unittest.main()
