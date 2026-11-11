"""
glb_writer.py: Pure-Python binary glTF 2.0 (.glb) exporter using struct.
No numpy, no external packages.
Matches Block 4 Plan §6.8 and glTF 2.0 specification.
"""

import json
import math
import struct
from typing import List, Sequence, Tuple


# glTF Component Types
COMPONENT_TYPE_UNSIGNED_SHORT = 5123  # 2 bytes
COMPONENT_TYPE_UNSIGNED_INT = 5125    # 4 bytes
COMPONENT_TYPE_FLOAT = 5126           # 4 bytes

# Buffer View Targets
TARGET_ARRAY_BUFFER = 34962          # Vertex attributes
TARGET_ELEMENT_ARRAY_BUFFER = 34963  # Indices


def write_glb(
    vertices: Sequence[Tuple[float, float, float]],
    normals: Sequence[Tuple[float, float, float]],
    indices: Sequence[int],
    output_path: str,
    mesh_name: str = "Mesh",
) -> int:
    """
    Writes a binary GLB file containing positions, normals, and triangle indices.
    Returns total triangle count.
    """
    vertex_count = len(vertices)
    index_count = len(indices)
    triangle_count = index_count // 3

    if vertex_count == 0 or index_count == 0:
        # Create empty dummy mesh or single triangle
        vertices = [(0.0, 0.0, 0.0), (0.001, 0.0, 0.0), (0.0, 0.001, 0.0)]
        normals = [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)]
        indices = [0, 1, 2]
        vertex_count = 3
        index_count = 3
        triangle_count = 1

    # 1. Calculate Min and Max for POSITION accessor (mandatory in glTF)
    min_x = min(v[0] for v in vertices)
    max_x = max(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    max_y = max(v[1] for v in vertices)
    min_z = min(v[2] for v in vertices)
    max_z = max(v[2] for v in vertices)

    # 2. Pack Binary Data into Chunk
    bin_buffer = bytearray()

    # Pack Positions (Vec3 Float32)
    pos_offset = len(bin_buffer)
    for v in vertices:
        bin_buffer.extend(struct.pack("<3f", float(v[0]), float(v[1]), float(v[2])))
    pos_length = len(bin_buffer) - pos_offset

    # Pack Normals (Vec3 Float32)
    norm_offset = len(bin_buffer)
    for n in normals:
        bin_buffer.extend(struct.pack("<3f", float(n[0]), float(n[1]), float(n[2])))
    norm_length = len(bin_buffer) - norm_offset

    # Pack Indices (Unsigned Short or Unsigned Int)
    idx_offset = len(bin_buffer)
    use_uint32 = vertex_count >= 65536
    if use_uint32:
        idx_comp_type = COMPONENT_TYPE_UNSIGNED_INT
        for idx in indices:
            bin_buffer.extend(struct.pack("<I", int(idx)))
    else:
        idx_comp_type = COMPONENT_TYPE_UNSIGNED_SHORT
        for idx in indices:
            bin_buffer.extend(struct.pack("<H", int(idx)))
    idx_length = len(bin_buffer) - idx_offset

    # Pad binary buffer to 4-byte boundary
    pad_bytes = (4 - (len(bin_buffer) % 4)) % 4
    bin_buffer.extend(b"\x00" * pad_bytes)

    # 3. Build JSON Chunk
    gltf_dict = {
        "asset": {
            "version": "2.0",
            "generator": "FRCBananaSim Fusion 360 Exporter"
        },
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": mesh_name}],
        "meshes": [
            {
                "name": mesh_name,
                "primitives": [
                    {
                        "attributes": {
                            "POSITION": 0,
                            "NORMAL": 1,
                        },
                        "indices": 2,
                        "mode": 4  # TRIANGLES
                    }
                ]
            }
        ],
        "accessors": [
            {
                "bufferView": 0,
                "byteOffset": 0,
                "componentType": COMPONENT_TYPE_FLOAT,
                "count": vertex_count,
                "type": "VEC3",
                "min": [min_x, min_y, min_z],
                "max": [max_x, max_y, max_z],
            },
            {
                "bufferView": 1,
                "byteOffset": 0,
                "componentType": COMPONENT_TYPE_FLOAT,
                "count": vertex_count,
                "type": "VEC3",
            },
            {
                "bufferView": 2,
                "byteOffset": 0,
                "componentType": idx_comp_type,
                "count": index_count,
                "type": "SCALAR",
            }
        ],
        "bufferViews": [
            {
                "buffer": 0,
                "byteOffset": pos_offset,
                "byteLength": pos_length,
                "target": TARGET_ARRAY_BUFFER,
            },
            {
                "buffer": 0,
                "byteOffset": norm_offset,
                "byteLength": norm_length,
                "target": TARGET_ARRAY_BUFFER,
            },
            {
                "buffer": 0,
                "byteOffset": idx_offset,
                "byteLength": idx_length,
                "target": TARGET_ELEMENT_ARRAY_BUFFER,
            }
        ],
        "buffers": [
            {
                "byteLength": len(bin_buffer)
            }
        ]
    }

    json_str = json.dumps(gltf_dict, separators=(",", ":"))
    json_bytes = bytearray(json_str.encode("utf-8"))

    # Pad JSON chunk with spaces to 4-byte boundary per glTF spec
    json_pad = (4 - (len(json_bytes) % 4)) % 4
    json_bytes.extend(b" " * json_pad)

    # 4. Assemble Final GLB File
    # Total length = 12 (Header) + 8 + len(JSON) + 8 + len(BIN)
    total_length = 12 + (8 + len(json_bytes)) + (8 + len(bin_buffer))

    with open(output_path, "wb") as f:
        # GLB Header
        f.write(struct.pack("<4sII", b"glTF", 2, total_length))

        # Chunk 0: JSON
        f.write(struct.pack("<I4s", len(json_bytes), b"JSON"))
        f.write(json_bytes)

        # Chunk 1: BIN
        f.write(struct.pack("<I4s", len(bin_buffer), b"BIN\x00"))
        f.write(bin_buffer)

    return triangle_count
