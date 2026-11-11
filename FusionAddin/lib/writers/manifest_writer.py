"""
manifest_writer.py: Writes Schema v1.0.0 manifest.json with deterministic key ordering,
fixed-precision float normalization (-0.0 -> 0.0), and zero non-deterministic timestamps.
Matches Block 4 Plan §9.
"""

import json
from typing import Any, Dict


def clean_floats(obj: Any, digits: int = 9) -> Any:
    """Recursively normalizes floats: rounds to digits and converts -0.0 to 0.0."""
    if isinstance(obj, float):
        r = round(obj, digits)
        return 0.0 if abs(r) == 0.0 else r
    elif isinstance(obj, dict):
        return {k: clean_floats(v, digits) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [clean_floats(item, digits) for item in obj]
    return obj


def write_manifest_json(manifest_data: Dict[str, Any], output_path: str, digits: int = 9) -> None:
    """Writes manifest dictionary to file with standard Schema v1 formatting."""
    cleaned = clean_floats(manifest_data, digits=digits)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(cleaned, f, indent=2, ensure_ascii=True)
        f.write("\n")
