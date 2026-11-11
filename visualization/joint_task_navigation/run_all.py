"""Render the main-style scene comparisons and joint-planning metric panels.

Run from any working directory, for example:
    python visualization/joint_task_navigation/run_all.py
Both --data-dir and --output-dir may override the default experiment paths.
"""
import hashlib
import json
from pathlib import Path
import platform
import re

import matplotlib
import numpy as np
from PIL import Image

from common import arguments, output_directory
from plot_scenes import generate as scenes
from plot_metrics import generate as metrics


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    args = arguments(__doc__)
    output_dir = output_directory(args.data_dir, args.output_dir)
    outputs, sources = [], set()
    for generate in (scenes, metrics):
        created, inputs = generate(args.data_dir, output_dir)
        outputs.extend(created)
        sources.update(inputs)
    checks = {}
    for path in outputs:
        if path.suffix == ".pdf":
            content = path.read_bytes()
            count = len(re.findall(rb"/Subtype\s*/Image", content))
            if count:
                raise ValueError(f"{path.name}: raster image object found in requested vector PDF")
            checks[path.name] = {"raster_image_objects": count, "embedded_truetype": b"/FontFile2" in content}
        elif path.suffix == ".png":
            with Image.open(path) as image:
                checks[path.name] = {"size_px": image.size, "dpi": image.info.get("dpi")}
    manifest = {
        "python": platform.python_version(), "matplotlib": matplotlib.__version__, "numpy": np.__version__,
        "style_reference": "results/figures/fig_main_scenario_publication.png",
        "semantic_encoding": {"tables": "gray", "tasks": "green circles", "navigation": "blue actual path",
                              "docks": "orange triangles", "start": "red star", "base": "blue footprint and heading"},
        "inputs_sha256": {str(path.resolve()): sha256(path) for path in sorted(sources)},
        "scripts_sha256": {path.name: sha256(path) for path in sorted(Path(__file__).parent.glob("*.py"))},
        "outputs_sha256": {path.name: sha256(path) for path in outputs},
        "render_checks": checks,
    }
    manifest_path = output_dir / "figure_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    for path in [*outputs, manifest_path]:
        print(path.resolve())


if __name__ == "__main__":
    main()
