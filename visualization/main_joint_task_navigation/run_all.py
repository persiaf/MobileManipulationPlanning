"""Render main-scene local-joint comparisons from completed CSV/JSON exports."""
import hashlib
import json
from pathlib import Path
import platform
import re

import matplotlib
import numpy as np
from PIL import Image

from main_common import STYLE_SOURCE, arguments, output_directory
from plot_main_scenes import generate as scenes
from plot_main_metrics import generate as metrics


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    args = arguments(__doc__)
    out = output_directory(args.data_dir, args.output_dir)
    outputs, sources = [], {STYLE_SOURCE}
    state_path = Path(args.data_dir)/"run_state.json"
    if state_path.is_file():
        state = json.loads(state_path.read_text(encoding="utf-8-sig"))
        if str(state.get("status", "")).upper() != "COMPLETE":
            raise ValueError("Experiment run_state is not COMPLETE; wait for the finished canonical export")
        sources.add(state_path)
    for generate in (scenes, metrics):
        created, used = generate(args.data_dir, out, args.details)
        outputs.extend(created)
        sources.update(used)
    checks = {}
    for path in outputs:
        if path.suffix == ".pdf":
            content = path.read_bytes()
            raster_count = len(re.findall(rb"/Subtype\s*/Image", content))
            if raster_count:
                raise ValueError(f"{path.name}: raster image object found in requested vector PDF")
            checks[path.name] = {"raster_image_objects": raster_count, "embedded_truetype": b"/FontFile2" in content}
        elif path.suffix == ".png":
            with Image.open(path) as image:
                checks[path.name] = {"size_px": image.size, "dpi": image.info.get("dpi")}
    manifest = {
        "python": platform.python_version(), "numpy": np.__version__, "matplotlib": matplotlib.__version__,
        "details": args.details,
        "style_reference": "results/figures/fig_main_scenario_publication.png",
        "style_helper": str(STYLE_SOURCE.resolve()),
        "inputs_sha256": {str(path.resolve()): sha256(path) for path in sorted(sources)},
        "scripts_sha256": {path.name: sha256(path) for path in sorted(Path(__file__).parent.glob("*.py"))},
        "outputs_sha256": {path.name: sha256(path) for path in outputs}, "render_checks": checks,
    }
    name = "figure_manifest_details.json" if args.details else "figure_manifest.json"
    manifest_path = out/name
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    for path in [*outputs, manifest_path]:
        print(path.resolve())


if __name__ == "__main__":
    main()
