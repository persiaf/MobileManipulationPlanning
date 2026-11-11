"""Render the completed failure-inclusive factorial comparison."""
import hashlib
import json
from pathlib import Path
import platform
import re

import matplotlib
import numpy as np
from PIL import Image

from batch_common import Batch, REFERENCE, STYLE_ADAPTER, TIE_TOL, arguments, output_directory
from plot_batch import generate


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    args = arguments(__doc__)
    batch = Batch(args.data_dir)
    out = output_directory(args.data_dir, args.output_dir)
    outputs, selection = generate(batch, out, args.representative_case)
    checks = {}
    for path in outputs:
        if path.suffix == ".pdf":
            data = path.read_bytes()
            raster_count = len(re.findall(rb"/Subtype\s*/Image", data))
            if raster_count:
                raise ValueError(f"{path.name}: scientific vector PDF contains raster images")
            checks[path.name] = {"raster_image_objects": raster_count, "embedded_truetype": b"/FontFile2" in data}
        elif path.suffix == ".png":
            with Image.open(path) as im:
                checks[path.name] = {"size_px": im.size, "dpi": im.info.get("dpi")}
    sources = batch.source_paths() | {STYLE_ADAPTER, Path(REFERENCE.__file__)}
    manifest = {"python": platform.python_version(), "numpy": np.__version__, "matplotlib": matplotlib.__version__,
                "scenario_count": len(batch.cases), "primary_raw_methods": ["minimum_seed", "reorder_only", "joint_local"],
                "tie_tolerance_absolute_J": TIE_TOL, "selection": selection,
                "style_reference": "results/figures/fig_main_scenario_publication.png",
                "inputs_sha256": {str(p.resolve()): sha256(p) for p in sorted(sources)},
                "scripts_sha256": {p.name: sha256(p) for p in sorted(Path(__file__).parent.glob("*.py"))},
                "outputs_sha256": {p.name: sha256(p) for p in outputs}, "render_checks": checks}
    path = out/"figure_manifest.json"
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for output in [*outputs, path]:
        print(output.resolve())


if __name__ == "__main__":
    main()
