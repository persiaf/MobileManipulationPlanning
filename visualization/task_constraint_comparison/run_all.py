"""Rebuild all three comparison figures from the exported experiment files.

Example (from the repository root):
    python visualization/task_constraint_comparison/run_all.py
    python visualization/task_constraint_comparison/run_all.py --data-dir PATH --output-dir PATH
"""
import hashlib
import json
from pathlib import Path
import platform

import matplotlib
import numpy as np

from common import arguments, output_directory
from plot_workspace import generate as workspace
from plot_dock_routes import generate as docks
from plot_metrics import generate as metrics


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    args = arguments(__doc__)
    out = output_directory(args.data_dir, args.output_dir)
    outputs = []
    sources = set()
    for generate in (workspace, docks, metrics):
        created, used = generate(args.data_dir, out)
        outputs.extend(created)
        sources.update(used)
    scripts = sorted(Path(__file__).parent.glob("*.py"))
    manifest = {
        "python": platform.python_version(),
        "matplotlib": matplotlib.__version__,
        "numpy": np.__version__,
        "rendering": {"pdf": "vector paths and TrueType fonts", "png_dpi": 300},
        "inputs_sha256": {str(path.resolve()): _sha256(path) for path in sorted(sources)},
        "scripts_sha256": {path.name: _sha256(path) for path in scripts},
        "outputs_sha256": {path.name: _sha256(path) for path in outputs},
    }
    manifest_path = out / "figure_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    includes = []
    figure_captions = (
        ("workspace_support", "workspace-support", "Sampled manipulation support under task-specific actions (top row) and matched Cartesian paths (bottom row). Colored samples admit a numerical IK sequence; gray samples denote finite-search failures. All panels share the same world-coordinate limits."),
        ("dock_routes", "dock-routes", "Candidate support, selected base poses, and exported navigation paths in the shared scene. Candidate colors show the maximum support ratio over sampled headings at each position. Rectangles and arrows show selected base footprints and orientations."),
        ("comparison_metrics", "comparison-metrics", "Candidate--target pair support, selected dock counts, navigation length, and observed scoring time. Values describe this exported run; error bars and statistical significance are not inferred. Hatched route bars, if any, denote failed navigation validation."),
    )
    for name, label, caption in figure_captions:
        includes.append("\\begin{figure*}[t]\n\\centering\n"
                        f"\\includegraphics[width=0.98\\textwidth]{{{name}.pdf}}\n"
                        f"% Full Chinese caption and exact values: {name}_caption.md\n"
                        f"\\caption{{{caption}}}\n"
                        f"\\label{{fig:{label}}}\n\\end{{figure*}}\n")
    (out / "latex_includes.tex").write_text("\n".join(includes), encoding="utf-8")
    for path in [*outputs, manifest_path, out / "latex_includes.tex"]:
        print(path.resolve())


if __name__ == "__main__":
    main()
