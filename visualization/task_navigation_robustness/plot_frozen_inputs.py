"""Plot actual frozen inputs before outcomes exist; never create result rows."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re

import numpy as np
from PIL import Image

from batch_common import (Case, DISTRIBUTION_LABELS, LAYOUT_LABELS, REFERENCE, STYLE_ADAPTER,
                          arguments, integer, num, read_csv, read_json, style)
from plot_batch import scenario_maps


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FrozenCase(Case):
    def __init__(self, root, spec):
        self.spec = spec
        self.case_id = spec["case_id"]
        self.layout, self.distribution = integer(spec["layout_id"]), integer(spec["distribution_id"])
        self.layout_label = spec.get("layout_label", LAYOUT_LABELS.get(self.layout, str(self.layout))).replace("_", " ").title()
        self.distribution_label = spec.get("distribution_label", DISTRIBUTION_LABELS.get(self.distribution, str(self.distribution))).replace("_", " ").title()
        self.directory = (root/spec["relative_dir"]).resolve()
        # The worker adds run-time fields to metadata.json later. Read the
        # immutable frozen scene definition so this input-only artifact and
        # its hashes remain valid after the outcome runs finish.
        definition_path = self.directory/"case_definition.json"
        definition = read_json(definition_path)
        cfg = definition["scene_cfg"]
        self.metadata = {"task_count": definition["task_count"], "scene": {
            "table_centers": cfg["table_positions"], "table_orientations_rad": cfg["table_orientations"],
            "table_length_m": cfg["table"]["length"], "table_width_m": cfg["table"]["width"],
            "map_width_m": cfg["map_width_m"], "map_height_m": cfg["map_height_m"],
            "border_thickness_m": cfg["border_thickness_m"], "start_pose": cfg["start_pose"]}}
        self.sources = {definition_path}
        self._tasks = None

    def tasks(self):
        if self._tasks is None:
            path = self.directory/"tasks.csv"
            rows = read_csv(path, ("task_id", "x_m", "y_m", "z_m"))
            self._tasks = {integer(r["task_id"]): (num(r["x_m"]), num(r["y_m"])) for r in rows}
            if len(self._tasks) != len(rows) or len(rows) != integer(self.metadata["task_count"]):
                raise ValueError(f"{self.case_id}: frozen task count/IDs mismatch")
            if not np.isfinite(np.asarray(list(self._tasks.values()))).all():
                raise ValueError(f"{self.case_id}: nonfinite frozen tasks")
            self.sources.add(path)
        return self._tasks


class FrozenBatch:
    def __init__(self, folder):
        self.directory = Path(folder).resolve()
        manifest = self.directory/"scenario_manifest.csv"
        specs = read_csv(manifest, ("case_id", "layout_id", "distribution_id", "relative_dir"))
        if len({row["case_id"] for row in specs}) != len(specs):
            raise ValueError("Duplicate frozen case IDs")
        self.cases = [FrozenCase(self.directory, row) for row in specs]
        self.layout_ids = tuple(sorted({case.layout for case in self.cases}))
        self.distribution_ids = tuple(sorted({case.distribution for case in self.cases}))
        self.layout_labels = {case.layout: case.layout_label for case in self.cases}
        self.distribution_labels = {case.distribution: case.distribution_label for case in self.cases}
        self.sources = {manifest, STYLE_ADAPTER, Path(REFERENCE.__file__)}
        for case in self.cases:
            case.tasks()
            self.sources.update(case.sources)


def main():
    args = arguments(__doc__)
    batch = FrozenBatch(args.data_dir)
    out = args.output_dir or Path(args.data_dir)/"input_figures"
    out.mkdir(parents=True, exist_ok=True)
    before = {str(p): sha256(p) for p in batch.sources}
    style()
    outputs = scenario_maps(batch, out, "frozen_inputs_overview", inputs_only=True)
    after = {str(p): sha256(p) for p in batch.sources}
    if before != after:
        raise ValueError("Input files changed while plotting; rerun once the metadata write finishes")
    checks = {}
    for p in outputs:
        if p.suffix == ".pdf":
            data = p.read_bytes()
            count = len(re.findall(rb"/Subtype\s*/Image", data))
            if count:
                raise ValueError("Frozen input figure should be vector")
            checks[p.name] = {"raster_image_objects": count, "embedded_truetype": b"/FontFile2" in data}
        if p.suffix == ".png":
            with Image.open(p) as im:
                checks[p.name] = {"size_px": im.size, "dpi": im.info.get("dpi")}
    manifest = {"status": "FROZEN_INPUTS_ONLY", "outcome_rows_read_or_created": 0,
                "scenario_count": len(batch.cases), "inputs_sha256_at_render": before,
                "scripts_sha256": {p.name: sha256(p) for p in Path(__file__).parent.glob("*.py")},
                "outputs_sha256": {p.name: sha256(p) for p in outputs}, "render_checks": checks}
    (out/"frozen_inputs_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    for p in outputs:
        print(p.resolve())


if __name__ == "__main__":
    main()
