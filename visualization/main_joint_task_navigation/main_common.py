"""General scene/data adapter for the actual main-scene experiment.

Reuse the existing publication palette and geometric drawing helpers without
changing the earlier two-table figure scripts or importing their fixed schema.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
STYLE_SOURCE = REPO_ROOT / "visualization" / "joint_task_navigation" / "common.py"
_spec = importlib.util.spec_from_file_location("_main_joint_reference_style", STYLE_SOURCE)
reference = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(reference)

DEFAULT_DATA = REPO_ROOT / "results" / "main_joint_task_navigation_20261111"
MODE_LABELS = reference.MODE_LABELS
METHOD_LABELS = {**reference.METHOD_LABELS, "minimum_seed": "Minimum seed", "joint_local": "Joint local search"}
METHOD_COLORS = {**reference.METHOD_COLORS, "minimum_seed": "#8A61AA", "joint_local": "#009E73"}
number, integer, boolean = reference.number, reference.integer, reference.boolean
style, save, caption = reference.style, reference.save, reference.caption
draw_map, draw_base, label_panel = reference.draw_map, reference.draw_base, reference.label_panel
PATH_COLOR, START_COLOR = reference.PATH_COLOR, reference.START_COLOR
TASK_FACE, TASK_EDGE = reference.TASK_FACE, reference.TASK_EDGE
ASSIGNMENT_COLOR = reference.ASSIGNMENT_COLOR
REFERENCES = (*reference.REFERENCES, STYLE_SOURCE)


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--details", action="store_true", help="Include all task IDs, visit labels and assignment links")
    return parser.parse_args()


def output_directory(data_dir, output_dir):
    directory = Path(output_dir) if output_dir else Path(data_dir) / "figures"
    directory.mkdir(parents=True, exist_ok=True)
    return directory


class Experiment:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        path = self.directory / "metadata.json"
        with path.open(encoding="utf-8-sig") as source:
            self.metadata = json.load(source)
        self.sources = {path}
        self._summary = None
        self.modes = None
        self.methods = None

    def rows(self, filename, required, allow_empty=False):
        path = self.directory / filename
        with path.open(newline="", encoding="utf-8-sig") as source:
            reader = csv.DictReader(source)
            missing = set(required)-set(reader.fieldnames or ())
            if missing:
                raise ValueError(f"{filename}: missing columns {sorted(missing)}")
            rows = list(reader)
        if not rows and not allow_empty:
            raise ValueError(f"{filename}: empty experiment export")
        self.sources.add(path)
        return rows

    def summaries(self):
        if self._summary is None:
            rows = self.rows("summary.csv", ("mode", "method", "covered_tasks", "target_count", "docks",
                                            "route_length_m", "navigation_cost", "route_valid"))
            records = {}
            for row in rows:
                key = row["mode"], row["method"]
                if key in records:
                    raise ValueError(f"summary.csv: duplicate mode/method result {key}")
                if row["mode"] not in MODE_LABELS or row["method"] not in METHOD_LABELS:
                    raise ValueError(f"summary.csv: unknown task/method label {key}")
                records[key] = row
            observed_modes = {key[0] for key in records}
            observed_methods = {key[1] for key in records}
            mode_order = self.metadata.get("mode_order", list(MODE_LABELS))
            method_order = self.metadata.get("methods", ["staged", "minimum_seed", "reorder_only", "joint_local", "joint"])
            self.modes = tuple(mode for mode in mode_order if mode in observed_modes)
            self.methods = tuple(method for method in method_order if method in observed_methods)
            if set(self.modes) != observed_modes or set(self.methods) != observed_methods:
                raise ValueError("metadata order does not name all observed results")
            if len(self.modes) != len(set(self.modes)) or len(self.methods) != len(set(self.methods)):
                raise ValueError("metadata orders must not duplicate modes or methods")
            expected = {(mode, method) for mode in self.modes for method in self.methods}
            if set(records) != expected:
                raise ValueError("summary.csv: incomplete mode/method comparison; wait for completed exports")
            self._summary = records
        return self._summary

    def scene_methods(self):
        self.summaries()
        comparator = "joint_local" if "joint_local" in self.methods else "joint"
        if "staged" not in self.methods or comparator not in self.methods:
            raise ValueError("Scene comparisons require staged and joint_local (or joint)")
        return "staged", comparator

    def scene(self):
        scene = dict(self.metadata["scene"])
        for field in ("map_width_m", "map_height_m", "table_length_m", "table_width_m"):
            if number(scene[field], field) <= 0:
                raise ValueError(f"scene.{field} must be positive")
        centers = np.asarray(scene["table_centers"], dtype=float)
        if centers.ndim != 2 or centers.shape[1] != 2 or not np.isfinite(centers).all():
            raise ValueError("scene.table_centers must be finite N-by-2 data")
        if "table_orientations_rad" not in scene and "table_orientations" in scene:
            scene["table_orientations_rad"] = scene["table_orientations"]
        return scene

    def footprint(self):
        footprint = self.metadata["robot_footprint"]
        length, width = number(footprint["length_m"]), number(footprint["width_m"])
        if min(length, width) <= 0:
            raise ValueError("Robot footprint dimensions must be positive")
        return length, width


def scene_legend(details=False):
    handles = reference.scene_legend()
    return handles if details else handles[:-1]


def valid_mission(row):
    valid = boolean(row["route_valid"]) and integer(row["covered_tasks"]) == integer(row["target_count"])
    if "mission_valid" in row:
        valid = valid and boolean(row["mission_valid"])
    return valid
