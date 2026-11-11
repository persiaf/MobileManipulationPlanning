"""Read failure-inclusive factorial exports and share the main-scene style."""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
STYLE_ADAPTER = ROOT / "visualization/main_joint_task_navigation/main_common.py"
_spec = importlib.util.spec_from_file_location("_robustness_main_style", STYLE_ADAPTER)
main_style = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_style)
DEFAULT_DATA = ROOT / "results/task_navigation_robustness_20261111"
MODES = ("position_only", "top_down_grasp", "table_wipe")
RAW_METHODS = ("staged", "minimum_seed", "reorder_only", "joint_local")
METHODS = ("minimum_seed", "reorder_only", "joint_local")
COMPARATORS = ("minimum_seed", "reorder_only")
TIE_TOL = 1e-7  # absolute weighted-cost difference, m-equivalent
ALIASES = {"minimum_seed": "staged_minimum", "reorder_only": "reorder_only", "joint_local": "joint_local"}
MODE_LABELS = main_style.MODE_LABELS
METHOD_LABELS = {"minimum_seed": "Staged + minimum repair", "reorder_only": "Reorder only", "joint_local": "Joint local search"}
METHOD_ZH = {"minimum_seed": "分阶段+共同最少覆盖修复", "reorder_only": "仅重排", "joint_local": "联合局部优化"}
METHOD_COLORS = {"minimum_seed": "#0072B2", "reorder_only": "#E69F00", "joint_local": "#009E73"}
LAYOUT_LABELS = {0: "Original", 1: "Staggered", 2: "Rotated"}
DISTRIBUTION_LABELS = {0: "Original", 1: "Clustered", 2: "Spread"}
style, save, caption = main_style.style, main_style.save, main_style.caption
draw_map, draw_base = main_style.draw_map, main_style.draw_base
PATH_COLOR, START_COLOR = main_style.PATH_COLOR, main_style.START_COLOR
TASK_FACE, TASK_EDGE = main_style.TASK_FACE, main_style.TASK_EDGE
REFERENCE = main_style.reference


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--representative-case", default="L0_D2",
                        help="Predeclared route example; default original layout / spread distribution")
    return parser.parse_args()


def output_directory(data_dir, requested=None):
    path = Path(requested) if requested else Path(data_dir)/"figures"
    path.mkdir(parents=True, exist_ok=True)
    return path


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def read_csv(path, required=(), allow_empty=False):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if set(required)-set(reader.fieldnames or ()):
            raise ValueError(f"{path}: missing columns {sorted(set(required)-set(reader.fieldnames or ()))}")
        rows = list(reader)
    if not rows and not allow_empty:
        raise ValueError(f"{path}: no exported rows")
    return rows


def num(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return math.nan


def integer(value):
    result = num(value)
    if not math.isfinite(result) or int(result) != result:
        raise ValueError(f"Expected finite integer, got {value!r}")
    return int(result)


def boolean(value):
    if str(value).strip().lower() in ("true", "1", "1.0"):
        return True
    if str(value).strip().lower() in ("false", "0", "0.0"):
        return False
    raise ValueError(f"Expected Boolean export, got {value!r}")


def full_tasks(row):
    return (integer(row["covered_tasks"]) == integer(row["target_count"]) and
            integer(row["verified_tasks"]) == integer(row["target_count"]))


def full_mission(row):
    return (boolean(row["mission_valid"]) and boolean(row["route_valid"]) and full_tasks(row) and
            math.isfinite(num(row["route_length_m"])) and math.isfinite(num(row["navigation_cost"])))


def minimum_mission(row):
    return (full_mission(row) and math.isfinite(num(row["minimum_docks"])) and
            integer(row["docks"]) == integer(row["minimum_docks"]))


def invalid_reason(row):
    if not full_tasks(row):
        return "TASK_COVERAGE_OR_RECHECK_INCOMPLETE"
    if not boolean(row["route_valid"]) or not boolean(row["mission_valid"]):
        return row.get("failure_reason", "").strip() or row.get("status", "INVALID_MISSION")
    if not math.isfinite(num(row["minimum_docks"])):
        return "NO_FINITE_MINIMUM_CERTIFICATE"
    if num(row["docks"]) != num(row["minimum_docks"]):
        return "DOCK_COUNT_NOT_MINIMUM"
    return "NONFINITE_COST_OR_LENGTH"


class Case:
    def __init__(self, root, spec):
        self.spec = spec
        self.case_id = spec["case_id"]
        self.layout = integer(spec["layout_id"])
        self.distribution = integer(spec["distribution_id"])
        self.layout_label = spec.get("layout_label", LAYOUT_LABELS.get(self.layout, str(self.layout))).replace("_", " ").title()
        self.distribution_label = spec.get("distribution_label", DISTRIBUTION_LABELS.get(self.distribution, str(self.distribution))).replace("_", " ").title()
        self.directory = (root/spec["relative_dir"]).resolve()
        self.sources = {self.directory/"metadata.json", self.directory/"summary.csv", self.directory/"run_state.json"}
        self.metadata = read_json(self.directory/"metadata.json")
        state = read_json(self.directory/"run_state.json")
        if str(state.get("status", "")).upper() != "COMPLETE":
            raise ValueError(f"{self.case_id}: scenario is not COMPLETE")
        required = ("mode", "method", "covered_tasks", "target_count", "verified_tasks", "docks", "minimum_docks",
                    "route_length_m", "navigation_cost", "route_valid", "mission_valid", "status")
        raw = read_csv(self.directory/"summary.csv", required)
        self.rows = {(r["mode"], r["method"]): r for r in raw}
        if len(self.rows) != len(raw) or set(self.rows) != {(mode, method) for mode in MODES for method in RAW_METHODS}:
            raise ValueError(f"{self.case_id}: expected unique complete 3-mode / 4-raw-method records")
        self._tasks = None
        self._routes = None

    def scene(self):
        scene = dict(self.metadata["scene"])
        angles = scene.get("table_orientations_rad", scene.get("table_orientations"))
        if angles is None:
            angles = [0.0]*len(scene["table_centers"])
        angles = np.asarray(angles, dtype=float).reshape(-1)
        if len(angles) != len(scene["table_centers"]) or not np.isfinite(angles).all():
            raise ValueError(f"{self.case_id}: table angles do not match centers")
        scene["table_orientations_rad"] = angles.tolist()
        return scene

    def tasks(self):
        if self._tasks is None:
            path = self.directory/"tasks.csv"
            rows = read_csv(path, ("task_id", "x_m", "y_m", "z_m"))
            tasks = {integer(r["task_id"]): (num(r["x_m"]), num(r["y_m"])) for r in rows}
            if len(tasks) != len(rows) or len(tasks) != integer(self.rows[MODES[0], METHODS[0]]["target_count"]):
                raise ValueError(f"{self.case_id}: task IDs/count are inconsistent")
            if not np.isfinite(np.asarray(list(tasks.values()))).all():
                raise ValueError(f"{self.case_id}: nonfinite task coordinate")
            self._tasks = tasks
            self.sources.add(path)
        return self._tasks

    def routes(self):
        if self._routes is not None:
            return self._routes
        dp, pp = self.directory/"docking_poses.csv", self.directory/"navigation_paths.csv"
        docks = read_csv(dp, ("mode", "method", "dock_id", "candidate_id", "x_m", "y_m", "yaw_rad"), allow_empty=True)
        paths = read_csv(pp, ("mode", "method", "point_id", "x_m", "y_m", "yaw_rad"), allow_empty=True)
        self.sources.update((dp, pp))
        result = {}
        for mode in MODES:
            for method in METHODS:
                key = mode, method
                summary = self.rows[key]
                selected = [r for r in docks if (r["mode"], r["method"]) == key]
                valid = minimum_mission(summary)
                ranks = bool(selected) and all(math.isfinite(num(r.get("visit_rank"))) and num(r["visit_rank"]) > 0 for r in selected)
                selected.sort(key=lambda r: integer(r["visit_rank"] if ranks else r["dock_id"]))
                if valid and len(selected) != integer(summary["docks"]):
                    raise ValueError(f"{self.case_id}/{key}: dock count differs")
                if ranks and [integer(r["visit_rank"]) for r in selected] != list(range(1, len(selected)+1)):
                    raise ValueError(f"{self.case_id}/{key}: invalid visit ranks")
                route = sorted((r for r in paths if (r["mode"], r["method"]) == key), key=lambda r: integer(r["point_id"]))
                xy = np.zeros((0, 2))
                if valid:
                    if len(route) < 2 or [integer(r["point_id"]) for r in route] != list(range(1, len(route)+1)):
                        raise ValueError(f"{self.case_id}/{key}: missing/duplicate path samples")
                    xy = np.asarray([[num(r["x_m"]), num(r["y_m"])] for r in route])
                    length = np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()
                    if not np.isfinite(xy).all() or abs(length-num(summary["route_length_m"])) > 1e-5:
                        raise ValueError(f"{self.case_id}/{key}: polyline length differs")
                result[key] = {"summary": summary, "docks": selected, "xy": xy, "valid": valid, "ranks": ranks}
        self._routes = result
        return result


class Batch:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        sp, mp, bp = (self.directory/name for name in ("run_state.json", "scenario_manifest.csv", "batch_summary.csv"))
        self.sources = {sp, mp, bp}
        self.state = read_json(sp)
        if str(self.state.get("status", "")).upper() != "COMPLETE":
            raise ValueError("Batch is not COMPLETE; do not publish a partial stability report")
        specs = read_csv(mp, ("case_id", "layout_id", "distribution_id", "seed", "relative_dir"))
        if len({r["case_id"] for r in specs}) != len(specs):
            raise ValueError("Manifest contains duplicate case IDs")
        if "scenario_count" in self.state and integer(self.state["scenario_count"]) != len(specs):
            raise ValueError("Manifest case count differs from completed batch state")
        self.cases = [Case(self.directory, r) for r in specs]
        self.by_id = {case.case_id: case for case in self.cases}
        self.layout_ids = tuple(sorted({case.layout for case in self.cases}))
        self.distribution_ids = tuple(sorted({case.distribution for case in self.cases}))
        self.layout_labels = {case.layout: case.layout_label for case in self.cases}
        self.distribution_labels = {case.distribution: case.distribution_label for case in self.cases}
        batch_rows = read_csv(bp, ("case_id", "mode", "method"))
        combined = {(r["case_id"], r["mode"], r["method"]): r for r in batch_rows}
        expected = {(case.case_id, mode, method) for case in self.cases for mode in MODES for method in RAW_METHODS}
        if len(combined) != len(batch_rows) or set(combined) != expected:
            raise ValueError("Batch summary must retain all scenarios and all four raw methods")
        compare = ("covered_tasks", "target_count", "verified_tasks", "docks", "minimum_docks", "route_length_m",
                   "navigation_cost", "route_valid", "mission_valid", "status")
        for case in self.cases:
            self.sources.update(case.sources)
            for mode in MODES:
                for method in RAW_METHODS:
                    row, local = combined[case.case_id, mode, method], case.rows[mode, method]
                    for key in compare:
                        if key in ("route_valid", "mission_valid"):
                            if boolean(row[key]) != boolean(local[key]):
                                raise ValueError(f"{case.case_id}/{mode}/{method}: batch/local {key} differ")
                            continue
                        a, b = num(row[key]), num(local[key])
                        equal = row[key] == local[key]
                        if math.isfinite(a) and math.isfinite(b):
                            equal = abs(a-b) <= 1e-9
                        elif math.isnan(a) and math.isnan(b) and key != "status":
                            equal = str(row[key]).strip().lower() in ("nan", "") and str(local[key]).strip().lower() in ("nan", "")
                        if not equal:
                            raise ValueError(f"{case.case_id}/{mode}/{method}: batch/local {key} differ")

    def source_paths(self):
        paths = set(self.sources)
        for case in self.cases:
            paths.update(case.sources)
        return paths

    def pairs(self):
        pairs = []
        for case in self.cases:
            for mode in MODES:
                primary = {method: case.rows[mode, method] for method in METHODS}
                minima = [num(r["minimum_docks"]) for r in primary.values()]
                common_minimum = all(math.isfinite(k) for k in minima) and len(set(minima)) == 1
                common_tasks = all(integer(r["target_count"]) == integer(primary[METHODS[0]]["target_count"]) for r in primary.values())
                common_eligible = all(minimum_mission(r) for r in primary.values()) and common_minimum and common_tasks
                common_reasons = [f"{name}:{invalid_reason(row)}" for name, row in primary.items() if not minimum_mission(row)]
                if not common_minimum:
                    common_reasons.append("CERTIFIED_MINIMUM_DIFFERS" if all(math.isfinite(k) for k in minima) else "NO_COMMON_FINITE_MINIMUM")
                if not common_tasks:
                    common_reasons.append("TASK_COUNT_DIFFERS")
                joint = case.rows[mode, "joint_local"]
                for comparator in COMPARATORS:
                    baseline = case.rows[mode, comparator]
                    same_minimum = num(baseline["minimum_docks"]) == num(joint["minimum_docks"])
                    same_tasks = integer(baseline["target_count"]) == integer(joint["target_count"])
                    pairwise_eligible = minimum_mission(baseline) and minimum_mission(joint) and same_minimum and same_tasks
                    eligible = common_eligible
                    jb, jj = num(baseline["navigation_cost"]), num(joint["navigation_cost"])
                    lb, lj = num(baseline["route_length_m"]), num(joint["route_length_m"])
                    if eligible and (jb <= 0 or lb <= 0):
                        raise ValueError("Paired percentage requires positive reference cost and length")
                    delta = jb-jj if eligible else math.nan
                    outcome = ("win" if delta > TIE_TOL else "loss" if delta < -TIE_TOL else "tie") if eligible else "ineligible"
                    pairs.append({"case_id": case.case_id, "layout_id": case.layout, "distribution_id": case.distribution,
                                  "seed": case.spec["seed"], "mode": mode, "comparator": comparator,
                                  "comparator_alias": ALIASES[comparator], "eligible": eligible,
                                  "common_three_method_eligible": common_eligible, "pairwise_eligible": pairwise_eligible,
                                  "baseline_valid_minimum": minimum_mission(baseline), "joint_valid_minimum": minimum_mission(joint),
                                  "baseline_J": jb, "joint_J": jj, "J_delta": delta,
                                  "J_gain_pct": 100*delta/jb if eligible else math.nan,
                                  "baseline_L_m": lb, "joint_L_m": lj,
                                  "L_gain_pct": 100*(lb-lj)/lb if eligible else math.nan,
                                  "outcome": outcome,
                                  "ineligible_reason": "" if eligible else "; ".join(common_reasons)})
        return pairs


def aggregate_pairs(pairs, ncases):
    result = []
    for mode in MODES:
        for comparator in COMPARATORS:
            chosen = [p for p in pairs if p["mode"] == mode and p["comparator"] == comparator]
            eligible = [p for p in chosen if p["eligible"]]
            values = np.asarray([p["J_gain_pct"] for p in eligible])
            lengths = np.asarray([p["L_gain_pct"] for p in eligible])
            q1, median, q3 = np.percentile(values, [25, 50, 75]) if len(values) else (math.nan,)*3
            result.append({"mode": mode, "comparator": comparator, "comparator_alias": ALIASES[comparator],
                           "total_scenarios": ncases, "eligible_pairs": len(eligible), "ineligible_pairs": ncases-len(eligible),
                           "wins": sum(p["outcome"] == "win" for p in eligible),
                           "ties": sum(p["outcome"] == "tie" for p in eligible),
                           "losses": sum(p["outcome"] == "loss" for p in eligible),
                           "J_gain_median_pct": float(median), "J_gain_q1_pct": float(q1), "J_gain_q3_pct": float(q3),
                           "J_gain_worst_pct": float(values.min()) if len(values) else math.nan,
                           "J_gain_best_pct": float(values.max()) if len(values) else math.nan,
                           "L_gain_median_pct": float(np.median(lengths)) if len(lengths) else math.nan,
                           "L_gain_worst_pct": float(lengths.min()) if len(lengths) else math.nan})
    return result


def mission_counts(batch):
    return [{"mode": mode, "method": method, "primary_alias": ALIASES[method], "total_scenarios": len(batch.cases),
             "full_task_support_and_recheck": sum(full_tasks(case.rows[mode, method]) for case in batch.cases),
             "full_missions": sum(full_mission(case.rows[mode, method]) for case in batch.cases),
             "full_minimum_missions": sum(minimum_mission(case.rows[mode, method]) for case in batch.cases)}
            for mode in MODES for method in METHODS]
