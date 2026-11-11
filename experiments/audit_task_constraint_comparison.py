"""Audit exported evidence without rerunning the MATLAB scoring code."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

MODES = ("position_only", "top_down_grasp", "table_wipe")


def read_rows(folder, name):
    with (folder / name).open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def truth(value):
    return str(value).lower() in {"1", "true"}


def corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((-length/2, -width/2), (length/2, -width/2),
                           (length/2, width/2), (-length/2, width/2))]


def overlaps(a, b):
    for poly in (a, b):
        for i in range(4):
            dx, dy = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            axis = (-dy, dx)
            pa = [p[0] * axis[0] + p[1] * axis[1] for p in a]
            pb = [p[0] * axis[0] + p[1] * axis[1] for p in b]
            if max(pa) < min(pb) - 1e-10 or max(pb) < min(pa) - 1e-10:
                return False
    return True


def audit(folder):
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
    summary = {r["mode"]: r for r in read_rows(folder, "summary.csv")}
    candidates = read_rows(folder, "candidate_support.csv")
    pairs = read_rows(folder, "pair_support.csv")
    docks = read_rows(folder, "docking_poses.csv")
    paths = read_rows(folder, "navigation_paths.csv")
    transfers = read_rows(folder, "plan_transfer.csv")
    tasks = read_rows(folder, "tasks.csv")
    scene = meta["scene"]
    obstacles = [corners(*p, 0, scene["table_length_m"], scene["table_width_m"])
                 for p in scene["table_centers"]]
    fp = meta["robot_footprint"]
    checks = []
    ref = None
    derived = {}
    for mode in MODES:
        cs = [r for r in candidates if r["mode"] == mode]
        identity = [(r["candidate_id"], r["table_id"], r["x_m"], r["y_m"], r["yaw_rad"]) for r in cs]
        if ref is None:
            ref = identity
        assert identity == ref, f"Candidate geometry differs: {mode}"
        ps = [r for r in pairs if r["mode"] == mode]
        feasible = [r for r in ps if truth(r["feasible"])]
        union = {r["task_id"] for r in feasible}
        sr = summary[mode]
        assert int(sr["pair_count"]) == len(ps)
        assert int(sr["supported_pairs"]) == len(feasible)
        assert int(sr["union_supported_count"]) == len(union)
        for row in feasible:
            assert float(row["max_position_error_m"]) <= 0.003 + 1e-10
            assert float(row["max_joint_step_rad"]) <= math.radians(25) + 1e-10
            tolerance = math.radians(5 if mode == "table_wipe" else 10)
            assert float(row["max_orientation_error_rad"]) <= tolerance + 1e-10
        ds = [r for r in docks if r["mode"] == mode]
        selected = {r["candidate_id"] for r in ds}
        assert len(ds) == int(sr["selected_docks"])
        lookup = {r["candidate_id"]: r for r in cs}
        for d in ds:
            c = lookup[d["candidate_id"]]
            assert all(abs(float(d[k]) - float(c[k])) < 1e-10 for k in ("x_m", "y_m", "yaw_rad"))
        selected_union = {r["task_id"] for r in feasible if r["candidate_id"] in selected}
        # Exact finite-pool set-cover lower bound. There are only five tasks
        # per table, so a 2^5-state DP can certify the minimum dock count.
        minimum_docks = 0
        for table_id in sorted({r["table_id"] for r in tasks}):
            task_ids = [r["task_id"] for r in tasks if r["table_id"] == table_id]
            bit = {tid: 1 << i for i, tid in enumerate(task_ids)}
            masks = []
            for c in cs:
                if c["table_id"] == table_id:
                    ids = {r["task_id"] for r in feasible if r["candidate_id"] == c["candidate_id"]}
                    masks.append(sum(bit[tid] for tid in ids))
            costs = [math.inf] * (1 << len(task_ids))
            costs[0] = 0
            for covered in range(len(costs)):
                if math.isfinite(costs[covered]):
                    for mask in masks:
                        costs[covered | mask] = min(costs[covered | mask], costs[covered] + 1)
            minimum_docks += costs[-1]
        ns = sorted((r for r in paths if r["mode"] == mode), key=lambda r: int(r["point_id"]))
        length = sum(math.hypot(float(b["x_m"])-float(a["x_m"]), float(b["y_m"])-float(a["y_m"]))
                     for a, b in zip(ns, ns[1:]))
        if truth(sr["route_valid"]):
            assert len(selected_union) == int(sr["target_count"]) == int(sr["verified_tasks"])
            assert abs(length - float(sr["route_length_m"])) < 1e-7
            for endpoint in (ns[0], ns[-1]):
                assert math.hypot(float(endpoint["x_m"])-scene["start_pose"][0],
                                  float(endpoint["y_m"])-scene["start_pose"][1]) < 1e-7
            for row in ns:
                poly = corners(float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]),
                               fp["length_m"], fp["width_m"])
                assert all(0 <= x <= scene["map_width_m"] and 0 <= y <= scene["map_height_m"] for x, y in poly)
                assert not any(overlaps(poly, obstacle) for obstacle in obstacles), f"Footprint intersects table: {mode}"
        else:
            assert not math.isfinite(float(sr["route_length_m"])), "Incomplete route was reported as a comparable cost"
        derived[mode] = {"pair_count": len(ps), "supported_pairs": len(feasible),
                         "selected_task_union": len(selected_union), "path_length_recomputed_m": length,
                         "path_samples_checked": len(ns), "minimum_docks_on_frozen_pool": minimum_docks}
        checks.append(f"{mode}: candidate identity, pair counts, limits, selected coverage, route length and sampled physical footprint")
    for row in transfers:
        selected = {r["candidate_id"] for r in docks if r["mode"] == row["planned_mode"]}
        supported = {r["task_id"] for r in pairs if r["mode"] == row["evaluated_mode"]
                     and r["candidate_id"] in selected and truth(r["feasible"])}
        assert len(supported) == int(row["supported_tasks"])
    checks.append("All nine cross-task plan-transfer counts independently reconstructed")
    diagnostics = json.loads((folder / "diagnostics.json").read_text(encoding="utf-8-sig"))
    for filename, count_key, support_key in (
            ("workspace.csv", "workspace_count", "workspace_supported_counts"),
            ("matched_path_workspace.csv", "matched_path_count", "matched_path_supported_counts")):
        data = read_rows(folder, filename)
        assert len(data) == diagnostics[count_key]
        assert len({(r["x_m"], r["y_m"], r["z_m"]) for r in data}) == len(data)
        assert [sum(truth(r[m]) for r in data) for m in MODES] == diagnostics[support_key]
        for r in data:
            assert all(r[m] in {"0", "1", "true", "false"} for m in MODES)
            assert abs(float(r["z_m"]) - scene["table_height_m"]) < 1e-10
            cx, cy = scene["table_centers"][0]
            assert abs(float(r["x_m"]) - cx) + meta["wipe_length_m"] / 2 <= scene["table_length_m"] / 2 + 1e-10
            assert abs(float(r["y_m"]) - cy) <= scene["table_width_m"] / 2 + 1e-10
    data = read_rows(folder, "matched_path_workspace.csv")
    assert sum(truth(r[MODES[1]]) and not truth(r[MODES[0]]) for r in data) == diagnostics["matched_axis_success_position_failure"]
    assert sum(truth(r[MODES[2]]) and not truth(r[MODES[1]]) for r in data) == diagnostics["matched_full_success_axis_failure"]
    checks.append("Workspace counts, binary support, distinct anchors, tabletop path bounds and numerical non-monotonicity counts")
    sources = ("experiments/task_constraint_comparison_setup.m", "experiments/comparison_manipulation_task.m",
               "experiments/task_constraint_effective_settings.m",
               "experiments/exp_task_constraint_comparison.m", "manipulator/evaluate_task_reachability.m",
               "manipulator/resolve_manipulation_task.m", "manipulator/solve_arm_task_ik.m",
               "config/vx300s_kinematic_model.m")
    root = Path(__file__).resolve().parents[1]
    manifest = {p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in sources}
    report = {"status": "PASS", "checks": checks, "derived": derived, "source_sha256": manifest,
              "limits": "Independent CSV/geometric audit, not independent IK, continuous-motion or arm-collision validation."}
    (folder / "evidence_audit.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", type=Path, default=Path("results/task_constraint_comparison_20261111"))
    args = parser.parse_args()
    print(json.dumps(audit(args.folder), indent=2))
