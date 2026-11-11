"""Independent finite-pool cover/tour, CSV and sampled-base geometry audit.

Uses only Python's standard library and exported evidence. It does not import
the MATLAB selector, solve IK, or certify continuous arm/base motion.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODES = ("position_only", "top_down_grasp", "table_wipe")
METHODS = ("staged", "reorder_only", "joint")
TOL = 1e-7


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as source:
        return list(csv.DictReader(source))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose(row):
    return tuple(float(row[k]) for k in ("x_m", "y_m", "yaw_rad"))


def corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in
            ((-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2))]


def overlaps(a, b):
    for polygon in (a, b):
        for i in range(4):
            dx = polygon[(i+1) % 4][0] - polygon[i][0]
            dy = polygon[(i+1) % 4][1] - polygon[i][1]
            pa = [-x*dy+y*dx for x, y in a]
            pb = [-x*dy+y*dx for x, y in b]
            if max(pa) < min(pb)-1e-10 or max(pb) < min(pa)-1e-10:
                return False
    return True


def exact_tour(groups, costs):
    best, route, evaluated = math.inf, None, 0
    for combination in itertools.product(*groups):
        selected = tuple(itertools.chain.from_iterable(combination))
        for order in itertools.permutations(selected):
            closed = (1,) + order + (1,)
            cost = sum(costs[a, b] for a, b in zip(closed, closed[1:]))
            evaluated += 1
            if cost < best:
                best, route = cost, closed
    assert math.isfinite(best), "No finite minimum-cover Hamiltonian tour"
    return {"navigation_cost": best, "one_optimal_closed_route": route,
            "routes_evaluated": evaluated}


def audit(folder):
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
    old = Path(meta["source_evidence"])
    if not old.is_absolute():
        old = ROOT / old
    prior_meta = json.loads((old / "metadata.json").read_text(encoding="utf-8-sig"))
    assert meta["source_geometry_signature"] == prior_meta["geometry_signature"]
    assert meta["model_revision"] == prior_meta["model_revision"]
    assert meta["scene"]["table_centers"] == prior_meta["scene"]["table_centers"]
    pool, pair_rows = rows(folder / "candidate_pool.csv"), rows(folder / "frozen_pair_support.csv")
    assert pair_rows == rows(old / "pair_support.csv"), "Frozen IK evidence changed"
    old_candidates = [r for r in rows(old / "candidate_support.csv") if r["mode"] == MODES[0]]
    assert len(pool) == len(old_candidates) == meta["candidate_count"]
    node = {r["candidate_id"]: int(r["node_index"]) for r in pool}
    assert len(node) == len(pool)
    assert set(node.values()) == set(range(2, len(pool)+2))
    poses = {r["candidate_id"]: pose(r) for r in pool}
    for new, previous in zip(pool, old_candidates):
        assert new["candidate_id"] == previous["candidate_id"]
        assert new["table_id"] == previous["table_id"] and pose(new) == pose(previous)
    tasks = rows(folder / "tasks.csv")
    assert tasks == rows(old / "tasks.csv")
    task_ids = {r["task_id"] for r in tasks}
    pair = {(r["mode"], r["candidate_id"], r["task_id"]): r for r in pair_rows}
    assert len(pair) == len(pair_rows)
    for mode in MODES:
        expected = {(mode, c["candidate_id"], t["task_id"])
                    for c in pool for t in tasks if c["table_id"] == t["table_id"]}
        assert {k for k in pair if k[0] == mode} == expected
    old_docks = rows(old / "docking_poses.csv")
    covers_exported = rows(folder / "minimum_covers.csv")
    needed, groups_by_mode, cover_certificates = set(), {}, []
    tables = sorted({r["table_id"] for r in tasks}, key=int)
    for mode in MODES:
        groups = []
        for ri, table in enumerate(tables, start=1):
            ids = [r["task_id"] for r in tasks if r["table_id"] == table]
            assert len(ids) <= 16, "Audit mask budget exceeded"
            bits = {tid: 1 << i for i, tid in enumerate(ids)}
            candidate_ids = [r["candidate_id"] for r in pool if r["table_id"] == table]
            masks = {cid: sum(bits[tid] for tid in ids if pair[mode, cid, tid]["feasible"] == "1")
                     for cid in candidate_ids}
            dp = [math.inf] * (1 << len(ids)); dp[0] = 0
            for covered in range(len(dp)):
                for mask in masks.values():
                    dp[covered | mask] = min(dp[covered | mask], dp[covered] + 1)
            k = int(dp[-1]); full = (1 << len(ids))-1
            active = [cid for cid in candidate_ids if masks[cid]]
            minimum = []
            for choice in itertools.combinations(active, k):
                mask = 0
                for cid in choice:
                    mask |= masks[cid]
                if mask == full:
                    minimum.append(tuple(node[cid] for cid in choice))
            assert minimum
            groups.append(minimum)
            exported = [r for r in covers_exported if r["mode"] == mode and int(r["region_id"]) == ri]
            assert len(exported) == len(minimum)
            assert len({r["cover_id"] for r in exported}) == len(exported)
            assert {tuple(int(i) for i in r["matrix_node_indices"].split("|")) for r in exported} == set(minimum)
            for row in exported:
                assert [node[cid] for cid in row["candidate_ids"].split("|")] == [int(i) for i in row["matrix_node_indices"].split("|")]
            core = {n for cover in minimum for n in cover}
            needed.update((1, n) for n in core); needed.update((n, 1) for n in core)
            needed.update((a, b) for cover in minimum for a in cover for b in cover if a != b)
            cover_certificates.append({"mode": mode, "table_id": int(table), "minimum_docks": k,
                                       "cover_count": len(minimum), "core_candidates": len(core)})
        for a, b in itertools.combinations(groups, 2):
            ac, bc = set(itertools.chain.from_iterable(a)), set(itertools.chain.from_iterable(b))
            needed.update((i, j) for i in ac for j in bc)
            needed.update((j, i) for i in ac for j in bc)
        fixed = [1] + [node[r["candidate_id"]] for r in old_docks if r["mode"] == mode]
        needed.update((a, b) for a in fixed for b in fixed if a != b)
        groups_by_mode[mode] = groups
    edges = rows(folder / "navigation_edges.csv")
    actual_edges = {(int(r["from_node"]), int(r["to_node"])) for r in edges}
    assert len(actual_edges) == len(edges) and actual_edges == needed
    node_pose = {node[cid]: p for cid, p in poses.items()}; node_pose[1] = tuple(meta["scene"]["start_pose"])
    costs, lam = {}, float(meta["lambda_theta_m_per_rad"])
    for row in edges:
        a, b = int(row["from_node"]), int(row["to_node"])
        pa, pb = node_pose[a], node_pose[b]
        yaw = abs(wrap(pb[2]-pa[2])); length = float(row["path_length_m"])
        assert abs(yaw-float(row["endpoint_yaw_change_rad"])) < 1e-10
        value = float(row["navigation_cost"])
        if row["path_found"] == "1":
            assert length + 1e-8 >= math.hypot(pb[0]-pa[0], pb[1]-pa[1])
            assert abs(value-length-lam*yaw) < 1e-10
        else:
            assert math.isinf(length) and math.isinf(value)
        costs[a, b] = value
    optima = {mode: exact_tour(groups_by_mode[mode], costs) for mode in MODES}
    summaries, docks = rows(folder / "summary.csv"), rows(folder / "docking_poses.csv")
    assignment, paths = rows(folder / "task_assignments.csv"), rows(folder / "navigation_paths.csv")
    assert {(s["mode"], s["method"]) for s in summaries} == set(itertools.product(MODES, METHODS))
    assert len(summaries) == len(MODES)*len(METHODS)
    scene, fp = meta["scene"], meta["robot_footprint"]
    obstacles = [corners(x, y, 0, scene["table_length_m"], scene["table_width_m"])
                 for x, y in scene["table_centers"]]
    derived, selected_sets = {}, {}
    for summary in summaries:
        mode, method = summary["mode"], summary["method"]
        match = lambda r: r["mode"] == mode and r["method"] == method
        ds = sorted(filter(match, docks), key=lambda r: int(r["visit_rank"]))
        aa = list(filter(match, assignment)); ps = sorted(filter(match, paths), key=lambda r: int(r["point_id"]))
        minimum = sum(len(g[0]) for g in groups_by_mode[mode])
        assert summary["route_valid"] == "1"
        assert (summary["exact_on_candidate_pool"] == "1") == (method == "joint")
        assert (summary["route_cost_optimal"] == "1") == (method == "joint")
        assert (summary["route_order_optimal"] == "1") == (method != "staged")
        assert len(aa) == len(task_ids) and {r["task_id"] for r in aa} == task_ids
        assert int(summary["covered_tasks"]) == int(summary["target_count"]) == len(task_ids)
        assert len(ds) == int(summary["docks"]) == int(summary["minimum_docks"]) == minimum
        assert [int(r["visit_rank"]) for r in ds] == list(range(1, len(ds)+1))
        for row in aa:
            support = pair[mode, row["candidate_id"], row["task_id"]]
            assert support["feasible"] == "1" and support["assigned_arm"] == row["arm"]
            dock = next(d for d in ds if d["dock_id"] == row["dock_id"])
            assert dock["candidate_id"] == row["candidate_id"]
        for dock in ds:
            assert pose(dock) == poses[dock["candidate_id"]]
            assert set(dock["covered_task_ids"].split("|")) == {r["task_id"] for r in aa if r["dock_id"] == dock["dock_id"]}
        for endpoint in (ps[0], ps[-1]):
            assert math.dist(pose(endpoint)[:2], scene["start_pose"][:2]) < TOL
            assert abs(wrap(pose(endpoint)[2]-scene["start_pose"][2])) < TOL
        length = sum(math.dist(pose(a)[:2], pose(b)[:2]) for a, b in zip(ps, ps[1:]))
        assert abs(length-float(summary["route_length_m"])) < TOL
        ordered = [tuple(scene["start_pose"])] + [pose(r) for r in ds] + [tuple(scene["start_pose"])]
        yaw = sum(abs(wrap(b[2]-a[2])) for a, b in zip(ordered, ordered[1:]))
        value = length + lam*yaw
        assert abs(value-float(summary["navigation_cost"])) < TOL
        closed = (1,) + tuple(node[r["candidate_id"]] for r in ds) + (1,)
        assert abs(sum(costs[a, b] for a, b in zip(closed, closed[1:]))-value) < TOL
        cursor = 0
        for dock in ds:
            target = pose(dock)
            hit = next((i for i in range(cursor, len(ps)) if math.dist(pose(ps[i])[:2], target[:2]) < TOL
                        and abs(wrap(pose(ps[i])[2]-target[2])) < TOL), None)
            assert hit is not None; cursor = hit
        margin = scene["border_thickness_m"]
        for row in ps:
            poly = corners(*pose(row), fp["length_m"], fp["width_m"])
            assert all(margin-TOL <= x <= scene["map_width_m"]-margin+TOL and
                       margin-TOL <= y <= scene["map_height_m"]-margin+TOL for x, y in poly)
            assert not any(overlaps(poly, obstacle) for obstacle in obstacles)
        selected = {r["candidate_id"] for r in ds}; selected_sets[mode, method] = selected
        fixed_optimum = None
        if method == "staged":
            assert selected == {r["candidate_id"] for r in old_docks if r["mode"] == mode}
        elif method == "reorder_only":
            assert selected == selected_sets[mode, "staged"]
            fixed_optimum = exact_tour([[tuple(node[cid] for cid in sorted(selected))]], costs)
            assert abs(fixed_optimum["navigation_cost"]-value) < TOL
        else:
            assert abs(optima[mode]["navigation_cost"]-value) < TOL
        derived[f"{mode}/{method}"] = {"covered_tasks": len(aa), "minimum_docks": minimum,
                                       "route_length_recomputed_m": length, "navigation_cost_recomputed": value,
                                       "endpoint_yaw_change_rad": yaw, "path_samples_checked": len(ps),
                                       "fixed_set_order_optimum": fixed_optimum}
    for mode in MODES:
        values = [derived[f"{mode}/{method}"]["navigation_cost_recomputed"] for method in METHODS]
        assert values[2] <= values[1]+TOL <= values[0]+2*TOL
    filenames = ("metadata.json", "candidate_pool.csv", "frozen_pair_support.csv", "minimum_covers.csv",
                 "navigation_edges.csv", "summary.csv", "docking_poses.csv", "task_assignments.csv", "navigation_paths.csv")
    report = {"status": "PASS", "cases_checked": len(derived), "necessary_directed_edges": len(edges),
              "asymmetric_edge_pairs": sum(abs(v-costs[b, a]) > 1e-9 for (a, b), v in costs.items())//2,
              "minimum_cover_certificates": cover_certificates, "independent_global_optima": optima,
              "cases": derived,
              "checks": ["Frozen geometry/model signature, candidate identity and unchanged IK CSV",
                         "Unique complete same-table support pairs; exact DP cardinalities and all minimum covers",
                         "Complete necessary directed edge set and length/yaw cost formula",
                         "Independent exhaustive cover products and every Hamiltonian visiting permutation",
                         "Exactly-once supported assignments; fixed reorder sets and exact reorder optima",
                         "Recomputed route/cost; dock position/yaw visits in rank order; closed start position/yaw",
                         "Sampled base polygon separation from physical tables and border"],
              "limits": "Exact only on the frozen regional support partition, minimum-cover layer and computed directed navigation graph with each selected dock visited once. Not an independent IK solver, continuous/swept motion test, arm collision/contact/physical validation, scalability or general superiority evidence.",
              "inputs_sha256": {f: hashlib.sha256((folder/f).read_bytes()).hexdigest() for f in filenames},
              "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (folder / "independent_audit.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", nargs="?", type=Path, default=ROOT / "results/joint_task_navigation_20261111")
    result = audit(parser.parse_args().folder)
    print(json.dumps({"status": result["status"], "cases_checked": result["cases_checked"],
                      "necessary_directed_edges": result["necessary_directed_edges"],
                      "independent_optimum_costs": {m: r["navigation_cost"] for m, r in result["independent_global_optima"].items()}}))
