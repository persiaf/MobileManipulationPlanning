"""Independent audit of navigation-only joint attribution evidence.

Reads exports and frozen support only; never invokes IK, navigation, or an
optimizer. Solver lower bounds are checked for provenance/arithmetic against
their saved MILP traces, not independently re-solved. Polygon/SAT primitives
are reused from the previous independent audit, not production geometry.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path

from audit_task_navigation_robustness import polygon, separated

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "results/task_navigation_robustness_20261111"
OUT = ROOT / "results/joint_attribution_20261113"
MODES = ("position_only", "top_down_grasp", "table_wipe")
METHODS = ("fixed_optimum", "legacy_strong_init", "balanced_raw_init", "balanced_strong_init")
CASES = tuple(f"L{l}_D{d}" for l in range(3) for d in range(3))
TOL = 1e-7


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def truth(value):
    return str(value).lower() in {"1", "true"}


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose(row):
    return tuple(float(row[k]) for k in ("x_m", "y_m", "yaw_rad"))


def same_pose(a, b):
    return math.dist(a[:2], b[:2]) < TOL and abs(wrap(a[2]-b[2])) < TOL


def same_cost(a, b):
    return (math.isinf(a) and math.isinf(b)) or abs(a-b) < TOL


def listify(value):
    return [value] if isinstance(value, dict) else value


def cost(route, edges):
    return sum(edges[a, b] for a, b in zip(route, route[1:]+route[:1]))


def load_edges(table, target):
    for row in table:
        key = int(row["from_node"]), int(row["to_node"])
        value = float(row["navigation_cost"])
        assert value >= 0 and not math.isnan(value)
        if key in target:
            assert same_cost(value, target[key]), f"Changed directed edge {key}"
        target[key] = value


def minimum_count(tids, candidate_ids, support):
    """Independent finite-set DP, preserving the original support labels."""
    bits = {tid: 1 << i for i, tid in enumerate(tids)}
    masks = [sum(bits[t] for t in tids if truth(support[c, t]["feasible"])) for c in candidate_ids]
    dp = [math.inf] * (1 << len(tids)); dp[0] = 0
    for state in range(len(dp)):
        for mask in masks:
            dp[state | mask] = min(dp[state | mask], dp[state]+1)
    assert math.isfinite(dp[-1])
    return int(dp[-1])


def check_certificate(cert, graph_rows, expected_nodes):
    graph = {}; load_edges(graph_rows, graph)
    expected_nodes = set(expected_nodes); n = len(expected_nodes)
    assert len(graph) == n*n and set(graph) == {(a, b) for a in expected_nodes for b in expected_nodes}
    assert set(cert["nodes"]) == expected_nodes and len(cert["nodes"]) == n
    route = cert["route_nodes"]
    assert route[0] == 1 and len(route) == n and set(route) == expected_nodes
    assert [cert["nodes"][i] for i in cert["route_local_zero_based"]] == route
    upper = cost(route, graph); lower = cert["lower_bound_raw"]
    assert math.isfinite(upper) and same_cost(upper, cert["upper_bound"])
    assert 0 <= lower <= upper+TOL
    assert same_cost(cert["absolute_gap"], max(0, upper-lower))
    assert cert["certified_optimal"] == (upper-lower <= TOL)
    guard = 1e-6 + 1e-9*max(1, abs(lower))
    assert abs(cert["lower_bound_conservative"]-max(0, lower-guard)) < 1e-12
    trace = cert["trace"]
    assert trace and cert["solver"] == "scipy.optimize.milp/HiGHS, binary assignment + DFJ cuts"
    prev_lower, prev_upper, elapsed = 0., math.inf, 0.
    for step in trace:
        assert step["lower_bound"] >= prev_lower-TOL
        assert step["upper_bound"] <= prev_upper+TOL
        assert step["lower_bound"] <= step["upper_bound"]+TOL
        assert step["elapsed_s"] >= elapsed
        if step["cycle_sizes"]:
            assert sum(step["cycle_sizes"]) == n and min(step["cycle_sizes"]) >= 2
        prev_lower, prev_upper, elapsed = step["lower_bound"], step["upper_bound"], step["elapsed_s"]
    assert same_cost(prev_lower, lower) and same_cost(prev_upper, upper)
    return {"upper_bound_recomputed": upper, "lower_bound_conservative": cert["lower_bound_conservative"],
            "absolute_gap": cert["absolute_gap"], "certified_optimal": cert["certified_optimal"]}


def audit_case(case_id):
    src, folder = SOURCE / "cases" / case_id, OUT / "cases" / case_id
    assert read(folder / "run_state.json")["status"] == "COMPLETE"
    summary = rows(folder / "summary.csv")
    expected = {(mode, method) for mode in MODES for method in METHODS}
    assert len(summary) == 12 and {(r["mode"], r["method"]) for r in summary} == expected
    lookup = {(r["mode"], r["method"]): r for r in summary}
    meta = read(src / "metadata.json"); geometry = meta["source_scene_config"]
    footprint = meta["robot_footprint"]
    tables = [polygon(*xy, yaw, geometry["table"]["length"], geometry["table"]["width"])
              for xy, yaw in zip(geometry["table_positions"], geometry["table_orientations"])]
    border = geometry["border_thickness_m"]; safe_checked = set()

    def safe(p):
        if p in safe_checked:
            return
        box = polygon(*p, footprint["length_m"], footprint["width_m"])
        assert all(border-TOL <= x <= geometry["map_width_m"]-border+TOL
                   and border-TOL <= y <= geometry["map_height_m"]-border+TOL for x, y in box), (case_id, p)
        assert all(separated(box, table) for table in tables), (case_id, p)
        safe_checked.add(p)

    candidates = {int(r["candidate_id"]): r for r in rows(src / "candidate_pool.csv")}
    tasks = rows(src / "tasks.csv"); tids_by_table = {}
    for task in tasks:
        tids_by_table.setdefault(int(task["table_id"]), []).append(int(task["task_id"]))
    docks, paths, assignments = (rows(folder / filename) for filename in
                                ("docking_poses.csv", "navigation_paths.csv", "task_assignments.csv"))
    old_docks = rows(src / "docking_poses.csv")
    events = listify(read(folder / "search_events.json"))
    event_by_key = {(r["mode"], r["method"]): r["search"] for r in events}
    assert len(events) == 12 and set(event_by_key) == expected
    edges = {}; load_edges(rows(src / "navigation_edges.csv"), edges)
    load_edges(rows(folder / "new_navigation_edges.csv"), edges)
    for i in range(1, len(candidates)+2):
        edges[i, i] = 0.
    for mode in MODES:
        load_edges(rows(src / f"seed_navigation_costs_{mode}_staged.csv"), edges)
    details, certs = {}, {}; assignments_checked = moves_checked = paths_checked = 0
    paired = {(r["case_id"], r["mode"]): r for r in rows(OUT / "paired_attribution.csv")}
    old_summary = {(r["mode"], r["method"]): r for r in rows(src / "summary.csv")}

    for mode in MODES:
        support = {(int(r["candidate_id"]), int(r["task_id"])): r for r in rows(src / f"pair_support_{mode}.csv")}
        minima = {}
        for t, tids in tids_by_table.items():
            ids = [c for c in candidates if int(candidates[c]["table_id"]) == t]
            minima[t] = minimum_count(tids, ids, support)
        saved_minima = [r for r in rows(src / "minimum_cover_certificates.csv") if r["mode"] == mode]
        assert {int(r["region_id"]): int(r["minimum_docks"]) for r in saved_minima} == minima

        def supported(route):
            assert route[0] == 1 and len(route) == len(set(route)) == sum(minima.values())+1
            selected = [node-1 for node in route[1:]]
            assert Counter(int(candidates[c]["table_id"]) for c in selected) == Counter(minima)
            for t, tids in tids_by_table.items():
                group = [c for c in selected if int(candidates[c]["table_id"]) == t]
                assert all(any(truth(support[c, tid]["feasible"]) for c in group) for tid in tids)

        for method, prefix, graph_name in (("minimum_seed", "fixed", f"seed_navigation_costs_{mode}_staged.csv"),
                                           ("joint_local", "old_joint_fixed", f"old_joint_graph_{mode}.csv")):
            original = sorted((r for r in old_docks if r["mode"] == mode and r["method"] == method), key=lambda r: int(r["visit_rank"]))
            expected_nodes = [1]+[int(d["candidate_id"])+1 for d in original]
            certificate = read(folder / f"{prefix}_{mode}.json")
            graph_rows = rows((src if prefix == "fixed" else folder) / graph_name)
            certs[f"{mode}/{prefix}"] = check_certificate(certificate, graph_rows, expected_nodes)

        for method in METHODS:
            key = mode, method; row = lookup[key]; search = event_by_key[key]
            assert truth(row["route_valid"]) and int(row["covered_tasks"]) == 36
            assert int(row["docks"]) == int(row["minimum_docks"]) == sum(minima.values())
            assert row["coverage_source"] == "frozen_task_support_replay_no_new_IK"
            assert 0 <= int(row["new_directed_requests"]) <= 1500 and 0 <= int(row["neighbor_evaluations"]) <= 10000
            assert int(row["new_directed_requests"]) == search["oracle_queries"]
            assert int(row["neighbor_evaluations"]) == search["neighbor_evaluations"]
            ds = sorted((d for d in docks if (d["mode"], d["method"]) == key), key=lambda d: int(d["visit_rank"]))
            ps = sorted((p for p in paths if (p["mode"], p["method"]) == key), key=lambda p: int(p["point_id"]))
            aa = [a for a in assignments if (a["mode"], a["method"]) == key]
            assert [int(d["visit_rank"]) for d in ds] == list(range(1, len(ds)+1))
            route = [1]+[int(d["candidate_id"])+1 for d in ds]; supported(route)
            assert route == search["matrix_visit_order"]
            assert sorted(int(a["task_id"]) for a in aa) == list(range(1, 37))
            for assignment in aa:
                candidate, task = int(assignment["candidate_id"]), int(assignment["task_id"])
                dock = next(d for d in ds if d["dock_id"] == assignment["dock_id"])
                assert int(dock["candidate_id"]) == candidate
                assert truth(support[candidate, task]["feasible"]) and assignment["arm"] == support[candidate, task]["assigned_arm"]
            for dock in ds:
                assert same_pose(pose(dock), pose(candidates[int(dock["candidate_id"])]))
                assert set(dock["covered_task_ids"].split("|")) == {a["task_id"] for a in aa if a["dock_id"] == dock["dock_id"]}
            assert len(ps) > 1 and [int(p["point_id"]) for p in ps] == list(range(1, len(ps)+1))
            points = [pose(p) for p in ps]
            assert same_pose(points[0], geometry["start_pose"]) and same_pose(points[-1], geometry["start_pose"])
            cursor = 0
            for dock in ds:
                hit = next((i for i in range(cursor, len(points)) if same_pose(points[i], pose(dock))), None)
                assert hit is not None, (case_id, key, dock["candidate_id"])
                cursor = hit
            length = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
            endpoint = [tuple(geometry["start_pose"])]+[pose(d) for d in ds]+[tuple(geometry["start_pose"])]
            yaw = sum(abs(wrap(b[2]-a[2])) for a, b in zip(endpoint, endpoint[1:]))
            actual = length+.15*yaw
            assert abs(length-float(row["route_length_m"])) < TOL
            assert abs(actual-float(row["navigation_cost"])) < TOL and abs(cost(route, edges)-actual) < TOL
            for point in points:
                safe(point)
            initial = search["minimum_cover_seed_matrix_visit_order"]; supported(initial)
            raw = sorted((d for d in old_docks if d["mode"] == mode and d["method"] == "minimum_seed"), key=lambda d: int(d["visit_rank"]))
            expected_init = [1]+[int(d["candidate_id"])+1 for d in raw] if method == "balanced_raw_init" else read(folder / f"fixed_{mode}.json")["route_nodes"]
            assert initial == expected_init
            moves = listify(search["accepted_moves"])
            assert len(moves) == int(row["accepted_moves"])
            previous, previous_cost = initial, cost(initial, edges)
            assert abs(previous_cost-search["minimum_cover_seed_cost"]) < TOL
            for event in moves:
                before, after = event["previous_matrix_visit_order"], event["matrix_visit_order"]
                supported(before); supported(after)
                assert before == previous and abs(previous_cost-event["previous_cost"]) < TOL
                assert abs(cost(before, edges)-event["previous_cost"]) < TOL
                assert abs(cost(after, edges)-event["new_cost"]) < TOL
                assert event["new_cost"] < event["previous_cost"]-1e-9
                if event["operation"] in {"directed_two_opt", "final_directed_order_descent"}:
                    assert set(before) == set(after)
                else:
                    assert event["operation"] in {"regional_minimum_cover_replacement", "balanced_regional_replacement"}
                    region = int(event["region_id"])
                    assert all(a == b or (a > 1 and b > 1 and int(candidates[a-1]["table_id"]) == int(candidates[b-1]["table_id"]) == region)
                               for a, b in zip(before, after))
                previous, previous_cost = after, event["new_cost"]
            assert previous == route and abs(previous_cost-actual) < TOL
            if truth(row["fixed_set_order_converged"]):
                assert search["fixed_set_order_converged"]
                for i in range(1, len(route)-1):
                    for j in range(i+1, len(route)):
                        trial = route[:i]+list(reversed(route[i:j+1]))+route[j+1:]
                        assert cost(trial, edges) >= actual-1e-8
                for i in range(1, len(route)):
                    reduced = route[:i]+route[i+1:]
                    for j in range(1, len(reduced)+1):
                        trial = reduced[:j]+[route[i]]+reduced[j:]
                        assert cost(trial, edges) >= actual-1e-8
            details[f"{mode}/{method}"] = {"J": actual, "L": length, "docks": len(ds), "tasks": len(aa),
                "path_samples": len(points), "strict_moves": len(moves), "route": route,
                "termination_reason": row["termination_reason"]}
            assignments_checked += len(aa); paths_checked += len(points); moves_checked += len(moves)

        pair = paired[case_id, mode]
        F, I, N, J = (details[f"{mode}/{method}"]["J"] for method in METHODS)
        S, C = (float(old_summary[mode, method]["navigation_cost"]) for method in ("minimum_seed", "joint_local"))
        fc, oc = certs[f"{mode}/fixed"], certs[f"{mode}/old_joint_fixed"]
        assert abs(F-fc["upper_bound_recomputed"]) < TOL and J <= F+TOL and I <= F+TOL
        values = {"staged_J": S, "old_joint_J": C, "fixed_optimum_J": F, "legacy_strong_J": I,
                  "balanced_raw_J": N, "new_joint_J": J, "order_gain": S-F, "docking_gain": F-J,
                  "allocation_gain": I-J, "initialization_gain": N-J,
                  "original_fixed_LB": fc["lower_bound_conservative"], "original_fixed_gap": fc["absolute_gap"],
                  "old_joint_fixed_J": oc["upper_bound_recomputed"], "old_joint_fixed_LB": oc["lower_bound_conservative"],
                  "old_joint_fixed_gap": oc["absolute_gap"], "gain_vs_fixed_pct": 100*(F-J)/F,
                  "gain_vs_old_joint_pct": 100*(C-J)/C,
                  "gain_vs_old_joint_fixed_pct": 100*(oc["upper_bound_recomputed"]-J)/oc["upper_bound_recomputed"]}
        for key, value in values.items():
            assert abs(float(pair[key])-value) < TOL, (case_id, mode, key)
        assert truth(pair["certified_vs_original_set"]) == (J < fc["lower_bound_conservative"]-TOL)
        assert truth(pair["certified_vs_old_joint_set"]) == (J < oc["lower_bound_conservative"]-TOL)
        original_set = set(details[f"{mode}/fixed_optimum"]["route"])
        new_set = set(details[f"{mode}/balanced_strong_init"]["route"])
        assert int(pair["changed_docks"]) == len(new_set-original_set)
        if truth(pair["certified_vs_original_set"]):
            assert new_set != original_set
    return {"case_id": case_id, "summary": summary, "details": details, "certificates": certs,
            "assignments_checked": assignments_checked, "moves_checked": moves_checked,
            "path_samples_checked": paths_checked, "unique_pose_geometry_checked": len(safe_checked)}


def main():
    freeze = read(OUT / "frozen_inputs.json")
    for name, digest in freeze["sha256"].items():
        assert sha(ROOT / name) == digest, f"Frozen evidence changed: {name}"
    combined = rows(OUT / "summary.csv"); pairs = rows(OUT / "paired_attribution.csv")
    assert len(combined) == 108 and len(pairs) == 27
    assert {(r["case_id"], r["mode"]) for r in pairs} == {(c, m) for c in CASES for m in MODES}
    cases = []
    for case_id in CASES:
        case = audit_case(case_id); cases.append(case)
        assert [r for r in combined if r["case_id"] == case_id] == case["summary"]
        print(f"AUDIT {case_id}: {case['path_samples_checked']} path samples, {case['moves_checked']} strict moves", flush=True)
    aggregate = {r["mode"]: r for r in rows(OUT / "aggregate_statistics.csv")}
    for mode in MODES:
        group = [r for r in pairs if r["mode"] == mode]; saved = aggregate[mode]
        assert int(saved["cases"]) == len(group) == 9
        for key, field in (("certified_vs_original", "certified_vs_original_set"), ("certified_vs_old_joint", "certified_vs_old_joint_set")):
            assert int(saved[key]) == sum(truth(r[field]) for r in group)
        for name in ("gain_vs_fixed", "gain_vs_old_joint", "gain_vs_old_joint_fixed"):
            assert abs(float(saved[name+"_median_pct"])-statistics.median(float(r[name+"_pct"]) for r in group)) < TOL
        for name, field in (("allocation", "allocation_gain"), ("init", "initialization_gain")):
            values = [float(r[field]) for r in group]
            assert int(saved[name+"_wins"]) == sum(v > TOL for v in values)
            assert int(saved[name+"_ties"]) == sum(abs(v) <= TOL for v in values)
            assert int(saved[name+"_losses"]) == sum(v < -TOL for v in values)
    result = {"status": "PASS", "frozen_files_unchanged": len(freeze["sha256"]), "cases": len(cases),
        "methods_checked": 108, "paired_claims_checked": 27, "fixed_set_certificates_checked": 54,
        "assignments_checked": sum(c["assignments_checked"] for c in cases),
        "strict_moves_checked": sum(c["moves_checked"] for c in cases),
        "path_samples_checked": sum(c["path_samples_checked"] for c in cases),
        "unique_pose_geometry_checked": sum(c["unique_pose_geometry_checked"] for c in cases),
        "certified_vs_original_set": sum(truth(r["certified_vs_original_set"]) for r in pairs),
        "certified_vs_old_joint_set": sum(truth(r["certified_vs_old_joint_set"]) for r in pairs),
        "scope": "Frozen support/min-cardinality replay, new sampled base geometry, path pricing, strict moves, and attribution arithmetic; no IK/navigation/optimization rerun.",
        "lower_bound_audit_scope": "Saved solver-bound trace, graph and incumbent provenance/arithmetic; not an independent MILP proof checker.",
        "auditor_sha256": sha(Path(__file__)),
        "case_details": [{k: v for k, v in c.items() if k != "summary"} for c in cases]}
    (OUT / "independent_audit.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "case_details"}, indent=2), flush=True)


if __name__ == "__main__":
    main()
