"""Independent, failure-inclusive audit of the task/navigation robustness grid.

This script reads exported evidence only. It does not import the production
selector, IK, navigation, or scene factory. Its geometry checks cover exported
base samples and task strokes, not continuous manipulation or physical success.
"""
from __future__ import annotations

import argparse
import csv
import functools
import hashlib
import itertools
import json
import math
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODES = ("position_only", "top_down_grasp", "table_wipe")
METHODS = ("staged", "minimum_seed", "reorder_only", "joint_local")
PRIMARY = ("minimum_seed", "reorder_only", "joint_local")
TOL = 1e-7


def rows(path, optional=False):
    if optional and not path.exists():
        return []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def truth(value):
    return str(value).lower() in {"true", "1"}


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose(row):
    return tuple(float(row[k]) for k in ("x_m", "y_m", "yaw_rad"))


def nodes(value):
    return tuple(int(x) for x in value.split("|")) if value else ()


def equal_cost(a, b):
    return (math.isinf(a) and math.isinf(b)) or abs(a-b) < TOL


def polygon(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in
            ((-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2))]


def separated(a, b):
    """Independent separating-axis test for two convex rectangles."""
    if max(p[0] for p in a) < min(p[0] for p in b) or max(p[0] for p in b) < min(p[0] for p in a) or \
            max(p[1] for p in a) < min(p[1] for p in b) or max(p[1] for p in b) < min(p[1] for p in a):
        return True
    for shape in (a, b):
        for i in range(len(shape)):
            dx = shape[(i+1) % len(shape)][0]-shape[i][0]
            dy = shape[(i+1) % len(shape)][1]-shape[i][1]
            aa = [-x*dy+y*dx for x, y in a]
            bb = [-x*dy+y*dx for x, y in b]
            if max(aa) < min(bb)-1e-10 or max(bb) < min(aa)-1e-10:
                return True
    return False


def table_local(x, y, center, angle):
    dx, dy = x-center[0], y-center[1]
    c, s = math.cos(angle), math.sin(angle)
    return c*dx+s*dy, -s*dx+c*dy


def check_fps(all_xy, selected_xy, seed):
    """Accept any valid floating-point tie order, without calling MATLAB."""
    all_xy, chosen = tuple(sorted(all_xy)), tuple(sorted(selected_xy))
    assert len(chosen) == min(8, len(all_xy))
    first_distance = min(math.dist(p, seed) for p in all_xy)

    @functools.lru_cache(None)
    def extend(mask):
        if mask == (1 << len(chosen))-1:
            return True
        selected = [chosen[i] for i in range(len(chosen)) if mask & (1 << i)]
        if not selected:
            eligible = [i for i, p in enumerate(chosen) if math.dist(p, seed) <= first_distance+1e-9]
        else:
            distances = {p: min(math.dist(p, q) for q in selected) for p in all_xy if p not in selected}
            farthest = max(distances.values())
            eligible = [i for i, p in enumerate(chosen) if not mask & (1 << i)
                        and distances[p] >= farthest-1e-9]
        return any(extend(mask | (1 << i)) for i in eligible)

    assert extend(0), "Geometry-only farthest-point selection cannot be reproduced"


def check_strokes(tasks, centers, angles, length, width, height):
    """Check the experiment's unchanged world-X 12 cm wipe stroke envelope."""
    for task in tasks:
        t = int(task["table_id"])-1
        x, y, z = (float(task[k]) for k in ("x_m", "y_m", "z_m"))
        assert abs(z-height) < TOL
        # The rectangle is convex, so checking both stroke endpoints also
        # checks every interpolated point. Include the anchor explicitly.
        for offset in (-.06, 0, .06):
            a, b = table_local(x+offset, y, centers[t], angles[t])
            assert abs(a) <= length/2+TOL and abs(b) <= width/2+TOL, \
                "A wipe stroke leaves its finite rotated tabletop"


def check_effective_samples(folder, tasks, centers, angles, length, width, height, required):
    anchors = {int(task["task_id"]): task for task in tasks}
    checked = {}
    for mode in MODES:
        path = folder / f"effective_tasks_{mode}.csv"
        if not path.exists():
            assert not required, f"Missing resolved ordered task samples: {path.name}"
            continue
        samples = rows(path)
        assert {int(row["task_id"]) for row in samples} == set(anchors)
        count = 0
        for tid, task in anchors.items():
            group = sorted((row for row in samples if int(row["task_id"]) == tid), key=lambda r: int(r["sample_id"]))
            assert [int(row["sample_id"]) for row in group] == list(range(1, len(group)+1))
            points = [tuple(float(row[k]) for k in ("x_m", "y_m", "z_m")) for row in group]
            anchor = tuple(float(task[k]) for k in ("x_m", "y_m", "z_m"))
            assert all(all(math.isfinite(v) for v in point) for point in points)
            if mode == "position_only":
                assert len(points) == 1 and math.dist(points[0], anchor) < TOL
            elif mode == "top_down_grasp":
                assert all(math.dist(point[:2], anchor[:2]) < TOL and height-TOL <= point[2] <= height+.04+TOL for point in points)
                assert abs(points[0][2]-height-.04) < TOL and abs(points[-1][2]-height-.04) < TOL
                assert min(math.dist(point, anchor) for point in points) < TOL
                assert all(math.dist(a, b) <= .02+TOL for a, b in zip(points, points[1:]))
            else:
                assert math.dist(points[0], (anchor[0]-.06, anchor[1], height)) < TOL
                assert math.dist(points[-1], (anchor[0]+.06, anchor[1], height)) < TOL
                assert all(abs(point[1]-anchor[1]) < TOL and abs(point[2]-height) < TOL for point in points)
                assert all(-TOL <= b[0]-a[0] <= .02+TOL for a, b in zip(points, points[1:]))
                t = int(task["table_id"])-1
                for point in points:
                    local = table_local(*point[:2], centers[t], angles[t])
                    assert abs(local[0]) <= length/2+TOL and abs(local[1]) <= width/2+TOL
            count += len(points)
        checked[mode] = count
    return checked


def certify_region(tids, candidate_ids, lookup):
    bits = {tid: 1 << i for i, tid in enumerate(tids)}
    masks = {cid: sum(bits[tid] for tid in tids if truth(lookup[cid, tid]["feasible"]))
             for cid in candidate_ids}
    dp = [math.inf]*(1 << len(tids)); dp[0] = 0
    for state in range(len(dp)):
        for mask in masks.values():
            dp[state | mask] = min(dp[state | mask], dp[state]+1)
    feasible = math.isfinite(dp[-1]); k = int(dp[-1]) if feasible else None
    count = 0
    if feasible:
        for choice in itertools.combinations([cid for cid in masks if masks[cid]], k):
            total = 0
            for cid in choice:
                total |= masks[cid]
            count += total == len(dp)-1
    return dict(feasible=feasible, minimum_docks=k, cover_count=count,
                masks=masks, full_mask=len(dp)-1)


def audit_case(folder, planned, allow_partial=False, geometry_only=False):
    meta = read_json(folder / "metadata.json")
    state = read_json(folder / "run_state.json") if (folder / "run_state.json").exists() else {}
    if not allow_partial and not geometry_only:
        assert state["status"] == "COMPLETE", "Scenario is not complete"
    scene, source = meta["scene"], meta["source_scene_config"]
    definition = meta.get("case_definition", {})
    case_id = planned["case_id"]
    if "case_id" in meta:
        assert meta["case_id"] == case_id
    if definition:
        assert definition.get("case_id", case_id) == case_id
        assert int(definition["layout_id"]) == int(planned["layout_id"])
        assert int(definition["distribution_id"]) == int(planned["distribution_id"])
        if "definition" in planned:
            assert definition == planned["definition"], "Per-case definition differs from the frozen factorial"
        assert source == definition["scene_cfg"]
        assert meta["reference_scene_config"] == definition["reference_scene_cfg"]
    centers = source["table_positions"]
    angles = source["table_orientations"]
    nt = int(source["num_tables"]); n = int(meta["task_count"])
    assert nt == meta["table_count"] == len(centers) == len(angles) == 16
    assert n == 36
    assert scene["table_centers"] == centers
    assert scene["start_pose"] == source["start_pose"] == [.5, .5, 0]
    assert scene["map_width_m"] == source["map_width_m"] == 10
    assert scene["map_height_m"] == source["map_height_m"] == 10
    length = scene["table_length_m"]; width = scene["table_width_m"]; height = scene["table_height_m"]
    assert length == source["table"]["length"] == 1.2
    assert width == source["table"]["width"] == .4
    assert height == source["table"]["height"] == .5
    assert meta["model_revision"].startswith("vx300s-urdf-0bb2b0e6")
    if not geometry_only:
        assert not meta["arm_collision_checked"] and not meta["hardware_calibrated"]
        assert meta["navigation_matrix_directed"] and meta["finite_nn_restarts"]
        assert meta["routing"]["directed_costs"] and meta["routing"]["finite_nn_restarts"]
        assert not meta["global_optimal"]
    fp = meta["robot_footprint"]
    tables = [polygon(*center, angle, length, width) for center, angle in zip(centers, angles)]
    border = source["border_thickness_m"]

    def safe(p):
        box = polygon(*p, fp["length_m"], fp["width_m"])
        assert all(border-TOL <= x <= source["map_width_m"]-border+TOL
                   and border-TOL <= y <= source["map_height_m"]-border+TOL for x, y in box)
        assert all(separated(box, table) for table in tables)

    tasks = rows(folder / "tasks.csv"); pool = rows(folder / "candidate_pool.csv")
    dense = rows(folder / "dense_to_subpool.csv"); counts = rows(folder / "table_candidate_counts.csv")
    assert len(tasks) == n and {int(r["task_id"]) for r in tasks} == set(range(1, n+1))
    table_tasks = {t: [r for r in tasks if int(r["table_id"]) == t] for t in range(1, nt+1)}
    assert all(2 <= len(group) <= 3 for group in table_tasks.values())
    for task in tasks:
        t = int(task["table_id"])-1
        local = table_local(float(task["x_m"]), float(task["y_m"]), centers[t], angles[t])
        assert abs(local[0]) <= length/2+TOL and abs(local[1]) <= width/2+TOL
        if definition.get("task_local_xy"):
            assert math.dist(local, definition["task_local_xy"][int(task["task_id"])-1]) < TOL
        if definition.get("task_table_ids"):
            assert int(task["table_id"]) == int(definition["task_table_ids"][int(task["task_id"])-1])
    check_strokes(tasks, centers, angles, length, width, height)
    effective_samples = check_effective_samples(folder, tasks, centers, angles, length, width, height,
                                                bool(definition) and not geometry_only)
    assert len(dense) == meta.get("dense_candidate_count", len(dense)) and len(pool) == meta["candidate_count"]
    valid_dense_count = sum(truth(r["collision_valid"]) for r in dense)
    assert valid_dense_count == meta.get("footprint_valid_dense_count", valid_dense_count)
    dense_id = {int(r["dense_candidate_id"]): r for r in dense}
    pool_by_id = {int(r["candidate_id"]): r for r in pool}
    assert len(dense_id) == len(dense) and len(pool_by_id) == len(pool)
    assert set(pool_by_id) == set(range(1, len(pool)+1))
    assert sum(truth(r["selected"]) for r in dense) == len(pool)
    for cid, candidate in pool_by_id.items():
        original = dense_id[int(candidate["dense_candidate_id"])]
        assert int(candidate["node_index"]) == cid+1 and int(original["candidate_id"]) == cid
        assert truth(original["selected"]) and truth(original["collision_valid"])
        assert pose(candidate) == pose(original)
        assert candidate["table_id"] == original["table_id"] and candidate["side_id"] == original["side_id"]
        safe(pose(candidate))
    for count in counts:
        t = int(count["table_id"]); center, angle = centers[t-1], angles[t-1]
        dr = [r for r in dense if int(r["table_id"]) == t]
        pr = [r for r in pool if int(r["table_id"]) == t]
        assert len(dr) == int(count["dense_candidates"])
        assert sum(truth(r["collision_valid"]) for r in dr) == int(count["footprint_valid_candidates"])
        assert len(pr) == int(count["retained_candidates"]) <= 64
        assert len(table_tasks[t]) == int(count["task_count"])
        for side in range(1, 5):
            valid = [r for r in dr if int(r["side_id"]) == side and truth(r["collision_valid"])]
            retained = [r for r in pr if int(r["side_id"]) == side]
            if not valid:
                assert not retained
                continue
            def xy(row):
                return tuple(round(v, 10) for v in table_local(float(row["x_m"]), float(row["y_m"]), center, angle))
            groups = {}
            for row in valid:
                groups.setdefault(xy(row), []).append(row)
            selected_xy = {xy(row) for row in retained}
            seed = ((0, width/2+.3), (0, -width/2-.3), (-length/2-.3, 0), (length/2+.3, 0))[side-1]
            normal = angle+(-math.pi/2, math.pi/2, 0, math.pi)[side-1]
            check_fps(groups, selected_xy, seed)
            for location in selected_xy:
                available = groups[location]; chosen = [r for r in retained if xy(r) == location]
                assert len(chosen) == min(2, len(available))
                def legal(order):
                    remaining = list(available)
                    for row, target in zip(order, (normal, normal+math.pi/12)):
                        error = abs(wrap(float(row["yaw_rad"])-target))
                        assert remaining
                        if error > min(abs(wrap(float(a["yaw_rad"])-target)) for a in remaining)+1e-9:
                            return False
                        remaining = [a for a in remaining if a["dense_candidate_id"] != row["dense_candidate_id"]]
                    return True
                assert any(legal(order) for order in itertools.permutations(chosen))

    if geometry_only:
        return dict(status="PASS_INPUT_GEOMETRY", case_id=case_id,
                    robot_signature=meta["robot_signature"], robot_footprint=fp,
                    immutable_pool_sha256={name: sha256(folder/name) for name in
                        ("candidate_pool.csv", "dense_to_subpool.csv", "table_candidate_counts.csv")},
                    scene_geometry=dict(tables=nt, tasks=n, candidate_count=len(pool), dense_candidates=len(dense)),
                    per_table_task_counts=[len(table_tasks[t]) for t in range(1, nt+1)],
                    finite_tabletop_wipe_strokes_checked=n, rotated_candidate_geometry_checked=len(pool),
                    frozen_case_definition_checked=bool(definition),
                    note="Input/preflight checks only: no IK, navigation outcome or manipulation-success conclusion")

    cert_rows = rows(folder / "minimum_cover_certificates.csv", True)
    manifest = read_json(folder / "main_library_geometry_signature.json")
    completed, lookup, minima, proofs, build_proofs = [], {}, {}, {}, {}
    for mode in MODES:
        proof_path = folder / f"ik_cache_{mode}.json"
        if not proof_path.exists():
            assert allow_partial
            continue
        proof = read_json(proof_path); pairs = rows(folder / f"pair_support_{mode}.csv")
        scored = rows(folder / f"candidate_support_{mode}.csv")
        expected = {(cid, int(task["task_id"])) for cid, candidate in pool_by_id.items()
                    for task in table_tasks[int(candidate["table_id"])]}
        look = {(int(row["candidate_id"]), int(row["task_id"])): row for row in pairs}
        assert set(look) == expected and len(look) == len(pairs) == proof["pair_count"]
        assert proof["geometry_directory_signature"] == manifest["signature"]
        assert proof["scorer_sources"] == manifest["scorer_sources"]
        assert not proof["independent_ik_validation"] and not proof["arm_collision_checked"]
        assert proof["request_count"] == len(pairs)
        assert proof["positive_hits"]+proof["exact_negative_hits"]+proof["misses"] == len(pairs)
        for field, kind in (("positive_hits", "positive_hit"), ("exact_negative_hits", "exact_negative_hit")):
            assert proof[field] == sum(row["cache_kind"] == kind for row in pairs)
        assert proof["witness_fallbacks"] == sum(row["cache_kind"] == "witness_fallback" for row in pairs)
        build_path = folder / f"ik_build_proof_{mode}.json"
        if build_path.exists():
            build = read_json(build_path)
            for field in ("signature", "scene_signature", "geometry_directory_signature", "scorer_sources", "pair_count",
                          "request_count", "positive_hits", "exact_negative_hits", "misses", "witness_fallbacks",
                          "direct_checks", "direct_disagreements", "arm_direct_checks", "witness_direct_search_failures"):
                assert build[field] == proof[field], f"Build/replay evidence changed for {mode}/{field}"
            if build["reused_from_disk"]:
                assert build["cache_scope"] == "signed_library_replay_without_new_witness_validation"
            else:
                assert build["cache_counters_reset_per_library"]
                assert build["cache_scope"] == "process_local_witness_buckets_with_per_library_counters"
            build_proofs[mode] = build
        support_counts = [sum(truth(look[cid, tid]["feasible"]) for cid, t in expected if t == tid)
                          for tid in range(1, n+1)]
        assert support_counts == proof["per_task_support_counts"]
        assert sum(v > 0 for v in support_counts) == proof["union_supported_count"]
        assert sum(truth(row["feasible"]) for row in pairs) == proof["supported_pairs"]
        angle_limit = math.radians(5 if mode == "table_wipe" else 10)
        assert proof["max_position_error_m"] <= .003+1e-10
        assert proof["max_orientation_error_rad"] <= angle_limit+1e-10
        for row in pairs:
            if truth(row["feasible"]):
                assert row["assigned_arm"] in {"left", "right"}
                assert float(row["max_position_error_m"]) <= .003+1e-10
                assert float(row["max_orientation_error_rad"]) <= angle_limit+1e-10
                assert float(row["max_joint_step_rad"]) <= math.radians(25)+1e-10
        assert len(scored) == len(pool)
        for row in scored:
            candidate = pool_by_id[int(row["candidate_id"])]
            assert pose(row) == pose(candidate)
            assert int(row["support_count"]) == sum(truth(look[int(row["candidate_id"]), int(t["task_id"])]["feasible"])
                                                    for t in table_tasks[int(candidate["table_id"])])
        regional = {}
        for t, group in table_tasks.items():
            ids = [cid for cid, candidate in pool_by_id.items() if int(candidate["table_id"]) == t]
            cert = certify_region([int(task["task_id"]) for task in group], ids, look)
            exported = [row for row in cert_rows if row["mode"] == mode and int(row["region_id"]) == t]
            assert len(exported) == 1
            row = exported[0]
            assert truth(row["feasible"]) == cert["feasible"] and int(row["cover_count"]) == cert["cover_count"]
            if cert["feasible"]:
                assert int(float(row["minimum_docks"])) == cert["minimum_docks"]
            else:
                assert math.isinf(float(row["minimum_docks"]))
            regional[t] = cert
        completed.append(mode); lookup[mode] = look; minima[mode] = regional; proofs[mode] = proof

    summary = rows(folder / "summary.csv"); docks = rows(folder / "docking_poses.csv", True)
    assignments = rows(folder / "task_assignments.csv", True); paths = rows(folder / "navigation_paths.csv", True)
    edges = rows(folder / "navigation_edges.csv", True); moves = rows(folder / "accepted_moves.csv", True)
    costs = {}; node_pose = {cid+1: pose(row) for cid, row in pool_by_id.items()}
    node_pose[1] = tuple(scene["start_pose"]); lam = float(meta["lambda_theta_m_per_rad"])
    assert lam == .15
    for row in edges:
        a, b = int(row["from_node"]), int(row["to_node"])
        assert (a, b) not in costs
        assert int(row["from_candidate_id"]) == a-1 and int(row["to_candidate_id"]) == b-1
        yaw = abs(wrap(node_pose[b][2]-node_pose[a][2]))
        assert abs(yaw-float(row["endpoint_yaw_change_rad"])) < TOL
        length, value = float(row["path_length_m"]), float(row["navigation_cost"])
        if truth(row["path_found"]):
            assert math.isfinite(length) and abs(value-length-lam*yaw) < TOL
            assert length+TOL >= math.dist(node_pose[a][:2], node_pose[b][:2])
        else:
            assert math.isinf(value)
        costs[a, b] = value

    def route_cost(route):
        return sum(costs[a, b] for a, b in zip(route, route[1:]+(1,)))

    def supported_route(mode, route):
        assert route[0] == 1 and len(set(route)) == len(route)
        for t, cert in minima[mode].items():
            assert cert["feasible"]
            ids = [node-1 for node in route[1:] if int(pool_by_id[node-1]["table_id"]) == t]
            assert len(ids) == cert["minimum_docks"]
            mask = 0
            for cid in ids:
                mask |= cert["masks"][cid]
            assert mask == cert["full_mask"]

    keys = {(row["mode"], row["method"]) for row in summary}
    assert len(keys) == len(summary)
    if not allow_partial:
        assert set(completed) == set(MODES)
        assert keys == set(itertools.product(MODES, METHODS)) and state["completed_cases"] == 12
    details, routes, sets, failures = {}, {}, {}, []
    for row in summary:
        mode, method = row["mode"], row["method"]
        if mode not in completed:
            assert allow_partial
            continue
        key = f"{mode}/{method}"
        match = lambda r: r["mode"] == mode and r["method"] == method
        assert not truth(row["global_optimal"])
        local = truth(row["local_optimum_certified"])
        assert not local or row["termination_reason"] == "complete_neighborhood_pass_without_improvement"
        events = sorted(filter(match, moves), key=lambda r: int(r["move_id"]))
        assert len(events) == int(row["accepted_moves"])
        previous, planned_cost = None, float(row["minimum_seed_cost"])
        for i, event in enumerate(events, 1):
            before, after = nodes(event["previous_nodes"]), nodes(event["new_nodes"])
            assert int(event["move_id"]) == i
            supported_route(mode, before); supported_route(mode, after)
            assert equal_cost(route_cost(before), float(event["previous_cost"]))
            assert equal_cost(route_cost(after), float(event["new_cost"]))
            assert float(event["new_cost"]) < float(event["previous_cost"])-1e-9
            assert previous is None or previous == before
            assert abs(float(event["previous_cost"])-planned_cost) < TOL
            if event["operation"] == "directed_two_opt":
                assert set(before) == set(after)
                changed = [j for j in range(len(before)) if before[j] != after[j]]
                lo, hi = min(changed), max(changed)
                assert lo >= 1 and before[:lo]+tuple(reversed(before[lo:hi+1]))+before[hi+1:] == after
            else:
                assert method == "joint_local" and event["operation"] == "regional_minimum_cover_replacement"
                t = int(event["region_id"])
                assert all(a == b or int(pool_by_id[a-1]["table_id"]) == int(pool_by_id[b-1]["table_id"]) == t
                           for a, b in zip(before, after))
            previous, planned_cost = after, float(event["new_cost"])
        if method in {"staged", "minimum_seed"}:
            assert not events
        if not truth(row["mission_valid"]):
            assert math.isnan(float(row["route_length_m"])) and math.isnan(float(row["navigation_cost"]))
            failure = dict(mode=mode, method=method, status=row["status"], reason=row["failure_reason"])
            failures.append(failure); details[key] = dict(mission_valid=False, accepted_moves_checked=len(events))
            continue
        assert truth(row["route_valid"]) and row["status"] == "READY"
        assert int(row["covered_tasks"]) == int(row["verified_tasks"]) == int(row["target_count"]) == n
        ds = sorted(filter(match, docks), key=lambda r: int(r["visit_rank"]))
        aa = list(filter(match, assignments)); ps = sorted(filter(match, paths), key=lambda r: int(r["point_id"]))
        assert len(ds) == int(row["docks"]) and len(aa) == n and len(ps) > 1
        assert {int(r["task_id"]) for r in aa} == set(range(1, n+1))
        assert [int(r["visit_rank"]) for r in ds] == list(range(1, len(ds)+1))
        for assignment in aa:
            pair = lookup[mode][int(assignment["candidate_id"]), int(assignment["task_id"])]
            assert truth(pair["feasible"]) and assignment["arm"] == pair["assigned_arm"]
            dock = next(d for d in ds if d["dock_id"] == assignment["dock_id"])
            assert dock["candidate_id"] == assignment["candidate_id"]
        for dock in ds:
            assert pose(dock) == pose(pool_by_id[int(dock["candidate_id"])])
            assert set(dock["covered_task_ids"].split("|")) == {a["task_id"] for a in aa if a["dock_id"] == dock["dock_id"]}
        route = (1,)+tuple(int(dock["candidate_id"])+1 for dock in ds)
        routes[mode, method] = route; sets[mode, method] = set(route[1:])
        minimum = sum(cert["minimum_docks"] for cert in minima[mode].values())
        assert int(row["minimum_docks"]) == minimum and len(ds) >= minimum
        if method != "staged":
            supported_route(mode, route)
            assert len(ds) == minimum and abs(route_cost(route)-planned_cost) < TOL
        for endpoint in (ps[0], ps[-1]):
            assert math.dist(pose(endpoint)[:2], scene["start_pose"][:2]) < TOL
            assert abs(wrap(pose(endpoint)[2]-scene["start_pose"][2])) < TOL
        assert [int(r["point_id"]) for r in ps] == list(range(1, len(ps)+1))
        length = sum(math.dist(pose(a)[:2], pose(b)[:2]) for a, b in zip(ps, ps[1:]))
        assert abs(length-float(row["route_length_m"])) < TOL
        ordered = [tuple(scene["start_pose"])]+[pose(dock) for dock in ds]+[tuple(scene["start_pose"])]
        yaw = sum(abs(wrap(b[2]-a[2])) for a, b in zip(ordered, ordered[1:]))
        cost = length+lam*yaw
        assert abs(cost-float(row["navigation_cost"])) < TOL and abs(route_cost(route)-cost) < TOL
        cursor = 0
        for dock in ds:
            p = pose(dock)
            hit = next((i for i in range(cursor, len(ps)) if math.dist(pose(ps[i])[:2], p[:2]) < TOL
                        and abs(wrap(pose(ps[i])[2]-p[2])) < TOL), None)
            assert hit is not None
            cursor = hit
        for point in ps:
            safe(pose(point))
        if method == "minimum_seed":
            assert abs(float(row["minimum_seed_cost"])-cost) < TOL
            assert abs(float(row["input_seed_cost"])-details[f"{mode}/staged"]["navigation_cost_recomputed"]) < TOL
        elif method in {"reorder_only", "joint_local"}:
            assert abs(float(row["input_seed_cost"])-details[f"{mode}/staged"]["navigation_cost_recomputed"]) < TOL
            assert abs(float(row["minimum_seed_cost"])-details[f"{mode}/minimum_seed"]["navigation_cost_recomputed"]) < TOL
            assert cost <= float(row["minimum_seed_cost"])+TOL
            if method == "reorder_only":
                assert sets[mode, method] == sets[mode, "minimum_seed"]
            assert route == (previous if events else routes[mode, "minimum_seed"])
        details[key] = dict(mission_valid=True, task_count=n, docks=len(ds), minimum_docks=minimum,
                            route_length_recomputed_m=length, navigation_cost_recomputed=cost,
                            path_samples_checked=len(ps), accepted_moves_checked=len(events),
                            termination_reason=row["termination_reason"], local_optimum_certified=local,
                            global_optimal=False)

    seed_checks = audit_seed_recovery(folder, docks, costs, routes, details)
    return dict(status="PASS" if not failures else "PASS_WITH_RECORDED_FAILURES", case_id=case_id,
                summary=summary, model_revision=meta["model_revision"], robot_signature=meta.get("robot_signature"),
                robot_footprint=fp, scorer_sources=manifest["scorer_sources"], local_search_budget=meta["local_search_budget"],
                immutable_pool_sha256={name: sha256(folder/name) for name in
                    ("candidate_pool.csv", "dense_to_subpool.csv", "table_candidate_counts.csv")},
                scene_geometry=dict(tables=nt, tasks=n, candidate_count=len(pool), dense_candidates=len(dense)),
                task_local_xy=[list(table_local(float(t["x_m"]), float(t["y_m"]), centers[int(t["table_id"])-1],
                                               angles[int(t["table_id"])-1])) for t in tasks],
                per_table_task_counts=[len(table_tasks[t]) for t in range(1, nt+1)],
                cases=details, recorded_failures=failures, common_seed_recovery_checks=seed_checks,
                minimum_cover_certificates={mode: {str(t): {k: v for k, v in cert.items() if k not in {"masks", "full_mask"}}
                    for t, cert in group.items()} for mode, group in minima.items()},
                finite_tabletop_wipe_strokes_checked=n, rotated_base_path_geometry_checked=True,
                resolved_task_samples_checked=effective_samples,
                ik_cache_diagnostics={mode: {k: proof[k] for k in
                    ("request_count", "positive_hits", "exact_negative_hits", "misses", "witness_fallbacks",
                     "direct_checks", "direct_disagreements", "witness_direct_search_failures")}
                    for mode, proof in proofs.items()},
                ik_library_build_scope={mode: {k: build[k] for k in
                    ("reused_from_disk", "cache_scope", "warm_bucket_count", "cache_counters_reset_per_library",
                     "direct_checks", "direct_disagreements", "witness_direct_search_failures")}
                    for mode, build in build_proofs.items()},
                cache_scope_note="Final planning loads an already-built library. Build-phase proofs distinguish new FK/IK checks from disk replay; counters in a replay are original library diagnostics, not new validation.")


def audit_seed_recovery(folder, docks, costs, routes, details):
    checks = {}
    records = read_json(folder / "navigation_seed_recovery.json")
    if isinstance(records, dict):
        records = [records]
    keys = {(r["mode"], r["method"]) for r in records}
    assert len(keys) == len(records)
    for record in records:
        mode, method = record["mode"], record["method"]
        key = f"{mode}/{method}"; recovery = record["recovery"]
        if not details[key]["mission_valid"]:
            continue
        if not recovery["enabled"]:
            assert record["route_order_source"] == "strategy_preferred" and recovery["status"] == "not_needed"
            checks[key] = dict(route_order_source="strategy_preferred")
            continue
        matrix = rows(folder / f"seed_navigation_costs_{mode}_{method}.csv")
        n = math.isqrt(len(matrix)); assert n*n == len(matrix)
        D, mapping = {}, {}
        for edge in matrix:
            a, b = int(edge["from_local"]), int(edge["to_local"])
            assert 1 <= a <= n and 1 <= b <= n and (a, b) not in D
            value = float(edge["navigation_cost"])
            assert not math.isnan(value) and value >= 0 and (a != b or value == 0)
            D[a, b] = value
            for local, node in ((a, int(edge["from_node"])), (b, int(edge["to_node"]))):
                assert local not in mapping or mapping[local] == node
                mapping[local] = node
            pair = int(edge["from_node"]), int(edge["to_node"])
            if pair in costs:
                assert equal_cost(value, costs[pair])
        selected = sorted((d for d in docks if d["mode"] == mode and d["method"] == method), key=lambda d: int(d["dock_id"]))
        assert mapping[1] == 1 and len(set(mapping.values())) == n
        assert [mapping[i] for i in range(2, n+1)] == [int(d["candidate_id"])+1 for d in selected]
        attempts = recovery["attempts"]
        if isinstance(attempts, dict):
            attempts = [attempts]
        expected, chosen, chosen_cost = [], None, None
        for start in range(1, n+1):
            route, unvisited = [start], set(range(1, n+1))-{start}
            while unvisited:
                nxt = min(unvisited, key=lambda j: (D[route[-1], j], j))
                route.append(nxt); unvisited.remove(nxt)
            at = route.index(1); route = route[at:]+route[:at]
            closed = route+[1]; edge_costs = [D[a, b] for a, b in zip(closed, closed[1:])]
            finite = all(math.isfinite(v) for v in edge_costs)
            expected.append((start, route, closed, finite, sum(edge_costs)))
            if finite:
                chosen, chosen_cost = route, sum(edge_costs)
                break
        exact_csv_reproduction = chosen is not None and len(attempts) == len(expected) and all(
            actual["visit_order"] == item[1] for actual, item in zip(attempts, expected))
        assert len(attempts) == recovery["attempt_count"]
        actual_attempts, rounding_ties = [], []
        for start, actual in enumerate(attempts, 1):
            route = actual["visit_order"]
            assert actual["start"] == start and len(route) == n and route[0] == 1 and set(route) == set(range(1, n+1))
            at = route.index(start); unrotated = route[at:]+route[:at]
            unvisited = set(range(1, n+1))-{start}
            for current, nxt in zip(unrotated, unrotated[1:]):
                assert nxt in unvisited
                canonical = min(unvisited, key=lambda j: (D[current, j], j))
                minimum = D[current, canonical]; selected_cost = D[current, nxt]
                if math.isinf(minimum):
                    assert nxt == canonical
                else:
                    # MATLAB writes 15 significant digits. A pair whose
                    # internal costs differ at the last bits can become a
                    # CSV tie, changing a canonical Python min/tour. Check
                    # the recorded NN choice within that export precision.
                    assert selected_cost <= minimum+2e-12
                    if nxt != canonical:
                        rounding_ties.append(dict(start=start, from_local=current, chosen_local=nxt,
                            canonical_csv_local=canonical, chosen_cost=selected_cost, minimum_cost=minimum))
                unvisited.remove(nxt)
            closed = route+[1]; edge_costs = [D[a, b] for a, b in zip(closed, closed[1:])]
            finite, value = all(math.isfinite(v) for v in edge_costs), sum(edge_costs)
            assert actual["closed_order"] == closed
            assert actual["closed_finite"] == actual["route_finite"] == finite
            assert (abs(actual["cost"]-value) < TOL if finite else actual["cost"] is None or math.isinf(actual["cost"]))
            assert finite == (start == len(attempts)), "The fixed-order recovery must stop at its first finite attempt"
            actual_attempts.append((start, route, closed, finite, value))
        start, chosen, _, _, chosen_cost = actual_attempts[-1]
        assert recovery["return_to_start"] and not recovery["two_opt_applied"]
        assert recovery["selected_start"] == start and recovery["recovered"] == (start != 1)
        assert recovery["initial_closed_finite"] == actual_attempts[0][3]
        source = "nearest_neighbor_finite_restart" if start != 1 else "nearest_neighbor"
        assert record["route_order_source"] == recovery["route_order_source"] == source
        assert tuple(mapping[i] for i in chosen) == routes[mode, method]
        assert abs(chosen_cost-details[key]["navigation_cost_recomputed"]) < TOL
        checks[key] = dict(route_order_source=source, initial_closed_finite=actual_attempts[0][3],
                           first_finite_start=start, attempts_independently_reproduced=len(actual_attempts),
                           exact_csv_tie_policy_reproduced=exact_csv_reproduction,
                           admissible_export_rounding_ties=rounding_ties)
    assert all(tuple(key.split("/")) in keys for key, detail in details.items() if detail["mission_valid"])
    return checks


def paired_statistics(summary, planned_case_ids):
    """Keep all denominators; no invented confidence intervals for this grid."""
    records = {(row["case_id"], row["mode"], row["method"]): row for row in summary}
    assert len(records) == len(summary)
    results = {}
    for mode in MODES:
        eligible, excluded = [], []
        for cid in planned_case_ids:
            group = [records[cid, mode, method] for method in PRIMARY]
            good = all(truth(row["mission_valid"]) and int(row["covered_tasks"]) == int(row["verified_tasks"]) == int(row["target_count"])
                       and math.isfinite(float(row["minimum_docks"])) and int(row["docks"]) == int(float(row["minimum_docks"]))
                       for row in group)
            if good:
                good = len({int(float(row["minimum_docks"])) for row in group}) == 1
            (eligible if good else excluded).append(cid)
        comparisons = {}
        for control in ("minimum_seed", "reorder_only"):
            values, raw_values, outcomes, per_case = [], [], Counter(), {}
            for cid in eligible:
                a, b = records[cid, mode, control], records[cid, mode, "joint_local"]
                before, after = float(a["navigation_cost"]), float(b["navigation_cost"])
                outcome = "win" if after < before-TOL else "loss" if after > before+TOL else "tie"
                outcomes[outcome] += 1
                gain = 100*(1-after/before); raw_gain = 100*(1-float(b["route_length_m"])/float(a["route_length_m"]))
                values.append(gain); raw_values.append(raw_gain)
                per_case[cid] = dict(outcome=outcome, navigation_reduction_percent=gain, route_length_reduction_percent=raw_gain)
            def quantile(data, p):
                if not data:
                    return None
                ordered = sorted(data); at = (len(ordered)-1)*p; lo = int(at); hi = math.ceil(at)
                return ordered[lo]+(ordered[hi]-ordered[lo])*(at-lo)
            comparisons[control] = dict(win=outcomes["win"], tie=outcomes["tie"], loss=outcomes["loss"],
                navigation_reduction_median_percent=quantile(values, .5),
                navigation_reduction_q1_percent=quantile(values, .25), navigation_reduction_q3_percent=quantile(values, .75),
                navigation_reduction_worst_percent=min(values) if values else None,
                route_length_reduction_median_percent=quantile(raw_values, .5),
                per_case=per_case)
        results[mode] = dict(planned_scenarios=len(planned_case_ids), eligible_scenarios=len(eligible),
                            eligible_case_ids=eligible, excluded_case_ids=excluded, comparisons=comparisons)
    return results


def compare_original_main(folder, scenario_audits):
    """Describe reproduction differences; never edit results to force a match."""
    if "L0_D0" not in scenario_audits or "cases" not in scenario_audits["L0_D0"]:
        return None
    original_path = ROOT / "results/main_joint_task_navigation_20261111/summary.csv"
    current_path = folder / "cases/L0_D0/summary.csv"
    original = {(r["mode"], r["method"]): r for r in rows(original_path)}
    current = {(r["mode"], r["method"]): r for r in rows(current_path)}
    assert set(current) == set(original)
    differences = []
    fields = ("route_length_m", "navigation_cost", "minimum_docks", "docks", "covered_tasks", "verified_tasks")
    for key, row in current.items():
        for field in fields:
            a, b = float(original[key][field]), float(row[field])
            if not ((math.isnan(a) and math.isnan(b)) or equal_cost(a, b)):
                differences.append(dict(mode=key[0], method=key[1], field=field, original=a, batch=b))
    names = ["tasks.csv", "candidate_pool.csv"]+[f"pair_support_{mode}.csv" for mode in MODES]
    hashes = {name: dict(original=sha256(original_path.parent/name), batch=sha256(current_path.parent/name)) for name in names}
    assert all(pair["original"] == pair["batch"] for pair in hashes.values())
    return dict(reference_case_count=len(original), input_support_hashes=hashes,
                exact_navigation_and_count_reproduction=not differences,
                navigation_or_count_differences=differences,
                joint_search_times_s={mode: dict(original=float(original[mode, "joint_local"]["selection_time_s"]),
                    batch=float(current[mode, "joint_local"]["selection_time_s"])) for mode in MODES},
                runtime_claim_supported=False,
                note="Same signed inputs; time limits/concurrency can change a budgeted incumbent. Differences are reported, never forced away.")


def directed_two_opt_converge(D, initial_route, max_neighbors=10000):
    """Whole-tour directed repricing, with deterministic best improvement.

    The graph is already priced. No IK, navigation, or path reconstruction is
    called. A partial final neighborhood may improve the incumbent but never
    earns a local-optimum flag.
    """
    import numpy as np
    matrix = np.asarray(D, dtype=float)
    route = np.asarray(initial_route, dtype=int).copy()
    assert matrix.ndim == 2 and matrix.shape[0] == matrix.shape[1]
    n = matrix.shape[0]
    assert matrix.shape == (len(route), len(route)) and route[0] == 0
    assert sorted(route.tolist()) == list(range(n)) and max_neighbors > 0
    assert not np.isnan(matrix).any() and (matrix >= 0).all() and (np.diag(matrix) == 0).all()

    def price(candidates):
        return matrix[candidates[:, :-1], candidates[:, 1:]].sum(axis=1)+matrix[candidates[:, -1], 0]

    value = float(price(route[None, :])[0]); assert math.isfinite(value)
    pairs = tuple(itertools.combinations(range(1, n), 2))
    evaluated = passes = 0; accepted = []
    certified = False; stop = "neighbor_budget"
    if not pairs:
        return route.tolist(), value, dict(neighbors_evaluated=0, passes=1, accepted_moves=0,
            local_optimum_certified=True, termination_reason="complete_neighborhood_without_improvement", accepted_cost_chain=[])
    while evaluated < max_neighbors:
        remaining = max_neighbors-evaluated
        tested = pairs[:remaining]
        candidates = np.repeat(route[None, :], len(tested), axis=0)
        for row, (lo, hi) in enumerate(tested):
            candidates[row, lo:hi+1] = candidates[row, lo:hi+1][::-1]
        costs = price(candidates); best = int(np.argmin(costs)); best_cost = float(costs[best])
        evaluated += len(tested); passes += 1
        full = len(tested) == len(pairs)
        if best_cost < value-1e-9:
            accepted.append([value, best_cost]); route, value = candidates[best].copy(), best_cost
            if not full:
                break
        else:
            if full:
                certified = True; stop = "complete_neighborhood_without_improvement"
            break
    assert sorted(route.tolist()) == list(range(n)) and route[0] == 0
    return route.tolist(), value, dict(neighbors_evaluated=evaluated, passes=passes,
        accepted_moves=len(accepted), local_optimum_certified=certified,
        termination_reason=stop, accepted_cost_chain=accepted)


def reorder_convergence_diagnostic(folder, max_neighbors=10000):
    """Post-hoc fixed-set graph diagnostic, excluded from primary methods."""
    import numpy as np
    manifest = rows(folder / "scenario_manifest.csv")
    output = []
    fields = ["case_id", "layout_id", "distribution_id", "mode", "status", "reason",
              "graph_only", "new_Astar_queries", "new_IK_queries", "new_complete_path_validation",
              "selected_set_matches_staged", "selected_set_preserved", "docks", "minimum_docks",
              "minimum_seed_J", "main_reorder_J", "main_joint_J", "diagnostic_graph_J",
              "main_reorder_minus_diagnostic_J", "joint_reduction_vs_diagnostic_percent",
              "neighbors_budget", "neighbors_evaluated", "passes", "accepted_moves",
              "local_optimum_certified", "termination_reason", "seed_nodes", "diagnostic_nodes",
              "accepted_cost_chain", "priced_matrix_sha256"]
    for plan in manifest:
        target = folder / plan["relative_dir"]
        summary_path = target / "summary.csv"
        summary = {(r["mode"], r["method"]): r for r in rows(summary_path)} if summary_path.exists() else {}
        docks = rows(target / "docking_poses.csv", optional=True)
        for mode in MODES:
            record = dict.fromkeys(fields, "")
            record.update(case_id=plan["case_id"], layout_id=plan["layout_id"], distribution_id=plan["distribution_id"],
                mode=mode, status="SKIPPED", graph_only=True, new_Astar_queries=0, new_IK_queries=0,
                new_complete_path_validation=False, selected_set_matches_staged=False,
                selected_set_preserved=False, neighbors_budget=max_neighbors)
            for field in ("minimum_seed_J", "main_reorder_J", "main_joint_J", "diagnostic_graph_J",
                          "main_reorder_minus_diagnostic_J", "joint_reduction_vs_diagnostic_percent"):
                record[field] = math.nan
            a = summary.get((mode, "minimum_seed")); raw = summary.get((mode, "staged"))
            if a is None or raw is None:
                record["reason"] = "scenario_mode_not_complete"
                output.append(record); continue
            if not truth(a["mission_valid"]) or not truth(raw["mission_valid"]):
                record["reason"] = "no_valid_staged_and_minimum_seed"
                output.append(record); continue
            seed = sorted((r for r in docks if r["mode"] == mode and r["method"] == "minimum_seed"),
                          key=lambda r: int(r["visit_rank"]))
            staged = [r for r in docks if r["mode"] == mode and r["method"] == "staged"]
            seed_nodes = (1,)+tuple(int(r["candidate_id"])+1 for r in seed)
            if {r["candidate_id"] for r in seed} != {r["candidate_id"] for r in staged}:
                record["reason"] = "minimum_seed_and_staged_selected_sets_differ"
                output.append(record); continue
            record["selected_set_matches_staged"] = True
            matrix_path = target / f"seed_navigation_costs_{mode}_staged.csv"
            assert matrix_path.exists()
            edges = rows(matrix_path); n = math.isqrt(len(edges)); assert n*n == len(edges)
            D = np.full((n, n), np.nan); mapping = {}
            for edge in edges:
                i, j = int(edge["from_local"])-1, int(edge["to_local"])-1
                assert np.isnan(D[i, j]); D[i, j] = float(edge["navigation_cost"])
                for local, node in ((i, int(edge["from_node"])), (j, int(edge["to_node"]))):
                    assert local not in mapping or mapping[local] == node
                    mapping[local] = node
            inverse = {node: local for local, node in mapping.items()}
            assert mapping[0] == 1 and len(inverse) == n and set(seed_nodes) == set(mapping.values())
            initial = [inverse[node] for node in seed_nodes]
            seed_price = float(D[initial[:-1], initial[1:]].sum()+D[initial[-1], 0])
            assert abs(seed_price-float(a["navigation_cost"])) < TOL
            route, value, stats = directed_two_opt_converge(D, initial, max_neighbors)
            result_nodes = tuple(mapping[local] for local in route)
            assert set(result_nodes) == set(seed_nodes) and result_nodes[0] == 1
            record.update(status="GRAPH_DIAGNOSTIC_COMPLETE", reason="", selected_set_preserved=True,
                docks=a["docks"], minimum_docks=a["minimum_docks"], minimum_seed_J=seed_price,
                diagnostic_graph_J=value, seed_nodes="|".join(map(str, seed_nodes)),
                diagnostic_nodes="|".join(map(str, result_nodes)), priced_matrix_sha256=sha256(matrix_path),
                **{k: v for k, v in stats.items() if k != "accepted_cost_chain"})
            record["accepted_cost_chain"] = json.dumps(stats["accepted_cost_chain"], separators=(",", ":"))
            b, c = summary.get((mode, "reorder_only")), summary.get((mode, "joint_local"))
            if b and truth(b["mission_valid"]):
                record["main_reorder_J"] = float(b["navigation_cost"])
                record["main_reorder_minus_diagnostic_J"] = float(b["navigation_cost"])-value
            if c and truth(c["mission_valid"]):
                record["main_joint_J"] = float(c["navigation_cost"])
                record["joint_reduction_vs_diagnostic_percent"] = 100*(1-float(c["navigation_cost"])/value)
            output.append(record)
    path = folder / "reorder_convergence_diagnostic.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(output)
    note = dict(post_hoc=True, graph_only=True, included_in_primary_methods=False,
        included_in_completion_or_eligibility_denominators=False, new_navigation_or_IK_queries=0,
        neighbor_budget_per_case_mode=max_neighbors, acceptance_tolerance=1e-9,
        algorithm=f"Deterministic best-improvement directed 2-opt, reprice every whole closed tour on the existing staged selected-set D; stop after a complete nonimproving pass or {max_neighbors} tested neighbors.",
        no_new_complete_path_validation=True, rows=len(output),
        diagnostic_rows=sum(r["status"] == "GRAPH_DIAGNOSTIC_COMPLETE" for r in output),
        csv_sha256=sha256(path), auditor_sha256=sha256(Path(__file__)),
        caveat="Same maximum passes does not equate accepted moves or work between reorder-only and joint search. This stronger fixed-set graph diagnostic cannot be presented as a newly validated mission or used to redefine the predeclared baseline.")
    (folder / "reorder_convergence_diagnostic.json").write_text(json.dumps(note, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return note


def audit_reorder_diagnostic(folder, summary):
    """Check the existing diagnostic endpoints; do not rerun its search."""
    import numpy as np
    path = folder / "reorder_convergence_diagnostic.csv"
    if not path.exists():
        return None
    report = read_json(folder / "reorder_convergence_diagnostic.json")
    assert report["post_hoc"] and report["graph_only"] and report["no_new_complete_path_validation"]
    assert not report["included_in_primary_methods"] and not report["included_in_completion_or_eligibility_denominators"]
    assert report["new_navigation_or_IK_queries"] == 0 and report["csv_sha256"] == sha256(path)
    diagnostic = rows(path); indexed = {(r["case_id"], r["mode"], r["method"]): r for r in summary}
    keys = {(r["case_id"], r["mode"]) for r in diagnostic}
    assert len(keys) == len(diagnostic) == 27
    assert keys == {(f"L{l}_D{d}", mode) for l, d in itertools.product(range(3), repeat=2) for mode in MODES}
    details, outcomes, b_outcomes = {}, {mode: Counter() for mode in MODES}, {mode: Counter() for mode in MODES}
    remaining = []
    for row in diagnostic:
        cid, mode = row["case_id"], row["mode"]; target = folder / "cases" / cid
        key = f"{cid}/{mode}"
        if row["status"] != "GRAPH_DIAGNOSTIC_COMPLETE":
            assert row["reason"]
            details[key] = dict(status=row["status"], reason=row["reason"])
            continue
        assert truth(row["graph_only"]) and not truth(row["new_complete_path_validation"])
        assert int(row["new_Astar_queries"]) == int(row["new_IK_queries"]) == 0
        assert truth(row["selected_set_matches_staged"]) and truth(row["selected_set_preserved"])
        matrix_path = target / f"seed_navigation_costs_{mode}_staged.csv"
        assert row["priced_matrix_sha256"] == sha256(matrix_path)
        matrix = rows(matrix_path); n = math.isqrt(len(matrix)); assert n*n == len(matrix)
        D, mapping = np.full((n, n), np.nan), {}
        for edge in matrix:
            a, b = int(edge["from_local"])-1, int(edge["to_local"])-1
            assert np.isnan(D[a, b]); D[a, b] = float(edge["navigation_cost"])
            for local, node in ((a, int(edge["from_node"])), (b, int(edge["to_node"]))):
                assert local not in mapping or mapping[local] == node
                mapping[local] = node
        inverse = {node: local for local, node in mapping.items()}
        assert len(inverse) == n and mapping[0] == 1
        docks = rows(target / "docking_poses.csv")
        def method_route(method):
            selected = sorted((d for d in docks if d["mode"] == mode and d["method"] == method), key=lambda d: int(d["visit_rank"]))
            return (1,)+tuple(int(d["candidate_id"])+1 for d in selected)
        seed, final = nodes(row["seed_nodes"]), nodes(row["diagnostic_nodes"])
        assert seed == method_route("minimum_seed")
        assert set(seed) == set(final) == set(method_route("staged")) == set(mapping.values())
        assert seed[0] == final[0] == 1
        def cost(route):
            local = [inverse[node] for node in route]
            return float(D[local[:-1], local[1:]].sum()+D[local[-1], 0])
        seed_cost, value = cost(seed), cost(final)
        a, b, c = (indexed[cid, mode, method] for method in PRIMARY)
        assert abs(seed_cost-float(a["navigation_cost"])) < TOL
        assert abs(seed_cost-float(row["minimum_seed_J"])) < TOL
        assert abs(value-float(row["diagnostic_graph_J"])) < TOL
        assert int(row["docks"]) == int(row["minimum_docks"]) == int(float(a["minimum_docks"]))
        chain = json.loads(row["accepted_cost_chain"])
        assert len(chain) == int(row["accepted_moves"])
        previous = seed_cost
        for before, after in chain:
            assert abs(before-previous) < TOL and after < before-1e-9
            previous = after
        assert abs(previous-value) < TOL
        assert 0 <= int(row["neighbors_evaluated"]) <= int(row["neighbors_budget"]) == 10000
        pairs = tuple(itertools.combinations(range(1, n), 2))
        def best_neighbor(route):
            local = np.array([inverse[node] for node in route], dtype=int)
            candidates = np.repeat(local[None, :], len(pairs), axis=0)
            for i, (lo, hi) in enumerate(pairs):
                candidates[i, lo:hi+1] = candidates[i, lo:hi+1][::-1]
            values = D[candidates[:, :-1], candidates[:, 1:]].sum(axis=1)+D[candidates[:, -1], 0]
            return float(np.min(values)) if len(values) else cost(route)
        diagnostic_neighbor = best_neighbor(final)
        if truth(row["local_optimum_certified"]):
            assert diagnostic_neighbor >= value-1e-9
            assert row["termination_reason"] == "complete_neighborhood_without_improvement"
            assert int(row["neighbors_evaluated"]) == int(row["passes"])*len(pairs)
        b_value, c_value = float(b["navigation_cost"]), float(c["navigation_cost"])
        assert abs(b_value-float(row["main_reorder_J"])) < TOL and abs(c_value-float(row["main_joint_J"])) < TOL
        assert abs(b_value-value-float(row["main_reorder_minus_diagnostic_J"])) < TOL
        assert abs(100*(1-c_value/value)-float(row["joint_reduction_vs_diagnostic_percent"])) < TOL
        b_route = method_route("reorder_only"); assert set(b_route) == set(seed)
        assert abs(cost(b_route)-b_value) < TOL
        b_neighbor = best_neighbor(b_route)
        if b_neighbor < b_value-1e-9:
            remaining.append(key)
        if truth(b["local_optimum_certified"]):
            assert b_neighbor >= b_value-1e-9
        def outcome(other):
            return "win" if other < value-TOL else "loss" if other > value+TOL else "tie"
        outcomes[mode][outcome(c_value)] += 1; b_outcomes[mode][outcome(b_value)] += 1
        details[key] = dict(status=row["status"], diagnostic_graph_J=value,
            main_reorder_J=b_value, main_joint_J=c_value,
            joint_vs_diagnostic=outcome(c_value), main_reorder_vs_diagnostic=outcome(b_value),
            diagnostic_full_neighborhood_certified=truth(row["local_optimum_certified"]),
            main_reorder_has_improving_two_opt_neighbor=b_neighbor < b_value-1e-9,
            main_reorder_best_neighbor_graph_J=b_neighbor)
    def normalized(group):
        return {mode: {key: counts[key] for key in ("win", "tie", "loss")} for mode, counts in group.items()}
    return dict(rows_checked=len(diagnostic), graph_only=True, new_Astar_or_IK_queries=0,
        optimizer_not_rerun=True, final_routes_and_complete_neighborhoods_repriced=True,
        joint_vs_diagnostic=normalized(outcomes), main_reorder_vs_diagnostic=normalized(b_outcomes),
        main_reorder_with_remaining_two_opt_improvement_count=len(remaining),
        main_reorder_with_remaining_two_opt_improvement_cases=remaining,
        cases=details,
        note="Best-improvement from the common seed can reach a different local minimum than the main first-improvement order. It need not dominate that main reorder result. This post-hoc graph diagnosis does not change the predeclared primary comparison.")


def audit_batch(folder, allow_partial=False):
    manifest_path = folder / "scenario_manifest.csv"
    planned = rows(manifest_path)
    assert len(planned) == 9 and len({r["case_id"] for r in planned}) == 9
    assert {(int(r["layout_id"]), int(r["distribution_id"])) for r in planned} == set(itertools.product(range(3), repeat=2))
    assert {r["case_id"] for r in planned} == {f"L{l}_D{d}" for l, d in itertools.product(range(3), repeat=2)}
    definitions_path = folder / "scenario_definitions.json"
    definitions = read_json(definitions_path)
    if isinstance(definitions, dict):
        definitions = definitions.get("cases", definitions.get("definitions"))
    assert isinstance(definitions, list) and len(definitions) == 9
    frozen = {definition["case_id"]: definition for definition in definitions}
    assert set(frozen) == {plan["case_id"] for plan in planned}
    for plan in planned:
        definition = frozen[plan["case_id"]]
        assert int(definition["layout_id"]) == int(plan["layout_id"])
        assert int(definition["distribution_id"]) == int(plan["distribution_id"])
        plan["definition"] = definition
    all_summary, audits, unfinished = [], {}, []
    batch_rows = rows(folder / "batch_summary.csv", optional=allow_partial)
    for plan in planned:
        relative = plan.get("relative_dir", plan["case_id"])
        target = (folder / relative).resolve()
        assert target.is_relative_to(folder.resolve())
        preparation_failure = (target / "scenario_failure.json").exists()
        if not preparation_failure:
            state_path = target / "run_state.json"
            ready = state_path.exists() and read_json(state_path).get("status") == "COMPLETE"
            if not ready:
                assert allow_partial, f"Scenario {plan['case_id']} is not complete"
                unfinished.append(plan["case_id"])
                continue
        if not (target / "metadata.json").exists() or preparation_failure:
            case_rows = [r for r in batch_rows if r["case_id"] == plan["case_id"]]
            if len(case_rows) != 12:
                assert allow_partial
                unfinished.append(plan["case_id"])
                continue
            assert {(r["mode"], r["method"]) for r in case_rows} == set(itertools.product(MODES, METHODS))
            assert all(not truth(r["mission_valid"]) and math.isnan(float(r["navigation_cost"]))
                       and math.isnan(float(r["route_length_m"])) and r["failure_reason"] for r in case_rows)
            assert preparation_failure, "A complete failed scenario requires a retained failure artifact"
            all_summary.extend(case_rows)
            audits[plan["case_id"]] = dict(status="RECORDED_FAILURE_BEFORE_CASE_ARTIFACTS", recorded_failures=case_rows)
            continue
        case = audit_case(target, plan, allow_partial)
        for row in case.pop("summary"):
            all_summary.append(dict(row, case_id=plan["case_id"]))
        audits[plan["case_id"]] = case
    assert allow_partial or set(audits) == {p["case_id"] for p in planned}
    complete = [a for a in audits.values() if "robot_signature" in a]
    if complete:
        signatures = {a["robot_signature"] for a in complete}
        assert None not in signatures and len(signatures) == 1, "Complete robot/mount/TCP/IK config must remain frozen"
        assert len({json.dumps(a["robot_footprint"], sort_keys=True) for a in complete}) == 1
        assert len({json.dumps(a["scorer_sources"], sort_keys=True) for a in complete}) == 1
        assert len({json.dumps(a["local_search_budget"], sort_keys=True) for a in complete}) == 1
        assert len({tuple(a["per_table_task_counts"]) for a in complete}) == 1
    for layout in range(3):
        group = [audits.get(f"L{layout}_D{d}") for d in range(3)]
        group = [a for a in group if a and "immutable_pool_sha256" in a]
        assert len({json.dumps(a["immutable_pool_sha256"], sort_keys=True) for a in group}) <= 1, \
            "Candidate identity must be frozen across task distributions within each layout"
    indexed = {(r["case_id"], r["mode"], r["method"]): r for r in batch_rows}
    assert len(indexed) == len(batch_rows)
    deferred_summary_keys = []
    for row in all_summary:
        key = row["case_id"], row["mode"], row["method"]
        if key not in indexed:
            assert allow_partial, f"Completed case is missing from batch summary: {key}"
            deferred_summary_keys.append(key)
            continue
        for field, value in row.items():
            if field not in indexed[key]:
                continue
            other = indexed[key][field]
            if value == other:
                continue
            try:
                a, b = float(value), float(other)
                assert (math.isnan(a) and math.isnan(b)) or equal_cost(a, b)
            except ValueError:
                raise AssertionError(f"Batch/case export mismatch for {key}/{field}: {value!r}/{other!r}")
    if not allow_partial:
        assert len(all_summary) == len(batch_rows) == 108
    planned_ids = [p["case_id"] for p in planned]
    stats = paired_statistics(all_summary, planned_ids) if len(all_summary) == 108 else None
    failures = [r for r in all_summary if not truth(r["mission_valid"])]
    result = dict(status="PARTIAL_PASS" if unfinished or len(all_summary) < 108 else "PASS_WITH_RECORDED_FAILURES" if failures else "PASS",
        planned_scenarios=9, scenarios_checked=len(audits), rows_checked=len(all_summary),
        valid_missions=sum(truth(r["mission_valid"]) for r in all_summary), recorded_failure_count=len(failures),
        fixed_primary_methods=list(PRIMARY), raw_staged_is_auxiliary=True, tie_tolerance_absolute_navigation_cost=TOL,
        paired_statistics=stats, cases=audits, unfinished_case_ids=unfinished,
        batch_summary_crosscheck_deferred_keys=deferred_summary_keys,
        original_main_reproduction=compare_original_main(folder, audits),
        reorder_convergence_diagnostic=(audit_reorder_diagnostic(folder, all_summary) if len(all_summary) == 108 else None),
        scenario_manifest_sha256=sha256(manifest_path), auditor_sha256=sha256(Path(__file__)),
        frozen_scenario_definitions_sha256=sha256(definitions_path),
        limits="Independent export/DP/oracle-cost and sampled rotated-geometry audit; not independently solved IK/A*, continuous collision, contact force or measured execution. Minima are regional finite-pool certificates. Deterministic nine-case grid does not establish a population confidence interval, global optimality or generic superiority.")
    (folder / "independent_audit.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def audit_inputs(folder):
    """Check every frozen input before waiting for expensive IK outputs."""
    planned = rows(folder / "scenario_manifest.csv")
    definitions = read_json(folder / "scenario_definitions.json")
    assert isinstance(definitions, list) and len(definitions) == len(planned) == 9
    frozen = {d["case_id"]: d for d in definitions}
    expected_ids = {f"L{l}_D{d}" for l, d in itertools.product(range(3), repeat=2)}
    assert set(frozen) == {p["case_id"] for p in planned} == expected_ids
    original = rows(ROOT / "results/main_joint_task_navigation_20261111/tasks.csv")
    original_centers = [(x, y) for y in (1.5, 4., 6.5, 9.) for x in (2., 4., 6., 8.)]
    original_local = [[float(t["x_m"])-original_centers[int(t["table_id"])-1][0],
                       float(t["y_m"])-original_centers[int(t["table_id"])-1][1]] for t in original]
    table_ids = [int(t["table_id"]) for t in original]
    counts = [table_ids.count(t) for t in range(1, 17)]
    audits = {}
    for plan in planned:
        definition = frozen[plan["case_id"]]; layout, distribution = int(plan["layout_id"]), int(plan["distribution_id"])
        assert definition["case_id"] == f"L{layout}_D{distribution}"
        assert definition["layout_id"] == layout and definition["distribution_id"] == distribution
        assert definition["task_table_ids"] == table_ids and definition["per_table_task_counts"] == counts
        assert all(math.dist(a, b) < TOL for a, b in zip(definition["original_task_local_xy"], original_local))
        expected_centers = [list(center) for center in original_centers]
        expected_angles = [0.]*16
        if layout == 1:
            expected_centers = [[x+.3*(-1 if i//4 % 2 == 0 else 1), y] for i, (x, y) in enumerate(original_centers)]
        elif layout == 2:
            expected_angles = [0., math.pi/6, -math.pi/6, math.pi/2]*4
        source = definition["scene_cfg"]
        assert all(math.dist(a, b) < TOL for a, b in zip(source["table_positions"], expected_centers))
        assert all(abs(wrap(a-b)) < TOL for a, b in zip(source["table_orientations"], expected_angles))
        expected_local = [point[:] for point in original_local]
        if distribution:
            for t in range(1, 17):
                ids = [i for i, value in enumerate(table_ids) if value == t]
                sign = -1 if t % 2 else 1
                points = ([[-.18, .12*sign], [.18, .12*sign], [0., .12*sign]] if distribution == 1 else
                          [[-.45, -.12*sign], [.45, .12*sign], [0., -.12*sign]])
                for i, point in zip(ids, points):
                    expected_local[i] = point
        assert len(definition["task_local_xy"]) == len(expected_local) == 36
        assert all(math.dist(a, b) < TOL for a, b in zip(definition["task_local_xy"], expected_local))
        plan["definition"] = definition
        target = (folder / plan["relative_dir"]).resolve()
        assert target.is_relative_to(folder.resolve())
        assert read_json(target / "case_definition.json") == definition
        audits[plan["case_id"]] = audit_case(target, plan, geometry_only=True)
    assert len({case["robot_signature"] for case in audits.values()}) == 1
    batch_meta = read_json(folder / "metadata.json")
    robot = read_json(folder / "frozen_robot.json")
    assert robot["manipulator"]["mode"] == "task_pose_ik"
    assert robot["manipulator"]["model"]["revision"].startswith("vx300s-urdf-0bb2b0e6")
    assert all(case["robot_signature"] == batch_meta["robot_signature"] and case["robot_footprint"] == robot["footprint"]
               for case in audits.values())
    for layout in range(3):
        assert len({json.dumps(audits[f"L{layout}_D{d}"]["immutable_pool_sha256"], sort_keys=True) for d in range(3)}) == 1
    result = dict(status="PASS_INPUT_GEOMETRY", scenarios_checked=9,
                  targets_checked=9*36, finite_tabletop_wipe_strokes_checked=9*36,
                  candidate_poses_checked=sum(a["scene_geometry"]["candidate_count"] for a in audits.values()),
                  factor_rules_independently_reconstructed=True, complete_robot_signatures_equal=True,
                  original_task_counts_and_anchors_checked=True, within_layout_candidate_pools_identical=True,
                  cases=audits, auditor_sha256=sha256(Path(__file__)),
                  frozen_scenario_definitions_sha256=sha256(folder / "scenario_definitions.json"),
                  frozen_robot_sha256=sha256(folder / "frozen_robot.json"),
                  note="Input-only audit, not an outcome or IK success verdict")
    (folder / "independent_input_audit.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def self_test():
    a = polygon(0, 0, 0, 1.2, .4)
    assert separated(a, polygon(2, 0, math.pi/4, 1.2, .4))
    assert not separated(a, polygon(.1, .1, math.pi/4, .5, .3))
    for angle in (0, math.pi/2, -.7):
        x, y = .32, -.12; c, s = math.cos(angle), math.sin(angle)
        assert math.dist(table_local(2+c*x-s*y, 4+s*x+c*y, [2, 4], angle), [x, y]) < 1e-12
    check_fps({(0, 0), (1, 0), (0, 1)}, {(0, 0), (1, 0), (0, 1)}, (0, 0))
    fake = {(1, 1): {"feasible": "1"}, (1, 2): {"feasible": "0"},
            (2, 1): {"feasible": "0"}, (2, 2): {"feasible": "1"}}
    cert = certify_region([1, 2], [1, 2], fake)
    assert cert["minimum_docks"] == 2 and cert["cover_count"] == 1
    # A directed four-node graph where reversing internal service nodes is
    # valuable; score every internal direction and the return, not four edges.
    D = [[0, 1, 9, 9], [9, 0, 1, 9], [9, 9, 0, 1], [1, 9, 9, 0]]
    route, cost, info = directed_two_opt_converge(D, [0, 3, 2, 1])
    assert route[0] == 0 and sorted(route) == [0, 1, 2, 3] and cost == 4
    assert info["local_optimum_certified"] and info["accepted_moves"] > 0
    _, _, limited = directed_two_opt_converge(D, [0, 3, 2, 1], max_neighbors=1)
    assert limited["neighbors_evaluated"] == 1 and not limited["local_optimum_certified"]
    print("Independent auditor self-checks PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", type=Path, default=Path("results/task_navigation_robustness_20261111"))
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--geometry-only", action="store_true")
    parser.add_argument("--reorder-diagnostic", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.geometry_only:
        result = audit_inputs(args.folder)
        print(json.dumps({k: result[k] for k in ("status", "scenarios_checked", "targets_checked", "candidate_poses_checked")}, ensure_ascii=False))
    elif args.reorder_diagnostic:
        result = reorder_convergence_diagnostic(args.folder)
        print(json.dumps({k: result[k] for k in ("graph_only", "rows", "diagnostic_rows", "new_navigation_or_IK_queries")}, ensure_ascii=False))
    else:
        result = audit_batch(args.folder, args.allow_partial)
        print(json.dumps({k: result[k] for k in ("status", "scenarios_checked", "rows_checked", "valid_missions", "recorded_failure_count")}, ensure_ascii=False))
