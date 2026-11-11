"""Independent audit of the full-default-scene local-joint experiment.

Standard-library evidence checks; no production selector or IK is imported.
This validates the finite support subpool and sampled exported base paths,
not physical execution, swept motion, or a globally optimal solution.
"""
from __future__ import annotations

import argparse
import csv
import functools
import hashlib
import itertools
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODES = ("position_only", "top_down_grasp", "table_wipe")
METHODS = ("staged", "minimum_seed", "reorder_only", "joint_local")
TOL = 1e-7


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as source:
        return list(csv.DictReader(source))


def truth(value):
    return str(value).lower() in {"true", "1"}


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose(row):
    return tuple(float(row[k]) for k in ("x_m", "y_m", "yaw_rad"))


def nodes(value):
    return tuple(int(x) for x in value.split("|")) if value else ()


def corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in
            ((-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2))]


def separated_from_table(poly, box):
    x0, x1, y0, y1 = box
    if max(p[0] for p in poly) < x0 or min(p[0] for p in poly) > x1 or \
            max(p[1] for p in poly) < y0 or min(p[1] for p in poly) > y1:
        return True
    rect = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    for shape in (poly, rect):
        for i in range(4):
            dx = shape[(i+1) % 4][0]-shape[i][0]
            dy = shape[(i+1) % 4][1]-shape[i][1]
            a = [-x*dy+y*dx for x, y in poly]
            b = [-x*dy+y*dx for x, y in rect]
            if max(a) < min(b)-1e-10 or max(b) < min(a)-1e-10:
                return True
    return False


def check_fps(all_xy, selected_xy, seed):
    """Check an admissible FPS tie order without assuming cross-language ties."""
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
            distance = {p: min(math.dist(p, q) for q in selected) for p in all_xy if p not in selected}
            farthest = max(distance.values())
            eligible = [i for i, p in enumerate(chosen) if not mask & (1 << i)
                        and distance[p] >= farthest-1e-9]
        return any(extend(mask | (1 << i)) for i in eligible)

    assert extend(0), "Retained XY set is not a geometry-only farthest-point selection"


def audit(folder, allow_partial=False):
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
    state = json.loads((folder / "run_state.json").read_text(encoding="utf-8-sig"))
    if not allow_partial:
        assert state["status"] == "COMPLETE", "Run incomplete; use --allow-partial for progress checks"
    # The common directed-matrix correction must not silently change any
    # geometry or IK evidence compared with the preserved initial trial.
    frozen_replay = None
    if meta.get("navigation_matrix_directed"):
        assert meta["finite_nn_restarts"]
        assert meta["routing"]["directed_costs"] and meta["routing"]["finite_nn_restarts"]
        initial = Path(meta["initial_trial_directory"])
        if not initial.is_absolute():
            initial = ROOT / initial
        assert initial.resolve() != folder.resolve()
        initial_state = json.loads((initial / "run_state.json").read_text(encoding="utf-8-sig"))
        assert initial_state["status"] == "COMPLETE" and initial_state["completed_cases"] == 12
        original_meta = json.loads((initial / "metadata.json").read_text(encoding="utf-8-sig"))
        assert not original_meta.get("navigation_matrix_directed", False)
        same_files = ["tasks.csv", "candidate_pool.csv", "dense_to_subpool.csv", "table_candidate_counts.csv",
                      "main_library_geometry_signature.json"]
        for mode in MODES:
            if (folder / f"ik_cache_{mode}.json").exists():
                same_files += [f"pair_support_{mode}.csv", f"candidate_support_{mode}.csv",
                               f"candidate_library_{mode}.mat"]
        frozen_hashes = {}
        for name in same_files:
            original_hash = hashlib.sha256((initial / name).read_bytes()).hexdigest()
            current_hash = hashlib.sha256((folder / name).read_bytes()).hexdigest()
            assert current_hash == original_hash, f"Frozen replay changed {name}"
            frozen_hashes[name] = current_hash
        initial_failures = [{"mode": r["mode"], "method": r["method"], "status": r["status"],
                             "reason": r["failure_reason"]}
                            for r in rows(initial / "summary.csv") if not truth(r["mission_valid"])]
        frozen_replay = {"initial_trial_directory": str(initial),
                         "common_navigation_matrix_directed": True,
                         "common_first_finite_nn_restarts": True,
                         "identical_geometry_and_support_sha256": frozen_hashes,
                         "initial_failures_preserved": initial_failures}
    scene, source = meta["scene"], meta["source_scene_config"]
    assert meta["model_revision"].startswith("vx300s-urdf-0bb2b0e6")
    assert not meta["arm_collision_checked"] and not meta["hardware_calibrated"]
    expected_centers = [[x, y] for y in (1.5, 4.0, 6.5, 9.0) for x in (2.0, 4.0, 6.0, 8.0)]
    assert scene["map_width_m"] == scene["map_height_m"] == 10
    assert scene["table_centers"] == source["table_positions"] == expected_centers
    assert source["random_seed"] == 2026 and source["num_tables"] == 16
    assert source["task_mode"] == "random" and source["table_orientations"] == [0]*16
    assert scene["start_pose"] == source["start_pose"]
    assert scene["table_length_m"] == source["table"]["length"] == 1.2
    assert scene["table_width_m"] == source["table"]["width"] == 0.4
    assert scene["table_height_m"] == source["table"]["height"] == 0.5
    tasks, pool = rows(folder / "tasks.csv"), rows(folder / "candidate_pool.csv")
    dense = rows(folder / "dense_to_subpool.csv"); table_counts = rows(folder / "table_candidate_counts.csv")
    assert len(tasks) == meta["task_count"] == 36
    assert {int(r["task_id"]) for r in tasks} == set(range(1, 37))
    table_tasks = {t: [r for r in tasks if int(r["table_id"]) == t] for t in range(1, 17)}
    for t, group in table_tasks.items():
        assert 2 <= len(group) <= 3
        cx, cy = expected_centers[t-1]
        for r in group:
            local = (float(r["x_m"])-cx, float(r["y_m"])-cy)
            assert any(math.dist(local, p) < 1e-10 for p in source["task_candidate_local"])
            assert float(r["z_m"]) == 0.5
    assert len(dense) == meta["dense_candidate_count"] == 26660
    assert len(pool) == meta["candidate_count"] == 906
    dense_id = {int(r["dense_candidate_id"]): r for r in dense}
    assert len(dense_id) == len(dense)
    selected = [r for r in dense if truth(r["selected"])]
    assert len(selected) == len(pool)
    assert sum(truth(r["collision_valid"]) for r in dense) == meta["footprint_valid_dense_count"]
    pool_by_id = {int(r["candidate_id"]): r for r in pool}
    assert set(pool_by_id) == set(range(1, len(pool)+1))
    for r in pool:
        cid = int(r["candidate_id"]); original = dense_id[int(r["dense_candidate_id"])]
        assert int(r["node_index"]) == cid+1 and truth(original["selected"]) and truth(original["collision_valid"])
        assert int(original["candidate_id"]) == cid and pose(r) == pose(original)
        assert r["table_id"] == original["table_id"] and r["side_id"] == original["side_id"]
    for count in table_counts:
        t = int(count["table_id"]); dr = [r for r in dense if int(r["table_id"]) == t]
        pr = [r for r in pool if int(r["table_id"]) == t]
        assert len(dr) == int(count["dense_candidates"])
        assert sum(truth(r["collision_valid"]) for r in dr) == int(count["footprint_valid_candidates"])
        assert len(pr) == int(count["retained_candidates"]) <= 64
        assert len(table_tasks[t]) == int(count["task_count"])
        cx, cy = expected_centers[t-1]
        for side in range(1, 5):
            valid = [r for r in dr if int(r["side_id"]) == side and truth(r["collision_valid"])]
            retained = [r for r in pr if int(r["side_id"]) == side]
            if not valid:
                assert not retained
                continue
            groups = {}
            for r in valid:
                xy = (round(float(r["x_m"])-cx, 10), round(float(r["y_m"])-cy, 10))
                groups.setdefault(xy, []).append(r)
            selected_xy = {(round(float(r["x_m"])-cx, 10), round(float(r["y_m"])-cy, 10)) for r in retained}
            seed = ((0, .5), (0, -.5), (-.9, 0), (.9, 0))[side-1]
            normal = (-math.pi/2, math.pi/2, 0, math.pi)[side-1]
            check_fps(groups, selected_xy, seed)
            for xy in selected_xy:
                available = groups[xy]
                chosen = [r for r in retained if (round(float(r["x_m"])-cx, 10), round(float(r["y_m"])-cy, 10)) == xy]
                assert len(chosen) == min(2, len(available))
                def legal_order(order):
                    remaining = list(available)
                    for r, target in zip(order, (normal, normal+math.pi/12)):
                        error = abs(wrap(float(r["yaw_rad"])-target))
                        if error > min(abs(wrap(float(a["yaw_rad"])-target)) for a in remaining)+1e-9:
                            return False
                        remaining = [a for a in remaining if a["dense_candidate_id"] != r["dense_candidate_id"]]
                    return True
                assert any(legal_order(order) for order in itertools.permutations(chosen))
    completed_modes, pair_lookup, minima, cache_proofs = [], {}, {}, {}
    certificates = rows(folder / "minimum_cover_certificates.csv") if (folder / "minimum_cover_certificates.csv").exists() else []
    manifest = json.loads((folder / "main_library_geometry_signature.json").read_text())
    for mode in MODES:
        proof_path = folder / f"ik_cache_{mode}.json"
        if not proof_path.exists():
            assert allow_partial
            continue
        p = rows(folder / f"pair_support_{mode}.csv"); c = rows(folder / f"candidate_support_{mode}.csv")
        proof = json.loads(proof_path.read_text(encoding="utf-8-sig"))
        if frozen_replay is not None and not proof["reused_from_disk"]:
            assert allow_partial, f"Replay recomputed or did not consume the {mode} support library"
            continue
        cache_proofs[mode] = proof
        expected = {(cid, int(t["task_id"])) for cid, candidate in pool_by_id.items()
                    for t in table_tasks[int(candidate["table_id"])]}
        look = {(int(r["candidate_id"]), int(r["task_id"])): r for r in p}
        assert set(look) == expected and len(look) == len(p) == proof["pair_count"] == 2037
        assert proof["geometry_directory_signature"] == manifest["signature"]
        assert proof["scorer_sources"] == manifest["scorer_sources"]
        assert not proof["independent_ik_validation"] and not proof["arm_collision_checked"]
        assert proof["request_count"] == len(p)
        assert proof["positive_hits"]+proof["exact_negative_hits"]+proof["misses"] == len(p)
        for field, kind in (("positive_hits", "positive_hit"), ("exact_negative_hits", "exact_negative_hit")):
            assert proof[field] == sum(r["cache_kind"] == kind for r in p)
        assert proof["witness_fallbacks"] == sum(r["cache_kind"] == "witness_fallback" for r in p)
        assert proof["max_position_error_m"] <= .003+1e-10
        angle_limit = math.radians(5 if mode == "table_wipe" else 10)
        assert proof["max_orientation_error_rad"] <= angle_limit+1e-10
        supported = [r for r in p if truth(r["feasible"])]
        assert len(supported) == proof["supported_pairs"]
        support_counts = [sum(truth(look[cid, tid]["feasible"]) for cid, t in expected if t == tid)
                          for tid in range(1, 37)]
        assert support_counts == proof["per_task_support_counts"]
        assert sum(v > 0 for v in support_counts) == proof["union_supported_count"]
        for r in supported:
            assert r["assigned_arm"] in {"left", "right"}
            assert float(r["max_position_error_m"]) <= .003+1e-10
            assert float(r["max_orientation_error_rad"]) <= angle_limit+1e-10
            assert float(r["max_joint_step_rad"]) <= math.radians(25)+1e-10
        assert len(c) == len(pool)
        for r in c:
            candidate = pool_by_id[int(r["candidate_id"])]; assert pose(r) == pose(candidate)
            tids = [int(t["task_id"]) for t in table_tasks[int(candidate["table_id"])]]
            assert int(r["support_count"]) == sum(truth(look[int(r["candidate_id"]), tid]["feasible"]) for tid in tids)
        regional = {}
        for t in range(1, 17):
            tids = [int(r["task_id"]) for r in table_tasks[t]]; bits = {tid: 1 << i for i, tid in enumerate(tids)}
            masks = {cid: sum(bits[tid] for tid in tids if truth(look[cid, tid]["feasible"]))
                     for cid, candidate in pool_by_id.items() if int(candidate["table_id"]) == t}
            dp = [math.inf]*(1 << len(tids)); dp[0] = 0
            for state_mask in range(len(dp)):
                for mask in masks.values():
                    dp[state_mask | mask] = min(dp[state_mask | mask], dp[state_mask]+1)
            feasible = math.isfinite(dp[-1]); k = int(dp[-1]) if feasible else None; count = 0
            if feasible:
                for choice in itertools.combinations([cid for cid in masks if masks[cid]], k):
                    total = 0
                    for cid in choice:
                        total |= masks[cid]
                    count += total == len(dp)-1
            exported = [r for r in certificates if r["mode"] == mode and int(r["region_id"]) == t]
            assert len(exported) == 1
            cert = exported[0]; assert truth(cert["feasible"]) == feasible and int(cert["cover_count"]) == count
            if feasible:
                assert int(float(cert["minimum_docks"])) == k
            else:
                assert math.isinf(float(cert["minimum_docks"]))
            regional[t] = {"minimum_docks": k, "cover_count": count, "masks": masks, "full_mask": len(dp)-1}
        completed_modes.append(mode); pair_lookup[mode] = look; minima[mode] = regional
    if not allow_partial:
        assert set(completed_modes) == set(MODES)
    summary = rows(folder / "summary.csv"); docks = rows(folder / "docking_poses.csv")
    assignments = rows(folder / "task_assignments.csv"); paths = rows(folder / "navigation_paths.csv")
    edges = rows(folder / "navigation_edges.csv"); moves = rows(folder / "accepted_moves.csv")
    costs = {}; node_pose = {cid+1: pose(r) for cid, r in pool_by_id.items()}; node_pose[1] = tuple(scene["start_pose"])
    lam = meta["lambda_theta_m_per_rad"]
    for r in edges:
        a, b = int(r["from_node"]), int(r["to_node"]); assert (a, b) not in costs
        assert int(r["from_candidate_id"]) == a-1 and int(r["to_candidate_id"]) == b-1
        yaw = abs(wrap(node_pose[b][2]-node_pose[a][2])); assert abs(yaw-float(r["endpoint_yaw_change_rad"])) < TOL
        length, value = float(r["path_length_m"]), float(r["navigation_cost"])
        if truth(r["path_found"]):
            assert abs(value-length-lam*yaw) < TOL
            assert length+TOL >= math.dist(node_pose[a][:2], node_pose[b][:2])
        else:
            assert math.isinf(value)
        costs[a, b] = value
    def route_cost(route):
        return sum(costs[a, b] for a, b in zip(route, route[1:]+(1,)))
    def supported_route(mode, route):
        assert route[0] == 1 and len(set(route)) == len(route)
        for t, cert in minima[mode].items():
            ids = [n-1 for n in route[1:] if int(pool_by_id[n-1]["table_id"]) == t]
            assert len(ids) == cert["minimum_docks"]
            mask = 0
            for cid in ids:
                mask |= cert["masks"][cid]
            assert mask == cert["full_mask"]
    valid_cases, failures, case_details, routes, sets = 0, [], {}, {}, {}
    boxes = [(x-.6, x+.6, y-.2, y+.2) for x, y in expected_centers]; fp = meta["robot_footprint"]
    keys = {(r["mode"], r["method"]) for r in summary}; assert len(keys) == len(summary)
    if not allow_partial:
        assert keys == set(itertools.product(MODES, METHODS)) and state["completed_cases"] == 12
    for s in summary:
        mode, method = s["mode"], s["method"]
        if mode not in completed_modes:
            assert allow_partial
            continue
        key = f"{mode}/{method}"; match = lambda r: r["mode"] == mode and r["method"] == method
        assert not truth(s["global_optimal"])
        local_flag = truth(s["local_optimum_certified"])
        assert not local_flag or s["termination_reason"] == "complete_neighborhood_pass_without_improvement"
        events = sorted(filter(match, moves), key=lambda r: int(r["move_id"]))
        assert len(events) == int(s["accepted_moves"])
        previous_new, planned = None, float(s["minimum_seed_cost"])
        for i, event in enumerate(events, start=1):
            before, after = nodes(event["previous_nodes"]), nodes(event["new_nodes"])
            assert int(event["move_id"]) == i
            supported_route(mode, before); supported_route(mode, after)
            assert abs(route_cost(before)-float(event["previous_cost"])) < TOL
            assert abs(route_cost(after)-float(event["new_cost"])) < TOL
            assert float(event["new_cost"]) < float(event["previous_cost"])-1e-9
            assert previous_new is None or before == previous_new
            assert abs(float(event["previous_cost"])-planned) < TOL
            if event["operation"] == "directed_two_opt":
                assert set(before) == set(after)
                changed = [j for j in range(len(before)) if before[j] != after[j]]
                lo, hi = min(changed), max(changed); assert lo >= 1
                assert before[:lo]+tuple(reversed(before[lo:hi+1]))+before[hi+1:] == after
            else:
                assert method == "joint_local" and event["operation"] == "regional_minimum_cover_replacement"
                t = int(event["region_id"])
                for a, b in zip(before, after):
                    if a != b:
                        assert int(pool_by_id[a-1]["table_id"]) == int(pool_by_id[b-1]["table_id"]) == t
            previous_new, planned = after, float(event["new_cost"])
        if method in {"staged", "minimum_seed"}:
            assert not events
        if not truth(s["mission_valid"]):
            assert math.isnan(float(s["route_length_m"])) and math.isnan(float(s["navigation_cost"]))
            failures.append({"mode": mode, "method": method, "status": s["status"], "reason": s["failure_reason"]})
            case_details[key] = {"mission_valid": False, "accepted_moves_checked": len(events)}
            continue
        valid_cases += 1; assert truth(s["route_valid"]) and s["status"] == "READY"
        assert int(s["covered_tasks"]) == int(s["verified_tasks"]) == int(s["target_count"]) == len(tasks)
        ds = sorted(filter(match, docks), key=lambda r: int(r["visit_rank"]))
        aa = list(filter(match, assignments)); ps = sorted(filter(match, paths), key=lambda r: int(r["point_id"]))
        assert len(ds) == int(s["docks"]) and len(aa) == len(tasks)
        assert {int(r["task_id"]) for r in aa} == set(range(1, len(tasks)+1))
        assert [int(r["visit_rank"]) for r in ds] == list(range(1, len(ds)+1))
        for r in aa:
            support = pair_lookup[mode][int(r["candidate_id"]), int(r["task_id"])]
            assert truth(support["feasible"]) and r["arm"] == support["assigned_arm"]
            d = next(d for d in ds if d["dock_id"] == r["dock_id"]); assert d["candidate_id"] == r["candidate_id"]
        for d in ds:
            assert pose(d) == pose(pool_by_id[int(d["candidate_id"])])
            assert set(d["covered_task_ids"].split("|")) == {r["task_id"] for r in aa if r["dock_id"] == d["dock_id"]}
        route = (1,)+tuple(int(d["candidate_id"])+1 for d in ds); routes[mode, method] = route
        sets[mode, method] = set(route[1:])
        minimum = sum(c["minimum_docks"] for c in minima[mode].values())
        assert int(s["minimum_docks"]) == minimum and len(ds) >= minimum
        if method != "staged":
            supported_route(mode, route); assert len(ds) == minimum
            assert abs(route_cost(route)-planned) < TOL
        for endpoint in (ps[0], ps[-1]):
            assert math.dist(pose(endpoint)[:2], scene["start_pose"][:2]) < TOL
            assert abs(wrap(pose(endpoint)[2]-scene["start_pose"][2])) < TOL
        assert [int(r["point_id"]) for r in ps] == list(range(1, len(ps)+1))
        length = sum(math.dist(pose(a)[:2], pose(b)[:2]) for a, b in zip(ps, ps[1:]))
        assert abs(length-float(s["route_length_m"])) < TOL
        ordered = [tuple(scene["start_pose"])]+[pose(d) for d in ds]+[tuple(scene["start_pose"])]
        yaw = sum(abs(wrap(b[2]-a[2])) for a, b in zip(ordered, ordered[1:]))
        value = length+lam*yaw
        assert abs(value-float(s["navigation_cost"])) < TOL and abs(route_cost(route)-value) < TOL
        cursor = 0
        for d in ds:
            target = pose(d)
            hit = next((i for i in range(cursor, len(ps)) if math.dist(pose(ps[i])[:2], target[:2]) < TOL
                        and abs(wrap(pose(ps[i])[2]-target[2])) < TOL), None)
            assert hit is not None; cursor = hit
        for r in ps:
            poly = corners(*pose(r), fp["length_m"], fp["width_m"])
            assert all(.1-TOL <= x <= 9.9+TOL and .1-TOL <= y <= 9.9+TOL for x, y in poly)
            assert all(separated_from_table(poly, box) for box in boxes)
        if method == "minimum_seed":
            assert abs(float(s["minimum_seed_cost"])-value) < TOL
            assert abs(float(s["input_seed_cost"])-case_details[f"{mode}/staged"]["navigation_cost_recomputed"]) < TOL
        elif method in {"reorder_only", "joint_local"}:
            assert abs(float(s["input_seed_cost"])-case_details[f"{mode}/staged"]["navigation_cost_recomputed"]) < TOL
            assert abs(float(s["minimum_seed_cost"])-case_details[f"{mode}/minimum_seed"]["navigation_cost_recomputed"]) < TOL
            assert value <= float(s["minimum_seed_cost"])+TOL
            if method == "reorder_only":
                assert sets[mode, method] == sets[mode, "minimum_seed"]
            if events:
                assert previous_new == route
            else:
                assert route == routes[mode, "minimum_seed"]
        case_details[key] = {"mission_valid": True, "task_count": len(aa), "docks": len(ds),
                             "minimum_docks": minimum, "route_length_recomputed_m": length,
                             "navigation_cost_recomputed": value, "path_samples_checked": len(ps),
                             "accepted_moves_checked": len(events), "termination_reason": s["termination_reason"],
                             "local_optimum_certified": local_flag, "global_optimal": False}
    status = "PARTIAL_PASS" if allow_partial and state["status"] != "COMPLETE" else ("PASS" if not failures else "PASS_WITH_RECORDED_FAILURES")
    seed_recovery_checks = {}
    if frozen_replay is not None:
        seed_records = json.loads((folder / "navigation_seed_recovery.json").read_text(encoding="utf-8-sig"))
        if isinstance(seed_records, dict):
            seed_records = [seed_records]
        seed_keys = {(r["mode"], r["method"]) for r in seed_records}
        assert len(seed_keys) == len(seed_records)
        for record in seed_records:
            mode, method = record["mode"], record["method"]
            key, recovery = f"{mode}/{method}", record["recovery"]
            if not case_details[key]["mission_valid"]:
                seed_recovery_checks[key] = {"route_order_source": record["route_order_source"],
                                             "mission_valid": False,
                                             "note": "Failure retained; not used for comparable-cost claims"}
                continue
            if not recovery["enabled"]:
                assert record["route_order_source"] == "strategy_preferred"
                assert recovery["status"] == "not_needed"
                seed_recovery_checks[key] = {"route_order_source": "strategy_preferred"}
                continue
            matrix = rows(folder / f"seed_navigation_costs_{mode}_{method}.csv")
            n = int(math.isqrt(len(matrix))); assert n*n == len(matrix)
            D, local_to_node = {}, {}
            for edge in matrix:
                a, b = int(edge["from_local"]), int(edge["to_local"])
                assert 1 <= a <= n and 1 <= b <= n and (a, b) not in D
                value = float(edge["navigation_cost"])
                assert not math.isnan(value) and value >= 0
                assert a != b or value == 0
                D[a, b] = value
                for local, node in ((a, int(edge["from_node"])), (b, int(edge["to_node"]))):
                    assert local not in local_to_node or local_to_node[local] == node
                    local_to_node[local] = node
                global_pair = int(edge["from_node"]), int(edge["to_node"])
                if global_pair in costs:
                    exported = costs[global_pair]
                    assert (math.isinf(exported) and math.isinf(value)) or abs(exported-value) < TOL
            assert local_to_node[1] == 1 and len(set(local_to_node.values())) == n
            selected = sorted((r for r in docks if r["mode"] == mode and r["method"] == method),
                              key=lambda r: int(r["dock_id"]))
            assert [local_to_node[i] for i in range(2, n+1)] == [int(r["candidate_id"])+1 for r in selected]
            attempts = recovery["attempts"]
            if isinstance(attempts, dict):
                attempts = [attempts]
            expected_attempts, chosen, chosen_cost, first_finite = [], None, None, False
            for start in range(1, n+1):
                route, unvisited = [start], set(range(1, n+1))-{start}
                while unvisited:
                    nxt = min(unvisited, key=lambda j: (D[route[-1], j], j))
                    route.append(nxt); unvisited.remove(nxt)
                at = route.index(1); route = route[at:]+route[:at]
                closed = route+[1]
                edge_costs = [D[a, b] for a, b in zip(closed, closed[1:])]
                finite = all(math.isfinite(v) for v in edge_costs)
                if start == 1:
                    first_finite = finite
                expected_attempts.append((start, route, closed, finite, sum(edge_costs)))
                if finite:
                    chosen, chosen_cost = route, sum(edge_costs)
                    break
            assert chosen is not None and len(attempts) == len(expected_attempts) == recovery["attempt_count"]
            for actual, (start, route, closed, finite, value) in zip(attempts, expected_attempts):
                assert actual["start"] == start and actual["visit_order"] == route and actual["closed_order"] == closed
                assert actual["closed_finite"] == actual["route_finite"] == finite
                if finite:
                    assert abs(actual["cost"]-value) < TOL
                else:
                    assert actual["cost"] is None or math.isinf(actual["cost"])
            selected_start = expected_attempts[-1][0]
            assert recovery["return_to_start"] and not recovery["two_opt_applied"]
            assert recovery["selected_start"] == selected_start and recovery["recovered"] == (selected_start != 1)
            assert recovery["initial_closed_finite"] == first_finite
            expected_source = "nearest_neighbor_finite_restart" if selected_start != 1 else "nearest_neighbor"
            assert record["route_order_source"] == recovery["route_order_source"] == expected_source
            assert tuple(local_to_node[i] for i in chosen) == routes[mode, method]
            assert abs(chosen_cost-case_details[key]["navigation_cost_recomputed"]) < TOL
            seed_recovery_checks[key] = {"route_order_source": expected_source,
                                         "initial_closed_finite": first_finite,
                                         "first_finite_start": selected_start,
                                         "attempts_independently_reproduced": len(attempts),
                                         "selected_set_preserved": True}
        assert all((s["mode"], s["method"]) in seed_keys for s in summary if truth(s["mission_valid"]))
    certificate_output = {m: {str(t): {k: v for k, v in cert.items() if k not in {"masks", "full_mask"}}
                              for t, cert in group.items()} for m, group in minima.items()}
    source_names = ("experiments/main_task_navigation_setup.m", "experiments/build_main_task_navigation_library.m",
                    "experiments/exp_main_task_navigation.m", "manipulator/evaluate_task_reachability_cached.m",
                    "planning_strategy/joint/plan_task_navigation_joint_local.m",
                    "global_planner/compute_path_cost_matrix.m", "global_planner/finite_nearest_neighbor_order.m",
                    "pipeline/stage3_global_planning.m")
    report = {"status": status, "run_state": state, "completed_modes_checked": completed_modes,
              "frozen_evidence_replay": frozen_replay,
              "common_seed_recovery_checks": seed_recovery_checks,
              "cases_checked": len(case_details), "valid_missions": valid_cases, "recorded_failures": failures,
              "main_geometry": {"tables": 16, "map_m": [10, 10], "tasks": len(tasks),
                                "dense_candidates": len(dense), "common_subpool_candidates": len(pool),
                                "geometric_fps_and_yaw_rule_checked": True},
              "minimum_cover_certificates": certificate_output, "cases": case_details,
              "ik_cache_diagnostics": {m: {k: p[k] for k in ("request_count", "positive_hits", "exact_negative_hits", "misses",
                  "witness_fallbacks", "direct_checks", "direct_disagreements", "witness_direct_search_failures",
                  "max_position_error_m", "max_orientation_error_rad")} for m, p in cache_proofs.items()},
              "checks": ["Default scene configuration and all 36 original-location task IDs (fresh default-build equality is separately covered by protocol tests)",
                         "26660-to-906 child mapping, original coordinates/yaws, geometric FPS/yaw selection and shared candidate identity",
                         "Every same-table pair preserved, cache counter accounting and reported residual bounds",
                         "Independent regional DP and enumeration of all minimum covers",
                         "Exactly-once supported task/arm assignments and fixed repaired-set reorder control",
                         "Every accepted replacement/reversal repriced using exported directed edges with strict descent",
                         "Actual route length/cost, dock/yaw visits, start return and sampled physical base/table/border separation",
                         "For corrected replays: original geometry/support/library hashes and independently reproduced first-finite fixed-set NN restarts",
                         "NaN comparable costs for failed missions, budget/local-optimum flags and no global-optimal claim"],
              "limits": "No independent IK solving, RNG reimplementation, continuous/swept arm/base collision, contact-force or physical validation. Minimum counts apply to the shared 906-candidate regional support subpool, not the full 26660-candidate pool. Local search is not globally optimal; recorded failure cases and fixed-set controls are not silently removed. Search callback counts exclude other stage3 navigation calls.",
              "source_sha256": {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in source_names},
              "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (folder / "independent_audit.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", nargs="?", type=Path, default=ROOT / "results/main_joint_task_navigation_20261111")
    parser.add_argument("--allow-partial", action="store_true")
    result = audit(**vars(parser.parse_args()))
    print(json.dumps({k: result[k] for k in ("status", "cases_checked", "valid_missions", "completed_modes_checked")}))
