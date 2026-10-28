"""Failure-aware, paired analysis of this frozen evidence run (stdlib only).

Uses current_evidence_path.txt beside this script unless --evidence-dir is
explicitly supplied. A discarded partial run is never selected implicitly.
Run after run_manifest.json says complete:
    python analyze_repair_evidence.py
For a labelled provisional snapshot only: add --allow-incomplete
Synthetic contract checks, without reading research data: add --self-test

No production modules are imported or edited. NaN/Inf/missing values become
JSON null; missing measurements are never replaced with zero. All ratios of
time/path use pairs with two successful, certified routes and matching scope.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path


VARIANTS = (
    "baseline", "lateral_coarse", "lateral_fine", "normal_coarse", "normal_fine",
    "yaw_coarse", "yaw_fine", "grid_coarse", "grid_fine", "cost_reach_half",
    "cost_reach_double", "cost_coverage_half", "cost_coverage_double",
    "selection_coverage_half", "selection_coverage_double",
)
PROFILES = ("C0", "C1", "C2", "C3")
EVENTS = ("ADD_LOCAL_OBSTACLE", "INVALIDATE_DOCK", "ADD_TASKS")
RAW_FILES = {
    "main": "main/main_raw.csv",
    "sensitivity": "sensitivity/sensitivity_raw.csv",
    "complex": "complex/complex_raw.csv",
    "dynamic": "dynamic/dynamic_raw.csv",
}
UNKNOWN = {"", "nan", "<missing>", "null", "none", "na", "n/a"}
PROTOCOL_KEYS = ("threshold", "return_to_start", "global_planner", "routing_mode", "sparse_k")
AUDIT_CLASSES = (
    "covered_in_library", "library_coverage_mismatch", "candidate_sampling_gap",
    "hard_vertical_certificate", "not_found_in_audited_samples",
)
NUMERIC_METRICS = (
    "candidate_coverage_percent", "geometric_coverage_percent", "independent_coverage_percent",
    "simulated_geometric_completion_percent", "candidate_count", "selected_docks",
    "path_length_m", "planning_time_s", "replanning_time_s", "wall_time_s",
    "workflow_wall_time_s", "certification_time_s", "path_queries",
)


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def truth(value):
    return str(value).strip().lower() in {"true", "1", "yes"}


def unknown(value):
    return str(value if value is not None else "").strip().lower() in UNKNOWN


def describe(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"available_n": 0, "mean": None, "median": None, "min": None, "max": None}
    return {"available_n": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "min": min(values), "max": max(values)}


def successful(row):
    allowed = {"SUCCESS", "NOOP"} if row.get("_study") == "dynamic" else {"SUCCESS"}
    return (row.get("status") in allowed and truth(row.get("path_safe"))
            and (row.get("_study") != "dynamic" or truth(row.get("certification_checked"))))


def key(row):
    study = row["_study"]
    if study == "dynamic":
        return (row.get("scene_id", ""), row.get("seed", ""),
                row.get("event_type", ""), row.get("mode", ""))
    return (row.get("scene_id", ""), row.get("seed", ""), row.get("variant", ""))


def same_protocol(a, b):
    mismatches = []
    for name in PROTOCOL_KEYS:
        av, bv = a.get(name), b.get(name)
        if name in {"threshold", "sparse_k"}:
            av, bv = number(av), number(bv)
        elif name == "return_to_start":
            av, bv = truth(av), truth(bv)
        if av is None or bv is None or av != bv:
            mismatches.append(name)
    return mismatches


def differences(a, b, metric):
    """b-a and b/a, only with finite measurements (strictly positive divisor)."""
    av, bv = number(a.get(metric)), number(b.get(metric))
    if av is None or bv is None:
        return {"delta": None, "ratio": None, "delta_percent": None}
    return {"delta": bv-av, "ratio": bv/av if av > 0 else None,
            "delta_percent": 100*(bv-av)/av if av > 0 else None}


def read_csv(path, evidence_root, inventory, issues):
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    relative = path.relative_to(evidence_root).as_posix()
    stable = before.st_size == after.st_size and before.st_mtime_ns == after.st_mtime_ns
    inventory.append({"path": relative, "sha256": hashlib.sha256(data).hexdigest(),
                      "bytes": len(data), "stable_during_read": stable})
    if not stable:
        issues.append({"kind": "file_changed_during_read", "file": relative})
    reader = csv.DictReader(io.StringIO(data.decode("utf-8-sig")))
    rows = []
    for index, row in enumerate(reader, 2):
        if None in row or any(v is None for v in row.values()):
            issues.append({"kind": "malformed_csv_row", "file": relative, "line": index})
            continue
        row["_source"] = relative
        row["_line"] = index
        rows.append(row)
    return rows


def aggregate_groups(rows):
    groups = defaultdict(list)
    for row in rows:
        group = (row["_study"], row.get("scene_id", ""), row.get("variant", ""),
                 row.get("event_type", ""), row.get("mode", ""))
        groups[group].append(row)
    result = []
    for group, members in sorted(groups.items()):
        ok = [r for r in members if successful(r)]
        failed = [r for r in members if not successful(r)]
        entry = dict(zip(("study", "scene_id", "variant", "event_type", "mode"), group))
        entry.update(attempted_n=len(members), successful_n=len(ok), failed_n=len(failed),
                     success_percent=100*len(ok)/len(members),
                     status_counts=dict(Counter(r.get("status", "") for r in members)),
                     failure_reasons=dict(Counter(r.get("failure_reason", "") or "unspecified"
                                                  for r in failed)),
                     timing_scope_counts=dict(Counter(r.get("timing_scope", "") for r in members)))
        entry["metrics"] = {}
        for metric in NUMERIC_METRICS:
            if not any(metric in r for r in members):
                continue
            entry["metrics"][metric] = {
                "all_attempts_available": describe(number(r.get(metric)) for r in members),
                "successful_only": describe(number(r.get(metric)) for r in ok),
                "failed_only_available": describe(number(r.get(metric)) for r in failed),
            }
        result.append(entry)
    return result


def sensitivity_pairs(rows, issues):
    baseline = defaultdict(list)
    for row in rows:
        if row.get("variant") == "baseline":
            baseline[(row.get("scene_id"), row.get("seed"))].append(row)
    output = []
    for row in rows:
        if row.get("variant") == "baseline":
            continue
        refs = baseline[(row.get("scene_id"), row.get("seed"))]
        pair = {"scene_id": row.get("scene_id"), "seed": row.get("seed"),
                "variant": row.get("variant"), "variant_status": row.get("status"),
                "both_successful": False, "eligible_for_time_path_comparison": False}
        if len(refs) != 1:
            pair["exclusion_reason"] = "missing_or_duplicate_same_seed_baseline"
            output.append(pair)
            continue
        ref = refs[0]
        geometry_ok = bool(row.get("task_geometry_signature")) and (
            row.get("task_geometry_signature") == ref.get("task_geometry_signature"))
        config_diff = same_protocol(ref, row)
        source_ok = bool(ref.get("implementation_signature")) and (
            ref.get("implementation_signature") == row.get("implementation_signature"))
        pair.update(baseline_status=ref.get("status"), geometry_signature_matches=geometry_ok,
                    protocol_mismatches=config_diff, implementation_signatures_match=source_ok,
                    baseline_successful=successful(ref), variant_successful=successful(row),
                    both_successful=successful(ref) and successful(row))
        pair["coverage_percent_points"] = differences(ref, row, "geometric_coverage_percent")["delta"]
        reasons = []
        if not geometry_ok:
            reasons.append("task_table_geometry_identity_missing_or_mismatched")
        if config_diff:
            reasons.append("protocol_mismatch")
        if not source_ok:
            reasons.append("implementation_identity_missing_or_mismatched")
        if not pair["both_successful"]:
            reasons.append("not_both_successful_certified_routes")
        if not ref.get("timing_scope") or ref.get("timing_scope") != row.get("timing_scope"):
            reasons.append("timing_scope_mismatch")
        pair["eligible_for_time_path_comparison"] = not reasons
        pair["exclusion_reason"] = ";".join(reasons)
        if not reasons:
            for metric in ("planning_time_s", "path_length_m", "selected_docks", "candidate_count"):
                pair[metric] = differences(ref, row, metric)
            bt, vt = number(ref.get("planning_time_s")), number(row.get("planning_time_s"))
            pair["baseline_over_variant_time_ratio"] = bt/vt if bt is not None and vt and vt > 0 else None
        if not geometry_ok or config_diff or not source_ok:
            issues.append({"kind": "invalid_sensitivity_pair_identity", "seed": row.get("seed"),
                           "variant": row.get("variant"), "reasons": reasons})
        output.append(pair)
    return output


def dynamic_pairs(rows, issues):
    grouped = defaultdict(lambda: defaultdict(list))
    for row in rows:
        grouped[(row.get("scene_id"), row.get("seed"), row.get("event_type"))][row.get("mode")].append(row)
    output = []
    for group, modes in sorted(grouped.items()):
        pair = dict(zip(("scene_id", "seed", "event_type"), group))
        pair.update(both_successful=False, eligible_for_time_path_comparison=False)
        if len(modes["LOCAL_REPLAN"]) != 1 or len(modes["FULL_REPLAN"]) != 1:
            pair["exclusion_reason"] = "missing_or_duplicate_local_full_row"
            output.append(pair)
            continue
        local, full = modes["LOCAL_REPLAN"][0], modes["FULL_REPLAN"][0]
        pair.update(local_status=local.get("status"), full_status=full.get("status"),
                    local_successful=successful(local), full_successful=successful(full),
                    both_successful=successful(local) and successful(full))
        config_diff = same_protocol(local, full)
        source_ok = bool(local.get("implementation_signature")) and (
            local.get("implementation_signature") == full.get("implementation_signature"))
        try:
            event_matches = (bool(local.get("event_geometry_json")) and
                             json.loads(local["event_geometry_json"]) == json.loads(full["event_geometry_json"]))
        except (ValueError, KeyError):
            event_matches = False
        timing_matches = (bool(local.get("timing_scope")) and
                          local.get("timing_scope") == full.get("timing_scope") and
                          local.get("certification_scope") == full.get("certification_scope"))
        pair.update(event_identity_matches=event_matches, protocol_mismatches=config_diff,
                    implementation_signatures_match=source_ok,
                    timing_scope_matches=timing_matches,
                    path_query_scope_matches=local.get("path_query_scope") == full.get("path_query_scope"))
        reasons = []
        if not pair["both_successful"]:
            reasons.append("not_both_successful_certified_routes")
        if config_diff or not event_matches:
            reasons.append("protocol_or_event_identity_mismatch")
        if not source_ok:
            reasons.append("implementation_identity_missing_or_mismatched")
        if not timing_matches:
            reasons.append("timing_scope_mismatch")
        lt, ft = number(local.get("replanning_time_s")), number(full.get("replanning_time_s"))
        ll, fl = number(local.get("path_length_m")), number(full.get("path_length_m"))
        if any(v is None or v <= 0 for v in (lt, ft, ll, fl)):
            reasons.append("missing_or_nonpositive_time_or_length")
        pair["eligible_for_time_path_comparison"] = not reasons
        pair["exclusion_reason"] = ";".join(reasons)
        if not reasons:
            pair.update(local_replanning_time_s=lt, full_replanning_time_s=ft,
                        local_path_length_m=ll, full_path_length_m=fl,
                        full_over_local_time_ratio=ft/lt,
                        local_minus_full_time_s=lt-ft,
                        local_over_full_path_ratio=ll/fl,
                        local_minus_full_path_m=ll-fl,
                        local_minus_full_path_percent=100*(ll-fl)/fl)
            faster, no_longer = lt <= ft, ll <= fl + 1e-9
            pair["tradeoff"] = ("faster_or_equal_and_no_longer" if faster and no_longer else
                                "faster_or_equal_but_longer" if faster else
                                "slower_but_no_longer" if no_longer else "slower_and_longer")
        if config_diff or not event_matches or not timing_matches or not source_ok:
            issues.append({"kind": "invalid_dynamic_pair_identity_or_scope", **pair})
        output.append(pair)
    return output


def pair_aggregates(pairs, group_name, dynamic=False):
    grouped = defaultdict(list)
    for pair in pairs:
        grouped[pair.get(group_name, "")].append(pair)
    result = []
    for name, members in sorted(grouped.items()):
        valid = [p for p in members if p.get("eligible_for_time_path_comparison")]
        entry = {group_name: name, "observed_pair_n": len(members),
                 "both_successful_n": sum(p.get("both_successful", False) for p in members),
                 "eligible_time_path_pair_n": len(valid), "excluded_pair_n": len(members)-len(valid),
                 "exclusion_reasons": dict(Counter(p.get("exclusion_reason") for p in members
                                                    if p.get("exclusion_reason")))}
        if dynamic:
            for metric in ("full_over_local_time_ratio", "local_minus_full_time_s",
                           "local_over_full_path_ratio", "local_minus_full_path_m",
                           "local_minus_full_path_percent"):
                entry[metric] = describe(p.get(metric) for p in valid)
            entry["tradeoff_counts"] = dict(Counter(p["tradeoff"] for p in valid))
            entry["pooled_full_over_local_time_ratio"] = (sum(p["full_replanning_time_s"] for p in valid) /
                sum(p["local_replanning_time_s"] for p in valid)) if valid else None
        else:
            entry["baseline_over_variant_time_ratio"] = describe(p.get("baseline_over_variant_time_ratio") for p in valid)
            for metric in ("planning_time_s", "path_length_m", "selected_docks", "candidate_count"):
                entry[metric+"_delta"] = describe(p.get(metric, {}).get("delta") for p in valid)
            entry["coverage_percent_points_all_available_pairs"] = describe(p.get("coverage_percent_points") for p in members)
        result.append(entry)
    return result


def audit_case_identity(path, all_rows):
    filename = path.name.removesuffix("_task_audit.csv")
    study = path.parts[0]
    if study == "dynamic":
        match = re.fullmatch(r"seed(\d+)", filename)
        seed = match.group(1) if match else ""
        scenes = {r.get("scene_id") for r in all_rows if r["_study"] == "dynamic" and r.get("seed") == seed}
        return {"study": "dynamic_base", "seed": seed,
                "scene_id": next(iter(scenes)) if len(scenes) == 1 else "", "variant": "baseline"}
    if study == "sensitivity":
        match = re.fullmatch(r"seed(\d+)_(.+)", filename)
        seed, variant = match.groups() if match else ("", "")
        refs = [r for r in all_rows if r["_study"] == study and r.get("seed") == seed and r.get("variant") == variant]
    else:
        match = re.fullmatch(r"(.+)_seed(\d+)", filename)
        scene, seed = match.groups() if match else ("", "")
        refs = [r for r in all_rows if r["_study"] == study and r.get("seed") == seed and r.get("scene_id") == scene]
    ref = refs[0] if len(refs) == 1 else {}
    return {"study": study, "seed": seed, "scene_id": ref.get("scene_id", ""),
            "variant": ref.get("variant", ""), "raw_row_matched": len(refs) == 1}


def audit_checks(root, all_rows, inventory, issues):
    cases, task_rows = [], []
    # Only these current, authoritative study roots belong to the core
    # matrix. Interrupted archives and separately timed supplements may
    # live beside them and must not contribute duplicate/invalid evidence.
    for path in authoritative_audit_paths(root):
        relative = path.relative_to(root)
        rows = read_csv(path, root, inventory, issues)
        info = audit_case_identity(relative, all_rows)
        counts = Counter(r.get("classification", "") for r in rows)
        entry = {**info, "file": relative.as_posix(), "task_case_observations": len(rows),
                 "classification_counts": dict(counts),
                 "library_coverage_mismatch_count": sum(truth(r.get("library_coverage_mismatch")) for r in rows),
                 "sampling_gap_count": sum(truth(r.get("candidate_sampling_gap")) for r in rows),
                 "hard_vertical_certificate_count": sum(truth(r.get("hard_vertical_certificate")) for r in rows),
                 "supported_count": sum(truth(r.get("audited_geometric_supported")) for r in rows),
                 "dense_samples": sum(number(r.get("dense_sample_count")) or 0 for r in rows),
                 "dense_collision_checks": sum(number(r.get("dense_collision_checks")) or 0 for r in rows)}
        ids = [r.get("task_id") for r in rows]
        if len(ids) != len(set(ids)):
            issues.append({"kind": "duplicate_audit_task_ids", "file": relative.as_posix()})
        for label in counts:
            if label not in AUDIT_CLASSES:
                issues.append({"kind": "unknown_audit_classification", "file": relative.as_posix(), "value": label})
        refs = [r for r in all_rows if r["_study"] == info["study"] and
                r.get("seed") == info["seed"] and r.get("scene_id") == info["scene_id"] and
                r.get("variant") == info["variant"]]
        if len(refs) == 1:
            ref = refs[0]
            for observed, expected_field in ((len(rows), "task_count"),
                    (entry["library_coverage_mismatch_count"], "independent_mismatch_count"),
                    (entry["sampling_gap_count"], "sampling_gap_count")):
                expected = number(ref.get(expected_field))
                if expected is not None and observed != expected:
                    issues.append({"kind": "audit_csv_summary_mismatch", "file": relative.as_posix(),
                                   "field": expected_field, "observed": observed, "expected": expected})
        cases.append(entry)
        task_rows.extend(rows)
    audit_rows = [r for r in all_rows if r.get("audit_status") == "PASS"]
    missing_pass_artifacts = []
    for row in audit_rows:
        study, seed = row["_study"], row.get("seed", "")
        if study == "sensitivity":
            filename = f"seed{seed}_{row.get('variant', '')}_task_audit.csv"
        else:
            filename = f"{row.get('scene_id', '')}_seed{seed}_task_audit.csv"
        relative = f"{study}/cases/{filename}"
        if not (root/relative).is_file():
            missing_pass_artifacts.append(relative)
            issues.append({"kind": "missing_pass_audit_csv", "file": relative})
    task_mismatches = [number(r.get("independent_mismatch_count")) for r in audit_rows]
    pair_mismatches = [number(r.get("independent_pair_mismatch_count")) for r in audit_rows]
    class_totals = Counter()
    for case in cases:
        class_totals.update(case["classification_counts"])
    total_task_mismatch = sum(c["library_coverage_mismatch_count"] for c in cases)
    return {"case_files_n": len(cases), "task_case_observations": len(task_rows),
            "observation_note": "Repeated tasks across variants/seeds are separate task-case observations, not unique physical tasks.",
            "classification_counts": dict(class_totals), "cases": cases,
            "raw_audit_status_counts": dict(Counter(r.get("audit_status", "NOT_APPLICABLE") for r in all_rows
                                                    if r["_study"] != "dynamic")),
            "checked_raw_audits_n": len(audit_rows),
            "missing_pass_audit_csvs": missing_pass_artifacts,
            "zero_task_mismatches_in_available_task_csvs": bool(cases) and total_task_mismatch == 0,
            "task_mismatch_count_in_available_csvs": total_task_mismatch,
            "zero_candidate_pair_mismatches_in_reported_audits": bool(pair_mismatches) and
                all(v == 0 for v in pair_mismatches),
            "raw_task_mismatch_count": sum(task_mismatches) if task_mismatches and None not in task_mismatches else None,
            "raw_candidate_pair_mismatch_count": sum(pair_mismatches) if pair_mismatches and None not in pair_mismatches else None,
            "task_ik_nonunknown_n": sum(not unknown(r.get("ik_feasible")) for r in task_rows),
            "task_actual_completion_nonunknown_n": sum(not unknown(r.get("actual_completed")) for r in task_rows),
            "missing_task_evidence_columns_n": sum("ik_feasible" not in r or
                 "actual_completed" not in r for r in task_rows)}


def authoritative_audit_paths(root):
    return sorted(path for study in RAW_FILES for path in
                  (root/study).rglob("*_task_audit.csv") if path.is_file())


def completeness(rows_by_study, manifest):
    seeds = [str(s) for s in manifest.get("seeds", [])]
    expected = {
        "main": {("MAIN", s, "baseline") for s in seeds},
        "sensitivity": {("SENS4", s, v) for s in seeds for v in VARIANTS},
        "complex": {(p, s, "fixed_protocol") for s in seeds for p in PROFILES},
        "dynamic": {("MAIN4", s, e, m) for s in seeds for e in EVENTS for m in ("LOCAL_REPLAN", "FULL_REPLAN")},
    }
    result = {}
    for study, rows in rows_by_study.items():
        observed = {key(r) for r in rows}
        result[study] = {"expected_n": len(expected[study]), "observed_rows_n": len(rows),
                         "unique_rows_n": len(observed),
                         "missing_expected_keys": sorted(expected[study]-observed),
                         "unexpected_keys": sorted(observed-expected[study])}
    return {"manifest_complete": truth(manifest.get("complete")), "expected_seeds": seeds,
            "expected_counts_basis": "Frozen run_repair_evidence_suite matrix: 3 MAIN + 45 sensitivity + 12 complex + 18 dynamic = 78 with 3 default seeds; warmup excluded.",
            "studies": result,
            "expected_total_rows": sum(v["expected_n"] for v in result.values()),
            "observed_total_rows": sum(v["observed_rows_n"] for v in result.values()),
            "all_expected_rows_present": bool(seeds) and all(not v["missing_expected_keys"] and
                not v["unexpected_keys"] and v["observed_rows_n"] == v["expected_n"] for v in result.values())}


def flatten(value, prefix=""):
    result = {}
    for name, item in value.items():
        field = prefix+name
        if isinstance(item, dict):
            result.update(flatten(item, field+"."))
        elif isinstance(item, (list, tuple)):
            result[field] = json.dumps(item, ensure_ascii=False, allow_nan=False)
        else:
            result[field] = item
    return result


def write_outputs(root, summary):
    target = root/"analysis_summary.json"
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
    temporary.replace(target)
    tables = []
    for section, label in (("groups", "failure_aware_group"), ("sensitivity_pairs", "sensitivity_pair"),
            ("sensitivity_paired_summary", "sensitivity_paired_summary"), ("dynamic_pairs", "dynamic_pair"),
            ("dynamic_paired_summary", "dynamic_paired_summary")):
        tables.extend({"record_type": label, **flatten(row)} for row in summary[section])
    tables.extend({"record_type": "audit_case", **flatten(row)} for row in summary["audit"]["cases"])
    fields = ["record_type"]+sorted({k for r in tables for k in r if k != "record_type"})
    temporary = root/"analysis_tables.csv.tmp"
    with temporary.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(tables)
    temporary.replace(root/"analysis_tables.csv")


def self_test():
    def row(study, status="SUCCESS", **changes):
        return {"_study": study, "scene_id": "SENS4", "seed": "2026", "variant": "baseline",
                "status": status, "path_safe": "true", "threshold": ".3", "return_to_start": "true",
                "global_planner": "convex_astar", "routing_mode": "sparse_knn", "sparse_k": "6",
                "timing_scope": "common", "task_geometry_signature": "same", "planning_time_s": "10",
                "implementation_signature": "synthetic_source",
                "replanning_time_s": "10", "path_length_m": "20", "geometric_coverage_percent": "100",
                "certification_checked": "true", "certification_scope": "external",
                "event_geometry_json": '{"type":"ADD_LOCAL_OBSTACLE"}', "path_query_scope": "requests",
                **changes}
    failed = row("sensitivity", "BLOCKED_COVERAGE", geometric_coverage_percent="80", path_length_m="NaN")
    group = aggregate_groups([row("sensitivity"), failed])[0]
    assert group["success_percent"] == 50
    assert group["metrics"]["path_length_m"]["successful_only"]["available_n"] == 1
    assert group["metrics"]["geometric_coverage_percent"]["all_attempts_available"]["mean"] == 90
    pairs = sensitivity_pairs([row("sensitivity"), row("sensitivity", variant="fine", planning_time_s="5")], [])
    assert pairs[0]["baseline_over_variant_time_ratio"] == 2
    pairs = sensitivity_pairs([row("sensitivity"), row("sensitivity", "ERROR", variant="fine")], [])
    assert not pairs[0]["eligible_for_time_path_comparison"]
    local = row("dynamic", mode="LOCAL_REPLAN", event_type="ADD_LOCAL_OBSTACLE", replanning_time_s="2", path_length_m="22")
    full = row("dynamic", mode="FULL_REPLAN", event_type="ADD_LOCAL_OBSTACLE", replanning_time_s="8")
    pair = dynamic_pairs([local, full], [])[0]
    assert pair["full_over_local_time_ratio"] == 4 and pair["tradeoff"] == "faster_or_equal_but_longer"
    assert not dynamic_pairs([local, {**full, "status": "ERROR"}], [])[0]["eligible_for_time_path_comparison"]
    source_issues = []
    assert not sensitivity_pairs([row("sensitivity"), row("sensitivity", variant="fine", implementation_signature="changed")], source_issues)[0]["eligible_for_time_path_comparison"]
    assert source_issues
    source_issues = []
    assert not dynamic_pairs([local, {**full, "implementation_signature": "changed"}], source_issues)[0]["eligible_for_time_path_comparison"]
    assert source_issues
    assert number("NaN") is None and number("Inf") is None and unknown("NaN")
    assert json.loads(json.dumps(group, allow_nan=False))["success_percent"] == 50
    import tempfile
    with tempfile.TemporaryDirectory(prefix="repair_analysis_selftest_") as directory:
        tmp = Path(directory)
        (tmp/"tiny.csv").write_text("status,ik_feasible\nSUCCESS,NaN\nERROR,NaN\n", encoding="utf-8-sig")
        inventory, issues = [], []
        read = read_csv(tmp/"tiny.csv", tmp, inventory, issues)
        assert len(read) == 2 and not issues and inventory[0]["stable_during_read"]
        sample = {"groups": [group], "sensitivity_pairs": pairs,
                  "sensitivity_paired_summary": [], "dynamic_pairs": [pair],
                  "dynamic_paired_summary": [], "audit": {"cases": []}}
        write_outputs(tmp, sample)
        assert json.loads((tmp/"analysis_summary.json").read_text())["groups"][0]["success_percent"] == 50
        with (tmp/"analysis_tables.csv").open(encoding="utf-8-sig", newline="") as handle:
            assert len(list(csv.DictReader(handle))) == 3
        current = tmp/"main/cases/MAIN_seed2026_task_audit.csv"
        current.parent.mkdir(parents=True)
        current.write_text("task_id,classification,ik_feasible,actual_completed\n1,covered_in_library,NaN,NaN\n", encoding="utf-8-sig")
        for discarded in ("dynamic_interrupted_20261028/base", "weight_completion_supplement/cases"):
            archive = tmp/discarded/"seed2026_task_audit.csv"
            archive.parent.mkdir(parents=True)
            archive.write_text("task_id,classification\n1,THIS_MUST_NOT_BE_READ\n", encoding="utf-8-sig")
        audit_inventory, audit_issues = [], []
        audit = audit_checks(tmp, [], audit_inventory, audit_issues)
        assert authoritative_audit_paths(tmp) == [current]
        assert audit["case_files_n"] == 1 and audit["task_case_observations"] == 1
        assert not audit_issues and len(audit_inventory) == 1
    print("analysis self-test: 12 synthetic contracts passed (including pair source identity, authoritative-root exclusion and temporary CSV/JSON roundtrip); no research CSV read or output written")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-incomplete", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--evidence-dir", type=Path, help="Explicit evidence directory override; incomplete runs still require --allow-incomplete.")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    script_dir = Path(__file__).resolve().parent
    pointer = script_dir/"current_evidence_path.txt"
    default_root = script_dir if (script_dir/"run_manifest.json").is_file() else script_dir/"evidence_20261027_221823"
    root = args.evidence_dir if args.evidence_dir else (
        Path(pointer.read_text(encoding="utf-8-sig").strip()) if pointer.is_file() else default_root)
    root = root.resolve()
    manifest = json.loads((root/"run_manifest.json").read_text(encoding="utf-8-sig"))
    if not truth(manifest.get("complete")) and not args.allow_incomplete:
        raise SystemExit("Run incomplete: no analysis outputs written. Wait for complete=true, or explicitly use --allow-incomplete for a provisional snapshot.")
    inventory, issues, rows_by_study = [], [], {}
    for study, filename in RAW_FILES.items():
        path = root/filename
        rows = read_csv(path, root, inventory, issues) if path.is_file() else []
        for item in rows:
            item["_study"] = study
        rows_by_study[study] = rows
        counts = Counter(key(r) for r in rows)
        for identity, n in counts.items():
            if n > 1:
                issues.append({"kind": "duplicate_raw_case", "study": study, "key": identity, "n": n})
    rows = [r for study_rows in rows_by_study.values() for r in study_rows]
    for row in rows:
        if number(row.get("threshold")) != .3 or not truth(row.get("return_to_start")):
            issues.append({"kind": "main_protocol_threshold_or_return_drift", "file": row["_source"], "line": row["_line"]})
        if row.get("status") in {"SUCCESS", "NOOP"} and not successful(row):
            issues.append({"kind": "success_without_safe_certified_path", "file": row["_source"], "line": row["_line"]})
        coverage = number(row.get("geometric_coverage_percent"))
        if successful(row) and (coverage is None or coverage < 100-1e-9):
            issues.append({"kind": "success_with_missing_or_partial_geometric_coverage", "file": row["_source"], "line": row["_line"]})
    sens = sensitivity_pairs(rows_by_study["sensitivity"], issues)
    dynamic = dynamic_pairs(rows_by_study["dynamic"], issues)
    audits = audit_checks(root, rows, inventory, issues)
    raw_ik = sum(not unknown(r.get("ik_feasibility_percent")) for r in rows)
    raw_actual = sum(not unknown(r.get("actual_completion_percent")) for r in rows)
    unknown_checks = {"raw_ik_nonunknown_n": raw_ik, "raw_actual_completion_nonunknown_n": raw_actual,
                      "task_ik_nonunknown_n": audits["task_ik_nonunknown_n"],
                      "task_actual_completion_nonunknown_n": audits["task_actual_completion_nonunknown_n"],
                      "missing_task_evidence_columns_n": audits["missing_task_evidence_columns_n"],
                      "missing_raw_evidence_columns_n": sum("ik_feasibility_percent" not in r or
                           "actual_completion_percent" not in r for r in rows)}
    unknown_checks["all_available_rows_keep_ik_and_measured_unknown"] = bool(rows) and not any(unknown_checks.values())
    summary = {
        "analysis_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
        "evidence_directory": str(root), "analysis_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "provisional": not truth(manifest.get("complete")), "run_manifest": manifest,
        "implementation_signatures": dict(Counter(r.get("implementation_signature", "") for r in rows)),
        "completeness": completeness(rows_by_study, manifest),
        "methodology": {
            "success_denominator": "Every observed attempted row, including blocked/error/unsafe outcomes; pending expected rows listed separately.",
            "success_definition": "SUCCESS and path_safe; dynamic also accepts NOOP and requires external certification_checked.",
            "metrics": "Availability counts accompany all/failed/successful summaries. Do not compare mixed timing scopes or treat failed paths/times as successful missions.",
            "paired_sensitivity": "Same scene+seed baseline, identical task/table geometry signature and robot protocol; time/path ratios restricted to two successful routes with equal timing scope.",
            "paired_dynamic": "Same scene+seed+event and protocol; speed/path tradeoff restricted to two successful externally certified routes with matching timing/certification scope and positive measured time/length.",
            "oracle_scope": "Independent implementation of configured geometric constraints, finite pose search. Finite search failure does not establish physical infeasibility; vertical certificate applies only to configured planar-base model.",
            "timing_uncertainty": "Three-seed pilot, sequential same process after discarded warmup; descriptive statistics only, no invented confidence intervals.",
            "ik_and_execution": "Geometric support and simulated verification provide no independent joint-space IK or measured physical execution evidence.",
        },
        "groups": aggregate_groups(rows), "sensitivity_pairs": sens,
        "sensitivity_paired_summary": pair_aggregates(sens, "variant"),
        "dynamic_pairs": dynamic, "dynamic_paired_summary": pair_aggregates(dynamic, "event_type", True),
        "audit": audits, "ik_measured_checks": unknown_checks,
        "data_quality_issues": issues, "source_csv_inventory": inventory,
    }
    summary["valid_complete_snapshot"] = (not summary["provisional"] and
        summary["completeness"]["all_expected_rows_present"] and not issues and
        unknown_checks["all_available_rows_keep_ik_and_measured_unknown"])
    write_outputs(root, summary)
    print(json.dumps({"observed_rows": len(rows), "provisional": summary["provisional"],
                      "valid_complete_snapshot": summary["valid_complete_snapshot"],
                      "data_quality_issues": len(issues), "output_directory": str(root)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
