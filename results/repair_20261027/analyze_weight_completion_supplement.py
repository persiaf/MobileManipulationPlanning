"""Analyze the separate 21-case MAIN4 weight batch without pooling core timings.

Defaults to current_evidence_path.txt beside this script. Outputs remain inside
weight_completion_supplement. Use --allow-incomplete only for labelled interim
results, or --self-test for synthetic checks without reading research data.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import analyze_repair_evidence as core


VARIANTS = ("baseline", "cost_reach_half", "cost_reach_double", "cost_coverage_half",
            "cost_coverage_double", "selection_coverage_half", "selection_coverage_double")


def paired_weights(rows, manifest, issues):
    pairs = core.sensitivity_pairs(rows, issues)
    by_key = defaultdict(list)
    for row in rows:
        by_key[(row.get("scene_id"), row.get("seed"), row.get("variant"))].append(row)
    for pair in pairs:
        refs = by_key[(pair["scene_id"], pair["seed"], "baseline")]
        variants = by_key[(pair["scene_id"], pair["seed"], pair["variant"])]
        if len(refs) != 1 or len(variants) != 1:
            continue
        ref, variant = refs[0], variants[0]
        expected_batch = manifest.get("batch_id")
        batch_ok = bool(expected_batch) and all(
            r.get("supplement_batch_id") == expected_batch and
            r.get("timing_comparison_group") == expected_batch for r in (ref, variant))
        expected_source = manifest.get("source_implementation_signature")
        source_ok = bool(expected_source) and all(
            r.get("implementation_signature") == expected_source for r in (ref, variant))
        pair.update(supplement_batch_id=expected_batch,
                    same_supplement_timing_batch=batch_ok, same_expected_source=source_ok,
                    timing_pair_domain="within_same_supplement_batch_only")
        extra = []
        if not batch_ok:
            extra.append("supplement_batch_identity_missing_or_mismatched")
        if not source_ok:
            extra.append("supplement_source_identity_missing_or_mismatched")
        if extra:
            pair["eligible_for_time_path_comparison"] = False
            pair["exclusion_reason"] = ";".join(filter(None, [pair.get("exclusion_reason"), *extra]))
            for metric in ("planning_time_s", "path_length_m", "selected_docks", "candidate_count",
                           "baseline_over_variant_time_ratio"):
                pair.pop(metric, None)
            issues.append({"kind": "invalid_supplement_pair_batch_or_source", "seed": pair["seed"],
                           "variant": pair["variant"], "reasons": extra})
    return pairs


def audit_checks(folder, rows, inventory, issues):
    cases, all_tasks = [], []
    for row in rows:
        filename = f"seed{row.get('seed', '')}_{row.get('variant', '')}_task_audit.csv"
        path = folder/"cases"/filename
        if not path.is_file():
            if row.get("audit_status") == "PASS":
                issues.append({"kind": "missing_pass_audit_csv", "file": f"cases/{filename}"})
            continue
        tasks = core.read_csv(path, folder, inventory, issues)
        classes = Counter(t.get("classification", "") for t in tasks)
        entry = {"seed": row.get("seed"), "variant": row.get("variant"),
                 "file": f"cases/{filename}", "task_case_observations": len(tasks),
                 "classification_counts": dict(classes),
                 "library_coverage_mismatch_count": sum(core.truth(t.get("library_coverage_mismatch")) for t in tasks),
                 "sampling_gap_count": sum(core.truth(t.get("candidate_sampling_gap")) for t in tasks)}
        expected = core.number(row.get("task_count"))
        if expected is not None and expected != len(tasks):
            issues.append({"kind": "audit_task_count_mismatch", "file": entry["file"],
                           "expected": expected, "observed": len(tasks)})
        if len({t.get("task_id") for t in tasks}) != len(tasks):
            issues.append({"kind": "duplicate_audit_task_ids", "file": entry["file"]})
        for label in classes:
            if label not in core.AUDIT_CLASSES:
                issues.append({"kind": "unknown_audit_classification", "file": entry["file"], "value": label})
        for observed_key, raw_key in (("library_coverage_mismatch_count", "independent_mismatch_count"),
                                      ("sampling_gap_count", "sampling_gap_count")):
            expected = core.number(row.get(raw_key))
            if expected is not None and expected != entry[observed_key]:
                issues.append({"kind": "audit_summary_mismatch", "file": entry["file"], "field": raw_key})
        cases.append(entry)
        all_tasks.extend(tasks)
    totals = Counter()
    for case in cases:
        totals.update(case["classification_counts"])
    return {"cases": cases, "case_files_n": len(cases), "task_case_observations": len(all_tasks),
            "observation_note": "Repeated seed tasks across weight variants are task-case observations, not independent physical tasks.",
            "classification_counts": dict(totals),
            "task_mismatch_count": sum(c["library_coverage_mismatch_count"] for c in cases),
            "raw_candidate_pair_mismatch_counts": core.describe(core.number(r.get("independent_pair_mismatch_count")) for r in rows),
            "task_ik_nonunknown_n": sum(not core.unknown(t.get("ik_feasible")) for t in all_tasks),
            "task_actual_nonunknown_n": sum(not core.unknown(t.get("actual_completed")) for t in all_tasks),
            "missing_task_evidence_columns_n": sum("ik_feasible" not in t or "actual_completed" not in t for t in all_tasks)}


def build_summary(folder):
    manifest = json.loads((folder/"weight_completion_manifest.json").read_text(encoding="utf-8-sig"))
    inventory, issues = [], []
    rows = core.read_csv(folder/"weight_completion_raw.csv", folder, inventory, issues)
    for row in rows:
        row["_study"] = "weight_completion_supplement"
        if core.number(row.get("threshold")) != .3 or not core.truth(row.get("return_to_start")):
            issues.append({"kind": "main_protocol_threshold_or_return_drift", "line": row["_line"]})
        if row.get("status") == "SUCCESS" and (not core.successful(row) or
                (core.number(row.get("geometric_coverage_percent")) or 0) < 100-1e-9):
            issues.append({"kind": "success_without_complete_safe_route", "line": row["_line"]})
        for flag in ("paired_geometry_checked", "paired_geometry_matches", "frozen_source_matches"):
            if row.get("status") == "SUCCESS" and not core.truth(row.get(flag)):
                issues.append({"kind": "successful_supplement_unverified_identity_flag", "flag": flag, "line": row["_line"]})
    seeds = [str(seed) for seed in manifest.get("seeds", [])]
    expected = {("MAIN4", seed, variant) for seed in seeds for variant in VARIANTS}
    observed = Counter(core.key(r) for r in rows)
    for identity, count in observed.items():
        if count > 1:
            issues.append({"kind": "duplicate_supplement_row", "key": identity, "n": count})
    completeness = {"expected_rows": len(expected), "observed_rows": len(rows),
                    "missing_expected_keys": sorted(expected-observed.keys()),
                    "unexpected_keys": sorted(observed.keys()-expected),
                    "manifest_expected_rows": manifest.get("expected_rows")}
    completeness["all_expected_rows_present"] = (bool(seeds) and len(rows) == len(expected)
        and not completeness["missing_expected_keys"] and not completeness["unexpected_keys"]
        and manifest.get("expected_rows") == len(expected))
    pairs = paired_weights(rows, manifest, issues)
    audits = audit_checks(folder, rows, inventory, issues)
    unknown = {"raw_ik_nonunknown_n": sum(not core.unknown(r.get("ik_feasibility_percent")) for r in rows),
               "raw_actual_nonunknown_n": sum(not core.unknown(r.get("actual_completion_percent")) for r in rows),
               "missing_raw_evidence_columns_n": sum("ik_feasibility_percent" not in r or "actual_completion_percent" not in r for r in rows),
               **{key: audits[key] for key in ("task_ik_nonunknown_n", "task_actual_nonunknown_n", "missing_task_evidence_columns_n")}}
    unknown["all_available_rows_keep_ik_and_measured_unknown"] = bool(rows) and not any(unknown.values())
    summary = {"analysis_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
               "analysis_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "core_analysis_sha256": hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest(),
               "study": "MAIN4_weight_completion_supplement", "directory": str(folder),
               "provisional": not core.truth(manifest.get("complete")), "run_manifest": manifest,
               "completeness": completeness,
               "methodology": {"timing_domain": "Compare same-seed variants inside this supplement batch only; do not pool absolute time with the earlier core batch.",
                    "pair_requirements": "Both complete safe routes, same scene/task signature, robot protocol, timing scope, supplement batch and expected implementation source.",
                    "failure_handling": "Retain all attempted rows in success denominators; time/path ratios only use eligible successful pairs.",
                    "evidence_scope": "Three-seed geometric pilot; independent geometric audit is not full joint-space IK or measured execution."},
               "implementation_signatures": dict(Counter(r.get("implementation_signature", "") for r in rows)),
               "groups": core.aggregate_groups(rows), "sensitivity_pairs": pairs,
               "sensitivity_paired_summary": core.pair_aggregates(pairs, "variant"),
               "audit": audits, "ik_measured_checks": unknown,
               "data_quality_issues": issues, "source_csv_inventory": inventory}
    summary["valid_complete_snapshot"] = (not summary["provisional"] and completeness["all_expected_rows_present"]
        and not issues and unknown["all_available_rows_keep_ik_and_measured_unknown"])
    return summary


def write_outputs(folder, summary):
    path = folder/"weight_completion_analysis_summary.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
    temporary.replace(path)
    rows = []
    for section in ("groups", "sensitivity_pairs", "sensitivity_paired_summary"):
        rows.extend({"record_type": section, **core.flatten(row)} for row in summary[section])
    fields = ["record_type"]+sorted({k for row in rows for k in row if k != "record_type"})
    temporary = folder/"weight_completion_analysis_tables.csv.tmp"
    with temporary.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(folder/"weight_completion_analysis_tables.csv")


def self_test():
    manifest = {"batch_id": "BATCH1", "source_implementation_signature": "source"}
    base = {"_study": "weight_completion_supplement", "scene_id": "MAIN4", "seed": "2026",
            "variant": "baseline", "status": "SUCCESS", "path_safe": "1", "threshold": ".3",
            "return_to_start": "1", "global_planner": "convex_astar", "routing_mode": "sparse_knn",
            "sparse_k": "6", "task_geometry_signature": "geometry", "timing_scope": "scope",
            "planning_time_s": "10", "path_length_m": "20", "geometric_coverage_percent": "100",
            "implementation_signature": "source", "supplement_batch_id": "BATCH1", "timing_comparison_group": "BATCH1"}
    variant = {**base, "variant": "cost_reach_half", "planning_time_s": "8"}
    issues = []
    pair = paired_weights([base, variant], manifest, issues)[0]
    assert pair["eligible_for_time_path_comparison"] and pair["baseline_over_variant_time_ratio"] == 1.25 and not issues
    for change in ({"supplement_batch_id": "BATCH2"}, {"timing_comparison_group": "BATCH2"},
                   {"implementation_signature": "different"}):
        issues = []
        pair = paired_weights([base, {**variant, **change}], manifest, issues)[0]
        assert not pair["eligible_for_time_path_comparison"] and "baseline_over_variant_time_ratio" not in pair and issues
    pair = paired_weights([base, {**variant, "status": "BLOCKED_ROUTING"}], manifest, [])[0]
    assert not pair["eligible_for_time_path_comparison"] and "baseline_over_variant_time_ratio" not in pair
    group = core.aggregate_groups([base, {**base, "status": "ERROR"}])[0]
    assert group["attempted_n"] == 2 and group["success_percent"] == 50
    import tempfile
    with tempfile.TemporaryDirectory(prefix="weight_analysis_selftest_") as directory:
        folder = Path(directory)
        (folder/"cases").mkdir()
        full_manifest = {**manifest, "seeds": [2026, 2027, 2028], "expected_rows": 21, "complete": True}
        (folder/"weight_completion_manifest.json").write_text(json.dumps(full_manifest), encoding="utf-8")
        rows = []
        for seed in full_manifest["seeds"]:
            for name in VARIANTS:
                item = {**base, "seed": str(seed), "variant": name,
                        "task_geometry_signature": f"geometry{seed}", "task_count": "1", "audit_status": "PASS",
                        "independent_pair_mismatch_count": "0", "independent_mismatch_count": "0", "sampling_gap_count": "0",
                        "paired_geometry_checked": "1", "paired_geometry_matches": "1", "frozen_source_matches": "1",
                        "ik_feasibility_percent": "NaN", "actual_completion_percent": "NaN"}
                rows.append(item)
                (folder/"cases"/f"seed{seed}_{name}_task_audit.csv").write_text(
                    "task_id,classification,library_coverage_mismatch,candidate_sampling_gap,ik_feasible,actual_completed\n"
                    "1,covered_in_library,0,0,NaN,NaN\n", encoding="utf-8-sig")
        with (folder/"weight_completion_raw.csv").open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary = build_summary(folder)
        assert summary["valid_complete_snapshot"] and not summary["data_quality_issues"]
        assert len(summary["sensitivity_pairs"]) == 18 and summary["audit"]["case_files_n"] == 21
        write_outputs(folder, summary)
        assert json.loads((folder/"weight_completion_analysis_summary.json").read_text(encoding="utf-8"))["valid_complete_snapshot"]
    print("supplement analysis self-test: 7 synthetic contracts passed (including 21-row CSV/task-audit/JSON roundtrip); no research data read")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path)
    parser.add_argument("--allow-incomplete", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    script_dir = Path(__file__).resolve().parent
    pointer = script_dir/"current_evidence_path.txt"
    default_root = script_dir if (script_dir/"run_manifest.json").is_file() else script_dir/"evidence_20261027_221823"
    root = args.evidence_dir if args.evidence_dir else (
        Path(pointer.read_text(encoding="utf-8-sig").strip()) if pointer.is_file() else default_root)
    folder = root.resolve()/"weight_completion_supplement"
    manifest = json.loads((folder/"weight_completion_manifest.json").read_text(encoding="utf-8-sig"))
    if not core.truth(manifest.get("complete")) and not args.allow_incomplete:
        raise SystemExit("Supplement incomplete: no outputs written; --allow-incomplete creates an explicitly provisional snapshot only.")
    summary = build_summary(folder)
    write_outputs(folder, summary)
    print(json.dumps({"observed_rows": summary["completeness"]["observed_rows"], "provisional": summary["provisional"],
                      "valid_complete_snapshot": summary["valid_complete_snapshot"],
                      "data_quality_issues": len(summary["data_quality_issues"]), "output_directory": str(folder)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
