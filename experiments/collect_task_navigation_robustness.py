"""Merge completed case CSVs without starting MATLAB or changing raw rows."""
from pathlib import Path
import argparse
import csv
import json


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def write_csv(path, fields, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def collect(folder):
    fields, manifest = read_csv(folder / "scenario_manifest.csv")
    combined, progress, raw_fields = [], [], None
    for case in manifest:
        directory = folder / case["relative_dir"]
        state = json.loads((directory / "run_state.json").read_text(encoding="utf-8-sig"))
        if state["status"] != "COMPLETE":
            raise RuntimeError(f"Case {case['case_id']} is not complete")
        current_fields, rows = read_csv(directory / "summary.csv")
        if len(rows) != 12 or (raw_fields is not None and current_fields != raw_fields):
            raise RuntimeError(f"Unexpected method rows/schema for {case['case_id']}")
        raw_fields = current_fields
        prefix = {key: case[key] for key in ("case_id", "layout_id", "distribution_id", "seed")}
        combined.extend(dict(prefix, **row) for row in rows)
        progress.append(dict(case, status=state["status"], completed_rows=len(rows)))
    assert len(manifest) == 9 and len(combined) == 108
    write_csv(folder / "batch_summary.csv", ["case_id", "layout_id", "distribution_id", "seed"] + raw_fields, combined)
    write_csv(folder / "scenario_progress.csv", fields + ["status", "completed_rows"], progress)
    state = dict(status="COMPLETE", scenario_count=9, completed_scenarios=9,
                 completed_rows=108, planned_raw_rows=108, planned_primary_rows=81)
    (folder / "run_state.json").write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(state))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("folder", nargs="?", type=Path,
                        default=Path("results/task_navigation_robustness_20261111"))
    collect(parser.parse_args().folder)
