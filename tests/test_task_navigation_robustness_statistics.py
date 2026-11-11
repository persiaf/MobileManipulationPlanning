"""Scientific-statistics boundaries using isolated in-memory export rows.

These fixtures are unit-test dictionaries, not generated research scenarios.
No experiment inputs, cached IK evidence or result CSVs are read or changed.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"visualization/task_navigation_robustness"))
from batch_common import (Batch, METHODS, MODES, TIE_TOL, aggregate_pairs,
                          minimum_mission, mission_counts)


def exported_row(cost, **changes):
    """One task and one certified dock, with finite cost unless overridden."""
    row = {"covered_tasks": "1", "verified_tasks": "1", "target_count": "1",
           "docks": "1", "minimum_docks": "1", "route_valid": "1", "mission_valid": "1",
           "route_length_m": str(cost-.5), "navigation_cost": str(cost),
           "status": "READY", "failure_reason": ""}
    row.update({key: str(value) for key, value in changes.items()})
    return row


def statistical_batch(count=1):
    """Exercise aggregation independently of the disk/scene reader."""
    batch = object.__new__(Batch)
    batch.cases = []
    for index in range(count):
        rows = {}
        for mode in MODES:
            for method, cost in (("staged", 10.0), ("minimum_seed", 10.0),
                                 ("reorder_only", 9.0), ("joint_local", 8.0)):
                rows[mode, method] = exported_row(cost)
        batch.cases.append(SimpleNamespace(case_id=f"statistics_fixture_{index}",
            layout=0, distribution=index, spec={"seed": "unit-test"}, rows=rows))
    return batch


def mode_pairs(batch, mode="position_only"):
    return {row["comparator"]: row for row in batch.pairs() if row["mode"] == mode}


class RobustnessStatisticsTests(unittest.TestCase):
    def test_negative_improvement_is_a_loss_and_is_not_clamped(self):
        """A worse joint cost stays negative in raw and aggregate metrics."""
        batch = statistical_batch()
        batch.cases[0].rows["position_only", "joint_local"] = exported_row(11.0)
        pair = mode_pairs(batch)["minimum_seed"]
        self.assertTrue(pair["eligible"])
        self.assertEqual(pair["outcome"], "loss")
        self.assertAlmostEqual(pair["J_delta"], -1.0)
        self.assertAlmostEqual(pair["J_gain_pct"], -10.0)
        aggregate = next(row for row in aggregate_pairs(batch.pairs(), 1)
                         if row["mode"] == "position_only" and row["comparator"] == "minimum_seed")
        self.assertEqual((aggregate["wins"], aggregate["ties"], aggregate["losses"]), (0, 0, 1))
        self.assertAlmostEqual(aggregate["J_gain_median_pct"], -10.0)
        self.assertAlmostEqual(aggregate["J_gain_worst_pct"], -10.0)

    def test_tie_uses_absolute_raw_cost_difference(self):
        """The predeclared tolerance is in J units, not relative percent."""
        batch = statistical_batch()
        batch.cases[0].rows["position_only", "joint_local"] = exported_row(10.0+TIE_TOL/2)
        self.assertEqual(mode_pairs(batch)["minimum_seed"]["outcome"], "tie")
        # A small relative difference at a large J is still an absolute loss.
        batch.cases[0].rows["position_only", "minimum_seed"] = exported_row(1e6)
        batch.cases[0].rows["position_only", "joint_local"] = exported_row(1e6+1e-5)
        pair = mode_pairs(batch)["minimum_seed"]
        self.assertTrue(pair["eligible"])
        self.assertEqual(pair["outcome"], "loss")

    def test_failed_nan_cost_is_ineligible_instead_of_zero(self):
        """Missing complete-mission cost cannot manufacture a short route."""
        batch = statistical_batch()
        batch.cases[0].rows["position_only", "joint_local"] = exported_row(8.0,
            navigation_cost="NaN", route_length_m="NaN", mission_valid=0,
            route_valid=0, status="ERROR", failure_reason="UNIT_TEST_NAVIGATION_FAILURE")
        for pair in mode_pairs(batch).values():
            self.assertFalse(pair["eligible"])
            self.assertEqual(pair["outcome"], "ineligible")
            self.assertTrue(math.isnan(pair["J_delta"]))
            self.assertTrue(math.isnan(pair["J_gain_pct"]))
            self.assertTrue(math.isnan(pair["L_gain_pct"]))

    def test_failure_remains_in_all_scenario_denominators(self):
        """Eligible gains and mission counts retain the planned denominator."""
        batch = statistical_batch(2)
        batch.cases[1].rows["position_only", "joint_local"] = exported_row(8.0,
            mission_valid=0, route_valid=0, navigation_cost="NaN", route_length_m="NaN",
            status="ERROR", failure_reason="UNIT_TEST_NAVIGATION_FAILURE")
        aggregates = [row for row in aggregate_pairs(batch.pairs(), 2) if row["mode"] == "position_only"]
        self.assertEqual(len(aggregates), 2)
        for row in aggregates:
            self.assertEqual(row["total_scenarios"], 2)
            self.assertEqual(row["eligible_pairs"], 1)
            self.assertEqual(row["ineligible_pairs"], 1)
            self.assertEqual(row["wins"]+row["ties"]+row["losses"], 1)
        joint = next(row for row in mission_counts(batch)
                     if row["mode"] == "position_only" and row["method"] == "joint_local")
        self.assertEqual((joint["full_minimum_missions"], joint["total_scenarios"]), (1, 2))

    def test_different_minimum_certificates_do_not_form_equal_dock_pairs(self):
        """Individually minimal results with different K are incomparable."""
        batch = statistical_batch()
        row = exported_row(8.0, docks=2, minimum_docks=2)
        self.assertTrue(minimum_mission(row))
        batch.cases[0].rows["position_only", "joint_local"] = row
        for pair in mode_pairs(batch).values():
            self.assertFalse(pair["eligible"])
            self.assertIn("CERTIFIED_MINIMUM_DIFFERS", pair["ineligible_reason"])
            self.assertTrue(math.isnan(pair["J_gain_pct"]))

    def test_third_method_failure_excludes_both_primary_comparisons(self):
        """Both references use the same three-method common-valid subset."""
        batch = statistical_batch()
        row = copy.deepcopy(batch.cases[0].rows["position_only", "reorder_only"])
        row.update(mission_valid="0", route_valid="0", status="ERROR",
                   failure_reason="UNIT_TEST_THIRD_METHOD_FAILURE")
        batch.cases[0].rows["position_only", "reorder_only"] = row
        pairs = mode_pairs(batch)
        self.assertTrue(pairs["minimum_seed"]["pairwise_eligible"])
        self.assertFalse(pairs["reorder_only"]["pairwise_eligible"])
        for pair in pairs.values():
            self.assertFalse(pair["eligible"])
            self.assertFalse(pair["common_three_method_eligible"])
            self.assertIn("reorder_only", pair["ineligible_reason"])
            self.assertTrue(math.isnan(pair["J_gain_pct"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, help="Optional validation-summary output, not experimental results")
    parser.add_argument("--log", type=Path, help="Optional concise unittest log")
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RobustnessStatisticsTests)
    test_names = [test.id() for test in suite]
    stream = io.StringIO()
    started = time.perf_counter()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    log = stream.getvalue()
    print(log, end="")
    record = {"status": "PASS" if result.wasSuccessful() else "FAIL", "tests_run": result.testsRun,
              "passed": result.testsRun-len(result.failures)-len(result.errors)-len(result.skipped),
              "failures": len(result.failures), "errors": len(result.errors), "skipped": len(result.skipped),
              "elapsed_s": time.perf_counter()-started, "tests": test_names,
              "scope": "in-memory statistical unit fixtures only; not experiment scenes, IK or navigation execution",
              "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (Path(__file__).resolve(), ROOT/"visualization/task_navigation_robustness/batch_common.py")}}
    if args.log:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text(log, encoding="utf-8")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(record, indent=2)+"\n", encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
