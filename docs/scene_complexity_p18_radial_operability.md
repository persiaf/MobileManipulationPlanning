# Scene Complexity P1.8 Radial Operability Audit

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

## 1. Radial Rule Provenance

The reachability evaluator computes horizontal radial distance from the arm
base to the task:

```text
d_horizontal = sqrt(dx^2 + dy^2)
```

The hard radial limit is:

```text
R_limit = robot_cfg.single_arm.reach_radius = 0.278 m
```

This value comes from `config/robot_config.m`. It is a manually configured
legacy value in the current code path, not a runtime value derived from DH
geometry or an external calibration table.

See also:

```text
docs/reachability_constraint_audit.md
```

## 2. Radial-Only Counterfactual

Experiment:

```text
experiments/exp_radial_operability_audit.m
```

Scope:

```text
F010 seed 2026
23 ORACLE_INFEASIBLE / FAIL_RADIAL tasks from P1.7
legacy_validated robot profile
```

The diagnostic ignores only the radial hard gate, while keeping collision,
vertical, backward, lateral, and score-threshold checks.

Result:

| Metric | Value |
|---|---:|
| Audited FAIL_RADIAL tasks | 23 |
| Radial-only blocked | 0 |
| Multi-constraint blocked | 23 |

This means radial is not the only controlling condition in the current
diagnostic. Removing the radial gate alone would not make these tasks pass the
rest of the existing model.

## 3. Required Radial Excess

No task had a pose that passed all non-radial conditions while only exceeding
the radial hard gate. Therefore, the minimum required radial excess statistics
are not defined:

```text
mean / median / min / max = NaN
```

This is evidence against making a simple radial-threshold change.

## 4. Isolated-Table Result

The isolated-table oracle keeps the target table, task, robot geometry,
reachability model, map boundary, and collision check, while removing other
tables from the diagnostic grid.

Result:

| Metric | Value |
|---|---:|
| Isolated-table feasible | 0 |
| Isolated-table infeasible | 23 |

The failures do not disappear when nearby table obstacles are removed.

## 5. Failure Classification

Final classification for the 23 audited tasks:

| Class | Count |
|---|---:|
| TASK_GEOMETRY_REACH_LIMIT | 23 |
| ENVIRONMENT_CLEARANCE_LIMIT | 0 |
| MODEL_RADIAL_OVERCONSERVATIVE | 0 |
| MIXED_OR_UNRESOLVED | 0 |

There is not enough evidence to label the radial model as over-conservative.
The stronger conclusion is that these tasks are not model-operable under the
current legacy reachability semantics.

## 6. Selected Action

Selected action:

```text
operability accounting
```

No reachability semantics were changed. Instead, the project now has a small
diagnostic helper:

```text
utils/task_operability_accounting.m
```

It reports both raw task coverage and feasible-task coverage. This keeps the
old metric intact while preventing model-infeasible tasks from being treated as
planner failures in later scene-complexity analysis.

For `F010 seed 2026`:

| Metric | Value |
|---|---:|
| Raw task count | 75 |
| Raw baseline coverage | 64.00% |
| Model-infeasible tasks | 23 |
| Model-feasible tasks | 52 |
| Feasible-task coverage | 92.31% |
| Candidate-generation residual | 4 |

The four residual feasible-but-uncovered tasks are the P1.7 candidate-generation
residual (`MISSING_HEADING = 3`, `POSITION_HEADING_INTERACTION = 1`). They are
not the dominant blocker and are left for later targeted repair.

## 7. P2 Gate

Decision:

```text
READY_FOR_P2
```

Reasons:

```text
reachability semantics are documented
scene geometric validity and task operability are separated
model-feasible task coverage is 92.31%
C0 regression remains 100%
no collision or reachability threshold relaxation was used
```

P2 should still preserve the raw coverage metric and additionally report
feasible-task coverage when operational infeasibility is present.
