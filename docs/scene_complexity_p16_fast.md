# Scene Complexity P1.6-Fast

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

## 1. Minimal Diagnosis

P1.6-Fast keeps the formal robot geometry at `legacy_validated` and stays at
the candidate layer unless the coverage gate is reached. The minimal sweep
uses a fixed grid layout, discrete tasks, normal difficulty, a 10 m by 10 m
map, 16 tables, seeds `2026` and `2027`, and four fixed table orientations:

```text
0 deg, +45 deg, -45 deg, +90 deg
```

All eight fixed-orientation cases reached 100% candidate-library coverage:

| Seed | 0 deg | +45 deg | -45 deg | 90 deg |
|---:|---:|---:|---:|---:|
| 2026 | 100% | 100% | 100% | 100% |
| 2027 | 100% | 100% | 100% | 100% |

This means that a single absolute table heading is not enough to reproduce the
P1.5 failure, and the tested `+45 deg` and `-45 deg` cases are symmetric at the
candidate-coverage level. The loss appears when random table orientations are
mixed across the scene, where nearby rotated obstacles and table-local task
geometry interact with the current candidate and reachability model.

## 2. Failure Mechanism

For the first failed validation case checked in detail, `F010 seed=2026`, the
baseline left 27 tasks uncovered. A local probe around those uncovered tasks
used:

```text
theta_face, theta_face +/- 15 deg, theta_face +/- 30 deg
```

with task-local side positions and the unchanged collision and reachability
criteria. `F010 seed=2026` had 64.00% baseline coverage, 4689 baseline
candidates, and 27 uncovered tasks. All 27 traced failures were classified as:

```text
REACHABILITY_MODEL_LIMIT
```

The observed failure is not a clean `HEADING_GAP`, and adding more headings
would mask the model issue rather than fix it.

## 3. Selected Repair

No repair was selected in this round.

The requested orientation-conditioned augmentation is only appropriate when
the failures are mainly `HEADING_GAP` or position-heading `MIXED`. Because the
minimal local probe produced collision-valid candidates with zero reachability
score, this round intentionally stops before adding more candidates.

## 4. Validation Result

The reproducible entry point is:

```text
experiments/exp_orientation_fast_validation.m
```

It writes:

```text
results/tables/orientation_fast_validation_summary.csv
results/tables/orientation_fast_validation_raw.csv
```

The summary CSV is the compact tracked result. The raw task-level failure trace
is local diagnostic output and should remain untracked.

The P1.5 three-seed coverage baselines remain the validation reference:

| Case | Baseline coverage | Position-only coverage | New orientation-aware coverage |
|---|---:|---:|---:|
| C0 | 100.00% | 100.00% | N/A, not implemented |
| F010 | 55.78% | 55.78% | N/A, stopped |
| F110 | 45.12% | 45.12% | N/A, stopped |
| F111 | 59.26% | 63.82% | N/A, stopped |
| C2 | 55.38% at seed 2026 | not rerun in P1.6-Fast | N/A, stopped |

## 5. Candidate Overhead

No orientation-aware candidate augmentation was implemented, so the
orientation-aware added candidate count is 0 and the added ratio is 0.

Position-only remains available as a baseline comparison, but it does not solve
the random-orientation discrete cases that motivated this round.

## 6. P2 Gate

Decision:

```text
STOP_FOR_REACHABILITY_MODEL_REVIEW
```

Reason:

```text
Collision-valid local candidates exist, but the current reachability model
assigns zero support to the uncovered tasks under the targeted heading probe.
```

P2 candidate pruning should not start until the reachability model semantics
for these orientation-coupled failures are reviewed.
