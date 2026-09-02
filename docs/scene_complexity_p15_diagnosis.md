# Scene Complexity P1.5 Diagnosis

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

## 1. Why P1.5 Was Added

P1 showed that C2 lowered candidate-library coverage from the C0/C1 baseline
while C3 returned to full coverage. C3 changes layout, orientation, and task
difficulty together, so it cannot be used to conclude that one task mode is
easier than another. P1.5 isolates the factors before any P2 candidate
pruning or fast-filter work.

## 2. Factor Isolation

Experiment: `experiments/exp_scene_complexity_factor_isolation.m`

Seeds: `2026, 2027, 2028`

Scope: candidate generation, baseline library coverage, and position-only
augmentation coverage. No Top-Down, Bidirectional, or stage3 full path matrix
was run.

| Factor | Layout | Orientation | Task Sampling | Baseline Coverage | Position-Only Coverage |
|---|---|---|---|---:|---:|
| F000 | grid | fixed | discrete | 100.00% | 100.00% |
| F001 | grid | fixed | continuous | 90.81% | 97.67% |
| F010 | grid | discrete_random | discrete | 55.78% | 55.78% |
| F011 | grid | discrete_random | continuous | 65.42% | 69.10% |
| F100 | jittered_grid | fixed | discrete | 100.00% | 100.00% |
| F101 | jittered_grid | fixed | continuous | 87.13% | 98.54% |
| F110 | jittered_grid | discrete_random | discrete | 45.13% | 45.13% |
| F111 | jittered_grid | discrete_random | continuous | 59.26% | 63.82% |

Main factor effects from this descriptive matrix:

- Layout alone does not reduce coverage: F000 and F100 are both 100%.
- Continuous sampling reduces coverage under fixed orientation, but
  position-only largely recovers it.
- Discrete random orientation is the largest single coverage drop in this
  matrix, including with discrete tasks.
- The strongest interaction is orientation plus jittered layout for discrete
  tasks: F110 is the lowest-coverage case.
- F111, the C2-like setting, remains below full coverage after position-only.

## 3. C2 Failure Geometry

Experiment: `experiments/exp_c2_candidate_failure_diagnosis.m`

The task-level trace records uncovered tasks, table-local positions, nearest
collision-valid candidate geometry, and reachability scores. Raw trace is
kept local in `results/tables/c2_failure_geometry_trace.csv`.

Summary:

| Geometry Class | Position-Only Category | Task Count | Mean Candidate-Task Distance | Mean Combined Reach Score |
|---|---|---:|---:|---:|
| CONTINUOUS_REACH_BOUNDARY | C_COLLISION_VALID_BUT_UNREACHABLE | 53 | 0.549 m | 0 |
| KNOWN_POSITION_GAP | C_COLLISION_VALID_BUT_UNREACHABLE | 26 | 0.563 m | 0 |
| KNOWN_POSITION_GAP | RECOVERED_BY_POSITION_ONLY | 10 | 0.489 m | 0 |

C2 is not only the previous S4 center-row position gap. A large portion is a
continuous reach-boundary geometry where collision-valid candidate bases
exist, but current arm reachability evaluates them as unreachable.

## 4. Position-Only Failure Analysis

Position-only improves F001, F101, F011, and F111, but it does not improve
F010 or F110. This means it helps when the dominant issue is candidate
position alignment for continuous samples, but it does not address orientation
coupled coverage loss.

For C2-like F111, position-only improved mean coverage from 59.26% to 63.82%.
The remaining failures are mostly category C:

```text
supplemental/collision-valid geometry exists, but reachability score remains 0
```

No evidence was found that position-only uses an unrotated table frame. The
augmentation computes task local coordinates with `rot2(-table_theta)` and
maps supplemental positions back with `rot2(table_theta)`.

## 5. Robot Geometry Alignment Audit

See `docs/robot_geometry_alignment_audit.md`.

Key point: the physical measurements `xa = 0.06 m`, `da = 0.40 m`, and
`h0 = 0.40 m` can be mapped to the current profile fields, but the physical
base-frame origin and vertical reference must still be confirmed. The
exploratory physical profile reduces C0 coverage from 100% to 64% and C2
coverage from 55.38% to 32.31%, so geometry alignment is not a cosmetic
change.

## 6. Implications for P2

P2 candidate pruning should not start yet. The current failure evidence says
the main unresolved issue is not candidate count alone. It is a coupled
coverage problem involving random table orientation, continuous task geometry,
and robot reachability semantics.

## 7. Decision Gate

Decision: `NOT_READY_FOR_P2`

Reasons:

- C2 failure geometry is partially explained, but still contains a strong
  orientation-coupled component that position-only does not recover.
- Robot physical geometry mapping is plausible but not fully confirmed at the
  coordinate-origin/reference-height level.
- The physical profile changes candidate coverage substantially, so P2 should
  not prune candidates before the geometry profile boundary is reviewed.
