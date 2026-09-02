# Scene Complexity P2 Candidate Efficiency

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

## 1. Pipeline Baseline

The candidate pipeline remains:

```text
raw candidate generation
collision filtering
reachability evaluation
candidate library
operability accounting
optional Position-only augmentation
selected docks
```

P2 does not change reachability semantics, collision margin, robot geometry,
Position-only augmentation, Bottom-Up, Top-Down, Bidirectional, Shared Lazy
Cache, `convex_astar`, or map size.

## 2. Quick Filter Definition

The conservative pair-level quick filter is:

```text
manipulator/candidate_reachability_quick_filter.m
```

It is controlled by:

```text
planner_cfg.candidate_pair_prefilter.enable
```

The default is `false`, preserving the existing baseline. The filter rejects a
candidate-task pair only if both arms fail at least one hard gate already used
by the legacy reachability evaluator:

```text
radial
vertical
backward
lateral
```

It does not approximate the final score and does not remove candidate poses.
It only reduces formal reachability calls for definitely impossible pairs.

## 3. False-Negative Audit

Experiment:

```text
experiments/exp_candidate_efficiency_p2.m
```

Scope:

```text
C0, C1, C2, C3, F010, F110, F111
seeds 2026, 2027, 2028
candidate-level only
```

Result:

```text
quick-filter false negatives = 0
```

## 4. Heading Shortlist

No new heading shortlist is enabled in this round. The current production
candidate generator already uses task-facing headings based on the task
centroid, individual task positions, table center, and side normal. P2-Fast
therefore keeps this part unchanged.

## 5. Coverage Preservation

Across the 21 case/seed runs:

| Metric | Current | Quick filter |
|---|---:|---:|
| Raw task coverage mean | 74.20% | 74.20% |
| Feasible-task coverage mean | 75.55% | 75.55% |
| Candidate-generation residual mean | 18.05 | 18.05 |
| Final candidate count mean | 5172.67 | 5172.67 |

Coverage degradation:

```text
0 pp
```

## 6. Evaluation Reduction

| Metric | Current | Quick filter |
|---|---:|---:|
| Candidate-task pairs mean | 23778.95 | 23778.95 |
| Formal reachability calls mean | 23778.95 | 2484.95 |
| Quick rejected pairs mean | 0 | 21294.00 |
| Formal valid pairs mean | 1375.29 | 1375.29 |

Mean formal reachability-call reduction:

```text
90.02%
```

## 7. Timing

Candidate generation time did not improve in MATLAB:

| Metric | Current | Quick filter |
|---|---:|---:|
| Candidate generation time mean | 7.605 s | 7.651 s |

The likely reason is MATLAB-level quick-filter function-call overhead. The
optimization is therefore useful as a semantic pair-pruning audit and as a
future vectorization target, but it should not be advertised as a wall-clock
speedup yet.

## 8. P3/P4 Gate

Candidate-level P2 passed:

```text
false negatives = 0
coverage degradation = 0 pp
C0 regression = 100%
formal reachability calls reduced by 90.02%
```

However, the required Bottom-Up smoke did not complete within the P2-Fast
budget and was interrupted during `C2 seed 2026` stage3 path validation.

Decision for enabling this optimization as a verified default:

```text
P2_OPTIMIZATION_REJECTED
```

The quick filter remains an opt-in, conservative diagnostic/performance path.
P2 failure should not block later P3/P4 research; later work can revisit this
filter with vectorization and a bounded Bottom-Up smoke harness.
