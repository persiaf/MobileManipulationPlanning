# P5-Fast: Region-Aware Local Dynamic Replanning

## 1. Motivation

P5 studies discrete local changes after a large static planning result has
already been computed. It is not a fourth static strategy. The intended flow is:

```text
existing static solution
    -> deterministic local change
    -> impact analysis
    -> local invalidation
    -> local repair
    -> route splice / reuse accounting
```

The frozen static baseline is Bottom-Up on L2 with sparse routing:

```text
scale      = L2
map        = 15 x 15 m
tables     = 28
regions    = 7
routing    = sparse_knn
k          = 4
prefilter  = OFF
robot      = legacy_validated
```

The implementation keeps the static planner semantics unchanged. P5 introduces
local replanning infrastructure in `replanning/` and experiment harness code in
`experiments/`.

## 2. Event definitions

The first P5-Fast version supports three deterministic event types:

| event | meaning | local repair target |
|---|---|---|
| `ADD_LOCAL_OBSTACLE` | place a local static obstacle near an existing route/sparse edge | invalidate affected sparse/path edges and reuse the remaining graph |
| `INVALIDATE_DOCK` | invalidate one selected docking pose | remove the affected dock, locally refresh coverage in its region, and insert a replacement dock |
| `ADD_TASKS` | add four tasks inside one existing region | account for added tasks and locally add coverage if existing docks are insufficient |

Events are deterministic under:

```text
planning_state + event_type + event_id
```

## 3. Impact analysis

`replanning/analyze_change_impact.m` reports:

```text
affected_regions
affected_tasks
invalid_docks
invalid_sparse_edges
candidate_refresh_regions
path_refresh_edges
region_locality_ratio
```

The repair boundary is the cluster region. The P5-Fast L2 runs affected one or
two regions out of seven, never all regions by default.

## 4. Local repair

`replanning/local_replan.m` records local repair accounting:

```text
recomputed_candidates
recomputed_docks
recomputed_path_edges
A* calls
fallback flag
coverage / feasible coverage / verified completion
unchanged_dock_ratio
reused_edge_ratio
```

For path-obstacle events, only invalidated graph edges are removed and the
remaining sparse graph is reused. For dock-invalidation and task-addition
events, only affected-region coverage is refreshed in the accounting layer.

## 5. Full-vs-local experiment

The resource-bounded experiment entry point is:

```matlab
exp_dynamic_local_replanning_p5()
```

It reuses frozen P3/P4 L2 state snapshots when available:

```text
results/tables/p3p4_state_l2_seed2026_bottom_up.mat
results/tables/p3p4_state_l2_seed2027_bottom_up.mat
```

The formal P5-Fast matrix is:

```text
L2 x seeds [2026, 2027] x events [D1, D2, D3] x [LOCAL_REPLAN, FULL_REPLAN reference]
```

The full-replan rows are marked as `REFERENCE_ESTIMATE` in this fast gate. They
use the frozen P3/P4 full-static effort as a resource-bounded reference instead
of rerunning the complete modified-scene static pipeline. This keeps the P5-Fast
gate short and avoids blocking on full baseline runtime, but it also means the
time-speedup and path-degradation columns should not be treated as final
paper-grade full-replan evidence.

## 6. Locality metrics

P5 reports:

```text
region_locality_ratio = affected_regions / total_regions
edge_recompute_ratio  = recomputed_edges / original_sparse_edges
dock_recompute_ratio  = recomputed_or_changed_docks / original_docks
```

It also reports state preservation:

```text
unchanged_dock_ratio
reused_edge_ratio
```

These metrics demonstrate that local replanning is not a renamed full replan.

## 7. Result

Formal outputs:

```text
results/tables/dynamic_local_replanning_p5_raw.csv
results/tables/dynamic_local_replanning_p5_summary.csv
```

Summary over L2 seeds 2026 and 2027:

| event | local successes | affected regions | invalid edges / docks | local A* calls | region locality | edge recompute | unchanged docks | reused edges | coverage / verified |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `ADD_LOCAL_OBSTACLE` | 2/2 | 2.0 | 2.5 edges | 2.5 | 0.2857 | 0.0209 | 1.0000 | 0.9791 | 100% / 100% |
| `INVALIDATE_DOCK` | 2/2 | 1.0 | 1 dock | 8.0 | 0.1429 | 0.0663 | 0.9802 | 0.9337 | 100% / 100% |
| `ADD_TASKS` | 2/2 | 1.0 | 4 tasks | 2.0 | 0.1429 | 0.0166 | 1.0000 | 0.9834 | 100% / 100% |

Reference-estimate local-vs-full means:

```text
mean affected-region ratio = 0.1905
mean edge-recompute ratio  = 0.0346
mean A* reduction vs full  = 0.9742
fallback count             = 0
```

## 8. Limitations

This is a P5-Fast validation. It proves deterministic event generation,
localized impact analysis, repair accounting, and state-preservation metrics on
the L2 sparse-routing baseline. It does not yet provide a fully rerun
modified-scene static baseline for every event, and it does not implement
continuous moving-obstacle prediction, RL/GNN policies, or shared-cache
selective invalidation.

Gate:

```text
P5_COMPLETE
```

Reason: D1/D2/D3 each have successful local-replan evidence, feasible coverage
and verified completion are preserved, affected regions remain local, fallback
is not used, and the P5 targeted tests plus P3/P4 targeted tests pass in
short-lived MATLAB processes.
