# P3/P4: Region-Structured Large-Scale Planning and Sparse Docking Path Graph

This maintenance stage moves the scalability focus from candidate-pair filtering
to global path-cost reduction. The existing planner strategies remain
Bottom-Up, Top-Down, and Bidirectional; sparse routing is an infrastructure
option used after docking-pose selection, not a fourth planning strategy.

## Scale profiles

The new P3/P4 profiles are configured through
`experiments/large_scale_profile_config.m` and generated parametrically through
the existing layout generator.

| scale | layout | map | tables | requested tasks | seed-2026 actual tasks | cluster regions |
|---|---|---:|---:|---:|---:|---:|
| L1 | cluster | 12 x 12 m | 20 | 90 | 90 | 5 |
| L2 | cluster | 15 x 15 m | 28 | 126 | 126 | 7 |
| L3 | cluster | 20 x 20 m | 40 | 180 | 181 | 7 |

`L1_JITTERED` is retained as a non-cluster control profile.

## Task-to-dock compression

The planner continues to use the project abstraction:

```text
Tasks -> Candidate Library -> Multi-task Docking Coverage -> Selected Docks -> Global Routing
```

The global routing graph is built over:

```text
start pose + selected docking poses
```

not over every task point. Experiments record:

```text
compression_ratio = N_task / N_selected_dock
```

## Region abstraction

The cluster generator now records `table_region_id`. Scene construction derives
`task_region_id`, and `summarize_region_metadata` can report:

```text
region_count
tables_per_region
tasks_per_region
candidate_count_per_region
dock_region_id
selected_docks_per_region
```

The previous table-level `regions` structure is preserved for candidate
generation and strategy code. P3/P4 maps selected dock poses back to the
cluster-level table region when sparse routing is used.

## Dense routing bottleneck

The dense baseline remains:

```matlab
planner_cfg.routing.mode = 'dense';
```

It calls the configured path planner for every pair among
`[start; selected_docking_poses]`. For `D` selected docks, the potential dense
pair count is:

```text
(D + 1) * D / 2
```

In an L1 seed-2026 smoke, Bottom-Up selected 33 docks, giving 561 dense pairs.
The dense stage3 smoke was manually stopped after roughly 90 seconds while still
inside path-cost matrix construction, so the dense L1 result should be treated
as resource-bounded and incomplete until rerun with a formal timeout wrapper.

## Sparse docking path graph

The sparse option is:

```matlab
planner_cfg.routing.mode = 'sparse_knn';
planner_cfg.routing.sparse_knn.k = 6;
planner_cfg.routing.sparse_knn.k_local = 4;
planner_cfg.routing.sparse_knn.k_cross = 1;
```

`global_planner/build_sparse_docking_graph.m` first creates Euclidean kNN edge
candidates and then validates only those candidate edges with the configured
path planner. It records:

```text
potential_dense_pairs
sparse_geometric_edges
actual_path_queries
actual_astar_calls
successful_edges
failed_edges
adaptive_bridge_edges_added
```

## Connectivity repair

`global_planner/validate_sparse_graph_connectivity.m` checks:

```text
start connected
all required docks reachable
region graph connected
```

If the graph is disconnected, the builder adds nearest component bridges and
counts them as adaptive bridge edges. It does not silently fall back to a dense
all-pairs matrix.

## Cache interaction

Sparse routing and shared lazy cache are separate mechanisms:

```text
sparse graph -> reduces proposed path pairs
shared lazy cache -> avoids repeated expensive solves
```

The Bidirectional four-mode experiment entry point is
`experiments/exp_sparse_cache_ablation.m`:

```text
B0 Dense  / Cache OFF
B1 Dense  / Cache ON
B2 Sparse / Cache OFF
B3 Sparse / Cache ON
```

The official P3/P4 baseline keeps:

```matlab
planner_cfg.candidate_pair_prefilter.enable = false;
```

## Experiment design

The resource-bounded aggregate-only entry point is
`experiments/exp_large_scale_sparse_routing.m`.

Recommended first matrix:

```text
L1: Dense, Sparse k=4, Sparse k=6, Sparse k=8; seeds [2026 2027 2028]
L2: Dense, Sparse k=selected; seeds [2026 2027 2028]
L3: Sparse k=selected; seeds [2026 2027 2028], plus one dense resource-bounded baseline
```

The scripts write compact CSV summaries only:

```text
results/tables/large_scale_sparse_routing_raw.csv
results/tables/large_scale_sparse_routing_summary.csv
results/tables/sparse_cache_ablation_raw.csv
results/tables/sparse_cache_ablation_summary.csv
```

## Current smoke results

`smoke` scale verified the experiment entry point:

| scale | mode | tasks | docks | dense pairs | actual A* calls | coverage | verified |
|---|---|---:|---:|---:|---:|---:|---:|
| smoke | dense | 4 | 4 | 10 | 10 | 100% | 100% |
| smoke | sparse k=4 | 4 | 4 | 10 | 10 | 100% | 100% |

L1 seed-2026 sparse-only smoke:

| tasks | tables | regions | docks | dense pairs | sparse edges | actual A* calls | coverage | verified | path |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 90 | 20 | 5 | 33 | 561 | 126 | 126 | 100% | 100% | 78.6026 m |

This gives a sparse A* reduction of approximately 77.5% relative to the dense
pair count for the same selected docking set. Route degradation is not yet
reported because the paired dense L1 route did not complete in the smoke run.

## Limitations and next gate

The current code establishes the P3/P4 infrastructure and validates it with
unit tests and smoke runs. A full paper-quality result table still requires
resource-bounded batch runs for L1/L2/L3 and the Bidirectional B0/B1/B2/B3
comparison.

Current gate:

```text
P3_P4_NEEDS_ONE_FIX
```

Reason: the sparse infrastructure is functioning, but the formal dense-vs-sparse
and cache-ablation result matrix is not complete yet.
