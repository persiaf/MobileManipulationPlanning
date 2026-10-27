# Footprint-Aware Global Navigation Report

## Scope

Global navigation now receives a footprint-inflated search map through the
existing `run_path_planner` entry point. The A* and other planner algorithms
were not modified. Candidate generation, task definitions, reachability,
docking selection, safety margins, and scene geometry were not changed.

The inflated map uses the configured rectangular chassis dimensions
`0.423 m x 0.366 m` and a conservative circumscribed radius of `0.2797 m`
(6 cells at the Cafeteria resolution of `0.05 m`). The original scene map is
retained for task/dock generation and independent footprint validation.

## Cafeteria complex12

| Metric | Before | After |
|---|---:|---:|
| Task coverage | 12/12 | 12/12 |
| Selected docks | 8 | 8 |
| Path length | 30.3128 m | 28.2995 m |
| Waypoints | 829 | 781 |
| Centerline collision | 0 | 0 |
| Rectangle footprint collision | 271 | 0 |
| Runtime | 21.1 s | 21.4 s |

The after-run waypoint-level validation is stored in
`results/cafeteria/footprint_validation/footprint_collision_report.csv`.

## Default 16-table baseline

The candidate and docking stages remain unchanged:

- candidates: `7893`
- task coverage: `36/36`
- selected docks: `27`

With footprint inflation enabled, the existing sparse docking graph could not
connect all selected dock endpoints. Consequently no valid global path,
path length, or collision count was reported for the footprint-aware run.
This is reported as a baseline connectivity limitation rather than being
hidden or repaired by changing the task, dock set, scene, reachability, or
safety threshold.

## Implementation

- `config/robot_config.m`: canonical footprint dimensions mirror chassis dimensions.
- `environment/build_footprint_inflated_map.m`: conservative map inflation.
- `global_planner/run_path_planner.m`: passes the inflated map into the existing planner implementation.

`FINAL_STATUS = CAFETERIA_FOOTPRINT_SAFE; DEFAULT_BASELINE_FOOTPRINT_GRAPH_BLOCKED`
