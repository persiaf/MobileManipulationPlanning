# Baseline Footprint-Aware Navigation Report

## Constraints

Task generation, candidate generation, docking selection, robot dimensions,
reachability, safety thresholds, and the A* implementation were unchanged.
Sparse graph repair now validates additional cross-component candidate edges
with the footprint-aware path planner before adding them.

## Default 16-table baseline

| Metric | Before | After |
|---|---:|---:|
| Candidates | 7893 | 7893 |
| Coverage | 36/36 | 36/36 |
| Selected docks | 27 | 27 |
| Centerline collision | unavailable (graph disconnected) | 0 |
| Footprint collision | unavailable (graph disconnected) | 0 |
| Path length | unavailable | 79.4383 m |
| Waypoints | unavailable | 2158 |
| Runtime | graph blocked | 21.687 s |

## Cafeteria complex12

| Metric | Before | After |
|---|---:|---:|
| Coverage | 12/12 | 12/12 |
| Selected docks | 8 | 8 |
| Centerline collision | 0 | 0 |
| Footprint collision | 0 | 0 |
| Path length | 28.2995 m | 28.0702 m |
| Waypoints | 781 | 775 |
| Runtime | 21.4 s | 20.894 s |

## Unified records

- `results/tables/footprint_baseline_round3.log`
- `results/tables/footprint_cafeteria_round3.log`
- `results/tables/footprint_experiment_record.csv`

`FINAL_STATUS = EXPERIMENT_READY`
