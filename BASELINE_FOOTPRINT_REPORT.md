> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

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
