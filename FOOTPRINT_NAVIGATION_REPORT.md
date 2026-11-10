> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

# Footprint-Aware Global Navigation Report

## 2026-10-28: default main-scene connectivity repaired

The default 16-table main program now completes without changing scene
geometry, robot dimensions, the 27 selected docks, or safety margins. If the
all-heading circumscribed-disk search fails, navigation tries rectangular
configuration-space slices at fixed headings. Each recovered route, including
endpoint translations and rotations, is sampled and footprint-checked on the
original safety-dilated scene map. This is a limited holonomic recovery, not a
complete SE(2) search; genuinely blocked paths are still rejected. Disable it
with `planner_cfg.navigation.fixed_heading_fallback = false` for comparisons.

The actual main script and its four default benchmark algorithms passed:
36/36 tasks verified, 27 docks, 90.581503 m main route including return, zero
footprint collisions, and exact endpoint position/yaw matching. The sparse
graph retained 91 valid edges and rejected 6 failed candidates. All 22 targeted
regression tests passed, including an impassable-gap rejection. Run records
are under `results/logs/main_scene_repair_20261028/` (local generated files).
The following sections describe the earlier implementation and limitation.

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
