# Scene Complexity Baseline P0

Date: 2026-09-02

Branch base: `research-large-scale-s4-validated-20260902`

This document freezes the current scene-generation baseline before the
scene-complexity scaling line. It records code facts observed in the
current repository, not target values for later experiments.

## Default Scene Config

Source: `config/scene_config.m`

- Map width: `10.0 m`
- Map height: `10.0 m`
- Grid resolution: `0.05 m/cell`
- Border thickness: `0.1 m`
- Table size: `1.200 m x 0.400 m x 0.500 m`
- Table mode: `custom`
- Table count: `16`
- Table orientations: all `0 rad`
- Task mode: `random`
- Task difficulty: `normal`
- Task count rule: `tasks_per_table_range = [2, 3]`
- Random seed: `2026`
- Start pose: `[0.5, 0.5, 0]`
- Docking distance range: `0.25 m` to `0.40 m`
- Docking distance step: `0.05 m`
- Docking lateral step: `0.15 m`

## Default Table Positions

| Table | x (m) | y (m) | theta (deg) |
|---:|---:|---:|---:|
| 1 | 2.0 | 1.5 | 0 |
| 2 | 4.0 | 1.5 | 0 |
| 3 | 6.0 | 1.5 | 0 |
| 4 | 8.0 | 1.5 | 0 |
| 5 | 2.0 | 4.0 | 0 |
| 6 | 4.0 | 4.0 | 0 |
| 7 | 6.0 | 4.0 | 0 |
| 8 | 8.0 | 4.0 | 0 |
| 9 | 2.0 | 6.5 | 0 |
| 10 | 4.0 | 6.5 | 0 |
| 11 | 6.0 | 6.5 | 0 |
| 12 | 8.0 | 6.5 | 0 |
| 13 | 2.0 | 9.0 | 0 |
| 14 | 4.0 | 9.0 | 0 |
| 15 | 6.0 | 9.0 | 0 |
| 16 | 8.0 | 9.0 | 0 |

## Default Discrete Task Candidates

Source: `cfg.task_candidate_local`

| Candidate | local x (m) | local y (m) |
|---:|---:|---:|
| 1 | -0.18 | 0.12 |
| 2 | 0.00 | 0.12 |
| 3 | 0.18 | 0.12 |
| 4 | -0.18 | -0.12 |
| 5 | 0.00 | -0.12 |
| 6 | 0.18 | -0.12 |

Task local-to-world conversion in the dynamic builder uses the table
SE(2) transform:

```matlab
world = rot2(theta) * local;
task_xy = table_center + world';
```

## Existing Scaled Scenes

Source: `utils/create_scaled_scene_config.m`

| Scale | Map | Grid | Tables | Spacing | Task Rule |
|---|---:|---:|---:|---:|---|
| `small` | `6 x 6 m` | `2 x 2` | 4 | `2.5 m` | `[2, 3]` |
| `medium` | `9 x 9 m` | `3 x 3` | 9 | `2.5 m` | `[2, 3]` |
| `large` | `12 x 12 m` | `4 x 4` | 16 | `2.5 m` | `[2, 3]` |

`experiments/large_scale_profile_config.m` maps these scaled scenes to
S1-S4. S1 uses the medium scene and 36 requested tasks. S2-S4 use the large
scene with requested task counts of 60, 90, and 120 respectively.

## Existing Hard Task Modes

Source: `utils/generate_hard_table_tasks.m`

- `normal`: random selection from `task_candidate_local`
- `hard_corner`: samples near table corners
- `hard_two_side`: samples on opposite long edges
- `hard_reach_boundary`: samples around table edges
- `hard_mixed`: mixes corner, two-side, and reach-boundary samples using
  `task.hard_mixed_ratio = [0.4, 0.4, 0.2]`

These modes already use the table rotation when converting local task points
to world coordinates.

## Robot Geometry Used By Planning

Source: `config/robot_config.m`

- Chassis length: `0.423 m`
- Chassis width: `0.366 m`
- Chassis height: `0.127 m`
- Dual-arm base spacing: `0.250 m`
- Single-arm reach radius: `0.278 m`
- Single-arm forward reach: `0.278 m`
- Single-arm backward reach: `0.197 m`
- Single-arm lateral reach: `0.278 m`
- Arm base mount x: `cfg.chassis.length / 2`
- Arm base mount z: `cfg.chassis.height + cfg.arm_dh.d1`

This P0 freeze does not change robot geometry, reachability thresholds,
collision margins, dock-selection semantics, planner semantics, or
`convex_astar`.

## Unified P1 Metric Contract

All scene-complexity experiments should report the following fields. Use
`NaN`/`N/A` when a field cannot be reliably obtained.

- Map width and height
- Table count, task count, and region count
- Raw, collision-valid, reachability-valid, final-library, and supplementary
  candidate counts
- Selected docking pose count
- Candidate-library coverage
- Verified completion
- Task, return, and total path length
- Candidate generation time, strategy time, path-cost time, and total planning
  time
- Logical path-cost queries
- Expensive planner calls
- Cache hits and misses
