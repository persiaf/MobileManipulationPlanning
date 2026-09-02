# Scene Complexity P1

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

This checkpoint adds a parameterized scene-generation layer for testing
scene-structure complexity while keeping the map scale and table count
roughly fixed. It does not modify the planning algorithms or tuning
parameters frozen in the validated large-scale checkpoint.

## Added Layout Modes

Implemented in `utils/generate_table_layout.m`:

- `grid`: deterministic regular grid baseline
- `jittered_grid`: regular grid plus bounded xy perturbation
- `cluster`: separated workspace clusters with local table groups

The default jitter range for scene-complexity configs is `0.20 m` and is
stored in `cfg.layout.jitter_range_m`.

## Added Orientation Modes

- `fixed`: all tables use `cfg.layout.fixed_orientation_rad`, default `0`
- `discrete_random`: samples from configurable degree candidates

The default discrete set is:

```text
0, +30, -30, +45, -45, 90 degrees
```

Continuous random orientation is intentionally not enabled in P1.

## Added Task Modes

Implemented in `utils/generate_scene_tasks.m`:

- `task.position_mode = 'discrete'`: preserves random selection from
  `cfg.task_candidate_local`
- `task.position_mode = 'continuous'`: samples local tabletop points inside
  bounds derived from table length, table width, and `task.edge_margin_m`

Existing difficulty names are preserved:

- `normal`
- `hard_corner`
- `hard_two_side`
- `hard_reach_boundary`
- `hard_mixed`

The hard modes continue to use existing geometry helpers and the same table
rotation transform as the baseline.

## Layout Validity

Implemented in `utils/validate_table_layout.m`.

Checks include:

- invalid or duplicate positions
- invalid orientations
- rotated table boundary collision
- rotated table-table overlap
- minimum aisle clearance

`build_scene_from_cfg` now performs bounded retry for `table_mode = 'layout'`
and stores the structured result in `scene.scene_cfg.layout_validation`.

## P1 Scene Set

Implemented in `experiments/scene_complexity_profile_config.m`.

| Case | Layout | Orientation | Task Sampling | Difficulty |
|---|---|---|---|---|
| C0 | `grid` | `fixed` | `discrete` | `normal` |
| C1 | `jittered_grid` | `fixed` | `discrete` | `normal` |
| C2 | `jittered_grid` | `discrete_random` | `continuous` | `normal` |
| C3 | `cluster` | `discrete_random` | `continuous` | `hard_mixed` |

All four cases use a `10 x 10 m` map, 16 tables, and `3-6` tasks per table.
With seed `2026`, the smoke run generated 75 tasks for C0.

## Experiment Driver

Implemented in `experiments/exp_scene_complexity_p1.m`.

The driver records the unified P1 fields and can compare:

- `augmentation_mode = 'off'`
- `augmentation_mode = 'position_only'`
- `bottom_up`
- `top_down`
- `bidirectional`

It writes:

- `results/tables/scene_complexity_p1_raw.csv`
- `results/tables/scene_complexity_p1_summary.csv`

The raw CSV is a local experiment product. The compact summary CSV is the
intended tracked result file for this checkpoint.

## Current Bottom-Up Summary

`results/tables/scene_complexity_p1_summary.csv` records the controlled
P1 matrix that was completed in this checkpoint:

| Case | Augmentation | Candidate Coverage | Final Candidates | Selected Docks | Strategy-Level Time |
|---|---|---:|---:|---:|---:|
| C0 | off | 100.00% | 7134 | 31 | 7.66 s |
| C0 | position_only | 100.00% | 7134 | 31 | 7.44 s |
| C1 | off | 100.00% | 6849 | 31 | 7.32 s |
| C1 | position_only | 100.00% | 6849 | 31 | 7.19 s |
| C2 | off | 55.38% | 3812 | 29 | 8.08 s |
| C2 | position_only | 60.00% | 3815 | 31 | 12.42 s |
| C3 | off | 100.00% | 5291 | 29 | 9.21 s |
| C3 | position_only | 100.00% | 5291 | 29 | 9.17 s |

C2 exposes the first P1 geometry gap: jittered layout plus discrete random
orientation plus continuous tasks lowers baseline candidate coverage, and
position-only augmentation provides only a small recovery. This is a
candidate-generation/reachability-boundary issue, not a scene-generator
validity issue.

## Full-Path Smoke Result

The C0 bottom-up full-path smoke completed successfully:

- tasks: 75
- candidate-library coverage: 100%
- verified completion: 100%
- selected docking poses: 31
- total path length: 69.05 m
- total measured planning time: 63.20 s

This is a baseline reproducibility smoke, not a full C0-C3 path-length
conclusion.

## Three-Strategy Runtime Note

A C0 top-down strategy-level run completed with 100% coverage but took about
208 s before stage3 path stitching. The full C0-C3 x augmentation x
three-strategy matrix was therefore not forced inside this P1 generator
checkpoint. The experiment driver supports it, but completing that matrix
should be treated as the next dedicated compute run rather than as scene
generator implementation work.

## Tests Added

- `test_scene_layout_determinism`
- `test_scene_layout_no_overlap`
- `test_scene_layout_inside_map`
- `test_rotated_table_task_transform`
- `test_continuous_task_inside_table`
- `test_scene_pattern_smoke`
- `test_scene_complexity_experiment_smoke`

These tests are scoped to scene generation, geometry consistency, and a small
experiment-driver smoke. They do not alter planner semantics.
