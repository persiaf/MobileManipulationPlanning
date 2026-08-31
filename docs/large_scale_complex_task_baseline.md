# Large-Scale Complex-Task Baseline

This experiment profiles the existing Bottom-Up, Top-Down, and Bidirectional planners as scene size grows. It is a measurement baseline, not a new planning method. All strategies use the same generated scene and shared candidate library for each seed.

## Scope and controls

- A single operating height is used for all tasks.
- Lift planning, lift timing, and continuous base/lift/arm co-optimization are excluded.
- The global planner remains the configured `convex_astar` baseline.
- Random seeds are explicit (`1:5` by default); each scale/seed is paired across strategies.
- Requested task counts are targets. The generated scene records both `requested_task_count` and `actual_task_count` because per-table random sampling can produce a nearby count.

The default profiles are `s1` (target 6 tasks), `s2` (30), `s3` (60), and `s4` (120). The driver uses the existing `create_scaled_scene_config` and expands task samples only for the 120-task profile.

## Run

```matlab
[raw, summary] = exp_large_scale_complex_tasks();
```

For a one-scale smoke check:

```matlab
[raw, summary] = exp_large_scale_complex_tasks(struct( ...
    'quick_mode', true, 'scales', {{'s1'}}, 'seeds', 1));
```

## Outputs

- `results/tables/large_scale_complex_tasks_raw.csv`: one row per scale, seed, and strategy, including failures.
- `results/tables/large_scale_complex_tasks_summary.csv`: mean, standard deviation, and median by scale and strategy.
- `results/figures/fig_large_scale_total_time.png`: task count versus total planning time.
- `results/figures/fig_large_scale_stage_time.png`: stage-time breakdown for Bottom-Up.
- `results/figures/fig_large_scale_path_cost_diagnostics.png`: path-cost queries and cache-hit ratio.

The raw schema records scene, region, task, candidate, timing, path, coverage, objective, cache, success, and failure-reason fields. A failed run remains in the raw CSV with `success=false` and a diagnostic `failure_reason`; it is not silently discarded from the experiment record.

## Interpretation boundary

Do not assume in advance that candidate generation, path-cost evaluation, strategy selection, route construction, or Bidirectional refinement is the bottleneck. Use the measured `T_*`, candidate counts, path-cost query counts, and cache ratios to select the next optimization target. This stage does not implement optimization changes.
