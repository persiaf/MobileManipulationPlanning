# Complex Scene Construction and Visualization

This visualization stage adds presentation-oriented V1/V2/V3 profiles and a
single MATLAB visualization interface without changing P0-P5 planning
semantics or formal experiment results.

## Profiles

| profile | map | tables | tasks at seed 2026 | regions | static obstacles | purpose |
|---|---:|---:|---:|---:|---:|---|
| V1 | 15 x 15 m | 28 | 131 | 7 | 0 | medium complex scene |
| V2 | 20 x 20 m | 40 | 181 | 7 | 4 | main paper/PPT high-complexity scene |
| V3 | 15 x 15 m | 28 | about 126 | 7 | P5 event obstacle | dynamic local replanning view |

Existing C0-C3 and L1-L3 profiles are unchanged. V2 uses deterministic cluster
layout, bounded jitter, discrete table orientations, continuous hard-mixed
tasks, and deterministic block obstacles represented directly in the occupancy
grid (`grid_map == 1`).

## Visualization entry points

```matlab
visualization_config
visualize_complex_scene
visualize_planning_overlay
visualize_dynamic_replan
```

The unified config supports map, region labels, rotated tables, table headings,
tasks, selected docks, sparse graph, final route, static obstacles, start pose,
legend/grid toggles, candidate stride, and PNG/FIG export.

Candidates are hidden by default to avoid clutter in scenes with thousands of
candidate poses.

## Demo commands

From the project root:

```matlab
init_paths
demo_complex_scene
demo_complex_planning
demo_dynamic_replanning
```

Generated PNGs:

```text
results/figures/complex_scene/V1_scene.png
results/figures/complex_scene/V2_scene.png
results/figures/complex_scene/V2_planning.png
results/figures/complex_scene/V3_dynamic_before.png
results/figures/complex_scene/V3_dynamic_after.png
```

## Validation

The visualization smoke tests verify deterministic profile generation, V2
layout validity, multiple regions, multiple table orientations, static
obstacles, scene rendering, planning overlay rendering, dynamic rendering, and
that visualization does not mutate its input scene.
