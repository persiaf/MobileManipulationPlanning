# Visualization Architecture

## Canonical configuration and primitives

New complex/paper visualization reads `visualization_config()`. Original stage drawing receives `legacy_visualization_config()` through `planner_cfg.visualization`. Shared maintained primitives are `oriented_rectangle_corners`, `draw_sparse_graph_connectivity`, `draw_selected_docks`, `draw_collision_free_route`, `build_scene_legend`, and `save_visualization_figure`.

| Function group | Files | Purpose | Config source | Used by | Scene | Candidates | Docks | Sparse graph | Route | Action |
|---|---|---|---|---|---:|---:|---:|---:|---:|---|
| V1 scene | `visualize_complex_scene` | region/table/task/static-obstacle view | canonical | V1/V2/V3 demos | yes | no | no | no | no | canonical scene visualizer |
| V2 overlay | `visualize_planning_overlay` | planning overlay | canonical | complex planning, dynamic view | via V1 | no | yes | yes | yes | canonical overlay |
| V3 dynamic | `visualize_dynamic_replan` | before/after event and affected region | canonical | P5 demo | via V2 | no | yes | yes | actual pre-change path plus repair annotations | retain |
| topology primitive | `draw_sparse_graph_connectivity` | straight adjacency lines | none | V2/V3 | no | no | no | yes | no | label as topology |
| actual route primitive | `draw_collision_free_route` | stored A*/planner waypoints | none | V2/V3 | no | no | no | no | yes | label collision-free route |
| dock primitive | `draw_selected_docks` | dock markers/headings | none | V2/V3 | no | no | yes | no | no | canonical maintained dock marker |
| legend/save | `build_scene_legend`, `save_visualization_figure` | deduplicated legend and output | canonical | V1/V2/V3 | n/a | n/a | n/a | n/a | n/a | shared |
| old stage scene | `draw_scene`, `draw_scene_background`, `plot_grid` | grid/table/task stage rendering | legacy adapter | stages 1-3 | yes | no | no | no | no | `LEGACY_COMPAT` |
| old candidate/dock | `draw_docking_result`, `draw_reachability_field`, `draw_local_manipulation_coverage`, `draw_robot` | detailed research diagnostics | legacy adapter | stage 2 and diagnostic use | partial | yes | yes | no | no | retain compatibility |
| old overview/path | `draw_comprehensive_overview`, `draw_global_path`, `animate_task_execution` | final stage summary/video | legacy adapter | main pipeline | yes | no | yes | no | actual waypoints | retain compatibility |
| grid compatibility | `plot_path`, `plot_square`, root `visualize_single_algorithm` | individual algorithm debug | explicit arguments | debug entry | grid | no | endpoints | no | grid path | retain |
| paper scenario | `plot_main_scenario_publication`, `plot_normal_vs_hard_scene`, `plot_normal_vs_hard_task_distribution` | paper scene panels | local publication options plus core configs | paper scripts | yes | varies | varies | no | actual path where present | migrate only with paper regeneration |
| result plots | `plot_ablation_bar`, `plot_hard_ablation_comparison`, `plot_hard_scenario_comparison`, `plot_scalability_curves` | CSV result charts | local plotting arguments | publication scripts | no | no | no | no | no | distinct from scene drawing |
| framework/style | `plot_framework_pipeline`, `apply_publication_style`, `format_method_label`, `plot_expand`, `save_publication_figures` | diagram/style/export | local | paper assets | no | no | no | no | no | retain |

## Route semantics

| Function/data | Data source | Drawn object |
|---|---|---|
| `draw_sparse_graph_connectivity` | `stats.sparse_graph.adjacency` and pose endpoints | `TOPOLOGICAL_EDGE` |
| dock visitation markers/labels | `stats.route_order` / selected poses | `DOCK_VISITATION_SEQUENCE` |
| `stitch_global_path` output | per-segment planner waypoint arrays | `ACTUAL_COLLISION_FREE_PATH` |
| `stitch_sparse_graph_path` output | stored waypoint path for expanded sparse edges | `ACTUAL_COLLISION_FREE_PATH` |
| `stats.global_path` | one of the two stitch functions | `ACTUAL_COLLISION_FREE_PATH` |
| P5 invalid edge overlay | change-impact edge indices | `INVALIDATED_TOPOLOGICAL_EDGE` |
| P5 repaired route state | local replan sparse data/path validity fields | repaired connectivity and collision validity |

The old phrase “final route” was ambiguous but the V2 data was already the stored waypoint path, not direct dock chords. The maintained legend now says `Collision-free route`; sparse straight lines say `Sparse graph connectivity`. The test `test_actual_path_visualization_uses_waypoints` verifies every stored waypoint reaches the plotted line unchanged.

## Duplicate drawing audit

The repeated oriented table/block transform is now centralized in `utils/oriented_rectangle_corners.m` for maintained visualizers and old overview/background views. `validate_scene_config` retains its validation-specific corner loop; changing it offers no algorithm benefit. Old stage and paper renderers remain intentionally separate because their visual contracts differ and regenerating historical figures is outside this pass.

## Legend policy

Repeated geometry uses hidden or empty display names. `build_scene_legend` selects one handle per nonempty name. Canonical names are `Table`, `Static obstacle`, `Task`, `Start`, `Selected dock`, `Sparse graph connectivity`, and `Collision-free route`; dynamic overlays add changed obstacle, invalidated edge, and affected region entries.
