# Project Call Graph

## Main pipeline

```text
main_mobile_manipulation_planning
  -> init_paths
  -> stage1_build_scene
       -> scene_config / robot_config / planner_config
       -> build_scene -> build_grid_map -> build_task_regions
  -> stage2_docking_planning
       -> build_candidate_library -> generate_docking_candidates
       -> run_planning_strategy
            -> plan_bottom_up | plan_top_down | plan_bidirectional
  -> stage3_global_planning
       -> compute_path_cost_matrix | build_sparse_docking_graph
       -> nearest_neighbor_order
       -> stitch_global_path | stitch_sparse_graph_path
       -> estimate_execution_time -> simulate_planned_task_execution
  -> stage4_benchmark_all
  -> draw_comprehensive_overview / stage5_generate_video
```

| Caller | Callee | File | Call type | Notes |
|---|---|---|---|---|
| main script | stage1_build_scene | `main_mobile_manipulation_planning.m` | direct | Builds base scene/configs |
| stage1_build_scene | scene/robot/planner config | `pipeline/stage1_build_scene.m` | direct | Base config layer |
| stage2_docking_planning | build_candidate_library | `pipeline/stage2_docking_planning.m` | direct | Shared candidates for all strategies |
| build_candidate_library | generate_docking_candidates | `planning_strategy/common/build_candidate_library.m` | direct | Candidate geometry and reachability |
| stage2_docking_planning | run_planning_strategy | `pipeline/stage2_docking_planning.m` | dispatch | Bottom-Up/Top-Down/Bidirectional |
| stage3_global_planning | build_sparse_docking_graph | `pipeline/stage3_global_planning.m` | conditional | `routing.mode=sparse_knn` |
| stage3_global_planning | compute_path_cost_matrix | `pipeline/stage3_global_planning.m` | conditional | Dense route cost matrix |
| stage3_global_planning | stitch sparse/global path | `pipeline/stage3_global_planning.m` | conditional | Produces stored collision-free waypoints |
| stage3_global_planning | execution estimate/simulation | `pipeline/stage3_global_planning.m` | direct | Does not control hardware |

## P3/P4 sparse routing

```text
large_scale_profile_config(L1/L2/L3)
  -> build_scene_from_cfg
prepare_p3p4_validation_state / run_p3p4_validation_route
  -> planner_config + explicit routing/cache overrides
  -> run_planning_strategy
  -> stage3_global_planning
       -> build_sparse_docking_graph
            -> query_path_with_shared_cache -> run_path_planner
            -> validate_sparse_graph_connectivity
       -> stitch_sparse_graph_path
```

The sparse adjacency lines are topological connectivity. `stitch_sparse_graph_path` expands each selected topology edge through its stored planner waypoint path; `stats.global_path` is therefore the actual route polyline.

## P5 local replanning

```text
exp_dynamic_local_replanning_p5 / demo_dynamic_replanning
  -> dynamic_event_config or frozen P3/P4 state
  -> apply_dynamic_event
  -> analyze_change_impact
  -> local_replan
       -> repair affected docks/edges only
       -> validate sparse connectivity and route collision
  -> visualize_dynamic_replan
```

## V1/V2/V3 visualization

```text
demo_complex_scene(V1) -> build_complex_visual_scene -> visualize_complex_scene
demo_complex_planning(V2) -> stage2/stage3 -> visualize_planning_overlay
demo_dynamic_replanning(V3) -> local_replan -> visualize_dynamic_replan
visualize_planning_overlay
  -> draw_sparse_graph_connectivity  [topology]
  -> draw_selected_docks
  -> draw_collision_free_route       [stored waypoints]
  -> build_scene_legend / save_visualization_figure
```

## Real-platform interface

```text
offline plan
  -> build_service_config
  -> export_mission_plan_json
  -> external ROS/navigation/manipulation executor
  -> import_ros_execution_log
```

`robot_config` defines the planning model. `real_platform_config` defines the serialization/execution contract and is intentionally separate. MATLAB does not issue real robot commands.

## Static-analysis boundary

The complete caller/callee columns are in `results/tables/project_function_inventory.csv`. Script execution, callbacks, local functions, and class method calls require manual interpretation and are not declared unused solely from this graph.
