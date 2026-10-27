# Code Architecture

Main chain: scene -> tasks -> candidates -> dock selection -> route -> footprint validation.

| Layer | Core files/functions | Input -> output |
|---|---|---|
| Environment | config/scene_config.m; environment/build_scene_from_cfg.m; pipeline/stage1_build_scene.m | profile -> scene/grid/tasks |
| Reachability | manipulator/; docking/build_candidate_library.m | robot/scene/tasks -> candidates |
| Docking/task planning | pipeline/stage2_docking_planning.m; planning_strategy/run_planning_strategy.m | candidates -> selected docks/plan |
| Global planning | global_planner/run_path_planner.m; pipeline/stage3_global_planning.m | map/poses -> waypoints |
| Replanning | replanning/apply_dynamic_event.m; replanning/local_replan.m | state/event -> local update |
| Experiment management | experiment_registry.m; experiment_config_builder.m; run_selected_experiment.m | profile/overrides -> result |
| Visualization | visualization/visualization_config.m; render helpers | result -> figures |
| Tests | tests/run_all_tests.m | code -> validation |

Legacy mains are useful for stage debugging; the registered runner is the recommended reproducible entry.
