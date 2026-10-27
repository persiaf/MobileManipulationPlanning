# Core File Index

| Category | File | Purpose | User recommendation |
|---|---|---|---|
| ENTRY POINT | main_mobile_manipulation_planning.m | Legacy staged baseline | Yes for debugging |
| ENTRY POINT | main_cafeteria_mobile_manipulation_planning.m | Cafeteria entry | Yes |
| ENTRY POINT | experiment_manager/run_selected_experiment.m | Unified runner | Yes, recommended |
| EXPERIMENT | experiment_manager/experiment_registry.m | Registry metadata | Yes |
| EXPERIMENT | experiment_manager/experiment_config_builder.m | Config builder | Yes |
| EXPERIMENT | experiment_manager/launch_mobile_manipulation_experiment_manager.m | GUI launcher | Yes |
| CONFIG | config/scene_config.m; robot_config.m; planner_config.m | Base config | Modify deliberately |
| PLANNING | planning_strategy/run_planning_strategy.m | Strategy dispatch | Usually no |
| PLANNING | global_planner/run_path_planner.m | Planner dispatch | Usually no |
| REPLANNING | experiments/exp_dynamic_local_replanning_p5.m | P5 validation | Yes for Stage 3 |
| VISUALIZATION | visualization/visualization_config.m | Canonical colors/options | Yes |
| TEST | tests/run_all_tests.m | Test suite | Yes |
| COMPATIBILITY | config/legacy_visualization_config.m | Legacy adapter | No |
