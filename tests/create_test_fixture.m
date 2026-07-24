function [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture()
%CREATE_TEST_FIXTURE Small deterministic scene for automated tests.

    scene_cfg = scene_config();
    scene_cfg.map_width_m = 4.0;
    scene_cfg.map_height_m = 4.0;
    scene_cfg.table_mode = 'custom';
    scene_cfg.table_positions = [2.0, 2.0];
    scene_cfg.table_orientations = 0;
    scene_cfg.num_tables = 1;
    scene_cfg.task_mode = 'manual';
    scene_cfg.manual_tasks = [1, 1.82, 2.12, 0.50; 1, 2.18, 1.88, 0.50];
    scene_cfg.start_pose = [0.5, 0.5, 0];
    scene_cfg.random_seed = 2026;

    scene = build_scene_from_cfg(scene_cfg);
    robot_cfg = robot_config();
    planner_cfg = planner_config();
    planner_cfg.global_planner = 'astar';
    planner_cfg.benchmark.enable = false;
    planner_cfg.execution.return_to_start = false;
    planner_cfg.runtime.headless = true;
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
end
