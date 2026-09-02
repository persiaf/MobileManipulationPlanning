function tests = test_position_augmentation_rotated_table_frame
%TEST_POSITION_AUGMENTATION_ROTATED_TABLE_FRAME Position-only uses table frame.
    tests = functiontests(localfunctions);
end

function testSupplementaryPoseTracksRotatedTask(testCase)
    scene_cfg = scene_config();
    scene_cfg.map_width_m = 4.0;
    scene_cfg.map_height_m = 4.0;
    scene_cfg.table_mode = 'custom';
    scene_cfg.table_positions = [2.0, 2.0];
    scene_cfg.table_orientations = deg2rad(45);
    scene_cfg.num_tables = 1;
    scene_cfg.task_mode = 'random';
    scene_cfg.task.position_mode = 'continuous';
    scene_cfg.task.difficulty = 'normal';
    scene_cfg.task.continuous_local_bounds = [0.20, 0.20; 0.10, 0.10];
    scene_cfg.tasks_per_table_range = [1, 1];
    scene_cfg.random_seed = 2026;
    scene = build_scene_from_cfg(scene_cfg);
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    robot_cfg = robot_config();
    planner_cfg = planner_config();
    planner_cfg.candidate_augmentation.enable = true;
    planner_cfg.candidate_augmentation.variant = 'position_only';
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    diag = library.augmentation_diagnostics;
    verifyGreaterThanOrEqual(testCase, diag.raw_supplementary_count, 1);
    if ~isempty(diag.selected_poses)
        pose = diag.selected_poses(1, :);
        center = scene.scene_cfg.table_positions(1, :);
        pose_local = (rot2(-scene.scene_cfg.table_orientations(1)) * (pose(1:2) - center)')';
        verifyGreaterThan(testCase, abs(pose_local(1)), 0.05);
    end
end
