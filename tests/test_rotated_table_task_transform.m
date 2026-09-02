function tests = test_rotated_table_task_transform
%TEST_ROTATED_TABLE_TASK_TRANSFORM Verify local task points use SE(2) rotation.
    tests = functiontests(localfunctions);
end

function testDiscreteTaskTransformAngles(testCase)
    angles = [0, 90, 45, -45];
    local = [0.30, 0.10];
    center = [2.0, 2.0];
    for deg = angles
        cfg = base_single_table_cfg(center, deg, local);
        scene = build_scene_from_cfg(cfg);
        expected = center + (rot2(deg2rad(deg)) * local')';
        verifyEqual(testCase, [scene.tasks(1).x, scene.tasks(1).y], expected, 'AbsTol', 1e-12);
    end
end

function cfg = base_single_table_cfg(center, deg, local)
    cfg = scene_config();
    cfg.map_width_m = 4.0;
    cfg.map_height_m = 4.0;
    cfg.table_mode = 'custom';
    cfg.table_positions = center;
    cfg.table_orientations = deg2rad(deg);
    cfg.num_tables = 1;
    cfg.task_mode = 'random';
    cfg.task.position_mode = 'discrete';
    cfg.task.difficulty = 'normal';
    cfg.task_candidate_local = local;
    cfg.tasks_per_table_range = [1, 1];
    cfg.random_seed = 2026;
    cfg.table_wall_margin_m = 0.4;
end
