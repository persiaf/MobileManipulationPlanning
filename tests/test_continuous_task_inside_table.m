function tests = test_continuous_task_inside_table
%TEST_CONTINUOUS_TASK_INSIDE_TABLE Continuous task samples remain on tabletop.
    tests = functiontests(localfunctions);
end

function testContinuousTasksRemainInsideLocalBounds(testCase)
    cfg = scene_complexity_profile_config('C2', 2026);
    scene = build_scene_from_cfg(cfg);
    margin = cfg.task.edge_margin_m;
    half_L = cfg.table.length / 2 - margin;
    half_W = cfg.table.width / 2 - margin;
    for i = 1:scene.num_tasks
        tid = scene.tasks(i).table_id;
        center = scene.scene_cfg.table_positions(tid, :);
        theta = scene.scene_cfg.table_orientations(tid);
        local = (rot2(-theta) * ([scene.tasks(i).x, scene.tasks(i).y] - center)')';
        verifyGreaterThanOrEqual(testCase, local(1), -half_L - 1e-12);
        verifyLessThanOrEqual(testCase, local(1), half_L + 1e-12);
        verifyGreaterThanOrEqual(testCase, local(2), -half_W - 1e-12);
        verifyLessThanOrEqual(testCase, local(2), half_W + 1e-12);
    end
end
