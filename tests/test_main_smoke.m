function tests = test_main_smoke
    tests = functiontests(localfunctions);
end

function testSmallPlanningPipeline(testCase)
    [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture();
    [poses, ~, summary, pose_info] = stage2_docking_planning( ...
        regions, scene, robot_cfg, planner_cfg);
    timing = struct('scene_time_s', 0, 'docking_time_s', 0);
    [~, path, ~, ~, ~, ~, stats] = stage3_global_planning(scene, poses, ...
        summary, task_table_map, robot_cfg, planner_cfg, pose_info, timing);
    verifyNotEmpty(testCase, poses);
    verifyNotEmpty(testCase, path);
    verifyEqual(testCase, stats.completed, scene.num_tasks);
end
