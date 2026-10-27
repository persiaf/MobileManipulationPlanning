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
    verifyEqual(testCase, stats.rc2_plan.covered_tasks, 1:scene.num_tasks);
    verifyEqual(testCase, sort(stats.rc2_plan.visit_order), 1:size(stats.rc2_plan.selected_docks,1)+1);
    verifyEqual(testCase, sort(stats.rc2_plan.route_order), 1:size(stats.rc2_plan.selected_docks,1)+1);
    verifyEqual(testCase, stats.rc2_plan.global_path, path);
    verifyEqual(testCase, [stats.planning_state.tasks.id], [scene.tasks.id]);
    verifyTrue(testCase, isfinite(stats.objective.final));
end
