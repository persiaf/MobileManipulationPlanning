function tests = test_bottom_up_regression
    tests = functiontests(localfunctions);
end

function testBottomUpLegacyAndStrategyMatch(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = plan_bottom_up(lib, regions, scene, robot_cfg, planner_cfg);
    [poses, ~, summary] = stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, lib);
    verifyEqual(testCase, size(poses), size(plan.selected_poses));
    verifyLessThan(testCase, max(abs(poses(:) - plan.selected_poses(:))), 1e-9);
    verifyEqual(testCase, collect_tasks(summary), plan.covered_tasks_predicted);
end

function tasks = collect_tasks(summary)
    tasks = [];
    for i = 1:numel(summary)
        tasks = union(tasks, summary{i}.covered_tasks, 'stable');
    end
end
