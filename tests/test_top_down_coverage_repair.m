function tests = test_top_down_coverage_repair
    tests = functiontests(localfunctions);
end

function testRepairDoesNotMarkImpossibleTasks(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    planner_cfg.strategy.name = 'top_down';
    planner_cfg.strategy.top_down.max_docks_per_region = 1;
    planner_cfg.strategy.top_down.max_repair_docks_per_region = 3;
    planner_cfg.strategy.common.route_shortlist_size = 3;
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(lib, regions, scene, robot_cfg, planner_cfg);
    verifyTrue(testCase, all(ismember(plan.covered_tasks_predicted, 1:scene.num_tasks)));
    verifyEqual(testCase, numel(unique(plan.covered_tasks_predicted)), numel(plan.covered_tasks_predicted));
end
