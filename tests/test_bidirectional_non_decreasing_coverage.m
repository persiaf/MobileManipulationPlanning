function tests = test_bidirectional_non_decreasing_coverage
    tests = functiontests(localfunctions);
end

function testBidirectionalKeepsBottomUpCoverage(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    bottom = plan_bottom_up(lib, regions, scene, robot_cfg, planner_cfg);
    planner_cfg.strategy.name = 'bidirectional';
    bi = run_planning_strategy(lib, regions, scene, robot_cfg, planner_cfg);
    verifyGreaterThanOrEqual(testCase, numel(bi.covered_tasks_predicted), ...
        numel(bottom.covered_tasks_predicted));
end
