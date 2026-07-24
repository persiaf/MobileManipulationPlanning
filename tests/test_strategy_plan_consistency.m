function tests = test_strategy_plan_consistency
    tests = functiontests(localfunctions);
end

function testPlanConsistencyAllStrategies(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    names = {'bottom_up', 'top_down', 'bidirectional'};
    for i = 1:numel(names)
        cfg = planner_cfg;
        cfg.strategy.name = names{i};
        cfg.strategy.common.route_shortlist_size = 5;
        plan = run_planning_strategy(lib, regions, scene, robot_cfg, cfg);
        verifyWarningFree(testCase, @() validate_plan_consistency(plan, scene));
        verifyTrue(testCase, isfield(plan, 'service_configs'));
    end
end
