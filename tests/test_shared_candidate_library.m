function tests = test_shared_candidate_library
    tests = functiontests(localfunctions);
end

function testThreeStrategiesShareLibrary(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    names = {'bottom_up', 'top_down', 'bidirectional'};
    signatures = strings(1, numel(names));
    totals = zeros(1, numel(names));
    for i = 1:numel(names)
        cfg = planner_cfg;
        cfg.strategy.name = names{i};
        cfg.strategy.common.route_shortlist_size = 5;
        plan = run_planning_strategy(lib, regions, scene, robot_cfg, cfg);
        verifyNotEmpty(testCase, plan.selected_poses);
        signatures(i) = string(lib.scene_signature);
        totals(i) = lib.total_candidates;
    end
    verifyEqual(testCase, numel(unique(signatures)), 1);
    verifyEqual(testCase, numel(unique(totals)), 1);
end
