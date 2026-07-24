function tests = test_top_down_region_order
    tests = functiontests(localfunctions);
end

function testRegionOrderUsesCandidateAnchors(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    planner_cfg.strategy.name = 'top_down';
    planner_cfg.strategy.common.route_shortlist_size = 5;
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(lib, regions, scene, robot_cfg, planner_cfg);
    verifyEqual(testCase, sort(plan.region_order), 1:numel(regions));
    verifyEqual(testCase, sort(plan.preferred_visit_order), 1:(size(plan.selected_poses, 1) + 1));
end
