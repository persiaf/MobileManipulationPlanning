function tests = test_preferred_visit_order
    tests = functiontests(localfunctions);
end

function testStage3UsesPreferredOrder(testCase)
    [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture();
    planner_cfg.strategy.name = 'top_down';
    planner_cfg.strategy.common.route_shortlist_size = 5;
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(lib, regions, scene, robot_cfg, planner_cfg);
    timing = struct('scene_time_s', 0, 'docking_time_s', 0, ...
        'candidate_generation_time_s', lib.total_generation_time_s);
    [~, ~, ~, visit_order, ~, ~, stats] = stage3_global_planning(scene, ...
        plan.selected_poses, plan.task_sequence_summary, task_table_map, robot_cfg, ...
        planner_cfg, plan.pose_info_list, timing, plan);
    verifyEqual(testCase, visit_order, plan.preferred_visit_order);
    verifyEqual(testCase, stats.route_order_source, 'strategy_preferred');
end
