function tests = test_strategy_comparison_smoke
    tests = functiontests(localfunctions);
end

function testComparisonLogicOnFixture(testCase)
    [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture();
    lib = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    names = {'bottom_up', 'top_down', 'bidirectional'};
    for i = 1:numel(names)
        cfg = planner_cfg;
        cfg.strategy.name = names{i};
        cfg.strategy.common.route_shortlist_size = 5;
        plan = run_planning_strategy(lib, regions, scene, robot_cfg, cfg);
        timing = struct('scene_time_s', 0, 'docking_time_s', 0, ...
            'candidate_generation_time_s', lib.total_generation_time_s);
        [~, path, ~, ~, ~, ~, stats] = stage3_global_planning(scene, ...
            plan.selected_poses, plan.task_sequence_summary, task_table_map, robot_cfg, ...
            cfg, plan.pose_info_list, timing, plan);
        verifyNotEmpty(testCase, path);
        verifyEqual(testCase, stats.strategy_name, names{i});
    end
end
