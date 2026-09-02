function tests = test_sparse_cache_semantic_identity
    tests = functiontests(localfunctions);
end

function testSparseCacheOnMatchesCacheOff(testCase)
    [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture();
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    off = run_case(false, scene, regions, task_table_map, robot_cfg, planner_cfg, library);
    on = run_case(true, scene, regions, task_table_map, robot_cfg, planner_cfg, library);

    verifyEqual(testCase, on.plan.covered_tasks_predicted, off.plan.covered_tasks_predicted);
    verifyEqual(testCase, on.visit_order, off.visit_order);
    verifyEqual(testCase, on.total_path, off.total_path, 'AbsTol', 1e-9);
    verifyEqual(testCase, on.stats.sparse_graph.adjacency, off.stats.sparse_graph.adjacency);
end

function result = run_case(enabled, scene, regions, task_table_map, robot_cfg, planner_cfg, library)
    cfg = planner_cfg;
    cfg.routing.mode = 'sparse_knn';
    cfg.strategy.name = 'bidirectional';
    cfg.strategy.bidirectional.use_shared_path_cost_cache = enabled;
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    [~, ~, ~, visit_order, ~, total_path, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    result.plan = plan;
    result.visit_order = visit_order;
    result.total_path = total_path;
    result.stats = stats;
end
