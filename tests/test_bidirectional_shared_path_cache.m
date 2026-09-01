function tests = test_bidirectional_shared_path_cache
%TEST_BIDIRECTIONAL_SHARED_PATH_CACHE Cache must preserve planning semantics.
    tests = functiontests(localfunctions);
end

function testCacheOnMatchesCacheOff(testCase)
    [scene, robot_cfg, planner_cfg, regions, task_table_map] = create_test_fixture();
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    baseline = run_case(false, scene, regions, task_table_map, robot_cfg, planner_cfg, library);
    cached = run_case(true, scene, regions, task_table_map, robot_cfg, planner_cfg, library);

    verifyEqual(testCase, cached.plan.covered_tasks_predicted, baseline.plan.covered_tasks_predicted);
    verifyEqual(testCase, cached.plan.selected_poses, baseline.plan.selected_poses, 'AbsTol', 1e-12);
    verifyEqual(testCase, cached.visit_order, baseline.visit_order);
    verifyEqual(testCase, cached.total_path, baseline.total_path, 'AbsTol', 1e-12);
    verifyEqual(testCase, cached.plan.objective_value, baseline.plan.objective_value, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, cached.cache.cache_hit_count, 0);
    verifyGreaterThan(testCase, cached.cache.stage3_cache_hits, 0);
    verifyLessThan(testCase, cached.cache.expensive_planner_calls, baseline.expensive_calls);
end

function testDirectedKeysRemainDistinct(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    context = SharedPathCostContext(scene, planner_cfg);
    context.setSource('BIDIRECTIONAL_REFINEMENT');
    p1 = scene.start_pose; p2 = [1.0, 1.0, 0];
    context.query(scene.grid_map, p1, p2, scene.map_info, planner_cfg);
    context.query(scene.grid_map, p2, p1, scene.map_info, planner_cfg);
    d = context.diagnostics();
    verifyEqual(testCase, d.cache_miss_count, 2);
    verifyEqual(testCase, d.cache_hit_count, 0);
end

function testChangedMapIsRejected(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    context = SharedPathCostContext(scene, planner_cfg);
    context.setSource('BIDIRECTIONAL_REFINEMENT');
    changed = scene.grid_map;
    free_idx = find(changed == 0, 1, 'first'); changed(free_idx) = 1;
    verifyError(testCase, @() context.query(changed, scene.start_pose, ...
        [1.0, 1.0, 0], scene.map_info, planner_cfg), ...
        'SharedPathCostContext:SceneChanged');
end

function result = run_case(enabled, scene, regions, task_table_map, robot_cfg, planner_cfg, library)
    cfg = planner_cfg; cfg.strategy.name = 'bidirectional';
    cfg.strategy.bidirectional.use_shared_path_cost_cache = enabled;
    path_cost_audit('reset', sprintf('identity_%d', enabled));
    path_cost_audit('set_context', 'bidirectional', 'OTHER', sprintf('identity_%d', enabled));
    plan = run_planning_strategy(library, regions, scene, robot_cfg, cfg);
    [~, ~, ~, visit_order, ~, total_path, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, cfg, plan.pose_info_list, struct(), plan);
    entries = path_cost_audit('snapshot'); path_cost_audit('disable');
    result.plan = plan; result.visit_order = visit_order; result.total_path = total_path;
    result.cache = stats.timing.shared_path_cache;
    result.expensive_calls = sum([entries.expensive_computation]);
end
