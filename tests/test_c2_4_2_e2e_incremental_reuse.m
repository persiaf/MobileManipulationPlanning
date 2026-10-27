function tests = test_c2_4_2_e2e_incremental_reuse
%TEST_C2_4_2_E2E_INCREMENTAL_REUSE Strict same-edge production-chain checks.
    tests = functiontests(localfunctions);
end

function testSameEdgeTwoEventsReuseWithoutFallback(testCase)
    state = fixture_state();
    frozen = state.rc2_plan;

    [changed1, event1] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    p0 = state.stats.sparse_graph.direct_paths{1, 2};
    event1.center = p0(max(2, floor(size(p0, 1) / 2)), 1:2) + [0.25, 0];
    event1.radius_m = 0.15;
    changed1.scene = state.scene;
    changed1.scene = add_test_obstacle(changed1.scene, event1.center, event1.radius_m);
    impact1 = analyze_change_impact(state, event1);
    [result1, state1, diagnostics1] = local_replan(state, changed1, event1, impact1);
    verifyEqual(testCase, result1.status, 'SUCCESS');
    verifyTrue(testCase, diagnostics1.incremental_dstar.path_safe);
    verifyFalse(testCase, diagnostics1.incremental_dstar.fallback_used);
    verifyTrue(testCase, diagnostics1.incremental_dstar.incremental_success_edge_count >= 1);
    verifyNotEmpty(testCase, state1.stats.path_search_states.states);
    first_edge = diagnostics1.incremental_dstar.edge_results(1).key;
    first_version = diagnostics1.incremental_dstar.edge_results(1).state_version;

    % The second event still enters through the deterministic production
    % event path. Add a distinct cell on the same selected edge so M1->M2 is
    % a real change while the edge identity and endpoints remain stable.
    [changed2, event2] = apply_dynamic_event(state1, 'ADD_LOCAL_OBSTACLE', 5);
    edge = event2.target_edge;
    p = state1.stats.sparse_graph.direct_paths{edge(1), edge(2)};
    verifyGreaterThan(testCase, size(p, 1), 3);
    q = p(max(2, floor(size(p, 1) / 2)), 1:2);
    event2.center = q;
    event2.radius_m = 0.25;
    changed2.scene = state1.scene;
    changed2.scene = add_test_obstacle(changed2.scene, event2.center, event2.radius_m);
    verifyGreaterThan(testCase, nnz(changed2.scene.grid_map ~= state1.scene.grid_map), ...
        nnz(changed1.scene.grid_map ~= state.scene.grid_map));
    impact2 = analyze_change_impact(state1, event2);
    verifyTrue(testCase, any(all(impact2.invalid_sparse_edges == edge, 2)) || ...
        any(all(impact2.invalid_sparse_edges == fliplr(edge), 2)));

    [result2, state2, diagnostics2] = local_replan(state1, changed2, event2, impact2);
    verifyEqual(testCase, result2.status, 'SUCCESS');
    verifyTrue(testCase, diagnostics2.incremental_dstar.path_safe);
    verifyFalse(testCase, diagnostics2.incremental_dstar.fallback_used);
    verifyTrue(testCase, diagnostics2.incremental_dstar.full_route_incremental_success);
    edge2 = diagnostics2.incremental_dstar.edge_results(1);
    verifyEqual(testCase, edge2.key, first_edge);
    verifyTrue(testCase, edge2.state_loaded);
    verifyTrue(testCase, edge2.incremental_reuse);
    verifyFalse(testCase, edge2.reinitialized);
    verifyEqual(testCase, edge2.actual_planner, 'dstar_lite');
    verifyTrue(testCase, edge2.path_found && edge2.path_safe && edge2.state_valid);
    verifyEqual(testCase, edge2.previous_state_version, first_version);
    verifyGreaterThan(testCase, edge2.state_version, first_version);
    verifyNotEmpty(testCase, state2.stats.path_search_states.states);
    verifyEqual(testCase, state2.rc2_plan, frozen);
    verifyEqual(testCase, state2.task_state, state1.task_state);
    verifyTrue(testCase, state2.path_validation.valid);
    verifyEqual(testCase, state2.stats.path_len, compute_path_length(state2.global_path), 'AbsTol', 1e-12);
end

function testFallbackClearsDstarState(testCase)
    [map, info, cfg, start_pose, goal_pose] = strict_wrapper_fixture();
    map(10, :) = 1;
    [path, ~, search_state, diagnostics] = run_path_planner(map, start_pose, goal_pose, info, cfg, ...
        'dstar_lite', struct('incremental_mode', true, 'dstar_state', []));
    verifyFalse(testCase, diagnostics.path_safe);
    verifyTrue(testCase, diagnostics.fallback_used);
    verifyEqual(testCase, diagnostics.actual_planner, 'astar');
    verifyFalse(testCase, diagnostics.search_state_valid);
    verifyEmpty(testCase, search_state);
    verifyLessThanOrEqual(testCase, size(path, 1), 2);
end

function state = fixture_state()
    scene = struct();
    scene.start_pose = [0.5, 0.5, 0];
    scene.num_tasks = 1;
    scene.grid_map = zeros(100, 100);
    scene.map_info = struct('resolution', 0.1, 'height_m', 10, 'width_m', 10, 'rows', 100, 'cols', 100);
    scene.scene_cfg.table_positions = [8.5, 8.5];
    scene.scene_cfg.table_region_id = 1;
    scene.scene_cfg.table_orientations = 0;
    scene.scene_cfg.table.length = 0.8; scene.scene_cfg.table.width = 0.5;
    scene.scene_cfg.task_offset_z = 0.5;
    scene.tasks = struct('id', 1, 'x', 8.5, 'y', 8.5, 'z', 0.5, 'status', 0, 'table_id', 1);
    scene.region_metadata = struct('region_count', 1, 'region_ids', 1, 'table_region_id', 1, ...
        'task_region_id', 1, 'tasks_per_region', 1, 'selected_docks_per_region', 1);
    state = struct('scale', 'fixture', 'seed', 1, 'scene', scene);
    state.plan = struct('selected_poses', [8.5, 8.5, 0], 'pose_to_region_map', 1, ...
        'covered_tasks_predicted', 1, 'task_sequence_summary', {{struct('covered_tasks', 1, 'table_id', 1, 'dock_index', 1)}}, ...
        'pose_info_list', {{struct('covered_tasks', 1, 'table_id', 1, 'dock_index', 1)}});
    state.robot_cfg = struct('footprint', struct('length_m', 0.423, 'width_m', 0.366, 'model', 'rectangle'));
    state.all_poses = [scene.start_pose; state.plan.selected_poses];
    state.candidate_library.total_candidates = 1;
    state.planner_cfg = struct('global_planner', 'dstar_lite', 'incremental_replanning_enabled', true, ...
        'footprint_aware_navigation', true, 'robot_cfg', state.robot_cfg, ...
        'astar', struct('allow_diagonal', true, 'heuristic_weight', 1));
    route = [1, 2, 1];
    graph = build_sparse_docking_graph(state.all_poses, state.scene.grid_map, state.scene.map_info, ...
        state.planner_cfg, [0; 1]);
    state.stats.sparse_graph = graph;
    [state.stats.global_path, state.stats.segment_stats] = stitch_sparse_graph_path(state.all_poses, route, graph);
    state.rc2_plan = struct('selected_docks', state.plan.selected_poses, 'route_order', route, ...
        'global_path', state.stats.global_path, 'path_segments', state.stats.segment_stats);
    state.task_state = struct('predicted_tasks', 1, 'commanded_tasks', 1, 'executed_tasks', 1, 'verified_tasks', 1);
end

function [map, info, cfg, start_pose, goal_pose] = strict_wrapper_fixture()
    map = zeros(20, 20); info = struct('rows', 20, 'cols', 20, 'resolution', 1, 'height_m', 20);
    cfg = struct('global_planner', 'dstar_lite', 'incremental_replanning_enabled', true, ...
        'footprint_aware_navigation', false, 'astar', struct('allow_diagonal', true, 'heuristic_weight', 1));
    start_pose = [1.5, 18.5, 0]; goal_pose = [17.5, 2.5, 0];
end

function scene = add_test_obstacle(scene, center, radius)
    [ci, cj] = world2grid(center(1), center(2), scene.map_info.resolution, scene.map_info.height_m);
    rr = max(1, ceil(radius / scene.map_info.resolution));
    for i = max(1, ci-rr):min(size(scene.grid_map, 1), ci+rr)
        for j = max(1, cj-rr):min(size(scene.grid_map, 2), cj+rr)
            [x, y] = grid2world(i, j, scene.map_info.resolution, scene.map_info.height_m);
            if hypot(x-center(1), y-center(2)) <= radius, scene.grid_map(i,j) = 1; end
        end
    end
end

