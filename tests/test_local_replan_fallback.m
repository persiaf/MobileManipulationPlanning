function tests = test_local_replan_fallback
    tests = functiontests(localfunctions);
end

function testFallbackWhenObstacleDisconnectsOnlyEdge(testCase)
    state = p5_fixture_state();
    state.all_poses = state.all_poses(1:2, :);
    state.plan.selected_poses = state.plan.selected_poses(1, :);
    state.plan.pose_to_region_map = 1;
    state.plan.pose_info_list = state.plan.pose_info_list(1);
    state.stats.route_order = [1, 2, 1];
    state.stats.sparse_graph.adjacency = [false true; true false];
    state.stats.sparse_graph.cost_matrix = [0 1; 1 0];
    state.stats.sparse_graph.region_ids = [0; 1];
    state.stats.sparse_graph.next_hop = [0 2; 1 0];
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyTrue(testCase, result.fallback);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
end
