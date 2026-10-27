function tests = test_local_replan_route_feasibility
    tests = functiontests(localfunctions);
end

function testPathObstacleRepairAvoidsInvalidEdge(testCase)
    state = p5_fixture_state();
    state.scene.start_pose = [0.5, 0.5, 0];
    state.all_poses(1, :) = state.scene.start_pose;
    state.robot_cfg = struct('footprint', struct('length_m', 0.423, ...
        'width_m', 0.366, 'model', 'rectangle'));
    state.planner_cfg = struct('global_planner', 'astar', ...
        'footprint_aware_navigation', true, 'robot_cfg', state.robot_cfg, ...
        'astar', struct('allow_diagonal', true, 'heuristic_weight', 1.0));
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyFalse(testCase, result.fallback);
    verifyTrue(testCase, result.sparse_graph_connected);
    verifyTrue(testCase, result.no_collision_invalid_route);
    verifyGreaterThanOrEqual(testCase, result.reused_edge_ratio, 0.5);
end

function testAddedTasksCoveredByLocalSupplement(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_TASKS', 1);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.added_tasks, 4);
    verifyEqual(testCase, result.model_feasible_added_tasks, 4);
    verifyEqual(testCase, result.feasible_task_coverage, 100);
    verifyLessThan(testCase, result.region_locality_ratio, 1);
end
