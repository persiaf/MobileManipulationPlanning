function tests = test_local_replan_route_feasibility
    tests = functiontests(localfunctions);
end

function testPathObstacleRepairAvoidsInvalidEdge(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
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
