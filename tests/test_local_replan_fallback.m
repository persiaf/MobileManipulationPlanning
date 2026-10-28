function tests = test_local_replan_fallback
    tests = functiontests(localfunctions);
end

function testFallbackWhenChangedMapHasNoTraversableCells(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
    % A local obstacle in open space admits a real detour. Block the map
    % explicitly to exercise the genuine no-route fallback boundary.
    changed.scene.grid_map(:)=1;
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyTrue(testCase, result.fallback);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyFalse(testCase,result.no_collision_invalid_route);
end
