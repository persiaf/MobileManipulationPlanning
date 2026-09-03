function tests = test_change_impact_locality
    tests = functiontests(localfunctions);
end

function testPathObstacleImpactIsLocal(testCase)
    state = p5_fixture_state();
    [~, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
    event.radius_m = 0.05;
    impact = analyze_change_impact(state, event);
    verifyGreaterThanOrEqual(testCase, impact.invalid_edge_count, 1);
    verifyLessThan(testCase, impact.affected_region_count, impact.total_regions);
    verifyLessThan(testCase, impact.region_locality_ratio, 1);
end
