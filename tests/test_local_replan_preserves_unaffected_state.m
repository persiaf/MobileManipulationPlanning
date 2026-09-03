function tests = test_local_replan_preserves_unaffected_state
    tests = functiontests(localfunctions);
end

function testDockInvalidationPreservesUnaffectedDocks(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyEqual(testCase, result.removed_docks, 1);
    verifyEqual(testCase, result.new_docks, 1);
    verifyGreaterThan(testCase, result.unchanged_dock_ratio, 0.5);
    verifyLessThan(testCase, result.dock_recompute_ratio, 1);
end
