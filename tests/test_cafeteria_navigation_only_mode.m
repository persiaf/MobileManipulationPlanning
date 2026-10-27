function tests = test_cafeteria_navigation_only_mode
    tests = functiontests(localfunctions);
end

function testProxyModeCompletesDockingAndPath(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    verifyEqual(testCase, result.planning_mode, 'navigation_docking_only');
    verifyEqual(testCase, result.candidate_coverage, 1, 'AbsTol', 0);
    verifyGreaterThan(testCase, result.selected_docks, 0);
    verifyGreaterThan(testCase, result.waypoint_count, 0);
    verifyEqual(testCase, result.path_collision_count, 0);
end

function testFullModeStillReportsManipulationBlock(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'full_manipulation');
    verifyEqual(testCase, result.status, 'CAFETERIA_CANDIDATE_COVERAGE_BLOCKED');
    verifyEqual(testCase, result.candidate_diagnostic.reason, 'HORIZONTAL_RADIAL_REACH_LIMIT');
end
