function tests = test_cafeteria_global_path_collision_free
    tests = functiontests(localfunctions);
end

function testActualPathOrDeclaredReachabilityBlock(testCase)
    result = run_cafeteria_planning('complex12', 2026, false);
    if strcmp(result.status, 'CAFETERIA_PLANNING_SUCCESS')
        verifyGreaterThan(testCase, result.waypoint_count, 1);
        verifyEqual(testCase, result.path_collision_count, 0);
        verifyEqual(testCase, result.footprint_collision_count, 0);
    else
        verifyEqual(testCase, result.status, 'CAFETERIA_REACHABILITY_BLOCKED');
        verifyTrue(testCase, all(strcmp({result.operability.reachability_result}, ...
            'TASK_GEOMETRY_REACH_LIMIT')));
        verifyEmpty(testCase, result.global_path);
    end
end
