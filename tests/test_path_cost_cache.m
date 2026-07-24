function tests = test_path_cost_cache
    tests = functiontests(localfunctions);
end

function testCacheHitAfterRepeat(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    cache = initialize_path_cost_cache(scene, planner_cfg);
    p1 = scene.start_pose;
    p2 = [1.0, 1.0, 0];
    [c1, cache] = get_cached_path_cost(cache, p1, p2, scene, planner_cfg);
    [c2, cache] = get_cached_path_cost(cache, p1, p2, scene, planner_cfg);
    verifyEqual(testCase, c2, c1);
    verifyGreaterThanOrEqual(testCase, cache.hits, 1);
end
