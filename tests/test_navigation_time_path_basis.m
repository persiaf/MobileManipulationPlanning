function tests = test_navigation_time_path_basis
    tests = functiontests(localfunctions);
end

function testNavigationTimeUsesTotalPath(testCase)
    cfg = planner_config();
    cfg.execution_time.navigation_speed_mps = 2.0;
    path = [0, 0; 6, 0; 6, 8; 0, 0];
    segment_stats = struct('path_length_m', {14, 10}, ...
        'is_return_segment', {false, true});
    stats = estimate_execution_time(path, [1, 2, 1], zeros(2, 3), ...
        {struct('covered_tasks', 1)}, segment_stats, cfg);
    verifyEqual(testCase, stats.navigation_time_s, 12, 'AbsTol', 1e-12);
    verifyEqual(testCase, stats.task_navigation_time_s, 7, 'AbsTol', 1e-12);
    verifyEqual(testCase, stats.return_navigation_time_s, 5, 'AbsTol', 1e-12);
end
