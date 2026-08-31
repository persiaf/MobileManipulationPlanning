function tests = test_path_length_decomposition
    tests = functiontests(localfunctions);
end

function testPathLengthWithReturnDecomposes(testCase)
    cfg = planner_config();
    cfg.execution.return_to_start = true;
    path = [0, 0; 3, 0; 3, 4; 0, 0];
    route_order = [1, 2, 1];
    segment_stats = struct('path_length_m', {7, 5}, ...
        'is_return_segment', {false, true});
    stats = estimate_execution_time(path, route_order, zeros(2, 3), ...
        {struct('covered_tasks', 1)}, segment_stats, cfg);
    verifyEqual(testCase, stats.task_path_len_m, 7, 'AbsTol', 1e-12);
    verifyEqual(testCase, stats.return_path_len_m, 5, 'AbsTol', 1e-12);
    verifyEqual(testCase, stats.total_path_len_m, 12, 'AbsTol', 1e-12);
end

function testPathLengthWithoutReturnHasZeroReturnPath(testCase)
    cfg = planner_config();
    cfg.execution.return_to_start = false;
    path = [0, 0; 3, 4];
    stats = estimate_execution_time(path, [1, 2], zeros(2, 3), ...
        {struct('covered_tasks', 1)}, struct('path_length_m', {}, ...
        'is_return_segment', {}), cfg);
    verifyEqual(testCase, stats.task_path_len_m, stats.total_path_len_m);
    verifyEqual(testCase, stats.return_path_len_m, 0);
end
