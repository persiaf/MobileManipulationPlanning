function tests = test_large_scale_benchmark_smoke
%TEST_LARGE_SCALE_BENCHMARK_SMOKE One tiny end-to-end benchmark trial.
    tests = functiontests(localfunctions);
end

function testQuickBenchmarkSchema(testCase)
    [raw, summary] = exp_large_scale_complex_tasks(struct('quick_mode', true, ...
        'scales', {{'smoke'}}, 'seeds', 1));
    verifyGreaterThanOrEqual(testCase, height(raw), 3);
    required = {'timestamp','scale_id','seed','strategy','requested_task_count', ...
        'actual_task_count','candidate_count','T_scene','T_candidate','T_strategy', ...
        'T_path_cost','T_route','T_execution_sim','T_total','path_cost_query_count', ...
        'path_cost_cache_hit','path_cost_cache_miss','cache_hit_ratio','success','failure_reason'};
    verifyTrue(testCase, all(ismember(required, raw.Properties.VariableNames)));
    verifyTrue(testCase, all(raw.success));
    verifyGreaterThanOrEqual(testCase, min(raw.candidate_count), 1);
    verifyGreaterThanOrEqual(testCase, min(raw.T_total), 0);
    verifyGreaterThanOrEqual(testCase, min(raw.cache_hit_ratio), 0);
    verifyLessThanOrEqual(testCase, max(raw.cache_hit_ratio), 1);
    verifyGreaterThanOrEqual(testCase, height(summary), 3);
end
