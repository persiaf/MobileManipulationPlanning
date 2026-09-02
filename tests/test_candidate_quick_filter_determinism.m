function tests = test_candidate_quick_filter_determinism
%TEST_CANDIDATE_QUICK_FILTER_DETERMINISM Benchmark is deterministic.
    tests = functiontests(localfunctions);
end

function testRepeatedRunMatches(testCase)
    options = struct('cases', {{'F010'}}, 'seeds', 2026, ...
        'variants', {{'E0_current','E1_quick_filter'}}, 'write_partial', false);
    options.write_outputs = false;
    first = exp_candidate_efficiency_p2(options);
    second = exp_candidate_efficiency_p2(options);
    verifyEqual(testCase, second.summary.formal_reachability_calls_mean, ...
        first.summary.formal_reachability_calls_mean);
    verifyEqual(testCase, second.summary.quick_rejected_pairs_mean, ...
        first.summary.quick_rejected_pairs_mean);
end
