function tests = test_local_reachability_oracle_determinism
%TEST_LOCAL_REACHABILITY_ORACLE_DETERMINISM Oracle output is deterministic.
    tests = functiontests(localfunctions);
end

function testRepeatedRunIsIdentical(testCase)
    options = struct('case_id', 'F010', 'seed', 2026, ...
        'max_uncovered_tasks', 2);
    first = exp_local_reachability_oracle(options);
    second = exp_local_reachability_oracle(options);

    verifyEqual(testCase, second.summary.oracle_feasible, first.summary.oracle_feasible);
    verifyEqual(testCase, second.summary.oracle_infeasible, first.summary.oracle_infeasible);
    verifyEqual(testCase, second.summary.primary_bottleneck, first.summary.primary_bottleneck);
    verifyEqual(testCase, second.task.oracle_class, first.task.oracle_class);
    verifyEqual(testCase, second.task.max_combined_reach_score, ...
        first.task.max_combined_reach_score, 'AbsTol', 1e-12);
end
