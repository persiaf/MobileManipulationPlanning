function tests = test_operational_feasibility_accounting
%TEST_OPERATIONAL_FEASIBILITY_ACCOUNTING Raw and feasible-task coverage.
    tests = functiontests(localfunctions);
end

function testFeasibleCoverageExcludesModelInfeasibleTasks(testCase)
    metrics = task_operability_accounting(1:5, [1, 2, 3], [4, 5]);
    verifyEqual(testCase, metrics.total_task_count, 5);
    verifyEqual(testCase, metrics.raw_task_coverage_percent, 60);
    verifyEqual(testCase, metrics.model_feasible_task_count, 3);
    verifyEqual(testCase, metrics.feasible_task_coverage_percent, 100);
end
