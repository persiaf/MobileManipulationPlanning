function tests = test_isolated_table_oracle_consistency
%TEST_ISOLATED_TABLE_ORACLE_CONSISTENCY Isolated-table fields are present.
    tests = functiontests(localfunctions);
end

function testIsolatedTableAuditIsDeterministic(testCase)
    options = struct('max_tasks', 2, 'write_outputs', false);
    first = exp_radial_operability_audit(options);
    second = exp_radial_operability_audit(options);
    verifyEqual(testCase, second.task.isolated_table_feasible, ...
        first.task.isolated_table_feasible);
    verifyEqual(testCase, second.task.final_classification, ...
        first.task.final_classification);
end
