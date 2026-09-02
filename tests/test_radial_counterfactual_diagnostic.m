function tests = test_radial_counterfactual_diagnostic
%TEST_RADIAL_COUNTERFACTUAL_DIAGNOSTIC P1.8 audit emits radial counters.
    tests = functiontests(localfunctions);
end

function testSmallAuditHasExpectedFields(testCase)
    outputs = exp_radial_operability_audit(struct('max_tasks', 2, ...
        'write_outputs', false));
    required = {'radial_only_blocked_count', 'multi_constraint_blocked_count', ...
        'R_excess_mean', 'main_rejection_condition'};
    verifyTrue(testCase, all(ismember(required, outputs.summary.Properties.VariableNames)));
    verifyEqual(testCase, outputs.summary.audited_fail_radial_tasks, 2);
    verifyEqual(testCase, outputs.summary.main_rejection_condition{1}, 'FAIL_RADIAL');
end
