function tests = test_candidate_failure_trace_consistency
%TEST_CANDIDATE_FAILURE_TRACE_CONSISTENCY Tiny C2 trace schema check.
    tests = functiontests(localfunctions);
end

function testC2TraceHasConsistentFields(testCase)
    [trace, summary] = exp_c2_candidate_failure_diagnosis(struct('seeds', 2026));
    required = {'scene_case','seed','task_id','table_id','task_local_x','task_local_y', ...
        'baseline_support_count','position_augmented_support_count', ...
        'candidate_to_task_distance','combined_reach_score','geometry_class'};
    verifyTrue(testCase, all(ismember(required, trace.Properties.VariableNames)));
    verifyGreaterThan(testCase, height(trace), 0);
    verifyGreaterThanOrEqual(testCase, min(trace.baseline_support_count), 0);
    verifyGreaterThanOrEqual(testCase, height(summary), 1);
end
