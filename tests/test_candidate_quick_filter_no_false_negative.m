function tests = test_candidate_quick_filter_no_false_negative
%TEST_CANDIDATE_QUICK_FILTER_NO_FALSE_NEGATIVE Quick filter never rejects valid pairs.
    tests = functiontests(localfunctions);
end

function testNoFalseNegativesOnSmallSet(testCase)
    outputs = exp_candidate_efficiency_p2(struct('cases', {{'F010'}}, ...
        'seeds', 2026, 'variants', {{'E0_current','E1_quick_filter'}}, ...
        'write_partial', false, 'write_outputs', false));
    e1 = outputs.raw(strcmp(outputs.raw.variant, 'E1_quick_filter'), :);
    verifyEqual(testCase, sum(e1.false_negative_count), 0);
end
