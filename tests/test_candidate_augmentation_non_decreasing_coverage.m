function tests = test_candidate_augmentation_non_decreasing_coverage
    tests = functiontests(localfunctions);
end

function testCoverageMonotonicity(testCase)
    [~, damaged, regions, scene, robot_cfg, cfg] = candidate_augmentation_fixture();
    [~, diagnostics] = augment_candidate_library( ...
        damaged, regions, scene, robot_cfg, cfg);
    verifyGreaterThanOrEqual(testCase, diagnostics.library_coverage_after, ...
        diagnostics.library_coverage_before);
    verifyGreaterThanOrEqual(testCase, diagnostics.candidate_count_after, ...
        diagnostics.baseline_candidate_count);
end
