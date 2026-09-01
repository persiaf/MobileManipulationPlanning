function tests = test_candidate_filtering_diagnostics
%TEST_CANDIDATE_FILTERING_DIAGNOSTICS Verify observed candidate-stage counts.
    tests = functiontests(localfunctions);
end

function testTaskSupportCountsMatchLibrary(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    d = library.regions(1).generation_diagnostics;
    verifyEqual(testCase, d.raw_candidate_count, ...
        d.collision_rejected_candidate_count + d.collision_valid_candidate_count);
    verifyEqual(testCase, d.collision_valid_candidate_count, d.library_candidate_count);
    verifyEqual(testCase, d.reachable_support_count, d.library_support_count);
    expected = zeros(numel(d.task_ids), 1);
    for ci = 1:numel(library.regions(1).candidates)
        expected = expected + ismember(d.task_ids, ...
            library.regions(1).candidates(ci).covered_tasks);
    end
    verifyEqual(testCase, d.library_support_count, expected);
end
