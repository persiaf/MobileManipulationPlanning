function tests = test_candidate_augmentation_noop_when_full_coverage
    tests = functiontests(localfunctions);
end

function testNoopForCoveredLibrary(testCase)
    scene = build_scene_from_cfg(large_scale_profile_config('smoke', 17));
    robot_cfg = robot_config(); cfg = planner_config();
    regions = build_task_regions(scene, assign_tasks_to_tables(scene));
    baseline = build_candidate_library(regions, scene, robot_cfg, cfg);
    cfg.candidate_augmentation.enable = true;
    [actual, diagnostics] = augment_candidate_library( ...
        baseline, regions, scene, robot_cfg, cfg);
    verifyEqual(testCase, diagnostics.uncovered_tasks_before, 0);
    verifyEqual(testCase, diagnostics.selected_supplementary_count, 0);
    verifyEqual(testCase, actual.total_candidates, baseline.total_candidates);
end
