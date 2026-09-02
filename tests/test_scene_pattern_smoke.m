function tests = test_scene_pattern_smoke
%TEST_SCENE_PATTERN_SMOKE Build C0-C3 without changing planners.
    tests = functiontests(localfunctions);
end

function testSceneComplexityPatternsBuild(testCase)
    for case_id = {'C0', 'C1', 'C2', 'C3'}
        cfg = scene_complexity_profile_config(case_id{1}, 2026);
        scene = build_scene_from_cfg(cfg);
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        verifyEqual(testCase, size(scene.scene_cfg.table_positions, 1), 16);
        verifyGreaterThanOrEqual(testCase, scene.num_tasks, 48);
        verifyLessThanOrEqual(testCase, scene.num_tasks, 96);
        verifyEqual(testCase, numel(regions), 16);
        verifyTrue(testCase, scene.scene_cfg.layout_validation.is_valid);
    end
end
