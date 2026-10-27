function tests = test_cafeteria_profile_determinism
    tests = functiontests(localfunctions);
end

function testFixedProfileIsDeterministic(testCase)
    a = cafeteria_scene_profile_config('complex12', 2026);
    b = cafeteria_scene_profile_config('complex12', 2026);
    verifyEqual(testCase, a.table_positions, b.table_positions, 'AbsTol', 0);
    verifyEqual(testCase, a.manual_tasks, b.manual_tasks, 'AbsTol', 0);
    verifyEqual(testCase, a.random_seed, 2026);
    verifyEqual(testCase, size(a.manual_tasks, 1), 12);
    verifyEqual(testCase, numel(a.active_table_ids), 8);
end

function testBasic3RemainsAvailable(testCase)
    cfg = cafeteria_scene_profile_config('basic3', 2026);
    verifyEqual(testCase, cfg.manual_tasks(:, 1)', [2, 4, 9]);
    verifyEqual(testCase, size(cfg.manual_tasks, 1), 3);
end

function testManualTasksUseCanonicalTransform(testCase)
    cfg = cafeteria_scene_profile_config('complex12', 2026);
    for i = 1:size(cfg.task_local_specs, 1)
        table_id = cfg.task_local_specs(i, 1);
        expected = table_local_to_world(cfg.table_positions(table_id, :), ...
            cfg.table_orientations(table_id), cfg.task_local_specs(i, 2:3));
        verifyEqual(testCase, cfg.manual_tasks(i, 1:3), ...
            [table_id, expected], 'AbsTol', 1e-12);
    end
end
