function tests = test_region_metadata_consistency
    tests = functiontests(localfunctions);
end

function testClusterRegionMetadataMatchesTablesAndTasks(testCase)
    scene = build_scene_from_cfg(large_scale_profile_config('L1', 2026));
    meta = scene.region_metadata;
    verifyEqual(testCase, numel(meta.table_region_id), size(scene.scene_cfg.table_positions, 1));
    verifyEqual(testCase, numel(meta.task_region_id), scene.num_tasks);
    verifyEqual(testCase, sum(meta.tables_per_region), size(scene.scene_cfg.table_positions, 1));
    verifyEqual(testCase, sum(meta.tasks_per_region), scene.num_tasks);
    verifyGreaterThanOrEqual(testCase, meta.region_count, 3);
end
