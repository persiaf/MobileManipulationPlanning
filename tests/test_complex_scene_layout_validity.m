function tests = test_complex_scene_layout_validity
    tests = functiontests(localfunctions);
end

function testV1V2BuildAndValidity(testCase)
    v1 = build_complex_visual_scene('V1', 2026);
    v2 = build_complex_visual_scene('V2', 2026);
    [ok, report] = validate_table_layout(v2.scene_cfg);
    verifyTrue(testCase, ok, report.failure_reason);
    verifyEqual(testCase, size(v1.scene_cfg.table_positions, 1), 28);
    verifyEqual(testCase, v1.region_metadata.region_count, 7);
    verifyEqual(testCase, size(v2.scene_cfg.table_positions, 1), 40);
    verifyGreaterThanOrEqual(testCase, v2.region_metadata.region_count, 7);
    verifyGreaterThanOrEqual(testCase, numel(unique(round(v2.scene_cfg.table_orientations, 8))), 2);
    verifyGreaterThanOrEqual(testCase, numel(v2.static_obstacles), 3);
end
