function tests = test_complex_scene_profile_determinism
    tests = functiontests(localfunctions);
end

function testV2Deterministic(testCase)
    a = build_complex_visual_scene('V2', 2026);
    b = build_complex_visual_scene('V2', 2026);
    verifyEqual(testCase, a.scene_cfg.table_positions, b.scene_cfg.table_positions, 'AbsTol', 1e-12);
    verifyEqual(testCase, a.scene_cfg.table_orientations, b.scene_cfg.table_orientations, 'AbsTol', 1e-12);
    verifyEqual(testCase, [[a.tasks.x]', [a.tasks.y]'], [[b.tasks.x]', [b.tasks.y]'], 'AbsTol', 1e-12);
    verifyEqual(testCase, numel(a.static_obstacles), numel(b.static_obstacles));
end
