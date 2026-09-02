function tests = test_scene_layout_determinism
%TEST_SCENE_LAYOUT_DETERMINISM Same scene-complexity config reproduces exactly.
    tests = functiontests(localfunctions);
end

function testJitteredGridIsDeterministic(testCase)
    cfg1 = scene_complexity_profile_config('C2', 2026);
    scene1 = build_scene_from_cfg(cfg1);
    cfg2 = scene_complexity_profile_config('C2', 2026);
    scene2 = build_scene_from_cfg(cfg2);

    verifyEqual(testCase, scene1.scene_cfg.table_positions, scene2.scene_cfg.table_positions, 'AbsTol', 1e-12);
    verifyEqual(testCase, scene1.scene_cfg.table_orientations, scene2.scene_cfg.table_orientations, 'AbsTol', 1e-12);
    pts1 = [[scene1.tasks.x]', [scene1.tasks.y]'];
    pts2 = [[scene2.tasks.x]', [scene2.tasks.y]'];
    verifyEqual(testCase, pts1, pts2, 'AbsTol', 1e-12);
end
