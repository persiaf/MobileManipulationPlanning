function tests = test_scene_complexity_factor_profiles
%TEST_SCENE_COMPLEXITY_FACTOR_PROFILES Validate P1.5 factor IDs.
    tests = functiontests(localfunctions);
end

function testFactorEndpoints(testCase)
    cfg000 = scene_complexity_factor_config('F000', 2026);
    verifyEqual(testCase, cfg000.layout.pattern, 'grid');
    verifyEqual(testCase, cfg000.layout.orientation_mode, 'fixed');
    verifyEqual(testCase, cfg000.task.position_mode, 'discrete');

    cfg111 = scene_complexity_factor_config('F111', 2026);
    verifyEqual(testCase, cfg111.layout.pattern, 'jittered_grid');
    verifyEqual(testCase, cfg111.layout.orientation_mode, 'discrete_random');
    verifyEqual(testCase, cfg111.task.position_mode, 'continuous');
end

function testFactorSceneBuilds(testCase)
    for id = {'F000','F001','F010','F011','F100','F101','F110','F111'}
        cfg = scene_complexity_factor_config(id{1}, 2026);
        scene = build_scene_from_cfg(cfg);
        verifyTrue(testCase, scene.scene_cfg.layout_validation.is_valid);
        verifyEqual(testCase, size(scene.scene_cfg.table_positions, 1), 16);
    end
end
