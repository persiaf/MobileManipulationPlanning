function tests = test_cafeteria_main_aisle_width
    tests = functiontests(localfunctions);
end

function testNominalAisleIsConnectedAndCorrectWidth(testCase)
    cfg = cafeteria_scene_profile_config('complex12', 2026);
    verifyEqual(testCase, cfg.cafeteria.main_aisle_x, [2.40, 4.50], 'AbsTol', 1e-12);
    verifyEqual(testCase, cfg.cafeteria.main_aisle_width_m, 2.10, 'AbsTol', 1e-12);
    verifyEqual(testCase, cfg.cafeteria.right_table_gap_m, 0.15, 'AbsTol', 1e-12);
    scene = build_cafeteria_scene('complex12', 2026);
    report = validate_cafeteria_scene(scene, robot_geometry_profile('physical_robot'));
    verifyTrue(testCase, report.main_aisle_connected);
end
