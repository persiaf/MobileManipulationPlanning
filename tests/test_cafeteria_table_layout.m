function tests = test_cafeteria_table_layout
    tests = functiontests(localfunctions);
end

function testNineFixedTablesDoNotOverlap(testCase)
    cfg = cafeteria_scene_profile_config('complex12', 2026);
    expected = [1.80 1.40; 1.80 4.00; 1.80 6.60; 5.10 1.40; ...
        6.45 1.40; 5.10 4.00; 6.45 4.00; 5.10 6.60; 6.45 6.60];
    verifyEqual(testCase, cfg.table_positions, expected, 'AbsTol', 1e-12);
    [ok, report] = validate_table_layout(cfg);
    verifyTrue(testCase, ok, report.failure_reason);
    verifyEqual(testCase, sum(contains(report.failures, '_overlap')), 0);
    verifyEqual(testCase, cfg.table_region_id, [1; 1; 1; 2; 2; 3; 3; 4; 4]);
    verifyEqual(testCase, cfg.table_min_spacing_m, 1.0, 'AbsTol', 0);
    verifyEqual(testCase, numel(unique(cfg.table_region_id)), 4);
end
