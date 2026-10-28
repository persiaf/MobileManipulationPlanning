function tests = test_footprint_sparse_edge_validation
    tests = functiontests(localfunctions);
end

function testInflatedMapUsesConfiguredFootprint(testCase)
    cfg = planner_config();
    verifyTrue(testCase, cfg.footprint_aware_navigation);
    info = struct('rows', 30, 'cols', 30, 'resolution', 0.1, 'height_m', 3);
    map = zeros(30, 30); map(15, 15) = 1;
    [inflated, meta] = build_footprint_inflated_map(map, robot_config().footprint, info);
    verifyGreaterThan(testCase, nnz(inflated), nnz(map));
    verifyEqual(testCase, meta.length_m, 0.423, 'AbsTol', 1e-12);
    verifyEqual(testCase, meta.width_m, 0.366, 'AbsTol', 1e-12);
end

function testInflationUsesMetricRadiusForDiagonalOffsets(testCase)
    info = struct('rows', 30, 'cols', 30, 'resolution', 0.1, 'height_m', 3);
    map = zeros(30, 30);
    map(15, 15) = 1;
    fp = struct('length_m', 0.423, 'width_m', 0.366, 'model', 'rectangle');
    [inflated, meta] = build_footprint_inflated_map(map, fp, info);

    verifyEqual(testCase, meta.radius_m, 0.5 * hypot(fp.length_m, fp.width_m), 'AbsTol', 1e-12);
    verifyEqual(testCase,meta.search_radius_m,meta.radius_m+sqrt(2)*info.resolution/2,'AbsTol',1e-12);
    verifyTrue(testCase,inflated(17,17)>0); % Finite obstacle square adds its half diagonal.
    verifyTrue(testCase,inflated(17,16)>0);
    verifyTrue(testCase,inflated(18,17)==0); % Outside the metric search radius, not ceil-cell dilation.
end
