function tests = test_cafeteria_complex12_path_collision_free
    tests = functiontests(localfunctions);
end

function testCenterlinePathIsCollisionFree(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    verifyGreaterThan(testCase, result.waypoint_count, 1);
    verifyEqual(testCase, result.path_collision_count, 0);
    [ok, bad] = validate_world_path_collision(result.scene.grid_map, result.global_path, ...
        result.scene.map_info, false);
    verifyTrue(testCase, ok);
    verifyEmpty(testCase, bad);
end

function testPathConnectsAllDocksAndReturnsToStart(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    path = result.global_path;
    start = result.scene.start_pose;
    % Segment endpoints come back grid-snapped to cell centres, so the first and
    % last waypoint sit within one cell of the requested start pose.
    tol = result.scene.map_info.resolution;
    verifyEqual(testCase, path(1, 1:2), start(1:2), 'AbsTol', tol);
    verifyEqual(testCase, path(end, 1:2), start(1:2), 'AbsTol', tol);
    verifyGreaterThan(testCase, result.path_length_m, 0);

    docks = result.plan.selected_poses;
    for i = 1:size(docks, 1)
        d = min(vecnorm(path(:, 1:2) - docks(i, 1:2), 2, 2));
        verifyLessThan(testCase, d, tol);
    end
end

function testPathLengthMatchesWaypointPolyline(testCase)
    result = run_cafeteria_planning('complex12', 2026, false, 'off', 'navigation_docking_only');
    path = result.global_path;
    polyline = sum(vecnorm(diff(path(:, 1:2), 1, 1), 2, 2));
    verifyEqual(testCase, polyline, result.path_length_m, 'RelTol', 1e-6);
end