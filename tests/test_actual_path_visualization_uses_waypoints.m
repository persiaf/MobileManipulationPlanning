function tests = test_actual_path_visualization_uses_waypoints
    tests = functiontests(localfunctions);
end

function testRouteLinePreservesStoredWaypoints(testCase)
    fig = figure('Visible', 'off');
    cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
    ax = axes(fig);
    waypoints = [0, 0, 0; 0.4, 0.8, 0; 1.1, 0.2, 0; 1.6, 1.0, 0];
    h = draw_collision_free_route(ax, waypoints);
    verifyEqual(testCase, h.XData(:), waypoints(:, 1));
    verifyEqual(testCase, h.YData(:), waypoints(:, 2));
    verifyEqual(testCase, h.DisplayName, 'Collision-free route');
end
