function h = draw_collision_free_route(ax, waypoints)
%DRAW_COLLISION_FREE_ROUTE Draw stored planner waypoints without simplification.

    if isempty(waypoints) || size(waypoints, 1) < 2 || size(waypoints, 2) < 2
        h = gobjects(0);
        return;
    end
    h = plot(ax, waypoints(:, 1), waypoints(:, 2), '-', ...
        'Color', [0.0 0.2 0.9], 'LineWidth', 2.8, ...
        'DisplayName', 'Collision-free route');
end
