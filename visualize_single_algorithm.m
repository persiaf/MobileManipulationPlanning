function visualize_single_algorithm(grid_map, start_pose, goal_pose, map_info, planner_cfg, algorithm)
    res = map_info.resolution;
    rows = map_info.rows;
    cols = map_info.cols;
    height_m = map_info.height_m;

    [si, sj] = world2grid(start_pose(1), start_pose(2), res, height_m);
    [gi, gj] = world2grid(goal_pose(1),  goal_pose(2),  res, height_m);
    si = max(1, min(rows, si)); sj = max(1, min(cols, sj));
    gi = max(1, min(rows, gi)); gj = max(1, min(cols, gj));

    start_grid = [si, sj];
    goal_grid = [gi, gj];

    clf;
    hold on;

    plot_grid(grid_map);

    path = run_path_planner(grid_map, start_pose, goal_pose, map_info, planner_cfg, algorithm);

    path_grid = zeros(size(path, 1), 2);
    for k = 1:size(path, 1)
        [ig, jg] = world2grid(path(k, 1), path(k, 2), res, height_m);
        path_grid(k, :) = [ig, jg];
    end

    plot_path(path_grid, res);

    plot_square(start_grid, [rows, cols], res, '#ff0000');
    plot_square(goal_grid, [rows, cols], res, '#00ff00');

    title(sprintf('%s - path length: %.3f m', algorithm, compute_path_length(path)), ...
        'FontSize', 14, 'Interpreter', 'none');

    axis equal;
    hold off;
end
