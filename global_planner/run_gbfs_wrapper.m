function path = run_gbfs_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    parent = zeros(rows, cols, 2);
    visited = false(rows, cols);

    h_si = sqrt((si - gi)^2 + (sj - gj)^2);
    open_list = [si, sj, h_si];

    max_iter = rows * cols;
    iter = 0;
    found = false;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;
        [~, min_idx] = min(open_list(:, 3));
        current = open_list(min_idx, 1:2);
        open_list(min_idx, :) = [];
        ci = current(1); cj = current(2);

        if visited(ci, cj)
            continue;
        end
        visited(ci, cj) = true;

        if ci == gi && cj == gj
            found = true;
            break;
        end

        for m = 1:size(moves, 1)
            ni = ci + moves(m, 1); nj = cj + moves(m, 2);
            if ni < 1 || ni > rows || nj < 1 || nj > cols
                continue;
            end
            if ~traversable(ni, nj) || visited(ni, nj)
                continue;
            end
            if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                if ~traversable(ci, nj) || ~traversable(ni, cj)
                    continue;
                end
            end
            parent(ni, nj, :) = [ci, cj];
            h = sqrt((ni - gi)^2 + (nj - gj)^2);
            open_list = [open_list; ni, nj, h];
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end
    path = reconstruct_path_grid(gi, gj, si, sj, parent, map_info.resolution, map_info.height_m);
end