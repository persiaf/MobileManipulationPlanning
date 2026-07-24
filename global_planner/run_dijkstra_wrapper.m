function path = run_dijkstra_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g_cost = inf(rows, cols);
    g_cost(si, sj) = 0;

    parent = zeros(rows, cols, 2);
    visited = false(rows, cols);

    moves = [-1, 0, 1;  0, 1, 1;  1, 0, 1;  0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    open_list = [si, sj, 0];

    max_iter = rows * cols;
    iter = 0;
    found = false;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;

        [~, min_idx] = min(open_list(:, 3));
        current = open_list(min_idx, 1:2);
        open_list(min_idx, :) = [];

        ci = current(1);
        cj = current(2);

        if visited(ci, cj)
            continue;
        end
        visited(ci, cj) = true;

        if ci == gi && cj == gj
            found = true;
            break;
        end

        for m = 1:size(moves, 1)
            ni = ci + moves(m, 1);
            nj = cj + moves(m, 2);
            move_cost = moves(m, 3);

            if ni < 1 || ni > rows || nj < 1 || nj > cols
                continue;
            end
            if ~traversable(ni, nj)
                continue;
            end
            if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                if ~traversable(ci, nj) || ~traversable(ni, cj)
                    continue;
                end
            end

            tentative_g = g_cost(ci, cj) + move_cost;

            if tentative_g < g_cost(ni, nj)
                g_cost(ni, nj) = tentative_g;
                parent(ni, nj, :) = [ci, cj];
                open_list = [open_list; ni, nj, tentative_g];
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end

    grid_path = [gi, gj];
    ci = gi;
    cj = gj;
    while ~(ci == si && cj == sj)
        pi = parent(ci, cj, 1);
        pj = parent(ci, cj, 2);
        if pi == 0 && pj == 0
            break;
        end
        grid_path = [pi, pj; grid_path];
        ci = pi;
        cj = pj;
    end

    path = convert_path_grid2world(grid_path, map_info);
end