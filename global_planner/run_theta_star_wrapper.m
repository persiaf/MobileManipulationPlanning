function path = run_theta_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g_cost = inf(rows, cols);
    g_cost(si, sj) = 0;
    parent = zeros(rows, cols, 2);
    parent(si, sj, :) = [si, sj];
    closed = false(rows, cols);
    open_set = false(rows, cols);

    h_start = sqrt((si - gi)^2 + (sj - gj)^2);
    open_list = [si, sj, h_start];
    open_set(si, sj) = true;

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    max_iter = rows * cols;
    iter = 0;
    found = false;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;
        [~, min_idx] = min(open_list(:, 3));
        current = open_list(min_idx, 1:2);
        open_list(min_idx, :) = [];
        open_set(current(1), current(2)) = false;

        ci = current(1); cj = current(2);
        if closed(ci, cj)
            continue;
        end
        closed(ci, cj) = true;

        if ci == gi && cj == gj
            found = true;
            break;
        end

        for m = 1:size(moves, 1)
            ni = ci + moves(m, 1); nj = cj + moves(m, 2);
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
            if closed(ni, nj)
                continue;
            end

            ppi = parent(ci, cj, 1); ppj = parent(ci, cj, 2);
            if line_of_sight(ppi, ppj, ni, nj, traversable)
                new_g = g_cost(ppi, ppj) + sqrt((ni - ppi)^2 + (nj - ppj)^2);
                if new_g < g_cost(ni, nj)
                    g_cost(ni, nj) = new_g;
                    parent(ni, nj, :) = [ppi, ppj];
                end
            else
                new_g = g_cost(ci, cj) + moves(m, 3);
                if new_g < g_cost(ni, nj)
                    g_cost(ni, nj) = new_g;
                    parent(ni, nj, :) = [ci, cj];
                end
            end

            h = sqrt((ni - gi)^2 + (nj - gj)^2);
            f_val = g_cost(ni, nj) + h;

            if open_set(ni, nj)
                for oi = 1:size(open_list, 1)
                    if open_list(oi, 1) == ni && open_list(oi, 2) == nj
                        open_list(oi, 3) = f_val;
                        break;
                    end
                end
            else
                open_list = [open_list; ni, nj, f_val];
                open_set(ni, nj) = true;
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end
    path = reconstruct_path_grid(gi, gj, si, sj, parent, map_info.resolution, map_info.height_m);
end

function los = line_of_sight(i1, j1, i2, j2, traversable)
% Use the same strict segment collision checker as the global planner.
% Table projection cells and safety-dilated table cells are not traversable.
    los = collision_free([i1, j1], [i2, j2], traversable);
end
