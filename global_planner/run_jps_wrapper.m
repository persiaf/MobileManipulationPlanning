function path = run_jps_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g_cost = inf(rows, cols);
    g_cost(si, sj) = 0;

    euclidean_h = zeros(rows, cols);
    for i = 1:rows
        for j = 1:cols
            euclidean_h(i, j) = sqrt((i - gi)^2 + (j - gj)^2);
        end
    end

    parent = zeros(rows, cols, 2);
    open_set = false(rows, cols);
    closed_set = false(rows, cols);

    open_list = [si, sj, euclidean_h(si, sj)];
    open_set(si, sj) = true;

    max_iter = rows * cols;
    iter = 0;
    found = false;

    dirs_all = [-1,  0; 1,  0; 0, -1; 0,  1;
                -1, -1; 1, -1; -1, 1; 1,  1];

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;

        [~, min_idx] = min(open_list(:, 3));
        current = open_list(min_idx, 1:2);
        open_list(min_idx, :) = [];
        open_set(current(1), current(2)) = false;

        ci = current(1);
        cj = current(2);

        if closed_set(ci, cj)
            continue;
        end
        closed_set(ci, cj) = true;

        if ci == gi && cj == gj
            found = true;
            break;
        end

        successors = identify_successors(ci, cj, gi, gj, parent, traversable, rows, cols);

        for s = 1:size(successors, 1)
            ni = successors(s, 1);
            nj = successors(s, 2);

            if closed_set(ni, nj)
                continue;
            end

            dx = ni - ci;
            dy = nj - cj;
            move_cost = sqrt(dx^2 + dy^2);
            tentative_g = g_cost(ci, cj) + move_cost;

            if tentative_g < g_cost(ni, nj)
                g_cost(ni, nj) = tentative_g;
                parent(ni, nj, :) = [ci, cj];
                f_val = tentative_g + euclidean_h(ni, nj);

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

function successors = identify_successors(ci, cj, gi, gj, parent, traversable, rows, cols)
    successors = [];

    if parent(ci, cj, 1) == 0 && parent(ci, cj, 2) == 0
        dirs = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; 1, -1; -1, 1; 1, 1];
        for d = 1:8
            dir_vec = dirs(d, :);
            jp = jump(ci, cj, dir_vec, gi, gj, traversable, rows, cols);
            if ~isempty(jp)
                successors = [successors; jp];
            end
        end
        return;
    end

    dx = ci - parent(ci, cj, 1);
    dy = cj - parent(ci, cj, 2);
    if dx ~= 0 && dy ~= 0
        ndx = sign(dx);
        ndy = sign(dy);
    else
        if dx ~= 0
            ndx = sign(dx);
        else
            ndx = 0;
        end
        if dy ~= 0
            ndy = sign(dy);
        else
            ndy = 0;
        end
    end

    if ndx ~= 0 && ndy ~= 0
        jp = jump(ci, cj, [ndx, ndy], gi, gj, traversable, rows, cols);
        if ~isempty(jp)
            successors = [successors; jp];
        end

        jp_h = jump(ci, cj, [ndx, 0], gi, gj, traversable, rows, cols);
        if ~isempty(jp_h)
            successors = [successors; jp_h];
        end

        jp_v = jump(ci, cj, [0, ndy], gi, gj, traversable, rows, cols);
        if ~isempty(jp_v)
            successors = [successors; jp_v];
        end
    else
        jp = jump(ci, cj, [ndx, ndy], gi, gj, traversable, rows, cols);
        if ~isempty(jp)
            successors = [successors; jp];
        end

        if ndx == 0
            if ~is_obstacle(ci-1, cj+ndy, traversable, rows, cols) && ...
               is_obstacle(ci-1, cj, traversable, rows, cols)
                jp = jump(ci, cj, [-1, ndy], gi, gj, traversable, rows, cols);
                if ~isempty(jp)
                    successors = [successors; jp];
                end
            end
            if ~is_obstacle(ci+1, cj+ndy, traversable, rows, cols) && ...
               is_obstacle(ci+1, cj, traversable, rows, cols)
                jp = jump(ci, cj, [1, ndy], gi, gj, traversable, rows, cols);
                if ~isempty(jp)
                    successors = [successors; jp];
                end
            end
        else
            if ~is_obstacle(ci+ndx, cj-1, traversable, rows, cols) && ...
               is_obstacle(ci, cj-1, traversable, rows, cols)
                jp = jump(ci, cj, [ndx, -1], gi, gj, traversable, rows, cols);
                if ~isempty(jp)
                    successors = [successors; jp];
                end
            end
            if ~is_obstacle(ci+ndx, cj+1, traversable, rows, cols) && ...
               is_obstacle(ci, cj+1, traversable, rows, cols)
                jp = jump(ci, cj, [ndx, 1], gi, gj, traversable, rows, cols);
                if ~isempty(jp)
                    successors = [successors; jp];
                end
            end
        end
    end
end

function jp = jump(ci, cj, dir, gi, gj, traversable, rows, cols)
    ni = ci + dir(1);
    nj = cj + dir(2);

    if ni < 1 || ni > rows || nj < 1 || nj > cols
        jp = [];
        return;
    end
    if ~traversable(ni, nj)
        jp = [];
        return;
    end

    if ni == gi && nj == gj
        jp = [ni, nj];
        return;
    end

    if dir(1) ~= 0 && dir(2) ~= 0
        if (~is_obstacle(ni-dir(1), nj, traversable, rows, cols) && ...
             is_obstacle(ni-dir(1), nj-dir(2), traversable, rows, cols)) || ...
           (~is_obstacle(ni, nj-dir(2), traversable, rows, cols) && ...
             is_obstacle(ni-dir(1), nj-dir(2), traversable, rows, cols))
            jp = [ni, nj];
            return;
        end

        jp_h = jump(ni, nj, [dir(1), 0], gi, gj, traversable, rows, cols);
        if ~isempty(jp_h)
            jp = [ni, nj];
            return;
        end

        jp_v = jump(ni, nj, [0, dir(2)], gi, gj, traversable, rows, cols);
        if ~isempty(jp_v)
            jp = [ni, nj];
            return;
        end
    else
        if dir(1) ~= 0
            nxt = [dir(1), 0];
            left_blocked  = is_obstacle(ni, nj-1, traversable, rows, cols) && ...
                            ~is_obstacle(ni+dir(1), nj-1, traversable, rows, cols);
            right_blocked = is_obstacle(ni, nj+1, traversable, rows, cols) && ...
                            ~is_obstacle(ni+dir(1), nj+1, traversable, rows, cols);
            if left_blocked || right_blocked
                jp = [ni, nj];
                return;
            end
        else
            nxt = [0, dir(2)];
            left_blocked  = is_obstacle(ni-1, nj, traversable, rows, cols) && ...
                            ~is_obstacle(ni-1, nj+dir(2), traversable, rows, cols);
            right_blocked = is_obstacle(ni+1, nj, traversable, rows, cols) && ...
                            ~is_obstacle(ni+1, nj+dir(2), traversable, rows, cols);
            if left_blocked || right_blocked
                jp = [ni, nj];
                return;
            end
        end
    end

    jp = jump(ni, nj, dir, gi, gj, traversable, rows, cols);
end

function obs = is_obstacle(i, j, traversable, rows, cols)
    if i < 1 || i > rows || j < 1 || j > cols
        obs = true;
    else
        obs = ~traversable(i, j);
    end
end