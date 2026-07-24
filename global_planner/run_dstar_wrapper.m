function path = run_dstar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    STATE_NEW      = 0;
    STATE_OPEN     = 1;
    STATE_CLOSED   = 2;

    state_t = STATE_NEW * ones(rows, cols);
    state_h = inf(rows, cols);
    state_k = inf(rows, cols);
    state_bi = zeros(rows, cols);
    state_bj = zeros(rows, cols);

    state_t(~traversable) = -1;

    state_h(gi, gj) = 0;
    state_k(gi, gj) = 0;
    state_t(gi, gj) = STATE_OPEN;
    open_i = gi;
    open_j = gj;

    max_iters = rows * cols;
    found = false;

    dk = [1,0; -1,0; 0,1; 0,-1; 1,1; -1,1; 1,-1; -1,-1];
    dcost = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];

    for iter = 1:max_iters
        if isempty(open_i)
            break;
        end

        best_k = inf;
        best_idx = 1;
        for n = 1:length(open_i)
            kval = state_k(open_i(n), open_j(n));
            if kval < best_k
                best_k = kval;
                best_idx = n;
            end
        end
        xi = open_i(best_idx); xj = open_j(best_idx);
        open_i(best_idx) = []; open_j(best_idx) = [];
        state_t(xi, xj) = STATE_CLOSED;

        if xi == si && xj == sj
            found = true;
            break;
        end

        for d = 1:8
            yi = xi + dk(d,1); yj = xj + dk(d,2);
            if yi < 1 || yi > rows || yj < 1 || yj > cols
                continue;
            end
            if state_t(yi, yj) == -1
                continue;
            end
            if dk(d,1) ~= 0 && dk(d,2) ~= 0
                if state_t(xi, yj) == -1 || state_t(yi, xj) == -1
                    continue;
                end
            end
            if yi == gi && yj == gj
                continue;
            end
            h_new = state_h(xi, xj) + dcost(d);
            if state_t(yi, yj) == STATE_NEW || h_new < state_h(yi, yj)
                state_bi(yi, yj) = xi;
                state_bj(yi, yj) = xj;
                state_h(yi, yj) = h_new;
                state_k(yi, yj) = h_new;
                if state_t(yi, yj) ~= STATE_OPEN
                    state_t(yi, yj) = STATE_OPEN;
                    open_i = [open_i; yi];
                    open_j = [open_j; yj];
                end
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end

    grid_path = [si, sj];
    ci = si; cj = sj;
    max_retrace = rows * cols;
    for step = 1:max_retrace
        if ci == gi && cj == gj
            break;
        end
        nxt_i = state_bi(ci, cj);
        nxt_j = state_bj(ci, cj);
        if nxt_i == 0 && nxt_j == 0
            break;
        end
        ci = nxt_i; cj = nxt_j;
        grid_path = [grid_path; ci, cj];
    end

    path = convert_path_grid2world(grid_path, map_info);
end