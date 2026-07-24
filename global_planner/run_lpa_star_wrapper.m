function path = run_lpa_star_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g = inf(rows, cols);
    rhs = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    rhs(gi, gj) = 0;

    keys1 = inf(rows, cols);
    keys2 = inf(rows, cols);
    open_list = [];
    open_map = false(rows, cols);

    keys1(gi, gj) = heuristic(gi, gj, si, sj);
    keys2(gi, gj) = 0;
    open_list = sub2ind([rows, cols], gi, gj);
    open_map(gi, gj) = true;

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    max_iter = rows * cols;
    found = false;

    for iter = 1:max_iter
        if isempty(open_list)
            break;
        end

        [~, min_idx] = min(keys1(open_list) + keys2(open_list));
        ulin = open_list(min_idx);
        [ui, uj] = ind2sub([rows, cols], ulin);
        open_list(min_idx) = [];
        open_map(ui, uj) = false;

        if g(ui, uj) > rhs(ui, uj)
            g(ui, uj) = rhs(ui, uj);
            if ui == si && uj == sj
                found = true;
            end
        else
            g(ui, uj) = inf;
            update_neighbors(ui, uj, gi, gj, g, rhs, parent, traversable, moves, rows, cols);
            if ui == si && uj == sj
                r = rhs(ui, uj);
                if r < inf
                    found = true;
                end
            end
        end

        for d = 1:size(moves, 1)
            vi = ui + moves(d, 1); vj = uj + moves(d, 2);
            if vi < 1 || vi > rows || vj < 1 || vj > cols || ~traversable(vi, vj)
                continue;
            end
            if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
                if ~traversable(ui, vj) || ~traversable(vi, uj)
                    continue;
                end
            end
            if vi == gi && vj == gj
                continue;
            end
            [rhs_val, best_pred] = calc_rhs(vi, vj, g, gi, gj, traversable, moves, rows, cols);
            if rhs_val < rhs(vi, vj)
                rhs(vi, vj) = rhs_val;
                parent(vi, vj, 1) = best_pred(1);
                parent(vi, vj, 2) = best_pred(2);
            end
            if g(vi, vj) ~= rhs(vi, vj)
                keys1(vi, vj) = min(g(vi, vj), rhs(vi, vj)) + heuristic(vi, vj, si, sj);
                keys2(vi, vj) = min(g(vi, vj), rhs(vi, vj));
                if ~open_map(vi, vj)
                    open_list = [open_list; sub2ind([rows, cols], vi, vj)];
                    open_map(vi, vj) = true;
                end
            end
        end

        if found
            break;
        end
    end

    if ~found
        warning('LPA* 未找到路径，回退到 A*. ');
        path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        return;
    end
    path = reconstruct_path_grid(si, sj, gi, gj, parent, map_info.resolution, map_info.height_m);
end

function [rhs_val, best_pred] = calc_rhs(vi, vj, g, gi, gj, traversable, moves, rows, cols)
    if vi == gi && vj == gj
        rhs_val = 0;
        best_pred = [0, 0];
        return;
    end
    best = inf;
    best_pred = [0, 0];
    for d = 1:size(moves, 1)
        nxi = vi + moves(d, 1); nxj = vj + moves(d, 2);
        if nxi < 1 || nxi > rows || nxj < 1 || nxj > cols || ~traversable(nxi, nxj)
            continue;
        end
        if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
            if ~traversable(vi, nxj) || ~traversable(nxi, vj)
                continue;
            end
        end
        val = g(nxi, nxj) + moves(d, 3);
        if val < best
            best = val;
            best_pred = [nxi, nxj];
        end
    end
    rhs_val = best;
end

function update_neighbors(ui, uj, gi, gj, g, rhs, parent, traversable, moves, rows, cols)
    for d = 1:size(moves, 1)
        vi = ui + moves(d, 1); vj = uj + moves(d, 2);
        if vi < 1 || vi > rows || vj < 1 || vj > cols || ~traversable(vi, vj)
            continue;
        end
        if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
            if ~traversable(ui, vj) || ~traversable(vi, uj)
                continue;
            end
        end
        if vi == gi && vj == gj
            continue;
        end
        best = inf;
        best_pred = [0, 0];
        for dp = 1:size(moves, 1)
            nxi = vi + moves(dp, 1); nxj = vj + moves(dp, 2);
            if nxi < 1 || nxi > rows || nxj < 1 || nxj > cols || ~traversable(nxi, nxj)
                continue;
            end
            if moves(dp, 1) ~= 0 && moves(dp, 2) ~= 0
                if ~traversable(vi, nxj) || ~traversable(nxi, vj)
                    continue;
                end
            end
            val = g(nxi, nxj) + moves(dp, 3);
            if val < best
                best = val;
                best_pred = [nxi, nxj];
            end
        end
        if best < rhs(vi, vj)
            rhs(vi, vj) = best;
            parent(vi, vj, 1) = best_pred(1);
            parent(vi, vj, 2) = best_pred(2);
        end
    end
end

function h = heuristic(i1, j1, i2, j2)
    h = sqrt((i1 - i2)^2 + (j1 - j2)^2);
end