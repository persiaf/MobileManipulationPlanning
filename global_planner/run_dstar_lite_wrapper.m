function path = run_dstar_lite_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g = inf(rows, cols);
    rhs = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    rhs(gi, gj) = 0;

    open_key1 = inf(rows, cols);
    open_key2 = inf(rows, cols);
    open_list = [];
    open_map = false(rows, cols);

    open_key1(gi, gj) = heuristic(gi, gj, si, sj);
    open_key2(gi, gj) = 0;
    open_list = sub2ind([rows, cols], gi, gj);
    open_map(gi, gj) = true;

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    max_iter = rows * cols;
    iter = 0;
    found = false;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;

        [~, min_idx] = min(open_key1(open_list) + open_key2(open_list));
        li = open_list(min_idx);
        [ui, uj] = ind2sub([rows, cols], li);
        open_list(min_idx) = [];
        open_map(ui, uj) = false;

        if g(ui, uj) > rhs(ui, uj)
            g(ui, uj) = rhs(ui, uj);
            if ui == si && uj == sj
                found = true;
            end
        else
            g(ui, uj) = inf;
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
                [rhs(vi, vj), parent(vi, vj, :)] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols);
                [g, rhs, open_list, open_map, open_key1, open_key2] = ...
                    update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj);
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
            [rhs(vi, vj), parent(vi, vj, :)] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols);
            [g, rhs, open_list, open_map, open_key1, open_key2] = ...
                update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj);
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end
    path = reconstruct_path_grid(si, sj, gi, gj, parent, map_info.resolution, map_info.height_m);
end

function h = heuristic(i1, j1, i2, j2)
    h = sqrt((i1 - i2)^2 + (j1 - j2)^2);
end

function [rhs_val, best_pred] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols)
    if vi == gi && vj == gj
        rhs_val = 0;
        best_pred = [0, 0];
        return;
    end
    best = inf;
    best_pred = [0, 0];
    for d = 1:size(moves, 1)
        pi = vi - moves(d, 1); pj = vj - moves(d, 2);
        if pi < 1 || pi > rows || pj < 1 || pj > cols || ~traversable(pi, pj)
            continue;
        end
        if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
            if ~traversable(vi, pj) || ~traversable(pi, vj)
                continue;
            end
        end
        val = g(pi, pj) + moves(d, 3);
        if val < best
            best = val;
            best_pred = [pi, pj];
        end
    end
    rhs_val = best;
end

function [g, rhs, open_list, open_map, open_key1, open_key2] = update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj)
    lin_idx = sub2ind(size(g), vi, vj);
    if ~open_map(vi, vj)
        if g(vi, vj) ~= rhs(vi, vj)
            open_map(vi, vj) = true;
            open_list = [open_list; lin_idx];
            open_key1(vi, vj) = min(g(vi, vj), rhs(vi, vj)) + heuristic(vi, vj, si, sj);
            open_key2(vi, vj) = min(g(vi, vj), rhs(vi, vj));
        end
    end
end