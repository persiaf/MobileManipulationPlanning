function path = run_voronoi_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, ~] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    traversable = (grid_map == 0);
    dist_to_obs = compute_obstacle_dist(grid_map, rows, cols);

    voronoi_free = false(rows, cols);
    moves8 = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    for i = 2:rows-1
        for j = 2:cols-1
            if ~traversable(i, j)
                continue;
            end
            center = dist_to_obs(i, j);
            is_ridge = false;
            for d = 1:8
                ni = i + moves8(d, 1); nj = j + moves8(d, 2);
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && traversable(ni, nj)
                    if dist_to_obs(ni, nj) < center - 0.1
                        is_ridge = true;
                        break;
                    end
                end
            end
            voronoi_free(i, j) = is_ridge || center > 1.5;
        end
    end

    clear_map = traversable & voronoi_free;

    if ~clear_map(si, sj)
        [si, sj] = snap_to_voronoi(si, sj, clear_map, rows, cols);
    end
    if ~clear_map(gi, gj)
        [gi, gj] = snap_to_voronoi(gi, gj, clear_map, rows, cols);
    end

    g_cost = inf(rows, cols);
    g_cost(si, sj) = 0;
    parent = zeros(rows, cols, 2);
    visited = false(rows, cols);

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    open_list = [si, sj, heuristic(si, sj, gi, gj)];
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
            if ~clear_map(ni, nj) || visited(ni, nj)
                continue;
            end
            if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                if ~clear_map(ci, nj) || ~clear_map(ni, cj)
                    continue;
                end
            end
            tentative_g = g_cost(ci, cj) + moves(m, 3);
            if tentative_g < g_cost(ni, nj)
                g_cost(ni, nj) = tentative_g;
                parent(ni, nj, :) = [ci, cj];
                h = heuristic(ni, nj, gi, gj);
                open_list = [open_list; ni, nj, tentative_g + h];
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end
    path = reconstruct_path_grid(gi, gj, si, sj, parent, map_info.resolution, map_info.height_m);
end

function dist = compute_obstacle_dist(grid_map, rows, cols)
    dist = inf(rows, cols);
    p=[]; q=[];
    [obs_i, obs_j] = find(grid_map >= 1);
    for k = 1:length(obs_i)
        i = obs_i(k); j = obs_j(k);
        if i >= 1 && i <= rows && j >= 1 && j <= cols
            dist(i, j) = 0;
            p = [p, i]; q = [q, j];
        end
    end
    idx = 1;
    while idx <= length(p)
        i = p(idx); j = q(idx);
        for di = -1:1
            for dj = -1:1
                if di == 0 && dj == 0
                    continue;
                end
                ni = i + di; nj = j + dj;
                if ni < 1 || ni > rows || nj < 1 || nj > cols
                    continue;
                end
                nd = dist(i, j) + sqrt(di^2 + dj^2);
                if nd < dist(ni, nj)
                    dist(ni, nj) = nd;
                    p = [p, ni]; q = [q, nj];
                end
            end
        end
        idx = idx + 1;
        if idx > rows * cols * 4
            break;
        end
    end
end

function h = heuristic(i1, j1, i2, j2)
    h = sqrt((i1 - i2)^2 + (j1 - j2)^2);
end

function [fi, fj] = snap_to_voronoi(si, sj, clear_map, rows, cols)
    fi = si; fj = sj;
    for r = 1:max(rows, cols)
        for di = -r:r
            for dj = -r:r
                ni = si + di; nj = sj + dj;
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && clear_map(ni, nj)
                    fi = ni; fj = nj; return;
                end
            end
        end
    end
end