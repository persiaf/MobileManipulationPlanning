function path = run_pso_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    n_particles = 20;
    max_iters = 50;
    w = 0.7;
    c1 = 1.5;
    c2 = 1.5;
    max_path_len = 100;

    positions = cell(n_particles, 1);
    velocities = cell(n_particles, 1);
    p_best_pos = cell(n_particles, 1);
    p_best_cost = inf(n_particles, 1);
    g_best_pos = [];
    g_best_cost = inf;

    moves8 = [-1,0; 0,1; 1,0; 0,-1; -1,-1; -1,1; 1,-1; 1,1];

    for p = 1:n_particles
        ci = si; cj = sj;
        pos = [ci, cj];
        for step = 1:max_path_len - 1
            if norm([ci-gi, cj-gj]) < 2
                break;
            end
            valid_moves = [];
            for m = 1:8
                ni = ci + moves8(m, 1); nj = cj + moves8(m, 2);
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && traversable(ni, nj)
                    if moves8(m, 1) ~= 0 && moves8(m, 2) ~= 0
                        if ~traversable(ci, nj) || ~traversable(ni, cj)
                            continue;
                        end
                    end
                    valid_moves = [valid_moves; ni, nj, m];
                end
            end
            if isempty(valid_moves)
                break;
            end
            dists = sqrt((valid_moves(:,1)-gi).^2 + (valid_moves(:,2)-gj).^2);
            [~, best_m] = min(dists);
            if rand < 0.7
                ci = valid_moves(best_m, 1);
                cj = valid_moves(best_m, 2);
            else
                r = randi(size(valid_moves, 1));
                ci = valid_moves(r, 1);
                cj = valid_moves(r, 2);
            end
            pos = [pos; ci, cj];
        end
        positions{p} = pos;
        velocities{p} = zeros(size(pos));
        p_best_pos{p} = pos;
        p_best_cost(p) = path_cost_pso(pos, traversable);
    end

    for iter = 1:max_iters
        for p = 1:n_particles
            pos = positions{p};
            vel = velocities{p};
            gb = g_best_pos;

            if ~isempty(gb)
                for k = 2:size(pos, 1)
                    r1 = rand; r2 = rand;
                    vel_i = w * vel(k, :) + ...
                             c1 * r1 * (p_best_pos{p}(k, :) - pos(k, :)) + ...
                             c2 * r2 * (gb(min(k, size(gb,1)), :) - pos(k, :));
                    vel(k, :) = vel_i;
                end
            end

            new_pos = round(pos + vel);
            for k = 1:size(new_pos, 1)
                new_pos(k, 1) = max(1, min(rows, new_pos(k, 1)));
                new_pos(k, 2) = max(1, min(cols, new_pos(k, 2)));
                if ~traversable(new_pos(k, 1), new_pos(k, 2))
                    new_pos(k, :) = pos(k, :);
                end
            end

            for k = 2:size(new_pos, 1)
                if ~edge_collision_free_ps(new_pos(k-1, :), new_pos(k, :), traversable)
                    new_pos(k, :) = pos(k, :);
                end
            end

            cost_new = path_cost_pso(new_pos, traversable);
            positions{p} = new_pos;
            velocities{p} = vel;

            if cost_new < p_best_cost(p)
                p_best_cost(p) = cost_new;
                p_best_pos{p} = new_pos;
            end
            if cost_new < g_best_cost
                g_best_cost = cost_new;
                g_best_pos = new_pos;
            end
        end

        if g_best_cost < inf && g_best_cost <= norm([si-gi, sj-gj]) * 1.5
            break;
        end
    end

    if isempty(g_best_pos)
        path = [start_pose; goal_pose];
        return;
    end

    smoothed = smooth_path_pso(g_best_pos, traversable);
    path = convert_path_grid2world(smoothed, map_info);
end

function c = path_cost_pso(pos, traversable)
    if size(pos, 1) < 2
        c = inf;
        return;
    end
    c = 0;
    for k = 1:size(pos, 1) - 1
        if ~edge_collision_free_ps(pos(k, :), pos(k+1, :), traversable)
            c = c + 1000;
        end
        c = c + norm(pos(k+1, :) - pos(k, :));
    end
end

function smoothed = smooth_path_pso(grid_path, traversable)
    if size(grid_path, 1) <= 2
        smoothed = grid_path;
        return;
    end
    smoothed = grid_path(1, :);
    ci = grid_path(1, 1); cj = grid_path(1, 2);
    for k = 2:size(grid_path, 1)
        if norm([grid_path(k,1)-ci, grid_path(k,2)-cj]) > 1
            if edge_collision_free_ps([ci, cj], grid_path(k, :), traversable)
                smoothed = [smoothed; grid_path(k, :)];
                ci = grid_path(k, 1); cj = grid_path(k, 2);
            end
        end
    end
end

function free = edge_collision_free_ps(p1, p2, traversable)
    di = p2(1) - p1(1); dj = p2(2) - p1(2);
    steps = max(abs(di), abs(dj));
    if steps == 0
        free = traversable(p1(1), p1(2));
        return;
    end
    n_check = steps * 5;
    rows_t = size(traversable, 1);
    cols_t = size(traversable, 2);
    for k = 0:n_check
        t = k / n_check;
        i = round(p1(1) + t * di);
        j = round(p1(2) + t * dj);
        if i < 1 || i > rows_t || j < 1 || j > cols_t
            free = false; return;
        end
        if ~traversable(i, j)
            free = false; return;
        end
    end
    free = true;
end