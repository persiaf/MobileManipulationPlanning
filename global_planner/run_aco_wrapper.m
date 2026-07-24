function path = run_aco_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    n_ants = 10;
    max_iters = 50;
    max_steps = 500;
    alpha = 1.5;
    beta = 3.0;
    rho = 0.1;
    q0 = 0.3;

    pheromone = ones(rows, cols) * 0.01;
    [col_grid, row_grid] = meshgrid(1:cols, 1:rows);
    dist_to_goal = sqrt((row_grid - gi).^2 + (col_grid - gj).^2);
    heuristic_map = zeros(rows, cols);
    mask = traversable;
    heuristic_map(mask) = 1.0 ./ (dist_to_goal(mask) + 0.1);

    moves = [-1, 0; 0, 1; 1, 0; 0, -1; -1,-1; -1, 1; 1, -1; 1, 1];
    move_cost = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];

    best_path_grid = [];
    best_path_cost = inf;

    all_paths = cell(n_ants, 1);
    all_costs = inf(n_ants, 1);

    for iter = 1:max_iters
        for ant = 1:n_ants
            ci = si; cj = sj;
            ant_path = [ci, cj];
            visited_ant = false(rows, cols);
            visited_ant(ci, cj) = true;
            ant_found = false;

            for step = 1:max_steps
                if ci == gi && cj == gj
                    ant_found = true;
                    break;
                end
                neighbors = [];
                probs = [];
                for m = 1:8
                    ni = ci + moves(m, 1); nj = cj + moves(m, 2);
                    if ni < 1 || ni > rows || nj < 1 || nj > cols
                        continue;
                    end
                    if ~traversable(ni, nj) || visited_ant(ni, nj)
                        continue;
                    end
                    if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                        if ~traversable(ci, nj) || ~traversable(ni, cj)
                            continue;
                        end
                    end
                    neighbors = [neighbors; ni, nj];
                    prob = (pheromone(ni, nj)^alpha) * (heuristic_map(ni, nj)^beta);
                    probs = [probs; prob];
                end
                if isempty(neighbors)
                    break;
                end
                probs = probs / sum(probs);
                if rand < q0
                    [~, sel] = max(probs);
                else
                    cp = cumsum(probs);
                    sel = find(cp > rand, 1);
                end
                ci = neighbors(sel, 1); cj = neighbors(sel, 2);
                visited_ant(ci, cj) = true;
                ant_path = [ant_path; ci, cj];
            end

            if ant_found
                path_cost = 0;
                for p = 2:size(ant_path, 1)
                    d = find(moves(:,1) == (ant_path(p,1)-ant_path(p-1,1)) & ...
                             moves(:,2) == (ant_path(p,2)-ant_path(p-1,2)));
                    if ~isempty(d)
                        path_cost = path_cost + move_cost(d);
                    end
                end
                all_paths{ant} = ant_path;
                all_costs(ant) = path_cost;

                if path_cost < best_path_cost
                    best_path_cost = path_cost;
                    best_path_grid = ant_path;
                end
            else
                all_paths{ant} = [];
                all_costs(ant) = inf;
            end
        end

        pheromone = pheromone * (1 - rho);

        for ant = 1:n_ants
            if ~isempty(all_paths{ant})
                delta = 1.0 / all_costs(ant);
                ap = all_paths{ant};
                for q = 1:size(ap, 1)
                    pheromone(ap(q,1), ap(q,2)) = pheromone(ap(q,1), ap(q,2)) + delta;
                end
            end
        end
    end

    if isempty(best_path_grid)
        path = [start_pose; goal_pose];
        return;
    end

    path = convert_path_grid2world(best_path_grid, map_info);
end