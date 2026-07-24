function path = run_ga_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    pop_size = 40;
    n_generations = 150;
    elite_count = 4;
    mutation_rate = 0.15;
    max_path_len = 100;

    moves8 = [-1,0; 0,1; 1,0; 0,-1; -1,-1; -1,1; 1,-1; 1,1];

    population = cell(pop_size, 1);
    fitness = inf(pop_size, 1);

    for i = 1:pop_size
        ci = si; cj = sj;
        chrom = [ci, cj];
        for step = 1:max_path_len - 1
            if norm([ci-gi, cj-gj]) < 2
                chrom = [chrom; gi, gj];
                break;
            end
            valid = [];
            for m = 1:8
                ni = ci + moves8(m, 1); nj = cj + moves8(m, 2);
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && traversable(ni, nj)
                    if moves8(m, 1) ~= 0 && moves8(m, 2) ~= 0
                        if ~traversable(ci, nj) || ~traversable(ni, cj)
                            continue;
                        end
                    end
                    valid = [valid; ni, nj];
                end
            end
            if isempty(valid)
                break;
            end
            if rand < 0.6
                d_goal = sqrt((valid(:,1)-gi).^2 + (valid(:,2)-gj).^2);
                [~, sel] = min(d_goal);
            else
                sel = randi(size(valid, 1));
            end
            ci = valid(sel, 1); cj = valid(sel, 2);
            chrom = [chrom; ci, cj];
        end
        population{i} = chrom;
        fitness(i) = ga_fitness(chrom, gi, gj, traversable);
    end

    global_best = [];
    global_best_fit = inf;

    for gen = 1:n_generations
        [~, sort_idx] = sort(fitness);
        population = population(sort_idx);
        fitness = fitness(sort_idx);

        if fitness(1) < global_best_fit
            global_best_fit = fitness(1);
            global_best = population{1};
        end

        if global_best_fit < inf && global_best_fit < norm([si-gi, sj-gj]) * 1.3
            break;
        end

        new_pop = population(1:elite_count);
        new_fit = fitness(1:elite_count);

        for i = elite_count+1:pop_size
            tournament = randperm(pop_size, 3);
            [~, p1_idx] = min(fitness(tournament));
            p1 = population{tournament(p1_idx)};

            tournament2 = randperm(pop_size, 3);
            [~, p2_idx] = min(fitness(tournament2));
            p2 = population{tournament2(p2_idx)};

            child = ga_crossover(p1, p2, traversable, rows, cols);

            if rand < mutation_rate
                child = ga_mutate(child, traversable, gi, gj, rows, cols);
            end

            new_pop{i} = child;
            new_fit(i) = ga_fitness(child, gi, gj, traversable);
        end

        population = new_pop;
        fitness = new_fit;
    end

    if isempty(global_best)
        warning('GA 未找到有效路径，回退到 A*。');
        path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        return;
    end

    smoothed = ga_smooth(global_best, traversable);
    path = convert_path_grid2world(smoothed, map_info);
end

function f = ga_fitness(chrom, gi, gj, traversable)
    n = size(chrom, 1);
    if n < 2
        f = inf;
        return;
    end
    for k = 1:n
        if ~traversable(chrom(k, 1), chrom(k, 2))
            f = inf;
            return;
        end
    end
    f = 0;
    for k = 1:n - 1
        if ~edge_collision_free_ga(chrom(k, :), chrom(k+1, :), traversable)
            f = f + 1000;
        end
        f = f + norm(chrom(k+1, :) - chrom(k, :));
    end
    end_dist = sqrt((chrom(end,1)-gi)^2 + (chrom(end,2)-gj)^2);
    f = f + 5 * end_dist;
end

function child = ga_crossover(p1, p2, traversable, rows, cols)
    n1 = size(p1, 1); n2 = size(p2, 1);
    if n1 < 3 || n2 < 3
        child = p1;
        return;
    end
    cp1 = randi([2, n1 - 1]);
    cp2 = randi([2, n2 - 1]);
    child = [p1(1:cp1, :); p2(cp2:end, :)];
    if size(child, 1) < 2
        child = p1;
        return;
    end
    if cp1 < size(p1, 1) && cp2 < size(p2, 1)
        if ~edge_collision_free_ga(p1(cp1, :), p2(cp2, :), traversable)
            child = p1;
        end
    end
end

function child = ga_mutate(chrom, traversable, gi, gj, rows, cols)
    child = chrom;
    if size(chrom, 1) < 3
        return;
    end
    idx = randi([2, size(chrom, 1) - 1]);
    moves = [-1,0; 0,1; 1,0; 0,-1; -1,-1; -1,1; 1,-1; 1,1];
    valid = [];
    for m = 1:8
        ni = chrom(idx, 1) + moves(m, 1); nj = chrom(idx, 2) + moves(m, 2);
        if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && traversable(ni, nj)
            if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                if ~traversable(chrom(idx, 1), nj) || ~traversable(ni, chrom(idx, 2))
                    continue;
                end
            end
            valid = [valid; ni, nj];
        end
    end
    if ~isempty(valid)
        d_to_goal = sqrt((valid(:,1)-gi).^2 + (valid(:,2)-gj).^2);
        [~, sel] = min(d_to_goal);
        new_val = valid(sel, :);
        if edge_collision_free_ga(chrom(idx-1, :), new_val, traversable) && ...
           edge_collision_free_ga(new_val, chrom(idx+1, :), traversable)
            child(idx, :) = new_val;
        end
    end
end

function smoothed = ga_smooth(grid_path, traversable)
    if size(grid_path, 1) <= 2
        smoothed = grid_path;
        return;
    end
    smoothed = grid_path(1, :);
    ci = grid_path(1, 1); cj = grid_path(1, 2);
    for k = 2:size(grid_path, 1)
        if ~(grid_path(k, 1) == ci && grid_path(k, 2) == cj)
            if edge_collision_free_ga([ci, cj], grid_path(k, :), traversable)
                smoothed = [smoothed; grid_path(k, :)];
                ci = grid_path(k, 1); cj = grid_path(k, 2);
            end
        end
    end
end

function free = edge_collision_free_ga(p1, p2, traversable)
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