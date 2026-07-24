function path = run_informed_rrt_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    max_nodes = 800;
    step_size = 5;
    search_radius = 10;
    goal_radius = 3;

    nodes = zeros(max_nodes, 2);
    parent = zeros(max_nodes, 1);
    cost = inf(max_nodes, 1);
    nodes(1, :) = [si, sj];
    cost(1) = 0;

    n_nodes = 1;
    found = false;
    best_goal_node = 0;
    best_goal_cost = inf;

    c_best = inf;
    c_min = norm([si - gi, sj - gj]);
    ellipse_enabled = false;

    for iter = 1:max_nodes
        if ellipse_enabled && c_best < inf
            rand_pt = sample_ellipse([si, sj], [gi, gj], c_best, c_min);
            i_pt = round(rand_pt(1)); j_pt = round(rand_pt(2));
            if i_pt < 1 || i_pt > rows || j_pt < 1 || j_pt > cols || ~traversable(i_pt, j_pt)
                rand_i = randi(rows); rand_j = randi(cols);
                while ~traversable(rand_i, rand_j)
                    rand_i = randi(rows); rand_j = randi(cols);
                end
                rand_pt = [rand_i, rand_j];
            end
        else
            rand_i = randi(rows); rand_j = randi(cols);
            while ~traversable(rand_i, rand_j)
                rand_i = randi(rows); rand_j = randi(cols);
            end
            rand_pt = [rand_i, rand_j];
        end

        [nearest_idx, ~] = nearest_node_ir(nodes(1:n_nodes, :), rand_pt);
        dir_vec = rand_pt - nodes(nearest_idx, :);
        nd = norm(dir_vec);
        if nd < 1e-6
            continue;
        end
        dir_vec = dir_vec / nd;
        new_pt = round(nodes(nearest_idx, :) + min(step_size, nd) * dir_vec);

        new_i = new_pt(1); new_j = new_pt(2);
        if new_i < 1 || new_i > rows || new_j < 1 || new_j > cols
            continue;
        end
        if ~traversable(new_i, new_j)
            continue;
        end
        if ~collision_free(nodes(nearest_idx, :), new_pt, traversable)
            continue;
        end

        neighbors = [];
        for ni = 1:n_nodes
            if norm(new_pt - nodes(ni, :)) <= search_radius
                neighbors = [neighbors, ni];
            end
        end

        min_cost = cost(nearest_idx) + norm(new_pt - nodes(nearest_idx, :));
        min_parent = nearest_idx;

        for ni = neighbors
            if collision_free(nodes(ni, :), new_pt, traversable)
                tentative = cost(ni) + norm(new_pt - nodes(ni, :));
                if tentative < min_cost
                    min_cost = tentative;
                    min_parent = ni;
                end
            end
        end

        n_nodes = n_nodes + 1;
        nodes(n_nodes, :) = new_pt;
        parent(n_nodes) = min_parent;
        cost(n_nodes) = min_cost;

        for ni = neighbors
            if collision_free(new_pt, nodes(ni, :), traversable)
                new_cost = cost(n_nodes) + norm(new_pt - nodes(ni, :));
                if new_cost < cost(ni)
                    parent(ni) = n_nodes;
                    cost(ni) = new_cost;
                end
            end
        end

        if norm(new_pt - [gi, gj]) <= goal_radius
            found = true;
            if cost(n_nodes) < best_goal_cost
                best_goal_cost = cost(n_nodes);
                best_goal_node = n_nodes;
                c_best = best_goal_cost;
                ellipse_enabled = true;
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end

    grid_path = [gi, gj];
    cn = best_goal_node;
    for step = 1:n_nodes
        grid_path = [nodes(cn, :); grid_path];
        cn = parent(cn);
        if cn == 0 || cn == 1
            break;
        end
    end
    grid_path = [nodes(1, :); grid_path];

    path = convert_path_grid2world(grid_path, map_info);
end

function pt = sample_ellipse(start_pt, goal_pt, c_best, c_min)
    center = (start_pt + goal_pt) / 2;
    C = diag([(c_best/2)^2, (c_best/2)^2 - (c_min/2)^2]);
    diff_vec = goal_pt - start_pt;
    theta = atan2(diff_vec(1), diff_vec(2));
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    L = sqrt(C(1,1));
    if C(2,2) <= 0
        C(2,2) = 1;
    end
    L2 = sqrt(C(2,2));

    if rand < 0.5 && L2 > 0
        r_ball = L2 * sqrt(rand());
    else
        r_ball = L * sqrt(rand());
    end
    angle = 2 * pi * rand();
    pt_ellipse = [r_ball * cos(angle); r_ball * sin(angle)];
    pt = (center' + R' * pt_ellipse)';
end

function [idx, dist] = nearest_node_ir(nodes, pt)
    d2 = sum((nodes - pt).^2, 2);
    [dist, idx] = min(d2);
    dist = sqrt(dist);
end