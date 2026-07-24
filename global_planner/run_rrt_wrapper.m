function path = run_rrt_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    max_nodes = 5000;
    step_size = 5;
    goal_bias = 0.1;
    goal_radius = 3;

    nodes = zeros(max_nodes, 2);
    parent = zeros(max_nodes, 1);
    nodes(1, :) = [si, sj];

    n_nodes = 1;
    found = false;
    goal_node = 0;

    for iter = 1:max_nodes
        if rand < goal_bias
            rand_pt = [gi, gj];
        else
            rand_i = randi(rows);
            rand_j = randi(cols);
            while ~traversable(rand_i, rand_j)
                rand_i = randi(rows);
                rand_j = randi(cols);
            end
            rand_pt = [rand_i, rand_j];
        end

        [nearest_idx, nearest_dist] = nearest_node(nodes(1:n_nodes, :), rand_pt);

        if nearest_dist < 1e-6
            continue;
        end

        dir_vec = (rand_pt - nodes(nearest_idx, :)) / nearest_dist;
        new_pt = round(nodes(nearest_idx, :) + step_size * dir_vec);

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

        n_nodes = n_nodes + 1;
        nodes(n_nodes, :) = new_pt;
        parent(n_nodes) = nearest_idx;

        if norm(new_pt - [gi, gj]) <= goal_radius
            found = true;
            goal_node = n_nodes;
            break;
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end

    grid_path = [gi, gj];
    cn = goal_node;
    max_path = n_nodes;
    for step = 1:max_path
        grid_path = [nodes(cn, :); grid_path];
        cn = parent(cn);
        if cn == 0 || cn == 1
            break;
        end
    end
    grid_path = [nodes(1, :); grid_path];

    path = convert_path_grid2world(grid_path, map_info);
end

function [idx, dist] = nearest_node(nodes, pt)
    d2 = sum((nodes - pt).^2, 2);
    [dist, idx] = min(d2);
    dist = sqrt(dist);
end