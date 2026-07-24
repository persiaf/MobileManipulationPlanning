function path = run_rrt_connect_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    max_nodes = 5000;
    step_size = 5;
    connect_radius = 3;

    nodes_a = zeros(max_nodes, 2); parent_a = zeros(max_nodes, 1);
    nodes_b = zeros(max_nodes, 2); parent_b = zeros(max_nodes, 1);
    nodes_a(1, :) = [si, sj]; nodes_b(1, :) = [gi, gj];

    n_a = 1; n_b = 1;
    connected = false;
    connect_node_a = 0; connect_node_b = 0;

    for iter = 1:max_nodes
        rand_i = randi(rows); rand_j = randi(cols);
        while ~traversable(rand_i, rand_j)
            rand_i = randi(rows); rand_j = randi(cols);
        end
        rand_pt = [rand_i, rand_j];

        [nearest_a, ~] = nearest_node_rc(nodes_a(1:n_a, :), rand_pt);
        dir_a = rand_pt - nodes_a(nearest_a, :);
        nd_a = norm(dir_a);
        if nd_a > 1e-6
            dir_a = dir_a / nd_a;
            new_a = round(nodes_a(nearest_a, :) + min(step_size, nd_a) * dir_a);
            if new_a(1) >= 1 && new_a(1) <= rows && new_a(2) >= 1 && new_a(2) <= cols && traversable(new_a(1), new_a(2))
                if collision_free(nodes_a(nearest_a, :), new_a, traversable)
                    n_a = n_a + 1;
                    nodes_a(n_a, :) = new_a;
                    parent_a(n_a) = nearest_a;

                    [nearest_b, dist_b] = nearest_node_rc(nodes_b(1:n_b, :), new_a);
                    if dist_b <= connect_radius
                        connected = true;
                        connect_node_a = n_a;
                        connect_node_b = nearest_b;
                        break;
                    end
                end
            end
        end

        [nearest_b, ~] = nearest_node_rc(nodes_b(1:n_b, :), nodes_a(n_a, :));
        dir_b = nodes_a(n_a, :) - nodes_b(nearest_b, :);
        nd_b = norm(dir_b);
        if nd_b > 1e-6
            dir_b = dir_b / nd_b;
            new_b = round(nodes_b(nearest_b, :) + min(step_size, nd_b) * dir_b);
            if new_b(1) >= 1 && new_b(1) <= rows && new_b(2) >= 1 && new_b(2) <= cols && traversable(new_b(1), new_b(2))
                if collision_free(nodes_b(nearest_b, :), new_b, traversable)
                    n_b = n_b + 1;
                    nodes_b(n_b, :) = new_b;
                    parent_b(n_b) = nearest_b;

                    [nearest_a2, dist_a2] = nearest_node_rc(nodes_a(1:n_a, :), new_b);
                    if dist_a2 <= connect_radius
                        connected = true;
                        connect_node_a = nearest_a2;
                        connect_node_b = n_b;
                        break;
                    end
                end
            end
        end
    end

    if ~connected
        path = [start_pose; goal_pose];
        return;
    end

    grid_path_a = [];
    cn = connect_node_a;
    for step = 1:n_a
        grid_path_a = [nodes_a(cn, :); grid_path_a];
        cn = parent_a(cn);
        if cn == 0 || cn == 1
            break;
        end
    end
    grid_path_a = [nodes_a(1, :); grid_path_a];

    grid_path_b = [];
    cn = connect_node_b;
    for step = 1:n_b
        grid_path_b = [nodes_b(cn, :); grid_path_b];
        cn = parent_b(cn);
        if cn == 0 || cn == 1
            break;
        end
    end
    grid_path_b = [grid_path_b; nodes_b(1, :)];

    grid_path = [grid_path_a; grid_path_b];

    path = convert_path_grid2world(grid_path, map_info);
end

function [idx, dist] = nearest_node_rc(nodes, pt)
    d2 = sum((nodes - pt).^2, 2);
    [dist, idx] = min(d2);
    dist = sqrt(dist);
end