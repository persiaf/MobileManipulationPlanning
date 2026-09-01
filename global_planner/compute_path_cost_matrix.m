function dist_matrix = compute_path_cost_matrix(poses, grid_map, map_info, planner_cfg)
    n = size(poses, 1);
    dist_matrix = inf(n, n);
    lambda_theta = 0;
    if isfield(planner_cfg, 'edge_cost') && isfield(planner_cfg.edge_cost, 'lambda_theta')
        lambda_theta = planner_cfg.edge_cost.lambda_theta;
    end

    for i = 1:n
        for j = i+1:n
            [~, d] = query_path_with_shared_cache(grid_map, poses(i,:), poses(j,:), map_info, planner_cfg);
            if d > 1e6 || isnan(d)
                d = inf;
            end

            if isfinite(d) && lambda_theta > 0
                delta_theta = abs(wrapToPiLocal(poses(j,3) - poses(i,3)));
                d = d + lambda_theta * delta_theta;
            end

            dist_matrix(i, j) = d;
            dist_matrix(j, i) = d;
        end
    end
end
