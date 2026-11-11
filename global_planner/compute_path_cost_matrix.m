function dist_matrix = compute_path_cost_matrix(poses, grid_map, map_info, planner_cfg)
    n = size(poses, 1);
    dist_matrix = inf(n, n);
    dist_matrix(1:n+1:end)=0;
    lambda_theta = 0;
    if isfield(planner_cfg, 'edge_cost') && isfield(planner_cfg.edge_cost, 'lambda_theta')
        lambda_theta = planner_cfg.edge_cost.lambda_theta;
    end

    % Opt-in for path oracles whose validated endpoint connectors/search
    % produce direction-dependent outcomes. Preserve the historical mirrored
    % matrix when this setting is absent or false.
    directed = isfield(planner_cfg, 'routing') && ...
        isfield(planner_cfg.routing, 'directed_costs') && ...
        isequal(planner_cfg.routing.directed_costs, true);
    if directed
        for i = 1:n
            for j = 1:n
                if i == j, continue; end
                [~, d, ok] = query_path_with_shared_cache( ...
                    grid_map, poses(i,:), poses(j,:), map_info, planner_cfg);
                if ~ok || ~isscalar(d) || ~isreal(d) || ~isfinite(d) || d < 0 || d > 1e6
                    d = inf;
                elseif lambda_theta > 0
                    d = d + lambda_theta * abs(wrapToPiLocal(poses(j,3) - poses(i,3)));
                end
                dist_matrix(i,j) = d;
            end
        end
        return;
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
