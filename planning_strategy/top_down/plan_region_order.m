function [region_order, cost_matrix, cache] = plan_region_order(scene, region_anchors, planner_cfg, cache)
%PLAN_REGION_ORDER Sort task regions by grid-planner cost between anchors.

    if nargin < 4 || isempty(cache)
        cache = initialize_path_cost_cache(scene, planner_cfg);
    end
    valid_regions = find_valid_regions(region_anchors);
    if isempty(valid_regions)
        region_order = [];
        cost_matrix = [];
        return;
    end
    [cost_matrix, cache] = build_region_cost_matrix(scene, region_anchors(valid_regions), planner_cfg, cache);
    order_nodes = nearest_neighbor_order(cost_matrix, 1, get_topdown_2opt(planner_cfg));
    order_nodes = order_nodes(order_nodes ~= 1);
    region_order = valid_regions(order_nodes - 1);
end

function valid_regions = find_valid_regions(region_anchors)
    if isstruct(region_anchors)
        valid = arrayfun(@(a) ~isempty(a.poses) && all(isfinite(a.representative_pose)), region_anchors);
        valid_regions = find(valid);
    else
        valid_regions = find(all(isfinite(region_anchors), 2));
    end
end

function tf = get_topdown_2opt(planner_cfg)
    tf = true;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'enable_region_2opt')
        tf = planner_cfg.strategy.top_down.enable_region_2opt;
    end
end
