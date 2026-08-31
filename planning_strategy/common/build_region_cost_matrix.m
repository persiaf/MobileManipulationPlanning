function [cost_matrix, cache, transition] = build_region_cost_matrix(scene, region_anchors, planner_cfg, cache)
%BUILD_REGION_COST_MATRIX Compute path costs over start plus region anchors.
% For anchor shortlists, region-to-region cost is the minimum feasible path
% cost over all candidate anchor pairs.

    if nargin < 4 || isempty(cache)
        cache = initialize_path_cost_cache(scene, planner_cfg);
    end
    if isstruct(region_anchors)
        anchor_sets = [{scene.start_pose}; arrayfun(@(a) a.poses, region_anchors(:), 'UniformOutput', false)];
        candidate_sets = [{0}; arrayfun(@(a) a.candidate_indices, region_anchors(:), 'UniformOutput', false)];
    else
        anchor_sets = mat2cell([scene.start_pose; region_anchors], ones(size(region_anchors, 1) + 1, 1), 3);
        candidate_sets = num2cell(zeros(numel(anchor_sets), 1));
    end
    n = numel(anchor_sets);
    cost_matrix = inf(n, n);
    transition = repmat(struct('best_from_candidate_index', 0, ...
        'best_to_candidate_index', 0, 'best_path_cost', inf), n, n);
    for i = 1:n
        cost_matrix(i, i) = 0;
    end
    for i = 1:n
        if isempty(anchor_sets{i}), continue; end
        for j = 1:n
            if i == j || isempty(anchor_sets{j}), continue; end
            [best_cost, best_from, best_to, cache] = min_anchor_pair_cost( ...
                anchor_sets{i}, anchor_sets{j}, candidate_sets{i}, candidate_sets{j}, ...
                scene, planner_cfg, cache);
            cost_matrix(i, j) = best_cost;
            transition(i, j).best_from_candidate_index = best_from;
            transition(i, j).best_to_candidate_index = best_to;
            transition(i, j).best_path_cost = best_cost;
        end
    end
end

function [best_cost, best_from, best_to, cache] = min_anchor_pair_cost(from_poses, to_poses, from_ids, to_ids, scene, planner_cfg, cache)
    best_cost = inf;
    best_from = 0;
    best_to = 0;
    pairs = nearest_anchor_pairs(from_poses, to_poses, max_pair_queries(planner_cfg));
    for k = 1:size(pairs, 1)
        pi = pairs(k, 1);
        qi = pairs(k, 2);
        [c, cache] = get_cached_path_cost(cache, from_poses(pi, :), to_poses(qi, :), scene, planner_cfg);
        if c < best_cost
            best_cost = c;
            best_from = safe_id(from_ids, pi);
            best_to = safe_id(to_ids, qi);
        end
    end
end

function pairs = nearest_anchor_pairs(from_poses, to_poses, max_queries)
    rows = zeros(size(from_poses, 1) * size(to_poses, 1), 3);
    k = 0;
    for pi = 1:size(from_poses, 1)
        for qi = 1:size(to_poses, 1)
            k = k + 1;
            rows(k, :) = [pi, qi, norm(from_poses(pi, 1:2) - to_poses(qi, 1:2))];
        end
    end
    rows = sortrows(rows, 3);
    rows = rows(1:min(size(rows, 1), max_queries), :);
    pairs = rows(:, 1:2);
end

function n = max_pair_queries(planner_cfg)
    n = 12;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'max_anchor_pair_queries')
        n = planner_cfg.strategy.top_down.max_anchor_pair_queries;
    end
end

function id = safe_id(ids, idx)
    if isempty(ids) || idx > numel(ids)
        id = 0;
    else
        id = ids(idx);
    end
end
