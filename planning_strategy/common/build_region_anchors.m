function region_anchors = build_region_anchors(candidate_library, planner_cfg)
%BUILD_REGION_ANCHORS Build feasible anchor shortlists for each region.
% Each region keeps high-coverage, high-reachability, low-cost and side-
% diverse representatives. Unique task coverers are preserved when possible.

    if nargin < 2
        planner_cfg = struct();
    end
    n = numel(candidate_library.regions);
    empty_anchor = struct('region_id', 0, 'candidate_indices', [], 'poses', [], ...
        'coverage_counts', [], 'mean_reachabilities', [], 'local_costs', [], ...
        'representative_pose', [nan, nan, nan]);
    region_anchors = repmat(empty_anchor, n, 1);
    for ri = 1:n
        candidates = candidate_library.regions(ri).candidates;
        region_anchors(ri).region_id = ri;
        if isempty(candidates)
            continue;
        end
        coverage = arrayfun(@(c) numel(c.covered_tasks), candidates);
        reach = arrayfun(@mean_candidate_reach, candidates);
        costs = [candidates.total_cost];
        score = coverage(:) * 1e6 + reach(:) * 1e3 - costs(:);
        [~, order] = sort(score, 'descend');

        keep = order(1:min(max_anchors(planner_cfg), numel(order)));
        if preserve_side_diversity(planner_cfg)
            keep = unique([keep(:); side_representatives(candidates, score)], 'stable');
        end
        if preserve_unique_task_cover(planner_cfg)
            keep = unique([keep(:); unique_task_coverers(candidates, score)], 'stable');
        end
        keep = keep(1:min(numel(keep), max(max_anchors(planner_cfg), numel(unique_task_coverers(candidates, score)))));

        region_anchors(ri).candidate_indices = reshape(keep, 1, []);
        region_anchors(ri).poses = reshape([candidates(keep).pose], 3, [])';
        region_anchors(ri).coverage_counts = coverage(keep);
        region_anchors(ri).mean_reachabilities = reach(keep);
        region_anchors(ri).local_costs = costs(keep);
        region_anchors(ri).representative_pose = candidates(keep(1)).pose;
    end
end

function r = mean_candidate_reach(candidate)
    if isfield(candidate, 'reach_scores') && ~isempty(candidate.reach_scores)
        r = mean(candidate.reach_scores(:, 3));
    else
        r = 0;
    end
end

function keep = side_representatives(candidates, score)
    keep = [];
    if ~isfield(candidates, 'side_id')
        return;
    end
    sides = unique([candidates.side_id]);
    for si = 1:numel(sides)
        idx = find([candidates.side_id] == sides(si));
        [~, best_local] = max(score(idx));
        keep(end + 1) = idx(best_local); %#ok<AGROW>
    end
end

function keep = unique_task_coverers(candidates, score)
    all_tasks = unique([candidates.covered_tasks]);
    keep = [];
    for ti = 1:numel(all_tasks)
        tid = all_tasks(ti);
        coverers = find(arrayfun(@(c) ismember(tid, c.covered_tasks), candidates));
        if numel(coverers) == 1
            keep(end + 1) = coverers; %#ok<AGROW>
        elseif ~isempty(coverers)
            [~, best_local] = max(score(coverers));
            keep(end + 1) = coverers(best_local); %#ok<AGROW>
        end
    end
end

function n = max_anchors(planner_cfg)
    n = 8;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'max_anchors_per_region')
        n = planner_cfg.strategy.top_down.max_anchors_per_region;
    end
end

function tf = preserve_side_diversity(planner_cfg)
    tf = true;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'preserve_side_diversity')
        tf = logical(planner_cfg.strategy.top_down.preserve_side_diversity);
    end
end

function tf = preserve_unique_task_cover(planner_cfg)
    tf = true;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'preserve_unique_task_cover')
        tf = logical(planner_cfg.strategy.top_down.preserve_unique_task_cover);
    end
end
