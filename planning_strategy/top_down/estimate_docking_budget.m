function budget = estimate_docking_budget(candidates, region, planner_cfg)
%ESTIMATE_DOCKING_BUDGET Estimate the initial docking count for one region.

    if isempty(candidates)
        budget = 0;
        return;
    end
    max_cov = 0;
    for i = 1:numel(candidates)
        max_cov = max(max_cov, numel(intersect(candidates(i).covered_tasks, region.task_ids)));
    end
    lower_bound = ceil(numel(region.task_ids) / max(max_cov, 1));
    max_docks = 3;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'max_docks_per_region')
        max_docks = planner_cfg.strategy.top_down.max_docks_per_region;
    end
    budget = min(max(lower_bound, 1), max_docks);
end
