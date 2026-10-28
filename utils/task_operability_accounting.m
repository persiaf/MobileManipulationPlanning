function metrics = task_operability_accounting(task_ids, covered_tasks, model_infeasible_tasks)
%TASK_OPERABILITY_ACCOUNTING Raw vs model-feasible task coverage metrics.
%
% This diagnostic does not change planner behavior. It only separates raw
% task coverage from coverage over tasks that are feasible under the current
% reachability/operability audit.

    task_ids = unique(task_ids(:)', 'stable');
    covered_tasks = unique(covered_tasks(:)', 'stable');
    model_infeasible_tasks = unique(model_infeasible_tasks(:)', 'stable');
    model_feasible_tasks = setdiff(task_ids, model_infeasible_tasks, 'stable');
    covered_model_feasible = intersect(covered_tasks, model_feasible_tasks, 'stable');

    metrics = struct();
    metrics.infeasibility_provenance = 'caller_supplied_unverified_set';
    metrics.independent_feasibility_evidence_available = false;
    metrics.independent_feasible_coverage_percent = NaN;
    metrics.total_task_count = numel(task_ids);
    metrics.raw_covered_tasks = numel(intersect(covered_tasks, task_ids, 'stable'));
    metrics.raw_task_coverage_percent = ...
        100 * metrics.raw_covered_tasks / max(metrics.total_task_count, 1);
    metrics.model_infeasible_task_count = numel(intersect(model_infeasible_tasks, task_ids, 'stable'));
    metrics.model_feasible_task_count = numel(model_feasible_tasks);
    metrics.covered_model_feasible_tasks = numel(covered_model_feasible);
    metrics.feasible_task_coverage_percent = ...
        100 * metrics.covered_model_feasible_tasks / max(metrics.model_feasible_task_count, 1);
end
