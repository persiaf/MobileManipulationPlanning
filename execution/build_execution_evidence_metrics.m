function e = build_execution_evidence_metrics(status, total_tasks)
%BUILD_EXECUTION_EVIDENCE_METRICS Rates retain provenance and unknown evidence.
    if ~isfield(status,'evidence') || ~isstruct(status.evidence)
        status = separate_execution_status(field_or(status,'predicted_tasks',[]), ...
            field_or(status,'commanded_tasks',[]),field_or(status,'executed_tasks',[]), ...
            field_or(status,'verified_tasks',[]),struct('source','unknown_legacy'));
    end
    e = status.evidence;
    e.total_tasks = total_tasks;
    e.geometric_predicted_task_count = count_or_nan(e.geometric_predicted_tasks,e.geometry_evidence_available);
    e.simulated_executed_task_count = count_or_nan(e.simulated_executed_tasks,e.simulation_evidence_available);
    e.simulated_geometric_verified_task_count = count_or_nan(e.simulated_geometric_verified_tasks,e.simulation_evidence_available);
    e.proxy_supported_task_count = count_or_nan(e.proxy_supported_tasks,e.proxy_evidence_available);
    e.ik_checked_task_count = count_or_nan(e.ik_checked_tasks,e.ik_evidence_available);
    e.ik_feasible_task_count = count_or_nan(e.ik_feasible_tasks,e.ik_evidence_available);
    e.measured_executed_task_count = count_or_nan(e.measured_executed_tasks,e.measured_evidence_available);
    e.measured_verified_task_count = count_or_nan(e.measured_verified_tasks,e.measured_evidence_available);
    denominator = max(total_tasks,1);
    e.geometric_coverage_rate = 100*e.geometric_predicted_task_count/denominator;
    e.simulated_geometric_completion_rate = 100*e.simulated_geometric_verified_task_count/denominator;
    e.proxy_coverage_rate = 100*e.proxy_supported_task_count/denominator;
    e.ik_checked_coverage_rate = 100*e.ik_checked_task_count/denominator;
    e.ik_feasibility_rate = 100*e.ik_feasible_task_count/denominator;
    e.ik_feasibility_among_checked_rate = 100*e.ik_feasible_task_count/max(e.ik_checked_task_count,1);
    e.measured_execution_rate = 100*e.measured_executed_task_count/denominator;
    e.actual_completion_rate = 100*e.measured_verified_task_count/denominator;
end

function n = count_or_nan(ids,available)
    if available, n = numel(unique(ids)); else, n = NaN; end
end

function value = field_or(s,name,fallback)
    if isstruct(s) && isfield(s,name), value = s.(name); else, value = fallback; end
end
