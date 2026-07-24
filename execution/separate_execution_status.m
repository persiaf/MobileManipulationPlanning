function status = separate_execution_status(predicted_tasks, commanded_tasks, executed_tasks, verified_tasks)
%SEPARATE_EXECUTION_STATUS Normalize the four execution-state levels.

    status.predicted_tasks = unique(predicted_tasks(:)', 'stable');
    status.commanded_tasks = unique(commanded_tasks(:)', 'stable');
    status.executed_tasks = unique(executed_tasks(:)', 'stable');
    status.verified_tasks = unique(verified_tasks(:)', 'stable');
end
