function [predicted_tasks, commanded_tasks] = get_planned_tasks_for_dock(task_sequence_summary, pose_info_list, dock_index)
%GET_PLANNED_TASKS_FOR_DOCK Return only the tasks assigned during stage two.

    predicted_tasks = [];
    commanded_tasks = [];
    if dock_index < 1 || dock_index > numel(task_sequence_summary)
        return;
    end
    summary = task_sequence_summary{dock_index};
    if isstruct(summary) && isfield(summary, 'covered_tasks')
        predicted_tasks = unique(summary.covered_tasks(:)', 'stable');
    end
    if dock_index <= numel(pose_info_list) && isstruct(pose_info_list{dock_index}) && ...
            isfield(pose_info_list{dock_index}, 'covered_tasks')
        commanded_tasks = unique(pose_info_list{dock_index}.covered_tasks(:)', 'stable');
    else
        commanded_tasks = predicted_tasks;
    end
    commanded_tasks = intersect(commanded_tasks, predicted_tasks, 'stable');
end
