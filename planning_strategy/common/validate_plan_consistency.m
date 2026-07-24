function validate_plan_consistency(plan, scene)
%VALIDATE_PLAN_CONSISTENCY Ensure selected poses and task assignments agree.

    if size(plan.selected_poses, 2) ~= 3 && ~isempty(plan.selected_poses)
        error('validate_plan_consistency:InvalidPoseShape', 'selected_poses must be Nx3.');
    end
    if numel(plan.task_sequence_summary) ~= size(plan.selected_poses, 1)
        error('validate_plan_consistency:SummaryMismatch', ...
            'task_sequence_summary count must match selected poses.');
    end
    illegal = setdiff(plan.covered_tasks_predicted, 1:scene.num_tasks);
    if ~isempty(illegal)
        error('validate_plan_consistency:InvalidTaskIds', ...
            'Plan contains invalid task ids: [%s].', num2str(illegal));
    end
end
