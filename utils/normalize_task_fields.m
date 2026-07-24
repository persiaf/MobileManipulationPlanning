function tasks = normalize_task_fields(tasks)
%NORMALIZE_TASK_FIELDS Add optional real-platform task fields compatibly.

    defaults = struct('object_pose_map', [], 'task_type', '', ...
        'left_grasp_pose', [], 'right_grasp_pose', [], ...
        'destination_pose', [], 'object_size', [], ...
        'required_height_range', [], 'policy_id', '');
    names = fieldnames(defaults);
    for i = 1:numel(tasks)
        for k = 1:numel(names)
            name = names{k};
            if ~isfield(tasks, name) || isempty(tasks(i).(name))
                tasks(i).(name) = defaults.(name);
            end
        end
    end
end
