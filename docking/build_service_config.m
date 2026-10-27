function service_config = build_service_config(id, pose_info, summary_entry, real_cfg)
%BUILD_SERVICE_CONFIG Convert a selected docking pose into an execution contract.

    if nargin < 4 || isempty(real_cfg)
        real_cfg = real_platform_config();
    end
    task_ids = unique(summary_entry.covered_tasks(:)', 'stable');
    assignments = cell(1, numel(task_ids));
    for k = 1:numel(task_ids)
        assignments{k} = 'either';
        if isfield(pose_info, 'arm_assign_map') && isa(pose_info.arm_assign_map, 'containers.Map') && ...
                isKey(pose_info.arm_assign_map, task_ids(k))
            assignments{k} = pose_info.arm_assign_map(task_ids(k));
        end
    end

    service_config = struct();
    service_config.id = id;
    service_config.pose2d = reshape(summary_entry.pose, 1, 3);
    service_config.lift_height_m = real_cfg.lift.default_height_m;
    service_config.table_id = summary_entry.table_id;
    service_config.task_ids = task_ids;
    service_config.arm_assignments = assignments;
    service_config.task_actions = repmat(struct('task_id',0,'assigned_arm','either', ...
        'reachability',true,'inter_task_transition_required',false),1,numel(task_ids));
    for k = 1:numel(task_ids)
        service_config.task_actions(k).task_id = task_ids(k);
        service_config.task_actions(k).assigned_arm = assignments{k};
        service_config.task_actions(k).inter_task_transition_required = k > 1;
    end
    service_config.inter_task_transition_required = numel(task_ids) > 1;
    service_config.arm_mode = infer_arm_mode(assignments, real_cfg.default_arm_mode);
    service_config.predicted_covered_task_count = safe_field(pose_info, 'covered_count', numel(task_ids));
    service_config.mean_reachability_score = safe_field(summary_entry, 'mean_reachability_score', NaN);
    service_config.minimum_reachability_score = safe_field(summary_entry, 'minimum_reachability_score', NaN);
    service_config.robust_success_fraction = safe_field(summary_entry, 'robust_success_fraction', NaN);
    service_config.position_tolerance_m = real_cfg.position_tolerance_m;
    service_config.yaw_tolerance_rad = real_cfg.yaw_tolerance_rad;
    service_config.policy_id = real_cfg.default_policy_id;
    service_config.expected_execution_time_s = NaN;
end

function mode = infer_arm_mode(assignments, default_mode)
    used = unique(assignments);
    used(strcmp(used, 'none')) = [];
    if isempty(used)
        mode = default_mode;
    elseif numel(used) == 1
        mode = used{1};
    elseif all(ismember(used, {'left', 'right'}))
        mode = 'dual_independent';
    else
        mode = default_mode;
    end
end

function value = safe_field(s, name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    end
end
