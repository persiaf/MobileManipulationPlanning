function exec_stats = estimate_execution_time(global_path, route_order, all_poses, task_sequence_summary, segment_stats, planner_cfg)
%ESTIMATE_EXECUTION_TIME Estimate simulated execution time with components.
% Units are seconds, meters and radians. The lift/replanning components are
% placeholders unless real controller logs are explicitly connected.

    if isempty(global_path)
        total_path_len_m = 0;
    else
        total_path_len_m = compute_path_length(global_path);
    end

    return_path_len_m = 0;
    for k = 1:length(segment_stats)
        if isfield(segment_stats(k), 'is_return_segment') && segment_stats(k).is_return_segment
            return_path_len_m = return_path_len_m + segment_stats(k).path_length_m;
        end
    end
    task_path_len_m = max(total_path_len_m - return_path_len_m, 0);

    time_cfg = get_time_cfg(planner_cfg);
    linear_speed = max(time_cfg.navigation_speed_mps, eps);
    angular_speed = max(time_cfg.turning_speed_radps, eps);

    [turn_angle_rad, num_turns] = estimate_path_turn_angle(global_path);

    navigation_time_s = total_path_len_m / linear_speed;
    return_time_s = return_path_len_m / linear_speed;
    turn_time_s = turn_angle_rad / angular_speed;

    dock_node_ids = route_order(route_order ~= 1);
    num_docks = length(dock_node_ids);
    docking_align_time_s = num_docks * time_cfg.docking_time_per_pose_s;

    manipulation_time_s = 0;
    arm_transition_time_s = 0;
    previous_side = [];
    num_side_switches = 0;
    num_arm_switches = 0;
    for kk = 1:num_docks
        summary_idx = dock_node_ids(kk) - 1;
        if summary_idx < 1 || summary_idx > length(task_sequence_summary)
            continue;
        end

        n_task = 0;
        if isfield(task_sequence_summary{summary_idx}, 'covered_tasks') && ~isempty(task_sequence_summary{summary_idx}.covered_tasks)
            n_task = length(task_sequence_summary{summary_idx}.covered_tasks);
        elseif isfield(task_sequence_summary{summary_idx}, 'covered_count')
            n_task = task_sequence_summary{summary_idx}.covered_count;
        end

        if planner_cfg.execution.dual_arm_parallel
            manipulation_time_s = manipulation_time_s + ceil(n_task / 2) * time_cfg.manipulation_time_per_task_s;
        else
            manipulation_time_s = manipulation_time_s + n_task * time_cfg.manipulation_time_per_task_s;
        end
        arm_transition_time_s = arm_transition_time_s + time_cfg.arm_transition_time_s;

        current_side = get_summary_field(task_sequence_summary{summary_idx}, 'side_id', []);
        if ~isempty(previous_side) && ~isempty(current_side) && current_side ~= previous_side
            num_side_switches = num_side_switches + 1;
        end
        if n_task > 1
            num_arm_switches = num_arm_switches + max(0, ceil(n_task / 2) - 1);
        end
        previous_side = current_side;
    end
    side_switch_time_s = num_side_switches * time_cfg.side_switch_time_s;
    lift_time_s = 0;
    replanning_time_s = 0;
    total_execution_time_s = navigation_time_s + turn_time_s + docking_align_time_s + ...
        manipulation_time_s + arm_transition_time_s + side_switch_time_s + ...
        lift_time_s + replanning_time_s;

    exec_stats.total_path_len_m = total_path_len_m;
    exec_stats.task_path_len_m = task_path_len_m;
    exec_stats.return_path_len_m = return_path_len_m;
    exec_stats.motion_time_s = navigation_time_s;
    exec_stats.navigation_time_s = navigation_time_s;
    exec_stats.return_time_s = return_time_s;
    exec_stats.turn_time_s = turn_time_s;
    exec_stats.turning_time_s = turn_time_s;
    exec_stats.docking_align_time_s = docking_align_time_s;
    exec_stats.docking_time_s = docking_align_time_s;
    exec_stats.manipulation_time_s = manipulation_time_s;
    exec_stats.arm_transition_time_s = arm_transition_time_s;
    exec_stats.side_switch_time_s = side_switch_time_s;
    exec_stats.lift_time_s = lift_time_s;
    exec_stats.replanning_time_s = replanning_time_s;
    exec_stats.total_execution_time_s = total_execution_time_s;
    exec_stats.total_estimated_time_s = total_execution_time_s;
    exec_stats.total_turn_angle_rad = turn_angle_rad;
    exec_stats.total_abs_yaw_change_rad = turn_angle_rad;
    exec_stats.num_turns = num_turns;
    exec_stats.num_docks = num_docks;
    exec_stats.num_side_switches = num_side_switches;
    exec_stats.num_arm_switches = num_arm_switches;
    exec_stats.time_breakdown = struct( ...
        'navigation_time_s', navigation_time_s, ...
        'docking_time_s', docking_align_time_s, ...
        'manipulation_time_s', manipulation_time_s, ...
        'arm_transition_time_s', arm_transition_time_s, ...
        'side_switch_time_s', side_switch_time_s, ...
        'turning_time_s', turn_time_s, ...
        'lift_time_s', lift_time_s, ...
        'replanning_time_s', replanning_time_s, ...
        'total_estimated_time_s', total_execution_time_s);
    components_sum = navigation_time_s + docking_align_time_s + manipulation_time_s + ...
        arm_transition_time_s + side_switch_time_s + turn_time_s + lift_time_s + replanning_time_s;
    assert(abs(total_execution_time_s - components_sum) < 1e-9, ...
        'estimate_execution_time:ComponentMismatch', ...
        'Execution-time total does not match component sum.');
end

function time_cfg = get_time_cfg(planner_cfg)
    time_cfg.navigation_speed_mps = planner_cfg.execution.nominal_linear_speed_mps;
    time_cfg.docking_time_per_pose_s = planner_cfg.execution.docking_align_time_s;
    time_cfg.manipulation_time_per_task_s = planner_cfg.execution.arm_task_time_s;
    time_cfg.arm_transition_time_s = planner_cfg.execution.arm_retract_time_s;
    time_cfg.side_switch_time_s = 0;
    time_cfg.turning_speed_radps = planner_cfg.execution.nominal_angular_speed_radps;
    time_cfg.lift_speed_mps = 0;
    time_cfg.replanning_penalty_s = 0;
    if isfield(planner_cfg, 'execution_time')
        names = fieldnames(planner_cfg.execution_time);
        for i = 1:numel(names)
            time_cfg.(names{i}) = planner_cfg.execution_time.(names{i});
        end
    end
end

function v = get_summary_field(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        v = s.(field_name);
    else
        v = default_value;
    end
end
