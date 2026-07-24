function experiment_stats = build_experiment_stats(scene, completed_task_ids, optimal_poses, global_path, exec_stats, timing, segment_stats, task_sequence_summary, execution_status)
    total_tasks = scene.num_tasks;
    if nargin < 9 || isempty(execution_status)
        execution_status = separate_execution_status(completed_task_ids, completed_task_ids, ...
            completed_task_ids, completed_task_ids);
    end
    completed_tasks = length(unique(execution_status.verified_tasks));
    uncompleted_tasks = max(total_tasks - completed_tasks, 0);
    num_docking_poses = size(optimal_poses, 1);

    if num_docking_poses > 0
        avg_tasks_per_dock = completed_tasks / num_docking_poses;
    else
        avg_tasks_per_dock = 0;
    end

    experiment_stats.task.total_tasks = total_tasks;
    experiment_stats.task.completed_tasks = completed_tasks;
    experiment_stats.task.uncompleted_tasks = uncompleted_tasks;
    experiment_stats.task.completion_rate = 100 * completed_tasks / max(total_tasks, 1);
    experiment_stats.task.num_docking_poses = num_docking_poses;
    experiment_stats.task.avg_tasks_per_dock = avg_tasks_per_dock;
    experiment_stats.task.predicted_tasks = numel(execution_status.predicted_tasks);
    experiment_stats.task.commanded_tasks = numel(execution_status.commanded_tasks);
    experiment_stats.task.executed_tasks = numel(execution_status.executed_tasks);
    experiment_stats.task.verified_tasks = numel(execution_status.verified_tasks);
    experiment_stats.task.predicted_coverage_rate = 100 * experiment_stats.task.predicted_tasks / max(total_tasks, 1);
    experiment_stats.task.commanded_task_rate = 100 * experiment_stats.task.commanded_tasks / max(total_tasks, 1);
    experiment_stats.task.verified_completion_rate = 100 * experiment_stats.task.verified_tasks / max(total_tasks, 1);
    experiment_stats.task.predicted_completion_rate = experiment_stats.task.predicted_coverage_rate;

    experiment_stats.path.total_path_len_m = exec_stats.total_path_len_m;
    experiment_stats.path.task_path_len_m = exec_stats.task_path_len_m;
    experiment_stats.path.return_path_len_m = exec_stats.return_path_len_m;
    experiment_stats.path.num_path_points = size(global_path, 1);
    experiment_stats.path.num_turns = exec_stats.num_turns;

    experiment_stats.planning.scene_time_s = getfield_default(timing, 'scene_time_s', 0);
    experiment_stats.planning.docking_time_s = getfield_default(timing, 'docking_time_s', 0);
    experiment_stats.planning.distance_matrix_time_s = getfield_default(timing, 'distance_matrix_time_s', 0);
    experiment_stats.planning.route_order_time_s = getfield_default(timing, 'route_order_time_s', 0);
    experiment_stats.planning.global_path_time_s = getfield_default(timing, 'global_path_time_s', 0);
    experiment_stats.planning.stage3_time_s = getfield_default(timing, 'stage3_time_s', ...
        experiment_stats.planning.distance_matrix_time_s + ...
        experiment_stats.planning.route_order_time_s + ...
        experiment_stats.planning.global_path_time_s);
    experiment_stats.planning.total_planning_time_s = getfield_default(timing, 'total_planning_time_s', 0);
    experiment_stats.planning.candidate_generation_time_s = getfield_default(timing, 'candidate_generation_time_s', 0);
    experiment_stats.planning.strategy_total_time_s = getfield_default(timing, 'strategy_total_time_s', 0);
    experiment_stats.planning.local_selection_time_s = getfield_default(timing, 'local_selection_time_s', 0);
    experiment_stats.planning.route_refinement_time_s = getfield_default(timing, 'route_refinement_time_s', 0);

    experiment_stats.execution.motion_time_s = exec_stats.motion_time_s;
    experiment_stats.execution.return_time_s = exec_stats.return_time_s;
    experiment_stats.execution.turn_time_s = exec_stats.turn_time_s;
    experiment_stats.execution.docking_align_time_s = exec_stats.docking_align_time_s;
    experiment_stats.execution.manipulation_time_s = exec_stats.manipulation_time_s;
    experiment_stats.execution.total_execution_time_s = exec_stats.total_execution_time_s;

    experiment_stats.total.estimated_total_time_s = ...
        experiment_stats.planning.total_planning_time_s + exec_stats.total_execution_time_s;

    experiment_stats.segment_stats = segment_stats;
    experiment_stats.task_sequence_summary = task_sequence_summary;
    experiment_stats.strategy_name = getfield_default(timing, 'strategy_name', 'bottom_up');
    experiment_stats.region_order = getfield_default(timing, 'region_order', []);
    experiment_stats.route_order_source = getfield_default(timing, 'route_order_source', '');
    experiment_stats.coverage_repair_count = getfield_default(timing, 'coverage_repair_count', 0);
    experiment_stats.strategy_objective_value = getfield_default(timing, 'strategy_objective_value', 0);
    experiment_stats.path_cache_hits = getfield_default(timing, 'path_cache_hits', 0);
    experiment_stats.path_cache_misses = getfield_default(timing, 'path_cache_misses', 0);
    experiment_stats.estimated_execution_time_s = exec_stats.total_execution_time_s;
    experiment_stats.estimated_total_time_s = experiment_stats.total.estimated_total_time_s;
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
