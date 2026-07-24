function print_experiment_stats(experiment_stats)
    fprintf('\n========== 实验统计结果 ==========\n');
    if isfield(experiment_stats, 'strategy_name')
        fprintf('Strategy: %s\n', experiment_stats.strategy_name);
    end
    fprintf('任务总数: %d\n', experiment_stats.task.total_tasks);
    fprintf('完成任务: %d\n', experiment_stats.task.completed_tasks);
    fprintf('完成率: %.1f%%\n', experiment_stats.task.completion_rate);
    if isfield(experiment_stats.task, 'predicted_coverage_rate')
        fprintf('预测覆盖率: %.1f%%\n', experiment_stats.task.predicted_coverage_rate);
        fprintf('指令任务率: %.1f%%\n', experiment_stats.task.commanded_task_rate);
        fprintf('验证完成率: %.1f%%\n', experiment_stats.task.verified_completion_rate);
    end
    fprintf('停靠次数: %d\n', experiment_stats.task.num_docking_poses);
    fprintf('平均每停靠点任务数: %.2f\n', experiment_stats.task.avg_tasks_per_dock);

    fprintf('\n路径统计:\n');
    fprintf('  任务路径长度: %.2f m\n', experiment_stats.path.task_path_len_m);
    fprintf('  返回起点路径长度: %.2f m\n', experiment_stats.path.return_path_len_m);
    fprintf('  总路径长度: %.2f m\n', experiment_stats.path.total_path_len_m);
    fprintf('  路径点数: %d\n', experiment_stats.path.num_path_points);
    fprintf('  转折次数: %d\n', experiment_stats.path.num_turns);

    fprintf('\n规划耗时:\n');
    fprintf('  场景构建时间: %.3f s\n', experiment_stats.planning.scene_time_s);
    fprintf('  停靠位姿规划时间: %.3f s\n', experiment_stats.planning.docking_time_s);
    fprintf('  Candidate generation time: %.3f s\n', getfield_default(experiment_stats.planning, 'candidate_generation_time_s', 0));
    fprintf('  Strategy total time: %.3f s\n', getfield_default(experiment_stats.planning, 'strategy_total_time_s', 0));
    fprintf('  距离矩阵计算时间 distance_matrix_time_s: %.3f s\n', getfield_default(experiment_stats.planning, 'distance_matrix_time_s', 0));
    fprintf('  访问顺序规划时间: %.3f s\n', experiment_stats.planning.route_order_time_s);
    fprintf('  全局路径规划时间: %.3f s\n', experiment_stats.planning.global_path_time_s);
    fprintf('  总规划时间: %.3f s\n', experiment_stats.planning.total_planning_time_s);

    fprintf('\n执行估计时间:\n');
    fprintf('  底盘运动时间: %.1f s\n', experiment_stats.execution.motion_time_s);
    fprintf('  转向时间: %.1f s\n', experiment_stats.execution.turn_time_s);
    fprintf('  停靠对准时间: %.1f s\n', experiment_stats.execution.docking_align_time_s);
    fprintf('  机械臂任务时间: %.1f s\n', experiment_stats.execution.manipulation_time_s);
    fprintf('  返回起点时间: %.1f s\n', experiment_stats.execution.return_time_s);
    fprintf('  执行估计总时间: %.1f s\n', experiment_stats.execution.total_execution_time_s);

    fprintf('\n任务总时间估计: %.1f s\n', experiment_stats.total.estimated_total_time_s);
    fprintf('==================================\n');
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
