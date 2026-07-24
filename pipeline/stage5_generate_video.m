function stage5_generate_video(scene, global_path, all_poses, visit_order, optimal_poses, task_sequence_summary, executed_tasks, robot_cfg, planner_cfg, stats)
    if nargin < 10
        stats = struct();
    end
    if runtime_flag(planner_cfg, 'disable_video', false) || runtime_flag(planner_cfg, 'disable_visualization', false)
        fprintf('\n--- Video generation skipped by runtime settings ---\n');
        return;
    end
    fprintf('\n--- 生成任务执行动画视频 ---\n');
    if isfield(stats, 'return_to_start') && stats.return_to_start
        fprintf('  检测到返航路径，动画末尾将显示返回起点阶段。\n');
    end
    fprintf('  生成中，请稍候...\n');
    animate_task_execution(scene, global_path, all_poses, visit_order, ...
                            optimal_poses, task_sequence_summary, executed_tasks, ...
                            robot_cfg, planner_cfg, stats);
end

function tf = runtime_flag(cfg, field_name, default_value)
    tf = default_value;
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, field_name)
        tf = logical(cfg.runtime.(field_name));
    end
end
