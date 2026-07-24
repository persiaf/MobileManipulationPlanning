function record = create_empty_result_record()
%CREATE_EMPTY_RESULT_RECORD 创建批量实验统一 record 模板。
%
%   该模板用于保证成功 trial 和失败 trial 的结构体字段完全一致，
%   避免 MATLAB 在 all_records(end+1)=record 时出现
%   “在不同结构体之间进行下标赋值”的错误。

    record = struct();

    % 场景与实验标识
    record.scene_id = '';
    record.seed = [];
    record.scale = '';
    record.trial_id = 0;
    record.trial_idx = 0;
    record.trial_seed = [];
    record.status = 'PENDING';
    record.error_message = '';

    % 场景规模
    record.num_tables = 0;
    record.num_tasks = 0;
    record.num_docking_poses = 0;

    % 任务完成情况
    record.completed_tasks = 0;
    record.completion_rate = 0;
    record.uncovered_tasks = 0;
    record.success = false;

    % 路径长度
    record.task_path_length = 0;
    record.return_path_length = 0;
    record.total_path_length = 0;

    % 路径形态
    record.num_turns = 0;
    record.num_path_points = 0;

    % 停靠代价
    record.docking_cost_total = 0;
    record.docking_cost_mean = 0;

    % 时间统计
    record.planning_time_total = 0;
    record.time_docking = 0;
    record.time_global = 0;
    record.execution_time_est = 0;

    % 算法与模式
    record.algorithm_name = '';
    record.method = 'full';
    record.ablation_mode = 'full';
    record.difficulty = 'normal';
    record.lnn_enable = false;
    record.lnn_alpha = 0;
    record.lnn_label_strategy = 'imitation';
    record.avg_candidate_cost = 0;
    record.fallback_count = 0;
end
