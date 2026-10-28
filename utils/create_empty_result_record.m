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
    record.completed_tasks = NaN;
    record.completion_rate = NaN;
    record.uncovered_tasks = NaN;
    record.success = false;
    record.metric_schema_version = '2.0';
    record.evidence_source = 'unknown_legacy';
    record.completion_semantics = 'unknown_legacy';
    record.success_scope = 'simulated_geometric_mission';
    record.geometry_evidence_available = false;
    record.simulation_evidence_available = false;
    record.proxy_evidence_available = false;
    record.ik_evidence_available = false;
    record.measured_evidence_available = false;
    record.predicted_completion_rate = NaN;
    record.verified_completion_rate = NaN;
    record.geometric_coverage_rate = NaN;
    record.simulated_geometric_completion_rate = NaN;
    record.proxy_coverage_rate = NaN;
    record.ik_checked_coverage_rate = NaN;
    record.ik_feasibility_rate = NaN;
    record.actual_completion_rate = NaN;
    record.geometric_predicted_task_count = NaN;
    record.simulated_geometric_verified_task_count = NaN;
    record.ik_checked_task_count = NaN;
    record.ik_feasible_task_count = NaN;
    record.measured_verified_task_count = NaN;
    record.executed_task_count = NaN;
    record.path_validation_available = false;
    record.path_safe = NaN;

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
    record.planning_time_total = NaN;
    record.time_docking = NaN;
    record.time_global = NaN;
    record.execution_time_est = NaN;
    record.execution_time_source = 'unavailable';
    record.actual_execution_time_s = NaN;
    record.planning_time_available = false;
    record.timing_scope = 'unspecified';
    record.timing_source = 'unspecified';
    time_fields = {'estimated_execution_time_s','estimated_total_time_s', ...
        'navigation_time_s','task_navigation_time_s','return_navigation_time_s', ...
        'docking_time_s','manipulation_time_s','arm_transition_time_s','side_switch_time_s', ...
        'turning_time_s','lift_time_s','replanning_time_s','mission_total_time_s', ...
        'candidate_generation_time_s','region_order_time_s','local_selection_time_s', ...
        'route_refinement_time_s'};
    for k = 1:numel(time_fields), record.(time_fields{k}) = NaN; end

    % 算法与模式
    record.algorithm_name = '';
    record.method = 'full';
    record.ablation_mode = 'full';
    record.difficulty = 'normal';
    record.avg_candidate_cost = 0;
    record.fallback_count = 0;
end
