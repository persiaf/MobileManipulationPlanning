function record = build_result_record(scene, optimal_poses, executed_tasks, global_path, total_path_len, stats, planner_cfg, pose_info_list, benchmark_results)
%BUILD_RESULT_RECORD 将一次实验输出统一整理为 record 结构体。
%
%   record = build_result_record(scene, optimal_poses, executed_tasks, ...
%                                global_path, total_path_len, stats, ...
%                                planner_cfg, pose_info_list, benchmark_results)
%
%   输入:
%       scene              - stage1_build_scene 返回的场景结构体
%       optimal_poses      - stage2 返回的最优停靠位姿 [Nx3]
%       executed_tasks     - stage3 执行的已完成任务ID列表
%       global_path        - stage3 拼接的全局路径 [Mx3]
%       total_path_len     - stage3 输出的全局路径总长度 (m)
%       stats              - stage3 输出的统计结构体
%       planner_cfg        - planner_config() 返回的规划器配置
%       pose_info_list     - stage2 任务序列 cell 数组 (可选, 传 {} 亦可)
%       benchmark_results  - stage4_benchmark_all 返回的 benchmark 结构体 (可选, 传 [] 亦可)
%
%   输出:
%       record - 结构体，包含以下字段:
%           .scene_id          - 场景标识字符串 (如 "N16_S2026")
%           .seed              - 随机种子
%           .num_tables        - 桌子数量
%           .num_tasks         - 任务点总数
%           .num_docking_poses - 停靠位姿数量
%           .completed_tasks   - 实际完成任务数
%           .completion_rate   - 任务完成率 (%)
%           .uncovered_tasks   - 未覆盖任务数
%           .task_path_length  - 任务段路径长度 (m)
%           .return_path_length- 返回起点段路径长度 (m)
%           .total_path_length - 全局路径总长度 (m)
%           .num_turns         - 路径转弯次数
%           .num_path_points   - 全局路径采样点数
%           .docking_cost_total- 所有停靠位姿代价总和
%           .docking_cost_mean - 停靠位姿代价均值
%           .planning_time_total- 总规划时间 (s)
%           .time_docking      - 停靠规划耗时 (s)
%           .time_global       - 全局路径规划耗时 (s)
%           .execution_time_est- 预估总执行时间 (s)
%           .algorithm_name    - 使用的全局规划器名称
%           .ablation_mode     - 消融实验模式 ('full' / 自定义字符串)
%           .fallback_count    - 路径碰撞回退次数
%           .success           - 本次实验是否成功 (logical)
%
%   示例 (主脚本末尾调用):
%       record = build_result_record(scene, optimal_poses, executed_tasks, ...
%                                    global_path, total_path_len, stats, ...
%                                    planner_cfg, pose_info_list, benchmark_results);
%
%   示例 (独立实验脚本):
%       % 先运行 main 获取工作区变量, 然后:
%       record = build_result_record(scene, optimal_poses, executed_tasks, ...
%                                    global_path, total_path_len, stats, ...
%                                    planner_cfg, {}, []);

    %% ---------- 安全默认值 ----------
    if nargin < 9 || isempty(benchmark_results)
        benchmark_results = [];
    end
    if nargin < 8 || isempty(pose_info_list)
        pose_info_list = {};
    end

    %% ---------- 1. 场景标识 ----------
    num_tables_val = safe_field(safe_field(scene, 'scene_cfg', struct()), 'num_tables', 0);
    seed_val       = safe_field(safe_field(scene, 'scene_cfg', struct()), 'random_seed', 'unknown');
    if isnumeric(seed_val)
        seed_str = num2str(seed_val);
    else
        seed_str = seed_val;
    end
    record.scene_id = sprintf('N%d_S%s', num_tables_val, seed_str);
    record.seed     = seed_val;

    %% ---------- 2. 场景规模 ----------
    record.num_tables = num_tables_val;
    record.num_tasks  = safe_field(scene, 'num_tasks', 0);
    record.num_docking_poses = size(optimal_poses, 1);

    %% ---------- 3. 任务完成情况 ----------
    record.completed_tasks = length(unique(executed_tasks));
    record.completion_rate = safe_div(100 * record.completed_tasks, max(record.num_tasks, 1));
    record.uncovered_tasks = max(record.num_tasks - record.completed_tasks, 0);
    record.success         = record.completed_tasks > 0;
    record.predicted_completion_rate = safe_field(stats, 'predicted_coverage_rate', record.completion_rate);
    record.verified_completion_rate = safe_field(stats, 'verified_completion_rate', record.completion_rate);

    %% ---------- 4. 路径长度 (优先从 exec_stats 取，否则用 total_path_len) ----------
    exec_stats = safe_field(stats, 'exec_stats', struct());
    record.task_path_length   = safe_field(exec_stats, 'task_path_len_m', 0);
    record.return_path_length = safe_field(exec_stats, 'return_path_len_m', 0);

    % 如果 exec_stats 中没有分段长度，则用 total_path_len 作为任务路径
    if record.task_path_length == 0 && record.return_path_length == 0
        record.task_path_length = total_path_len;
        record.return_path_length = 0;
    end
    record.total_path_length = total_path_len;

    %% ---------- 5. 路径形态 ----------
    record.num_turns       = safe_field(exec_stats, 'num_turns', 0);
    record.num_path_points = size(global_path, 1);

    %% ---------- 6. 停靠代价 ----------
    [record.docking_cost_total, record.docking_cost_mean] = compute_docking_cost(pose_info_list);

    %% ---------- 7. 时间统计 ----------
    timing = safe_field(stats, 'timing', struct());
    record.planning_time_total = safe_field(timing, 'total_planning_time_s', 0);
    record.time_docking        = safe_field(timing, 'docking_time_s', 0);
    record.time_global         = safe_field(timing, 'global_path_time_s', 0);
    record.execution_time_est  = safe_field(exec_stats, 'total_execution_time_s', 0);
    record.estimated_execution_time_s = record.execution_time_est;
    record.estimated_total_time_s = record.planning_time_total + record.execution_time_est;
    record.candidate_generation_time_s = safe_field(timing, 'candidate_generation_time_s', 0);
    record.region_order_time_s = safe_field(timing, 'region_order_time_s', 0);
    record.local_selection_time_s = safe_field(timing, 'local_selection_time_s', 0);
    record.route_refinement_time_s = safe_field(timing, 'route_refinement_time_s', 0);
    record.path_cache_hits = safe_field(timing, 'path_cache_hits', 0);
    record.path_cache_misses = safe_field(timing, 'path_cache_misses', 0);

    %% ---------- 8. 算法与模式 ----------
    record.algorithm_name = safe_field(planner_cfg, 'global_planner', 'astar');
    record.strategy_name = safe_field(stats, 'strategy_name', ...
        safe_field(safe_field(planner_cfg, 'strategy', struct()), 'name', 'bottom_up'));
    record.route_order_source = safe_field(stats, 'route_order_source', '');
    record.region_order = [];
    if isfield(stats, 'plan') && isstruct(stats.plan) && isfield(stats.plan, 'region_order')
        record.region_order = stats.plan.region_order;
    end
    record.coverage_repair_count = safe_field(stats, 'coverage_repair_count', 0);
    record.strategy_objective_value = safe_field(stats, 'strategy_objective_value', 0);
    record.ablation_mode  = safe_field(safe_field(planner_cfg, 'docking', struct()), 'ablation_mode', 'full');
    record.method         = record.ablation_mode;
    record.difficulty      = safe_field(safe_field(safe_field(scene, ...
                                'scene_cfg', struct()), 'task', struct()), ...
                                'difficulty', 'normal');
    record.avg_candidate_cost = record.docking_cost_mean;

    %% ---------- 9. 回退统计 ----------
    record.fallback_count = compute_fallback_count(benchmark_results, record.algorithm_name);

    %% ---------- 10. 批量实验默认字段 ----------
    % 这些字段在主程序中保持默认值，在随机批量实验中会被外部覆盖。
    record.scale = '';
    record.trial_id = 0;
    record.trial_idx = 0;
    record.trial_seed = seed_val;
    record.status = 'OK';
    record.error_message = '';

end

%% ======================== 内部辅助函数 ==========================

function v = safe_field(s, field_name, default_val)
%SAFE_FIELD 安全读取结构体字段，缺失时返回 default_val
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_val;
    end
end

function r = safe_div(num, den)
%SAFE_DIV 安全除法，分母为 0 时返回 0
    if den == 0
        r = 0;
    else
        r = num / den;
    end
end

function [cost_total, cost_mean] = compute_docking_cost(pose_info_list)
%COMPUTE_DOCKING_COST 从 pose_info_list 汇总停靠代价
    cost_total = 0;
    cost_mean  = 0;
    if isempty(pose_info_list)
        return;
    end

    cost_sum = 0;
    valid_count = 0;
    for i = 1:length(pose_info_list)
        info = pose_info_list{i};
        if isempty(info)
            continue;
        end
        % 优先取 cost_breakdown.total，其次取直接字段 total_cost
        if isfield(info, 'cost_breakdown') && ~isempty(info.cost_breakdown) ...
                && isfield(info.cost_breakdown, 'total')
            cost_sum = cost_sum + info.cost_breakdown.total;
            valid_count = valid_count + 1;
        elseif isfield(info, 'total_cost')
            cost_sum = cost_sum + info.total_cost;
            valid_count = valid_count + 1;
        end
    end

    cost_total = cost_sum;
    if valid_count > 0
        cost_mean = cost_sum / valid_count;
    else
        cost_mean = 0;
    end
end

function fb = compute_fallback_count(benchmark_results, algo_name)
%COMPUTE_FALLBACK_COUNT 从 benchmark_results 中提取当前算法的回退次数
    fb = 0;
    if isempty(benchmark_results) || ~isstruct(benchmark_results)
        return;
    end
    for i = 1:length(benchmark_results)
        if isfield(benchmark_results(i), 'algorithm') ...
                && strcmpi(benchmark_results(i).algorithm, algo_name)
            fb = safe_field(benchmark_results(i), 'fallback_count', 0);
            return;
        end
    end
    % 若算法不在 benchmark_results 中（如仅跑主流程），回退计数为 0
end
