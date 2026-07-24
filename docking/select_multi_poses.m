function [selected_poses, pose_info] = select_multi_poses(candidates, region, scene, robot_cfg, planner_cfg)
    if isempty(candidates)
        selected_poses = [];
        pose_info = [];
        fprintf('    该区域无候选停靠位姿！\n');
        return;
    end

    % --- 读取消融实验模式 ---
    ablation_mode = 'full';
    if isfield(planner_cfg, 'docking') && isfield(planner_cfg.docking, 'ablation_mode')
        ablation_mode = planner_cfg.docking.ablation_mode;
    end
    if strcmpi(ablation_mode, 'nearest_docking')
        [selected_poses, pose_info] = select_nearest_docking(candidates, region, scene, robot_cfg);
        return;
    end

    % --- 预计算 coverage cost 归一化权重（仅消融模式需要） ---
    w_c_normalized = 0;
    need_coverage_cost_adj = ismember(lower(ablation_mode), ...
        {'no_coverage_cost', 'no_coverage_all'});
    if need_coverage_cost_adj
        cw = planner_cfg.cost_weights;
        w_sum = cw.w_reach + cw.w_coverage + cw.w_orientation + ...
                cw.w_safety + cw.w_approach + cw.w_maneuver;
        if w_sum > 0
            w_c_normalized = cw.w_coverage / w_sum;
        end
    end

    % --- 确定消融标志 ---
    is_no_cov_priority = ismember(lower(ablation_mode), ...
        {'no_coverage_priority', 'no_coverage_all'});
    is_no_cov_cost     = need_coverage_cost_adj;  % no_coverage_cost 或 no_coverage_all
    is_no_reach        = strcmpi(ablation_mode, 'no_reachability');

    all_task_ids = region.task_ids;
    num_total_tasks = length(all_task_ids);
    remaining_tasks = all_task_ids;

    weights = get_bottom_up_weights(planner_cfg);
    a1 = weights.coverage_gain;
    a2 = weights.reachability;
    a3 = weights.local_cost;
    a4 = weights.intra_region_move;
    a5 = weights.side_switch;
    a_sum = a1 + a2 + a3 + a4 + a5;
    a1 = a1 / a_sum;
    a2 = a2 / a_sum;
    a3 = a3 / a_sum;
    a4 = a4 / a_sum;
    a5 = a5 / a_sum;

    selected_poses = [];
    pose_info = struct('pose', {}, 'covered_tasks', {}, 'covered_count', {}, ...
                       'arm_assign_map', {}, 'table_id', {}, 'side_id', {}, ...
                       'cost_breakdown', {}, 'remaining_after', {});

    available = true(1, length(candidates));
    prev_pose = [];
    prev_side = [];
    % 消融: single_docking_per_table -> 仅允许 1 次停靠
    if strcmpi(ablation_mode, 'single_docking_per_table')
        max_iter = 1;
    else
        max_iter = num_total_tasks;
    end
    iter = 0;

    fprintf('    桌子 %d: 共 %d 个任务点, %d 个候选位姿', ...
            region.table_id, num_total_tasks, length(candidates));
    if ~strcmpi(ablation_mode, 'full')
        fprintf('  [消融: %s]', ablation_mode);
    end
    fprintf('\n');

    while ~isempty(remaining_tasks) && iter < max_iter
        iter = iter + 1;

        best_idx = 0;
        best_score = -inf;
        best_new_tasks = [];
        best_pose_local = [];
        best_total_cost_dbg = 0;      % 用于调试打印
        best_new_coverage_dbg = 0;    % 用于调试打印

        for ci = 1:length(candidates)
            if ~available(ci)
                continue;
            end

            new_tasks = intersect(candidates(ci).covered_tasks, remaining_tasks);
            if isempty(new_tasks)
                continue;
            end

            new_count = length(new_tasks);
            remaining_count = length(remaining_tasks);

            % ---- A. coverage priority term ----
            new_coverage_term = new_count / max(remaining_count, 1);

            % 消融: 去掉 coverage priority (no_coverage_priority / no_coverage_all)
            if is_no_cov_priority
                new_coverage_term = 0;
            end

            % ---- B. reachability term ----
            new_reach_scores = [];
            for ti_idx = 1:length(new_tasks)
                tid = new_tasks(ti_idx);
                task_local_idx = find(all_task_ids == tid, 1);
                if ~isempty(task_local_idx)
                    cs = candidates(ci).reach_scores(task_local_idx, 3);
                    new_reach_scores = [new_reach_scores, cs]; %#ok<AGROW>
                end
            end
            if isempty(new_reach_scores)
                mean_reach_new = 0;
            else
                mean_reach_new = mean(new_reach_scores);
            end

            % 消融: no_reachability -> 可达性评分项清零
            if is_no_reach
                mean_reach_new = 0;
            end

            % ---- C. total cost (含/不含 coverage cost) ----
            raw_total_cost = candidates(ci).total_cost;
            if is_no_cov_cost
                % 从 total_cost 中剔除 coverage cost 贡献
                cov_raw = candidates(ci).cost_breakdown.coverage;
                effective_total_cost = raw_total_cost - w_c_normalized * cov_raw;
            else
                effective_total_cost = raw_total_cost;
            end

            % ---- D. move cost ----
            if ~isempty(prev_pose)
                move_dist = norm(candidates(ci).pose(1:2) - prev_pose(1:2));
                move_cost = min(1, move_dist / 3.0);
            else
                move_cost = 0;
            end

            % ---- E. side penalty ----
            if ~isempty(prev_side) && isfield(candidates(ci), 'side_id')
                if candidates(ci).side_id ~= prev_side
                    side_penalty = planner_cfg.candidate.side_switch_penalty;
                else
                    side_penalty = 0;
                end
            else
                side_penalty = 0;
            end

            % ---- F. 综合评分 ----
            select_score = a1 * new_coverage_term ...
                         + a2 * mean_reach_new ...
                         - a3 * effective_total_cost ...
                         - a4 * move_cost ...
                         - a5 * side_penalty;

            if select_score > best_score
                best_score           = select_score;
                best_idx             = ci;
                best_new_tasks       = new_tasks;
                best_pose_local      = candidates(ci).pose;
                best_total_cost_dbg  = raw_total_cost;
                best_new_coverage_dbg = new_count;
            end
        end

        if best_idx == 0
            warning('  桌子 %d: 剩余 %d 个任务无法被任何候选停靠位姿覆盖! 任务ID: [%s]', ...
                    region.table_id, length(remaining_tasks), strtrim(sprintf('%d ', remaining_tasks)));
            break;
        end

        selected_poses = [selected_poses; best_pose_local]; %#ok<AGROW>
        remaining_tasks = setdiff(remaining_tasks, best_new_tasks);

        arm_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
        for ti_idx = 1:length(best_new_tasks)
            tid = best_new_tasks(ti_idx);
            task_local_idx = find(all_task_ids == tid, 1);
            if ~isempty(task_local_idx) && task_local_idx <= length(candidates(best_idx).arm_assignment)
                arm_map(tid) = candidates(best_idx).arm_assignment{task_local_idx};
            end
        end

        % --- 修正 cost_breakdown（若消融去掉了 coverage cost） ---
        cb = candidates(best_idx).cost_breakdown;
        if is_no_cov_cost
            cov_raw = cb.coverage;
            cb.coverage = 0;
            cb.total    = cb.total - w_c_normalized * cov_raw;
        end

        info_entry.pose            = best_pose_local;
        info_entry.covered_tasks   = best_new_tasks;
        info_entry.covered_count   = length(best_new_tasks);
        info_entry.arm_assign_map  = arm_map;
        info_entry.table_id        = region.table_id;
        info_entry.side_id         = candidates(best_idx).side_id;
        info_entry.cost_breakdown  = cb;
        info_entry.remaining_after = length(remaining_tasks);
        pose_info(end+1) = info_entry; %#ok<AGROW>

        prev_pose = best_pose_local;
        prev_side = candidates(best_idx).side_id;
        available(best_idx) = false;

        % ---- 调试打印：mode / 覆盖 / score / cost ----
        fprintf(['    停靠 %d: (%.2f, %.2f, %.1f°), 侧%d, ' ...
                 '覆盖=%d, 剩余=%d | score=%.4f  cost=%.4f  [%s]\n'], ...
                iter, best_pose_local(1), best_pose_local(2), ...
                rad2deg(best_pose_local(3)), ...
                candidates(best_idx).side_id, length(best_new_tasks), ...
                length(remaining_tasks), best_score, best_total_cost_dbg, ...
                ablation_mode);
    end

    n_selected = size(selected_poses, 1);
    covered_total = num_total_tasks - length(remaining_tasks);
    fprintf('    桌子 %d 汇总: %d 次停靠, 覆盖 %d/%d 任务, 未覆盖 %d\n', ...
            region.table_id, n_selected, covered_total, num_total_tasks, length(remaining_tasks));

    if ~isempty(remaining_tasks)
        fprintf('    未覆盖任务ID: [%s]\n', strtrim(sprintf('%d ', remaining_tasks)));
    end

end

function weights = get_bottom_up_weights(planner_cfg)
    weights.coverage_gain = 0.35;
    weights.reachability = 0.25;
    weights.local_cost = 0.20;
    weights.intra_region_move = 0.10;
    weights.side_switch = 0.10;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'bottom_up') && ...
            isfield(planner_cfg.strategy.bottom_up, 'weights')
        user_weights = planner_cfg.strategy.bottom_up.weights;
        names = fieldnames(weights);
        for wi = 1:numel(names)
            if isfield(user_weights, names{wi})
                weights.(names{wi}) = user_weights.(names{wi});
            end
        end
    end
end

%% ======================== 消融实验: nearest_docking =========================
function [selected_poses, pose_info] = select_nearest_docking(candidates, region, scene, robot_cfg)
%SELECT_NEAREST_DOCKING 消融模式: 仅选择距离桌子中心最近的候选位姿。
%   不进行迭代多停靠，直接返回距离 region.center 最近的 1 个有效候选。

    % TODO: 当前实现仅使用桌子中心作为参考点。
    %       可进一步优化为使用任务质心 (task_centroid)，使最近停靠更有针对性。
    table_center = region.center;

    best_idx = 0;
    best_dist = inf;
    for ci = 1:length(candidates)
        d = norm(candidates(ci).pose(1:2) - table_center);
        if d < best_dist
            best_dist = d;
            best_idx = ci;
        end
    end

    if best_idx == 0
        selected_poses = [];
        pose_info = [];
        fprintf('    桌子 %d (nearest_docking): 无候选位姿\n', region.table_id);
        return;
    end

    selected_poses = candidates(best_idx).pose;

    best_new_tasks = candidates(best_idx).covered_tasks;
    num_total_tasks = length(region.task_ids);
    covered_count = length(best_new_tasks);

    arm_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    for ti_idx = 1:length(best_new_tasks)
        tid = best_new_tasks(ti_idx);
        task_local_idx = find(region.task_ids == tid, 1);
        if ~isempty(task_local_idx) && task_local_idx <= length(candidates(best_idx).arm_assignment)
            arm_map(tid) = candidates(best_idx).arm_assignment{task_local_idx};
        end
    end

    info_entry.pose            = candidates(best_idx).pose;
    info_entry.covered_tasks   = best_new_tasks;
    info_entry.covered_count   = covered_count;
    info_entry.arm_assign_map  = arm_map;
    info_entry.table_id        = region.table_id;
    info_entry.side_id         = candidates(best_idx).side_id;
    info_entry.cost_breakdown  = candidates(best_idx).cost_breakdown;
    info_entry.remaining_after = num_total_tasks - covered_count;
    pose_info = info_entry;

    fprintf('    桌子 %d (nearest_docking): 1次停靠, 覆盖 %d/%d 任务, 距离 %.2f m\n', ...
            region.table_id, covered_count, num_total_tasks, best_dist);
end
