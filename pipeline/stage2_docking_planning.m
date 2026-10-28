function [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list, plan] = ...
    stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, candidate_library)
%STAGE2_DOCKING_PLANNING Select docking poses through the configured strategy.

    fprintf('\n=== Stage 2: RCH-MT-MMP docking pose planning ===\n');
    validate_experiment_protocol(planner_cfg);
    if nargin < 5 || isempty(candidate_library)
        candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    else
        validate_candidate_library(candidate_library, regions, scene, robot_cfg, planner_cfg);
    end

    plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg);
    plan.candidate_library = candidate_library;
    plan.reachability_summary = build_reachability_summary(candidate_library, false);
    plan.robust_reachability_summary = build_reachability_summary(candidate_library, true);
    plan.reachability_mode = planner_cfg.reachability.mode;
    plan.robust_reachability_summary.error_model = planner_cfg.docking_error_model;
    plan.robust_reachability_summary.success_fraction_min = ...
        planner_cfg.reachability.robust_success_fraction_min;
    optimal_poses = plan.selected_poses;
    pose_to_region_map = plan.pose_to_region_map;
    task_sequence_summary = plan.task_sequence_summary;
    pose_info_list = plan.pose_info_list;

    if isfield(plan, 'status') && strcmpi(plan.status, 'BLOCKED')
        fprintf('STAGE2_STATUS = BLOCKED\nSTAGE2_REASON = %s\n', plan.reason);
        return;
    end

    fprintf('\n=== Strategy summary: %s ===\n', plan.strategy_name);
    fprintf('Total candidates: %d\n', candidate_library.total_candidates);
    fprintf('Selected docking poses: %d\n', plan.num_docking_poses);
    fprintf('Predicted coverage: %.1f%% (%d/%d tasks)\n', ...
        plan.metrics.predicted_completion_rate, numel(plan.covered_tasks_predicted), scene.num_tasks);
    fprintf('Local cost total: %.4f\n', plan.local_cost_total);
    fprintf('%-6s %-8s %-8s %-22s %-8s %-8s\n', ...
        'pose', 'region', 'table', 'pose(x,y,theta_deg)', 'tasks', 'cost');
    fprintf('%s\n', repmat('-', 1, 72));
    for i = 1:size(optimal_poses, 1)
        c = 0;
        if i <= numel(pose_info_list) && isfield(pose_info_list{i}, 'cost_breakdown') && ...
                isfield(pose_info_list{i}.cost_breakdown, 'total')
            c = pose_info_list{i}.cost_breakdown.total;
        end
        fprintf('%-6d %-8d %-8d (%6.2f,%6.2f,%7.1f) %-8d %-8.4f\n', ...
            i, pose_to_region_map(i), task_sequence_summary{i}.table_id, ...
            optimal_poses(i, 1), optimal_poses(i, 2), rad2deg(optimal_poses(i, 3)), ...
            task_sequence_summary{i}.covered_count, c);
    end

    covered_count = numel(plan.covered_tasks_predicted);
    if covered_count < scene.num_tasks
        plan.status = 'BLOCKED';
        plan.reason = 'PARTIAL_TASK_COVERAGE';
        fprintf('STAGE2_STATUS = BLOCKED\nSTAGE2_REASON = PARTIAL_TASK_COVERAGE\n');
        return;
    end

    if planner_cfg.visualization.show_docking && ~runtime_flag(planner_cfg, 'disable_visualization', false)
        hFig2 = figure('Name', 'Stage 2: docking pose planning', 'Position', [80, 80, 900, 700]);
        draw_scene(scene, hFig2);
        draw_docking_result(optimal_poses, scene, hFig2, [], pose_info_list, planner_cfg.visualization);
        title(sprintf('Stage 2 docking poses - %s - %d service poses', ...
            plan.strategy_name, size(optimal_poses, 1)), 'FontSize', 12);
        drawnow;
    end
end

function summary = build_reachability_summary(library, robust)
    scores = []; fractions = []; covered_pairs = 0; total_pairs = 0;
    for ri = 1:numel(library.regions)
        cs = library.regions(ri).candidates;
        for ci = 1:numel(cs)
            if robust
                if isfield(cs, 'robust_reachability_summary')
                    q = [cs(ci).robust_reachability_summary.success_fraction];
                    fractions = [fractions, q(isfinite(q))]; %#ok<AGROW>
                    covered_pairs = covered_pairs + numel(cs(ci).robust_covered_tasks);
                    total_pairs = total_pairs + numel(cs(ci).robust_reachability_summary);
                end
            else
                q = cs(ci).nominal_reach_scores(:,3)';
                scores = [scores, q]; %#ok<AGROW>
                covered_pairs = covered_pairs + numel(cs(ci).nominal_covered_tasks);
                total_pairs = total_pairs + numel(q);
            end
        end
    end
    if robust, scores = fractions; end
    summary = struct('mode', ternary(robust,'robust','nominal'), ...
        'mean_score', safe_mean(scores), 'minimum_score', safe_min(scores), ...
        'covered_candidate_task_pairs', covered_pairs, 'candidate_task_pairs', total_pairs);
end

function value = safe_mean(x)
    if isempty(x), value = NaN; else, value = mean(x); end
end

function value = safe_min(x)
    if isempty(x), value = NaN; else, value = min(x); end
end

function value = ternary(condition, yes, no)
    if condition, value = yes; else, value = no; end
end

function tf = runtime_flag(cfg, field_name, default_value)
    tf = default_value;
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, field_name)
        tf = logical(cfg.runtime.(field_name));
    end
end
