function [optimal_poses, pose_to_region_map, task_sequence_summary, pose_info_list, plan] = ...
    stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, candidate_library)
%STAGE2_DOCKING_PLANNING Select docking poses through the configured strategy.

    fprintf('\n=== Stage 2: RCH-MT-MMP docking pose planning ===\n');
    if nargin < 5 || isempty(candidate_library)
        candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    else
        validate_candidate_library(candidate_library, regions);
    end

    plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg);
    optimal_poses = plan.selected_poses;
    pose_to_region_map = plan.pose_to_region_map;
    task_sequence_summary = plan.task_sequence_summary;
    pose_info_list = plan.pose_info_list;

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

    if isempty(optimal_poses)
        error('stage2_docking_planning:NoFeasibleDockingPose', ...
            'No feasible docking pose was selected.');
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

function tf = runtime_flag(cfg, field_name, default_value)
    tf = default_value;
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, field_name)
        tf = logical(cfg.runtime.(field_name));
    end
end
