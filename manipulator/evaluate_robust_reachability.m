function [result, arm_assignment] = evaluate_robust_reachability(pose, target_xyz, robot_cfg, threshold, error_model, task)
%EVALUATE_ROBUST_REACHABILITY Evaluate configured reach under docking offsets.
% Task-pose IK keeps one arm across all offset samples. The historical
% position-only call retains its per-sample either-arm geometric semantics.

    if nargin < 4 || isempty(threshold), threshold = 0.3; end
    if nargin < 5 || isempty(error_model)
        error_model.position_offsets_m = [0, 0; 0.02, 0; -0.02, 0; 0, 0.02; 0, -0.02];
        error_model.yaw_offsets_rad = [0, 0.03, -0.03];
    end
    if ~isfield(error_model, 'position_offsets_m') || ~isfield(error_model, 'yaw_offsets_rad')
        error('evaluate_robust_reachability:InvalidErrorModel', ...
            'Error model must define position_offsets_m and yaw_offsets_rad.');
    end
    offsets = error_model.position_offsets_m;
    yaw_offsets = error_model.yaw_offsets_rad(:)';
    if ~isnumeric(offsets) || ~isnumeric(yaw_offsets) || ...
            size(offsets, 2) ~= 2 || isempty(offsets) || isempty(yaw_offsets) || ...
            any(~isfinite(offsets(:))) || any(~isfinite(yaw_offsets(:)))
        error('evaluate_robust_reachability:InvalidErrorModel', ...
            'Position offsets must be M-by-2 and yaw offsets must be nonempty.');
    end
    scores = zeros(size(offsets, 1) * numel(yaw_offsets), 1);
    arm_scores = zeros(numel(scores), 2);
    has_task = nargin >= 6 && ~isempty(task);
    task_pose_mode = has_task && isfield(robot_cfg, 'manipulator') && ...
        isfield(robot_cfg.manipulator, 'mode') && ...
        strcmp(robot_cfg.manipulator.mode, 'task_pose_ik');
    arm_assignment = 'either';
    idx = 0;
    for i = 1:size(offsets, 1)
        for j = 1:numel(yaw_offsets)
            idx = idx + 1;
            perturbed = pose + [offsets(i, :), yaw_offsets(j)];
            perturbed(3) = wrapToPiLocal(perturbed(3));
            if has_task
                [ls, rs, scores(idx)] = evaluate_task_reachability( ...
                    perturbed, task, robot_cfg, threshold);
                arm_scores(idx, :) = [ls, rs];
            else
                [~, ~, scores(idx)] = evaluate_dual_arm_reachability( ...
                    perturbed, target_xyz, robot_cfg, threshold);
            end
        end
    end
    if task_pose_mode
        worst_scores = min(arm_scores, [], 1);
        [~, selected] = max(worst_scores);
        if worst_scores(1) == worst_scores(2)
            [~, selected] = max(mean(arm_scores > 0, 1));
        end
        scores = arm_scores(:, selected);
        arm_names = {'left', 'right'};
        arm_assignment = arm_names{selected};
        if ~any(scores > 0), arm_assignment = 'none'; end
        success_fraction = mean(scores > 0);
    else
        success_fraction = mean(scores >= threshold);
    end
    result = struct('minimum_score', min(scores), 'mean_score', mean(scores), ...
        'success_fraction', success_fraction, 'sample_count', numel(scores), ...
        'threshold', threshold, 'geometric_only', ~task_pose_mode, ...
        'error_model_source', getfield_default(error_model, 'source', 'unspecified'), ...
        'error_model_measured', getfield_default(error_model, 'measured', false));
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = default_value; end
end
