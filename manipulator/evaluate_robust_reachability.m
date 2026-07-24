function result = evaluate_robust_reachability(pose, target_xyz, robot_cfg, threshold, error_model)
%EVALUATE_ROBUST_REACHABILITY Evaluate geometric reach under docking offsets.

    if nargin < 4 || isempty(threshold), threshold = 0.3; end
    if nargin < 5 || isempty(error_model)
        error_model.position_offsets_m = [0, 0; 0.02, 0; -0.02, 0; 0, 0.02; 0, -0.02];
        error_model.yaw_offsets_rad = [0, 0.03, -0.03];
    end
    offsets = error_model.position_offsets_m;
    yaw_offsets = error_model.yaw_offsets_rad;
    scores = zeros(size(offsets, 1) * numel(yaw_offsets), 1);
    idx = 0;
    for i = 1:size(offsets, 1)
        for j = 1:numel(yaw_offsets)
            idx = idx + 1;
            perturbed = pose + [offsets(i, :), yaw_offsets(j)];
            [~, ~, scores(idx)] = evaluate_dual_arm_reachability( ...
                perturbed, target_xyz, robot_cfg, threshold);
        end
    end
    result = struct('minimum_score', min(scores), 'mean_score', mean(scores), ...
        'success_fraction', mean(scores > 0), 'sample_count', numel(scores), ...
        'geometric_only', true);
end
