function [score, details] = evaluate_bimanual_reachability(pose, target_pair_xyz, robot_cfg, threshold)
%EVALUATE_BIMANUAL_REACHABILITY Geometric two-arm reachability interface.
%   This does not perform IK or collision-aware coordinated motion planning.

    if nargin < 4 || isempty(threshold)
        threshold = 0.3;
    end
    details = struct('status', 'not_evaluated', 'left_score', 0, ...
        'right_score', 0, 'is_ik_verified', false, 'message', '');
    score = 0;
    if ~isnumeric(target_pair_xyz) || ~isequal(size(target_pair_xyz), [2, 3])
        details.message = 'target_pair_xyz must contain [left_target; right_target].';
        return;
    end

    left_score = evaluate_single_arm_reachability(pose, target_pair_xyz(1, :), ...
        'left', robot_cfg, threshold);
    right_score = evaluate_single_arm_reachability(pose, target_pair_xyz(2, :), ...
        'right', robot_cfg, threshold);
    score = min(left_score, right_score);
    details.left_score = left_score;
    details.right_score = right_score;
    if score > 0
        details.status = 'geometrically_reachable';
    else
        details.status = 'unreachable';
    end
    details.message = 'Geometric screening only; coordinated IK is not verified.';
end
