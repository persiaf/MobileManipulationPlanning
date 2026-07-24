function [left_score, right_score, combined_score, arm_assignment] = evaluate_dual_arm_reachability(pose, target_xyz, robot_cfg, threshold)
    if nargin < 4
        threshold = 0.3;
    end

    left_score  = evaluate_single_arm_reachability(pose, target_xyz, 'left',  robot_cfg, threshold);
    right_score = evaluate_single_arm_reachability(pose, target_xyz, 'right', robot_cfg, threshold);
    combined_score = max(left_score, right_score);

    if combined_score <= 0
        arm_assignment = 'none';
    elseif left_score >= right_score
        arm_assignment = 'left';
    else
        arm_assignment = 'right';
    end
end