function [reject, reason] = candidate_reachability_quick_filter(pose, target_xyz, robot_cfg)
%CANDIDATE_REACHABILITY_QUICK_FILTER Conservative pair-level hard-gate filter.
%
% Rejects only when both arms fail at least one hard gate that is already used
% by evaluate_single_arm_reachability. The filter does not approximate the
% final score; it is a definitely-impossible precheck.

    left = arm_hard_gate_pass(pose, target_xyz, 'left', robot_cfg);
    right = arm_hard_gate_pass(pose, target_xyz, 'right', robot_cfg);
    reject = ~(left.pass || right.pass);
    if reject
        reason = left.reason;
        if strcmp(reason, 'PASS_HARD_GATES')
            reason = right.reason;
        end
    else
        reason = 'PASS_HARD_GATES';
    end
end

function out = arm_hard_gate_pass(pose, target_xyz, arm_side, robot_cfg)
    if strcmp(arm_side, 'left')
        base_offset = [robot_cfg.arm_base_mount_x, robot_cfg.dual_arm.base_spacing / 2];
    else
        base_offset = [robot_cfg.arm_base_mount_x, -robot_cfg.dual_arm.base_spacing / 2];
    end

    arm_base = [pose(1); pose(2)] + rot2(pose(3)) * base_offset';
    delta = [target_xyz(1) - arm_base(1); target_xyz(2) - arm_base(2)];
    dz = target_xyz(3) - robot_cfg.arm_base_mount_z;
    d_horizontal = hypot(delta(1), delta(2));
    local_dir = rot2(-pose(3)) * delta;

    out.pass = false;
    if d_horizontal < 0.01 || d_horizontal > robot_cfg.single_arm.reach_radius
        out.reason = 'FAIL_RADIAL';
        return;
    end
    if dz < robot_cfg.single_arm.reach_z_min || dz > robot_cfg.single_arm.reach_z_max
        out.reason = 'FAIL_VERTICAL';
        return;
    end
    if local_dir(1) < -robot_cfg.single_arm.reach_backward
        out.reason = 'FAIL_BACKWARD';
        return;
    end
    if abs(local_dir(2)) > robot_cfg.single_arm.reach_lateral * 1.1
        out.reason = 'FAIL_LATERAL';
        return;
    end

    out.pass = true;
    out.reason = 'PASS_HARD_GATES';
end
