function score = evaluate_single_arm_reachability(pose, target_xyz, arm_side, robot_cfg, threshold)
    if nargin < 5
        threshold = 0.3;
    end
    if nargin < 4
        arm_side = 'left';
    end

    if strcmp(arm_side, 'left')
        base_offset = [robot_cfg.arm_base_mount_x,  robot_cfg.dual_arm.base_spacing / 2];
    else
        base_offset = [robot_cfg.arm_base_mount_x, -robot_cfg.dual_arm.base_spacing / 2];
    end

    R_mat = rot2(pose(3));
    arm_base = [pose(1); pose(2)] + R_mat * base_offset';
    arm_base_z = robot_cfg.arm_base_mount_z;

    dx = target_xyz(1) - arm_base(1);
    dy = target_xyz(2) - arm_base(2);
    dz = target_xyz(3) - arm_base_z;

    d_horizontal = sqrt(dx^2 + dy^2);

    R_min = 0.01;
    R_max = robot_cfg.single_arm.reach_radius;
    z_min = robot_cfg.single_arm.reach_z_min;
    z_max = robot_cfg.single_arm.reach_z_max;

    if d_horizontal < R_min || d_horizontal > R_max || dz < z_min || dz > z_max
        score = 0;
        return;
    end

    R_local = rot2(-pose(3));
    local_dir = R_local * [dx; dy];

    forward_reach  = robot_cfg.single_arm.reach_forward;
    backward_reach = robot_cfg.single_arm.reach_backward;
    lateral_reach  = robot_cfg.single_arm.reach_lateral;

    % 机械臂并非只能向前工作：根据模型，单臂有约0.197 m后向折叠可达距离。
    % 原来的 -0.05 m 限制过严，会把桌角/侧边任务错误判为不可达。
    if local_dir(1) < -backward_reach || abs(local_dir(2)) > lateral_reach * 1.1
        score = 0;
        return;
    end

    dist_score = 1.0 - (d_horizontal / R_max);
    center_score = 1.0 - abs(d_horizontal - (R_min + R_max)/2) / ((R_max - R_min)/2);
    center_score = max(0, center_score);

    ideal_z = (z_min + z_max) / 2;
    z_score = 1.0 - abs(dz - ideal_z) / ((z_max - z_min) / 2);
    z_score = max(0, z_score);

    forward_score = local_dir(1) / forward_reach;
    forward_score = max(0, min(1, forward_score));

    lateral_penalty = abs(local_dir(2)) / lateral_reach;

    raw_score = 0.30 * dist_score + 0.25 * center_score + ...
                0.20 * z_score + 0.15 * forward_score + 0.10 * (1 - lateral_penalty);
    raw_score = max(0, min(1, raw_score));

    if raw_score < threshold
        score = 0;
    else
        score = raw_score;
    end
end