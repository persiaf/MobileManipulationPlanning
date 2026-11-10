function cfg = task_manipulation_config(robot_cfg)
%TASK_MANIPULATION_CONFIG Opt-in kinematic experiment; mounts are unmeasured.
%   robot.manipulator = task_manipulation_config(robot) enables the VX300s
%   chain without changing the historical chassis or geometric baseline.
%   Replace both mounting transforms with measured base_link->arm_base_link
%   transforms before making claims about a physical Mobile ALOHA platform.
    if nargin < 1, robot_cfg = robot_config(); end
    cfg.mode = 'task_pose_ik';
    cfg.model = vx300s_kinematic_model();
    cfg.left_mount_T_base_arm = eye(4);
    cfg.left_mount_T_base_arm(1:3,4) = [robot_cfg.arm_base_mount_x; ...
        robot_cfg.dual_arm.base_spacing/2; robot_cfg.arm_base_mount_z];
    cfg.right_mount_T_base_arm = cfg.left_mount_T_base_arm;
    cfg.right_mount_T_base_arm(2,4) = -robot_cfg.dual_arm.base_spacing/2;
    cfg.mount_source = 'inherited_planner_mount_unmeasured';
    cfg.hardware_calibrated = false;
end
