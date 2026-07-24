function cfg = robot_config()
    cfg.name = 'MobileManipulator';
    cfg.platform_type = 'mobile_dual_arm';
    cfg.frames.map = 'map';
    cfg.frames.base = 'base_link';
    cfg.frames.left_arm_base = 'left_arm_base';
    cfg.frames.right_arm_base = 'right_arm_base';

    cfg.chassis.length   = 0.423;
    cfg.chassis.width    = 0.366;
    cfg.chassis.height   = 0.127;

    cfg.dual_arm.base_spacing = 0.250;

    cfg.single_arm.reach_radius  = 0.278;
    cfg.single_arm.reach_z_min   = -0.142;
    cfg.single_arm.reach_z_max   = 0.334;
    cfg.single_arm.reach_forward = 0.278;
    cfg.single_arm.reach_backward = 0.197;
    cfg.single_arm.reach_lateral = 0.278;

    cfg.dual_arm.collab_forward_depth  = 0.25;
    cfg.dual_arm.collab_lateral_width  = 0.31;

    cfg.arm_dh.d1 = 0.0563;
    cfg.arm_dh.a2 = 0.10931;
    cfg.arm_dh.a3 = 0.10051;
    cfg.arm_dh.d4 = 0.000953;
    cfg.arm_dh.d5 = 0.0681;

    cfg.arm_dh.offset_j2 = deg2rad(-7.78);
    cfg.arm_dh.offset_j3 = deg2rad(9.32);
    cfg.arm_dh.offset_j4 = deg2rad(88.46);

    cfg.joint_limits.j1 = deg2rad([-90, 90]);
    cfg.joint_limits.j2 = deg2rad([-8, 125]);
    cfg.joint_limits.j3 = deg2rad([-170, 15]);
    cfg.joint_limits.j4 = deg2rad([-90, 90]);
    cfg.joint_limits.j5 = deg2rad([-180, 180]);

    cfg.arm_base_mount_x = cfg.chassis.length / 2;
    cfg.arm_base_mount_z = cfg.chassis.height + cfg.arm_dh.d1;
end
