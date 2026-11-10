function cfg = manipulation_task_profile(task_type)
%MANIPULATION_TASK_PROFILE Task constraints in world and ee_gripper frames.
%   VX300s ee_gripper_link advances along +X. A wiping pad can instead use
%   its own T_ee_tcp and tool_axis; rotation_world describes THAT TCP frame.
    if nargin < 1 || isempty(task_type), task_type = 'position_only'; end
    cfg.task_type = lower(char(task_type));
    cfg.orientation_mode = 'none';
    cfg.tool_axis = [1;0;0];
    cfg.axis_direction_world = [0;0;-1];
    cfg.rotation_world = [0,0,1; 0,1,0; -1,0,0];
    cfg.T_ee_tcp = eye(4);
    cfg.position_tolerance_m = 0.003;
    cfg.orientation_tolerance_rad = deg2rad(10);
    cfg.approach_distance_m = 0;
    cfg.waypoints_world = zeros(0,3);
    cfg.sample_spacing_m = 0.02;
    cfg.max_joint_step_rad = deg2rad(25);
    cfg.arm_side = 'either';
    cfg.surface_normal_world = [0;0;1];
    cfg.contact_height_tolerance_m = 0.003;
    switch cfg.task_type
        case 'position_only'
        case 'top_down_grasp'
            cfg.orientation_mode = 'axis';
            cfg.approach_distance_m = 0.04;
        case 'table_wipe'
            cfg.orientation_mode = 'full';
            cfg.orientation_tolerance_rad = deg2rad(5);
        otherwise
            error('manipulation_task_profile:UnknownTaskType', ...
                'Unknown kinematic task type "%s".', cfg.task_type);
    end
end
