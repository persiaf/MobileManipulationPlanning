function cfg = real_platform_config()
%REAL_PLATFORM_CONFIG Offline planning and real-platform interface values.

    cfg.enabled = false;
    cfg.schema_version = '1.0';
    cfg.frame_id = 'map';
    cfg.scene_id = 'simulation_default';
    cfg.planner_version = 'rch-mt-mmp-matlab';

    cfg.position_tolerance_m = 0.03;
    cfg.yaw_tolerance_rad = 0.05;
    cfg.default_policy_id = '';
    cfg.default_arm_mode = 'either';

    cfg.lift.enabled = false;
    cfg.lift.candidate_heights_m = [0.00, 0.15, 0.30];
    cfg.lift.min_height_m = 0.00;
    cfg.lift.max_height_m = 0.60;
    cfg.lift.default_height_m = 0.00;

    cfg.io.mission_plan_file = fullfile('results', 'real', 'mission_plan.json');
    cfg.io.execution_log_dir = fullfile('results', 'logs');
end
