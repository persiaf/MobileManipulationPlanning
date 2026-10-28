function cfg = apply_experiment_protocol(cfg)
%APPLY_EXPERIMENT_PROTOCOL Use one robot, threshold, return and timing basis.
% Algorithm/routing/sampling/weights remain explicit experimental variables.
    standard = planner_config();
    cfg.robot_cfg = validate_main_robot_config(standard.robot_cfg);
    cfg.reachability.threshold = standard.reachability.threshold;
    cfg.reachability.mode = 'nominal';
    cfg.execution.return_to_start = true;
    cfg.benchmark.include_return_to_start = true;
    cfg.benchmark.order_mode = 'fixed_order';
    cfg.experiment_protocol = experiment_protocol_config(standard);
    cfg.experiment_protocol.enforce = true;
    validate_experiment_protocol(cfg);
end
