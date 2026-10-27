function override = cafeteria_experiment_override()
%CAFETERIA_EXPERIMENT_OVERRIDE Shared planner settings for cafeteria demos.

    override.robot.arm_base_mount_z = 0.416;
    override.robot.arm_base_mount_z_source = 'user_confirmed_lower_bound';
    override.planner.strategy.name = 'bottom_up';
    override.planner.global_planner = 'convex_astar';
    override.planner.routing.mode = 'dense';
    override.planner.candidate_pair_prefilter.enable = false;
    override.planner.execution.return_to_start = true;
    override.planner.inflation_radius_m = 0.25;
    override.planner.candidate_augmentation.enable = false;
end
