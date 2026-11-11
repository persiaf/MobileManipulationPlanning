function settings=task_constraint_effective_settings(setup)
%TASK_CONSTRAINT_EFFECTIVE_SETTINGS Export actual task overrides, not IK defaults.
    if nargin<1,setup=task_constraint_comparison_setup();end
    settings.description=['Model IK tolerances are defaults. Task-level values below override them; ' ...
        'subsequent Cartesian samples use one warm-start seed.'];
    settings.model_ik_defaults=setup.robot.manipulator.model.ik;
    settings.task_profiles=struct();settings.matched_path_profiles=struct();
    fields={'task_type','orientation_mode','position_tolerance_m','orientation_tolerance_rad', ...
        'sample_spacing_m','max_joint_step_rad','approach_distance_m','arm_side', ...
        'tool_axis','T_ee_tcp','rotation_world','axis_direction_world', ...
        'surface_normal_world','contact_height_tolerance_m'};
    for k=1:numel(setup.mode_order)
        mode=setup.mode_order{k};
        for matched=[false,true]
            task=comparison_manipulation_task(setup.scene.tasks(1),mode,matched);
            cfg=resolve_manipulation_task(task);profile=struct();
            for j=1:numel(fields),profile.(fields{j})=cfg.(fields{j});end
            profile.first_sample_max_seeds=setup.robot.manipulator.model.ik.max_seeds;
            profile.continuation_max_seeds=1;
            profile.max_iterations_per_seed=setup.robot.manipulator.model.ik.max_iterations;
            if matched,settings.matched_path_profiles.(mode)=profile;
            else,settings.task_profiles.(mode)=profile;end
        end
    end
end
