function scene = demo_cafeteria_scene(task_mode, seed)
%DEMO_CAFETERIA_SCENE Print effective config and save the scene figure.

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if isnumeric(task_mode)
        seed = task_mode;
        task_mode = 'complex12';
    elseif nargin < 2 || isempty(seed)
        seed = 2026;
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    override = cafeteria_experiment_override();
    profile_override.scene = cafeteria_scene_profile_config(task_mode, seed);
    effective = resolve_effective_config('profile', 'CAFETERIA_1X3_2X3', ...
        'seed', seed, 'robot_profile', 'physical_robot', ...
        'profile_override', profile_override, 'experiment_override', override);
    print_effective_config(effective);
    scene = build_scene_from_cfg(effective.scene);
    vis_cfg = effective.visualization;
    vis_cfg.figure_visible = 'off';
    vis_cfg.output_dir = fullfile(root, 'results', 'figures', 'cafeteria');
    visualize_cafeteria_scene(scene, vis_cfg, ...
        'title', sprintf('CAFETERIA %s Scene', upper(char(task_mode))), ...
        'filename', sprintf('cafeteria_%s_scene', lower(char(task_mode))));
    close all force;
end
