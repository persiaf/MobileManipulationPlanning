function scene = demo_complex_scene(profile_id, seed)
%DEMO_COMPLEX_SCENE Build and save one deterministic complex scene figure.

    if nargin < 1 || isempty(profile_id), profile_id = 'V2'; end
    if nargin < 2 || isempty(seed), seed = 2026; end
    profile_id = upper(char(profile_id));

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    effective = resolve_effective_config('profile', profile_id, 'seed', seed);
    print_effective_config(effective);
    vis_cfg = effective.visualization;
    vis_cfg.figure_visible = 'off';
    vis_cfg.save_png = true;

    scene = build_complex_visual_scene(profile_id, seed);
    visualize_complex_scene(scene, vis_cfg, ...
        'title', sprintf('%s Complex Scene', profile_id), ...
        'filename', sprintf('%s_scene', profile_id));
end
