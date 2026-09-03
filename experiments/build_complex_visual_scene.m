function scene = build_complex_visual_scene(profile_id, seed)
%BUILD_COMPLEX_VISUAL_SCENE Build V1/V2/V3 display scenes without changing baselines.

    if nargin < 1 || isempty(profile_id), profile_id = 'V2'; end
    if nargin < 2 || isempty(seed), seed = 2026; end
    cfg = large_scale_profile_config(profile_id, seed);
    scene = build_scene_from_cfg(cfg);
    if isfield(cfg, 'visual_static_obstacles')
        scene = apply_static_obstacles_to_scene(scene, cfg.visual_static_obstacles);
    else
        scene = apply_static_obstacles_to_scene(scene, []);
    end
end
