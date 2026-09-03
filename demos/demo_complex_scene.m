function demo_complex_scene()
%DEMO_COMPLEX_SCENE Build and save V1/V2 complex scene figures.

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    vis_cfg = visualization_config();
    vis_cfg.figure_visible = 'off';
    vis_cfg.save_png = true;

    scene_v1 = build_complex_visual_scene('V1', 2026);
    visualize_complex_scene(scene_v1, vis_cfg, 'title', 'V1 Medium Complex Scene', ...
        'filename', 'V1_scene');

    scene_v2 = build_complex_visual_scene('V2', 2026);
    visualize_complex_scene(scene_v2, vis_cfg, 'title', 'V2 High Complex Scene', ...
        'filename', 'V2_scene');
end
