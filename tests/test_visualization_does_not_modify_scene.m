function tests = test_visualization_does_not_modify_scene
    tests = functiontests(localfunctions);
end

function testSceneVisualizerIsReadOnly(testCase)
    vis_cfg = visualization_config();
    vis_cfg.figure_visible = 'off';
    vis_cfg.save_png = false;
    scene = build_complex_visual_scene('V1', 2026);
    before = scene;
    fig = visualize_complex_scene(scene, vis_cfg, 'title', 'readonly', 'filename', '');
    close(fig);
    verifyEqual(testCase, scene, before);
end
