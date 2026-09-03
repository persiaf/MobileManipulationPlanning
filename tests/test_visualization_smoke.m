function tests = test_visualization_smoke
    tests = functiontests(localfunctions);
end

function testScenePlanningAndDynamicRender(testCase)
    vis_cfg = visualization_config();
    vis_cfg.figure_visible = 'off';
    vis_cfg.save_png = false;
    scene = build_complex_visual_scene('V1', 2026);
    fig1 = visualize_complex_scene(scene, vis_cfg, 'title', 'smoke', 'filename', '');
    verifyTrue(testCase, isvalid(fig1));
    close(fig1);

    state = p5_fixture_state();
    fig2 = visualize_planning_overlay(state.scene, state.plan, state.stats, vis_cfg, ...
        'title', 'overlay smoke', 'filename', '');
    verifyTrue(testCase, isvalid(fig2));
    close(fig2);

    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    figs = visualize_dynamic_replan(state, event, result, vis_cfg, ...
        'title', 'dynamic smoke', 'prefix', '');
    verifyTrue(testCase, isvalid(figs.before));
    verifyTrue(testCase, isvalid(figs.after));
    close(figs.before); close(figs.after);
end
