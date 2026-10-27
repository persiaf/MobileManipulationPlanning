function tests = test_cafeteria_a4_inside_table
    tests = functiontests(localfunctions);
end

function testFullMarkerFootprintsAreInside(testCase)
    scene = build_cafeteria_scene('complex12', 2026);
    report = validate_cafeteria_scene(scene, robot_geometry_profile('physical_robot'));
    verifyEqual(testCase, scene.scene_cfg.task_marker.size, [0.297, 0.210], 'AbsTol', 1e-12);
    verifyTrue(testCase, all(report.a4_inside_table));
    verifyEqual(testCase, sum(report.a4_inside_table), 12);
end

function testVisualizationUsesShortA4Labels(testCase)
    scene = build_cafeteria_scene('complex12', 2026);
    vis = visualization_config();
    vis.figure_visible = 'off';
    vis.save_png = false;
    vis.save_fig = false;
    fig = visualize_cafeteria_scene(scene, vis);
    cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
    labels = string(get(findobj(fig, 'Type', 'Text'), 'String'));
    for i = 1:12
        verifyTrue(testCase, any(labels == "A" + i));
    end
    verifyFalse(testCase, any(startsWith(labels, "Task T")));
end
