function tests = test_visualization_config_single_source
    tests = functiontests(localfunctions);
end

function testPlannerCompatibilityViewMatchesAdapter(testCase)
    planner = planner_config();
    verifyEqual(testCase, planner.visualization, legacy_visualization_config());
end

function testMaintainedVisualizersDoNotReadPlannerVisualization(testCase)
    names = {'visualize_complex_scene.m','visualize_planning_overlay.m', ...
        'visualize_dynamic_replan.m','draw_collision_free_route.m', ...
        'draw_sparse_graph_connectivity.m','draw_selected_docks.m'};
    root = fileparts(fileparts(mfilename('fullpath')));
    for i = 1:numel(names)
        source = fileread(fullfile(root, 'visualization', names{i}));
        verifyEmpty(testCase, regexp(source, 'planner_cfg\.visualization', 'once'));
        verifyEmpty(testCase, regexp(source, 'planner_config\(\)\.visualization', 'once'));
    end
end
