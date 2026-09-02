function tests = test_sparse_graph_determinism
    tests = functiontests(localfunctions);
end

function testSparseGraphIsDeterministic(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    cfg = planner_cfg;
    cfg.routing.mode = 'sparse_knn';
    poses = [scene.start_pose; 1 1 0; 2 1 0; 2 2 0; 1 2 0];
    region_ids = [0; 1; 1; 2; 2];
    g1 = build_sparse_docking_graph(poses, scene.grid_map, scene.map_info, cfg, region_ids);
    g2 = build_sparse_docking_graph(poses, scene.grid_map, scene.map_info, cfg, region_ids);
    verifyEqual(testCase, g1.geometric_adjacency, g2.geometric_adjacency);
    verifyEqual(testCase, g1.adjacency, g2.adjacency);
    verifyEqual(testCase, g1.shortest_cost_matrix, g2.shortest_cost_matrix, 'AbsTol', 1e-12);
end
