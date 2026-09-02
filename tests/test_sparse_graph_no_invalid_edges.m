function tests = test_sparse_graph_no_invalid_edges
    tests = functiontests(localfunctions);
end

function testSparseGraphHasNoSelfEdgesAndFiniteCosts(testCase)
    [scene, ~, planner_cfg] = create_test_fixture();
    cfg = planner_cfg;
    cfg.routing.sparse_knn.k = 3;
    poses = [scene.start_pose; 1 1 0; 2 1 0; 2 2 0; 1 2 0];
    g = build_sparse_docking_graph(poses, scene.grid_map, scene.map_info, cfg, [0; 1; 1; 2; 2]);
    verifyFalse(testCase, any(diag(g.adjacency)));
    [ii, jj] = find(triu(g.adjacency, 1));
    for e = 1:numel(ii)
        verifyTrue(testCase, isfinite(g.cost_matrix(ii(e), jj(e))));
        verifyNotEmpty(testCase, g.direct_paths{ii(e), jj(e)});
    end
end
