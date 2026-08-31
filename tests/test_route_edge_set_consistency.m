function tests = test_route_edge_set_consistency
    tests = functiontests(localfunctions);
end

function testDockEdgesExcludeStartByDefault(testCase)
    n = normalize_visit_order([1, 2, 3, 1], 2, struct('node_indexing', 'auto'));
    edges = build_route_edge_set(n);
    verifyEqual(testCase, edges.directed_edges, [1, 2]);
    verifyFalse(testCase, edges.includes_start_edge);
    verifyFalse(testCase, edges.includes_return_edge);
end

function testStartAndReturnEdgesAreExplicit(testCase)
    n = normalize_visit_order([1, 2, 3, 1], 2, struct('node_indexing', 'auto'));
    edges = build_route_edge_set(n, struct('include_start_edge', true, ...
        'include_return_edge', true));
    verifyEqual(testCase, edges.start_edges, [0, 1]);
    verifyEqual(testCase, edges.return_edges, [2, 0]);
    verifyEqual(testCase, edges.num_edges, 3);
end
