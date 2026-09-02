function tests = test_sparse_graph_connectivity
    tests = functiontests(localfunctions);
end

function testConnectivityReportRequiresAllDocks(testCase)
    adjacency = false(4);
    adjacency(1, 2) = true; adjacency(2, 1) = true;
    adjacency(3, 4) = true; adjacency(4, 3) = true;
    report = validate_sparse_graph_connectivity(adjacency, [0; 1; 2; 2]);
    verifyFalse(testCase, report.ok);
    verifyFalse(testCase, report.all_required_docks_reachable);
end

function testConnectedRegionGraphPasses(testCase)
    adjacency = false(4);
    adjacency(1, 2) = true; adjacency(2, 1) = true;
    adjacency(2, 3) = true; adjacency(3, 2) = true;
    adjacency(3, 4) = true; adjacency(4, 3) = true;
    report = validate_sparse_graph_connectivity(adjacency, [0; 1; 2; 2]);
    verifyTrue(testCase, report.ok);
    verifyTrue(testCase, report.region_graph_connected);
end
