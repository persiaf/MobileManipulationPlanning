function tests = test_normalize_visit_order
    tests = functiontests(localfunctions);
end

function testGlobalOrderRemovesStartAndReturn(testCase)
    n = normalize_visit_order([1, 3, 2, 1], 2, struct('node_indexing', 'auto'));
    verifyTrue(testCase, n.is_valid);
    verifyTrue(testCase, n.includes_start);
    verifyTrue(testCase, n.includes_return);
    verifyEqual(testCase, n.dock_order, [2, 1]);
end

function testLocalOrderRemainsLocal(testCase)
    n = normalize_visit_order([2, 1, 3], 3, struct('node_indexing', 'auto'));
    verifyTrue(testCase, n.is_valid);
    verifyFalse(testCase, n.includes_start);
    verifyEqual(testCase, n.dock_order, [2, 1, 3]);
end

function testInvalidDuplicateDockIsReported(testCase)
    n = normalize_visit_order([1, 2, 2], 3, struct('node_indexing', 'local'));
    verifyFalse(testCase, n.is_valid);
    verifyNotEmpty(testCase, n.error_message);
end
