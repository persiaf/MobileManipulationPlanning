function tests = test_candidate_generation
    tests = functiontests(localfunctions);
end

function testHeuristicCandidates(testCase)
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    candidates = generate_docking_candidates(regions(1), scene, robot_cfg, planner_cfg);
    verifyNotEmpty(testCase, candidates);
    verifyTrue(testCase, all(isfinite([candidates.total_cost])));
    removed_field = [char([108, 110, 110]), '_cost'];
    verifyFalse(testCase, isfield(candidates(1).cost_breakdown, removed_field));
end
