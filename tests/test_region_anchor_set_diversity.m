function tests = test_region_anchor_set_diversity
    tests = functiontests(localfunctions);
end

function testAnchorShortlistPreservesSideAndUniqueCoverage(testCase)
    cfg = planner_config();
    cfg.strategy.top_down.max_anchors_per_region = 2;
    cfg.strategy.top_down.preserve_side_diversity = true;
    cfg.strategy.top_down.preserve_unique_task_cover = true;

    candidates = repmat(empty_candidate(), 1, 4);
    candidates(1) = make_candidate([0, 0, 0], [1, 2], 1, 1.0, 1.0, 10);
    candidates(2) = make_candidate([1, 0, 0], 1, 2, 0.9, 2.0, 20);
    candidates(3) = make_candidate([2, 0, 0], 3, 3, 0.8, 3.0, 30);
    candidates(4) = make_candidate([3, 0, 0], 2, 1, 0.7, 4.0, 40);

    candidate_library.regions = struct('candidates', candidates);
    anchors = build_region_anchors(candidate_library, cfg);
    kept = anchors(1).candidate_indices;

    verifyTrue(testCase, ismember(1, kept), 'Best high-coverage anchor should be retained.');
    verifyTrue(testCase, ismember(2, kept), 'Side-diverse representative should be retained.');
    verifyTrue(testCase, ismember(3, kept), 'Unique task coverer should be retained.');
    verifyEqual(testCase, size(anchors(1).poses, 2), 3);
end

function c = empty_candidate()
    c = struct('pose', [0, 0, 0], 'covered_tasks', [], 'side_id', 0, ...
        'reach_scores', zeros(0, 3), 'total_cost', 0, 'candidate_id', 0);
end

function c = make_candidate(pose, tasks, side_id, reachability, cost, id)
    c = empty_candidate();
    c.pose = pose;
    c.covered_tasks = tasks;
    c.side_id = side_id;
    c.reach_scores = [tasks(:), tasks(:), reachability * ones(numel(tasks), 1)];
    c.total_cost = cost;
    c.candidate_id = id;
end
