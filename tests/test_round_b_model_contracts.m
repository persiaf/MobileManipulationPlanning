function tests = test_round_b_model_contracts
    tests = functiontests(localfunctions);
end

function testRobustZeroOffsetMatchesNominal(testCase)
    robot = robot_config();
    pose = [1.0, 1.0, 0.2];
    target = [1.25, 1.05, 0.5];
    model = struct('source','test','measured',false, ...
        'position_offsets_m',[0 0], 'yaw_offsets_rad',0);
    [~, ~, nominal_score] = evaluate_dual_arm_reachability(pose, target, robot, 0.3);
    robust = evaluate_robust_reachability(pose, target, robot, 0.3, model);
    verifyEqual(testCase, robust.sample_count, 1);
    verifyEqual(testCase, robust.minimum_score, nominal_score, 'AbsTol', 1e-12);
    verifyEqual(testCase, robust.success_fraction, double(nominal_score >= 0.3));
end

function testRobustExpandedErrorCannotImproveConservativeCase(testCase)
    robot = robot_config();
    pose = [1.0, 1.0, 0.0];
    target = [1.25, 1.05, 0.5];
    small = struct('position_offsets_m',[0 0], 'yaw_offsets_rad',0);
    large = struct('position_offsets_m',[0 0; 2 2], 'yaw_offsets_rad',[0 pi]);
    a = evaluate_robust_reachability(pose, target, robot, 0.3, small);
    b = evaluate_robust_reachability(pose, target, robot, 0.3, large);
    verifyLessThanOrEqual(testCase, b.success_fraction, a.success_fraction);
end

function testObjectiveFieldsRemainSeparate(testCase)
    p = planner_config();
    c = struct('total_candidates',0,'total_generation_time_s',0);
    s = struct('num_tasks',0);
    plan = build_unified_plan('bottom_up', zeros(0,3), [], {}, {}, [], [], c, s, p, struct(), struct());
    verifyTrue(testCase, isfield(plan.objective, 'stage2'));
    verifyFalse(testCase, isfield(plan.objective, 'final'));
    verifyEqual(testCase, plan.objective_value_stage2, plan.objective.stage2);
end

function testCacheSignatureChangesWithSingleCell(testCase)
    p = planner_config();
    s = struct('grid_map',zeros(4), 'map_info',struct('resolution',0.1));
    a = build_path_cache_signature(s.grid_map, s.map_info, p);
    s.grid_map(2,3) = 1;
    b = build_path_cache_signature(s.grid_map, s.map_info, p);
    verifyNotEqual(testCase, a, b);
    p.robot_cfg.footprint.length_m = p.robot_cfg.footprint.length_m + 0.01;
    c = build_path_cache_signature(zeros(4), struct('resolution',0.1), p);
    verifyNotEqual(testCase, a, c);
end

function testRobustModeFeedsCandidateCoverageAndSelection(testCase)
    [scene, robot, planner, regions] = create_test_fixture();
    planner.reachability.mode = 'robust';
    library = build_candidate_library(regions, scene, robot, planner);
    verifyTrue(testCase, isfield(library.regions(1).candidates, 'nominal_covered_tasks'));
    verifyTrue(testCase, isfield(library.regions(1).candidates, 'robust_covered_tasks'));
    plan = run_planning_strategy(library, regions, scene, robot, planner);
    verifyEqual(testCase, plan.status, 'READY');
    for i = 1:numel(plan.task_sequence_summary)
        verifyTrue(testCase, isfield(plan.task_sequence_summary{i}, 'robust_success_fraction'));
        verifyGreaterThanOrEqual(testCase, plan.task_sequence_summary{i}.robust_success_fraction, ...
            planner.reachability.robust_success_fraction_min);
    end
    nominal = planner;
    nominal.reachability.mode = 'nominal';
    verifyError(testCase, @() validate_candidate_library( ...
        library, regions, scene, robot, nominal), 'validate_candidate_library:StaleLibrary');
end

function testOrdinaryCacheRejectsChangedMap(testCase)
    p = planner_config();
    scene = struct('grid_map',zeros(5), 'map_info',struct('resolution',0.1));
    cache = initialize_path_cost_cache(scene, p);
    scene.grid_map(3,3) = 1;
    verifyError(testCase, @() get_cached_path_cost(cache,[0 0 0],[0.2 0.2 0],scene,p), ...
        'get_cached_path_cost:StaleCache');
end

function testServiceSemanticsAvoidBimanualAmbiguity(testCase)
    pose_info.arm_assign_map = containers.Map('KeyType','double','ValueType','char');
    pose_info.arm_assign_map(1) = 'left';
    pose_info.arm_assign_map(2) = 'right';
    pose_info.covered_count = 2;
    summary = struct('covered_tasks',[1 2], 'pose',[1 2 0], 'table_id',1);
    service = build_service_config(1, pose_info, summary, real_platform_config());
    verifyEqual(testCase, service.arm_mode, 'dual_independent');
    verifyEqual(testCase, service.predicted_covered_task_count, 2);
    verifyFalse(testCase, isfield(service, 'predicted_reachability'));
end

function testPlanningStateRetainsContract(testCase)
    scene = struct('tasks',struct('id',1));
    rc2 = struct('sparse_graph',[], 'selected_docks',[1 2 0]);
    state = build_planning_state(scene, struct(), struct(), [], rc2, [], struct());
    verifyEqual(testCase, state.rc2_plan.selected_docks, [1 2 0]);
    verifyEqual(testCase, state.tasks.id, 1);
end

function testBottomUpScoreTermsHaveExpectedDirection(testCase)
    w = struct('coverage_gain',1,'reachability',1,'local_cost',1, ...
        'intra_region_move',1,'side_switch',1);
    base = compute_bottom_up_selection_score(0.5,0.5,0.5,0.5,0.5,w);
    verifyGreaterThan(testCase, compute_bottom_up_selection_score(0.8,0.5,0.5,0.5,0.5,w), base);
    verifyGreaterThan(testCase, compute_bottom_up_selection_score(0.5,0.8,0.5,0.5,0.5,w), base);
    verifyLessThan(testCase, compute_bottom_up_selection_score(0.5,0.5,0.8,0.5,0.5,w), base);
    verifyLessThan(testCase, compute_bottom_up_selection_score(0.5,0.5,0.5,0.8,0.5,w), base);
    verifyLessThan(testCase, compute_bottom_up_selection_score(0.5,0.5,0.5,0.5,0.8,w), base);
end

function testSelectionNeverAddsZeroCoverageCandidate(testCase)
    p = planner_config();
    p.strategy.bottom_up.weights = struct('coverage_gain',0.35,'reachability',0.25, ...
        'local_cost',0.20,'intra_region_move',0.10,'side_switch',0.10);
    p.docking.ablation_mode = 'full';
    a = make_candidate([1 0 0], 1, [1 0 0]);
    b = make_candidate([2 0 0], [], [0 0 0]);
    region = struct('task_ids',[1 2], 'table_id',1, 'center',[0 0]);
    [poses, info] = select_multi_poses([a b], region, struct(), struct(), p);
    verifyEqual(testCase, size(poses,1), 1);
    verifyEqual(testCase, info(1).covered_tasks, 1);
end

function candidate = make_candidate(pose, task, scores)
    candidate = struct('pose',pose,'covered_tasks',task,'reach_scores',repmat(scores,2,1), ...
        'arm_assignment',{{'left';'left'}},'side_id',1,'total_cost',0.2, ...
        'cost_breakdown',struct('coverage',0,'total',0.2));
end
