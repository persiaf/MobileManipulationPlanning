function tests = test_c2_1_path_repair
    tests = functiontests(localfunctions);
end

function testObstacleBuildsValidatedNextPath(testCase)
    state = c2_fixture();
    original_docks = state.plan.selected_poses;
    original_assignment = state.plan.task_sequence_summary;
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyNotEmpty(testCase, result.global_path);
    verifyTrue(testCase, diagnostics.path_validation.path_exists);
    verifyTrue(testCase, diagnostics.path_validation.map_collision_free);
    verifyTrue(testCase, diagnostics.path_validation.footprint_checked);
    verifyTrue(testCase, diagnostics.path_validation.footprint_collision_free);
    verifyTrue(testCase, diagnostics.path_validation.world_grid_consistent);
    verifyEqual(testCase, next_state.plan.selected_poses, original_docks);
    verifyEqual(testCase, next_state.plan.task_sequence_summary, original_assignment);
end

function testNoImpactReturnsNoopWithValidatedPath(testCase)
    state = c2_fixture();
    changed = state;
    far=[changed.scene.map_info.width_m-0.7,changed.scene.map_info.height_m-0.7];
    [ci, cj] = world2grid(far(1), far(2), changed.scene.map_info.resolution, ...
        changed.scene.map_info.height_m);
    verifyTrue(testCase,ci>=1&&ci<=changed.scene.map_info.rows&&cj>=1&&cj<=changed.scene.map_info.cols);
    changed.scene.grid_map(ci, cj) = 1;
    event = struct('type', 'ADD_LOCAL_OBSTACLE', 'event_id', 1, ...
        'center', far, 'radius_m', 0.05);
    impact = analyze_change_impact(state, event);
    verifyEmpty(testCase, impact.invalid_sparse_edges);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'NOOP');
    verifyNotEmpty(testCase, next_state.global_path);
    verifyTrue(testCase, next_state.path_validation.map_collision_free);
end

function testDisconnectedMapReturnsFallback(testCase)
    state = c2_fixture();
    changed = state;
    changed.scene.grid_map(:) = 1;
    event = struct('type', 'ADD_LOCAL_OBSTACLE', 'event_id', 99, ...
        'center', [0, 0], 'radius_m', 1);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyTrue(testCase, result.fallback);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, next_state.path_repair_version, 'FAILED');
end

function testOriginalRc2AndTaskStateArePreserved(testCase)
    state = c2_fixture();
    state.task_state = struct('predicted_tasks', 1:6, 'commanded_tasks', 1:3, ...
        'executed_tasks', 1:2, 'verified_tasks', 1);
    state.rc2_plan = struct('selected_docks', state.plan.selected_poses, ...
        'route_order', state.stats.route_order, 'global_path', [0 0 0; 1 1 0]);
    frozen = state.rc2_plan;
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact); %#ok<ASGLU>
    verifyEqual(testCase, state.rc2_plan, frozen);
    verifyEqual(testCase, next_state.rc2_plan, frozen);
    verifyEqual(testCase, result.verified_completion, 100 / 6, 'AbsTol', 1e-10);
end

function testRoundBPlanningStateCanTransitionWithoutReconstruction(testCase)
    legacy = c2_fixture();
    rc2 = struct('status', 'READY', 'reason', '', ...
        'selected_docks', legacy.plan.selected_poses, ...
        'task_assignment', {legacy.plan.task_sequence_summary}, ...
        'covered_tasks', legacy.plan.covered_tasks_predicted, ...
        'uncovered_tasks', [], 'visit_order', legacy.stats.route_order(1:end-1), ...
        'route_order', legacy.stats.route_order, ...
        'global_path', legacy.stats.global_path, ...
        'path_segments', legacy.stats.segment_stats, ...
        'candidate_library', legacy.candidate_library, ...
        'reachability_summary', struct(), 'robust_reachability_summary', struct(), ...
        'sparse_graph', legacy.stats.sparse_graph, 'reachability_mode', 'nominal');
    state = build_planning_state(legacy.scene, legacy.robot_cfg, legacy.planner_cfg, ...
        legacy.candidate_library, rc2, struct('verified_tasks', 1));
    frozen = state.rc2_plan;
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyTrue(testCase, ~isempty(next_state.current_plan.global_path));
    verifyEqual(testCase, next_state.rc2_plan, frozen);
    verifyEqual(testCase, next_state.task_state.verified_tasks, 1);
end

function testLengthComesFromActualPath(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.final_path_length, compute_path_length(result.global_path), 'AbsTol', 1e-12);
    verifyEqual(testCase, next_state.stats.path_len, compute_path_length(next_state.global_path), 'AbsTol', 1e-12);
end

function testReturnSegmentIsIncludedAndContinuous(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result, ~, diagnostics] = local_replan(state, changed, event, impact);
    verifyTrue(testCase, strcmp(result.status, 'SUCCESS') || strcmp(result.status, 'NOOP'));
    verifyTrue(testCase, diagnostics.path_validation.segments_continuous);
    verifyEqual(testCase, result.path_segments(end).to_id, 1);
    verifyTrue(testCase, result.path_segments(end).is_return_segment);
end

function state = c2_fixture()
    state = p5_fixture_state();
end
