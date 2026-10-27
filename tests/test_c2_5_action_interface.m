function tests = test_c2_5_action_interface
%TEST_C2_5_ACTION_INTERFACE Minimal deterministic C2-5 contract checks.
    tests = functiontests(localfunctions);
end

function testFixedActionOrderAndUnsupportedEvent(testCase)
    state = basic_state();
    event = struct('type','ADD_TASKS');
    impact = struct('invalid_sparse_edges', zeros(0,2));
    [mask, d] = evaluate_replanning_action_mask(state, state, event, impact);
    verifyEqual(testCase, d.actions, {'KEEP','REPLAN_PATH','SWITCH_DOCK','REALLOCATE_TASK'});
    verifyEqual(testCase, mask, false(1,4));
    [result, next_state] = execute_replanning_action(state, state, event, impact, 'REPLAN_PATH');
    verifyEqual(testCase, result.status, 'UNSUPPORTED_EVENT');
    verifyEqual(testCase, next_state.rc2_plan, state.rc2_plan);
end

function testKeepRequiresSafeCurrentPath(testCase)
    state = basic_state();
    [~,event] = apply_dynamic_event(state,'ADD_LOCAL_OBSTACLE',1);
    impact = struct('invalid_sparse_edges', zeros(0,2));
    [mask, ~] = evaluate_replanning_action_mask(state, state, event, impact);
    verifyTrue(testCase, mask(1));
    blocked = state;
    [ci, cj] = world2grid(state.stats.global_path(round(size(state.stats.global_path,1)/2),1), ...
        state.stats.global_path(round(size(state.stats.global_path,1)/2),2), ...
        blocked.scene.map_info.resolution, blocked.scene.map_info.height_m);
    blocked.scene.grid_map(max(1,ci-1):min(100,ci+1),max(1,cj-1):min(100,cj+1)) = 1;
    event.center=state.stats.global_path(round(size(state.stats.global_path,1)/2),1:2);
    impact=analyze_change_impact(state,event);
    [mask2, ~] = evaluate_replanning_action_mask(state, blocked, event, impact);
    verifyFalse(testCase, mask2(1));
    [result, ~] = execute_replanning_action(state, blocked, event, impact, 'KEEP');
    verifyEqual(testCase, result.status, 'ACTION_REJECTED');
end

function testIllegalActionDoesNotChangeAssignment(testCase)
    state = basic_state();
    [~,event] = apply_dynamic_event(state,'ADD_LOCAL_OBSTACLE',1);
    impact = struct('invalid_sparse_edges', zeros(0,2));
    frozen = state.plan;
    [result, next_state] = execute_replanning_action(state, state, event, impact, 'REALLOCATE_TASK');
    verifyEqual(testCase, result.status, 'ACTION_REJECTED');
    verifyEqual(testCase, next_state.plan, frozen);
    verifyEqual(testCase, next_state.rc2_plan, state.rc2_plan);
end

function testPathActionIsConnectedToLocalReplan(testCase)
    state = basic_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 1);
    event.center = [4.5, 4.5]; event.radius_m = 0.15;
    changed.scene = state.scene;
    changed.scene.grid_map(45:47,45:47) = 1;
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = execute_replanning_action(state, changed, event, impact, 'REPLAN_PATH');
    verifyTrue(testCase, isfield(diagnostics, 'low_level'));
    verifyEqual(testCase, result.requested_action, 'REPLAN_PATH');
    verifyEqual(testCase, next_state.rc2_plan, state.rc2_plan);
    verifyTrue(testCase, any(strcmp(result.status, {'SUCCESS','FALLBACK_REQUIRED','NOOP'})));
end

function testDockAndTaskActionsConnectToLocalReplan(testCase)
    state = planning_fixture();
    dock_event = apply_event_only(state, 'INVALIDATE_DOCK');
    dock_impact = analyze_change_impact(state, dock_event);
    [dock_result, dock_state] = execute_replanning_action(state, state, dock_event, dock_impact, 'SWITCH_DOCK');
    verifyEqual(testCase, dock_result.requested_action, 'SWITCH_DOCK');
    verifyTrue(testCase, dock_result.action_legal);
    verifyTrue(testCase, any(strcmp(dock_result.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    verifyEqual(testCase, dock_state.rc2_plan, state.rc2_plan);

    task_id = state.plan.task_sequence_summary{1}.covered_tasks(1);
    task_event = struct('type','TASK_COVERAGE_FAILURE','event_id',901, ...
        'target_task_id',task_id,'target_dock_index',1,'target_region',1, ...
        'invalidated_coverage_pairs',[task_id 1]);
    task_impact = analyze_change_impact(state, task_event);
    [task_result, task_state] = execute_replanning_action(state, state, task_event, task_impact, 'REALLOCATE_TASK');
    verifyEqual(testCase, task_result.requested_action, 'REALLOCATE_TASK');
    verifyTrue(testCase, task_result.action_legal);
    verifyTrue(testCase, any(strcmp(task_result.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    verifyEqual(testCase, task_state.rc2_plan, state.rc2_plan);
end

function event = apply_event_only(state, type)
    [~, event] = apply_dynamic_event(state, type, 1);
end

function state = planning_fixture()
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(library, regions, scene, robot_cfg, planner_cfg);
    plan.candidate_library = library;
    [scene, ~, ~, ~, ~, ~, stats] = stage3_global_planning(scene, ...
        plan.selected_poses, plan.task_sequence_summary, {}, robot_cfg, ...
        planner_cfg, plan.pose_info_list, struct(), plan);
    state = stats.planning_state;
    state.task_state = struct('predicted_tasks',plan.covered_tasks_predicted, ...
        'commanded_tasks',[],'executed_tasks',[],'verified_tasks',[]);
    for i=1:numel(scene.tasks), scene.tasks(i).status=0; end
    state.scene = scene;
    state = normalize_planning_state(state);
    state = complete_replanning_route(state);
end

function state = basic_state()
    scene = struct('start_pose',[0.5,0.5,0], 'grid_map',zeros(100,100), ...
        'map_info',struct('resolution',0.1,'height_m',10,'width_m',10,'rows',100,'cols',100), ...
        'num_tasks',1, 'tasks',struct('id',1,'x',8.5,'y',8.5,'z',0.5,'status',0,'table_id',1));
    scene.scene_cfg = struct('table_positions',[8.5,8.5],'table_region_id',1, ...
        'table_orientations',0,'table',struct('length',0.8,'width',0.5),'task_offset_z',0.5);
    scene.region_metadata = struct('region_count',1,'region_ids',1,'table_region_id',1, ...
        'task_region_id',1,'tasks_per_region',1,'selected_docks_per_region',1);
    state = struct('scene',scene,'scale','fixture','seed',1);
    state.robot_cfg = struct('footprint',struct('length_m',0.423,'width_m',0.366,'model','rectangle'));
    state.plan = struct('selected_poses',[8.5,8.5,0],'pose_to_region_map',1, ...
        'covered_tasks_predicted',1,'task_sequence_summary',{{struct('covered_tasks',1,'table_id',1,'dock_index',1)}}, ...
        'pose_info_list',{{struct('covered_tasks',1,'table_id',1,'dock_index',1)}});
    state.all_poses = [scene.start_pose; state.plan.selected_poses];
    state.planner_cfg = struct('global_planner','dstar_lite','incremental_replanning_enabled',true, ...
        'footprint_aware_navigation',true,'robot_cfg',state.robot_cfg, ...
        'astar',struct('allow_diagonal',true,'heuristic_weight',1));
    graph = build_sparse_docking_graph(state.all_poses, scene.grid_map, scene.map_info, state.planner_cfg, [0;1]);
    state.stats.sparse_graph = graph;
    [state.stats.global_path,state.stats.segment_stats] = stitch_sparse_graph_path(state.all_poses,[1,2,1],graph);
    state.stats.route_order = [1,2,1];
    state.rc2_plan = struct('selected_docks',state.plan.selected_poses,'route_order',[1,2,1], ...
        'global_path',state.stats.global_path,'path_segments',state.stats.segment_stats);
    state.task_state = struct('predicted_tasks',1,'commanded_tasks',1,'executed_tasks',1,'verified_tasks',1);
    state = normalize_planning_state(state);
    state = complete_replanning_route(state);
end
