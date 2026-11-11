function tests=test_finite_nearest_neighbor_order
    tests=functiontests(localfunctions);
end

function testDirectedTrapRestartsAndRecordsEveryAttempt(testCase)
    D=[0 1 2;1 0 Inf;Inf 1 0];
    [order,r]=finite_nearest_neighbor_order(D,1,true);
    verifyEqual(testCase,order,[1 3 2]);
    verifyEqual(testCase,r.selected_start,2);verifyEqual(testCase,r.attempt_count,2);
    verifyFalse(testCase,r.initial_closed_finite);verifyTrue(testCase,r.recovered);
    verifyEqual(testCase,r.route_order_source,'nearest_neighbor_finite_restart');
    verifyFalse(testCase,r.attempts(1).route_finite);
    verifyTrue(testCase,r.attempts(2).closed_finite);
    verifyEqual(testCase,r.attempts(2).cost,4);
end

function testReturnEdgeIsCheckedAndFirstFiniteIsNotBest(testCase)
    D=[0 1 5 1.5;2 0 1 8;Inf 2 0 1;Inf 3 .5 0];
    [order,r]=finite_nearest_neighbor_order(D,1,true);
    verifyEqual(testCase,order,[1 3 4 2]);
    verifyEqual(testCase,r.selected_start,3);verifyEqual(testCase,r.attempt_count,3);
    verifyEqual(testCase,r.attempts(1).bad_edges,[4 1]);
    verifyEqual(testCase,r.attempts(end).cost,11);
    cheaper=[1 4 3 2 1];cost=sum(D(sub2ind(size(D),cheaper(1:end-1),cheaper(2:end))));
    verifyEqual(testCase,cost,6);verifyGreaterThan(testCase,r.attempts(end).cost,cost);
    [open,open_r]=finite_nearest_neighbor_order(D,1,false);
    verifyEqual(testCase,open,[1 2 3 4]);verifyEqual(testCase,open_r.selected_start,1);
    verifyFalse(testCase,open_r.initial_closed_finite);
end

function testAllFailedRestartsReportFiniteSearchFailure(testCase)
    D=[0 Inf;Inf 0];
    verifyError(testCase,@()finite_nearest_neighbor_order(D,1,true), ...
        'finite_nearest_neighbor_order:NoFiniteOrder');
end

function testStage3DefaultFalseAndPreferredOrderContracts(testCase)
    [scene,robot,cfg,regions,task_map]=create_test_fixture();cfg.execution.return_to_start=true;
    lib=build_candidate_library(regions,scene,robot,cfg);
    plan=run_planning_strategy(lib,regions,scene,robot,cfg);
    [~,~,~,original,~,~,a]=stage3_global_planning(scene,plan.selected_poses, ...
        plan.task_sequence_summary,task_map,robot,cfg,plan.pose_info_list,struct(),plan);
    cfg.routing.finite_nn_restarts=false;
    [~,~,~,same,~,~,b]=stage3_global_planning(scene,plan.selected_poses, ...
        plan.task_sequence_summary,task_map,robot,cfg,plan.pose_info_list,struct(),plan);
    verifyEqual(testCase,same,original);verifyEqual(testCase,a.route_order_source,b.route_order_source);
    verifyFalse(testCase,a.navigation_seed_recovery.enabled);verifyFalse(testCase,b.navigation_seed_recovery.enabled);
    cfg.routing.finite_nn_restarts=true;
    cfg.routing.directed_costs=true;cfg.routing.mode='dense';
    [~,~,~,~,~,~,enabled]=stage3_global_planning(scene,plan.selected_poses, ...
        plan.task_sequence_summary,task_map,robot,cfg,plan.pose_info_list,struct(),plan);
    verifyTrue(testCase,enabled.navigation_seed_recovery.enabled);
    verifyEqual(testCase,enabled.navigation_seed_recovery.selected_start,1);
    verifyEqual(testCase,enabled.navigation_seed_recovery.attempt_count,1);
    verifySize(testCase,enabled.navigation_seed_recovery.cost_matrix, ...
        [size(plan.selected_poses,1)+1,size(plan.selected_poses,1)+1]);
    plan.preferred_visit_order=[1,size(plan.selected_poses,1)+1:-1:2];
    [~,~,~,preferred,~,~,c]=stage3_global_planning(scene,plan.selected_poses, ...
        plan.task_sequence_summary,task_map,robot,cfg,plan.pose_info_list,struct(),plan);
    verifyEqual(testCase,preferred,plan.preferred_visit_order);
    verifyEqual(testCase,c.route_order_source,'strategy_preferred');
    verifyEqual(testCase,c.navigation_seed_recovery.status,'not_needed');
    verifyEqual(testCase,c.navigation_seed_recovery.route_order_source,'strategy_preferred');
    verifyEqual(testCase,c.navigation_seed_recovery.attempt_count,0);
    verifyFalse(testCase,isfield(c.navigation_seed_recovery,'cost_matrix'));
end
