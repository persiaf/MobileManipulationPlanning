function tests = test_benchmark_protocol_scope
    tests = functiontests(localfunctions);
end

function testFixedRouteScopeAndEveryQueryProvenance(testCase)
    [scene,docks,cfg] = fixture();
    results = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg, ...
        struct('route_order',[1,2,3,1]));
    verifyEqual(testCase,{results.requested_algorithm},{'astar','jps'});
    for k = 1:numel(results)
        r = results(k); trace = jsondecode(r.planner_query_provenance_json);
        verifyTrue(testCase,r.success);
        verifyEqual(testCase,r.comparison_level,'fixed_route_path_planner_pipeline');
        verifyEqual(testCase,r.timing_scope,'fixed_route_path_planner_queries+stitch');
        verifyTrue(testCase,contains(r.excluded_time_scope,'visit_order_preprocessing'));
        verifyEqual(testCase,r.matrix_query_count,0);
        verifyEqual(testCase,r.route_query_count,3);
        verifyEqual(testCase,r.path_query_count,3);
        verifyEqual(testCase,r.num_successful_segments,3);
        verifyEqual(testCase,numel(trace),3);
        verifyTrue(testCase,all(strcmp({trace.query_scope},'route_segment')));
        verifyTrue(testCase,all([trace.path_valid]));
        verifyTrue(testCase,r.actual_planner_known);
        verifyTrue(testCase,r.return_to_start);
        verifyTrue(testCase,r.return_segment_completed);
        verifyEqual(testCase,r.threshold,cfg.reachability.threshold);
        verifyNotEmpty(testCase,r.observed_actual_planners);
    end
end

function testIndependentOrderUsesDifferentExplicitScope(testCase)
    [scene,docks,cfg] = fixture(); cfg.benchmark.algorithms = {'astar'};
    cfg.benchmark.order_mode = 'independent_order';
    r = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg,struct());
    trace = jsondecode(r.planner_query_provenance_json);
    verifyTrue(testCase,r.success);
    verifyEqual(testCase,r.comparison_level,'fixed_docks_route_and_path_planning_pipeline');
    verifyEqual(testCase,r.matrix_query_count,3);
    verifyEqual(testCase,r.route_query_count,3);
    verifyEqual(testCase,r.path_query_count,6);
    verifyEqual(testCase,sum(strcmp({trace.query_scope},'distance_matrix')),3);
    verifyTrue(testCase,contains(r.timing_scope,'visit_order'));
    verifyFalse(testCase,contains(r.excluded_time_scope,'visit_order'));
end

function testOmittedDockCannotBeSuccessfulPartialRoute(testCase)
    [scene,docks,cfg] = fixture(); cfg.benchmark.algorithms = {'astar'};
    r = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg, ...
        struct('route_order',[1,2,1]));
    verifyFalse(testCase,r.success);
    verifyFalse(testCase,r.required_nodes_visited);
    verifyEqual(testCase,r.failure_reason,'VISIT_ORDER_OMITS_DOCKS');
end

function testFailedLaterSegmentDoesNotLeaveSuccessfulPrefix(testCase)
    [scene,docks,cfg] = fixture(); cfg.benchmark.algorithms = {'astar'};
    [i,j] = world2grid(docks(2,1),docks(2,2),scene.map_info.resolution,scene.map_info.height_m);
    scene.grid_map(i,j) = 1;
    r = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg, ...
        struct('route_order',[1,2,3,1]));
    verifyFalse(testCase,r.success);
    verifyEqual(testCase,r.num_required_segments,3);
    verifyEqual(testCase,r.num_successful_segments,1);
    verifyEqual(testCase,r.num_attempted_segments,2);
    verifyEqual(testCase,r.failed_query_count,1);
    verifyEqual(testCase,r.num_path_points,0);
    verifyTrue(testCase,isinf(r.path_length_m));
    verifyFalse(testCase,r.return_segment_completed);
end

function testConvexInternalFallbackIdentityStaysUnknown(testCase)
    [scene,docks,cfg] = fixture(); docks = docks(1,:);
    cfg.benchmark.algorithms = {'convex_astar'};
    cfg.convex_astar.enable = false;
    cfg.convex_astar.enable_fallback = true;
    cfg.convex_astar.fallback_planner = 'jps';
    r = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg, ...
        struct('route_order',[1,2,1]));
    verifyEqual(testCase,r.requested_algorithm,'convex_astar');
    verifyTrue(testCase,r.success);
    verifyFalse(testCase,r.actual_planner_known);
    verifyTrue(testCase,contains(r.observed_actual_planners,'internal_fallback'));
    trace = jsondecode(r.planner_query_provenance_json);
    verifyFalse(testCase,any([trace.actual_planner_known]));
end

function testReturnConditionAndWarningStateArePreserved(testCase)
    [scene,docks,cfg] = fixture(); docks = docks(1,:);
    cfg.benchmark.algorithms = {'astar'}; cfg.benchmark.include_return_to_start = false;
    before = warning('query','run_path_planner:collision');
    cleanup = onCleanup(@() warning(before.state,'run_path_planner:collision')); %#ok<NASGU>
    warning('off','run_path_planner:collision');
    r = stage4_benchmark_all(scene,[scene.start_pose;docks],docks,cfg, ...
        struct('route_order',[1,2,1]));
    after = warning('query','run_path_planner:collision');
    verifyFalse(testCase,r.return_to_start);
    verifyFalse(testCase,r.return_segment_completed);
    verifyEqual(testCase,r.route_query_count,1);
    verifyEqual(testCase,after.state,'off');
end

function [scene,docks,cfg] = fixture()
    cfg = planner_config(); cfg.benchmark.enable = true;
    cfg.benchmark.algorithms = {'astar','jps'};
    cfg.benchmark.include_extended = false; cfg.benchmark.include_stochastic = false;
    cfg.benchmark.include_convex_astar = false; cfg.benchmark.order_mode = 'fixed_order';
    cfg.benchmark.include_return_to_start = true;
    scene = struct(); scene.grid_map = zeros(40,40);
    scene.grid_map([1,end],:) = 1; scene.grid_map(:,[1,end]) = 1;
    scene.map_info = struct('rows',40,'cols',40,'resolution',0.1,'height_m',4,'width_m',4);
    scene.start_pose = [0.8,0.8,0];
    docks = [1.8,0.8,0;2.8,1.8,pi/2];
end
