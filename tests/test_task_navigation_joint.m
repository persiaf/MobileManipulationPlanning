function tests=test_task_navigation_joint
%TEST_TASK_NAVIGATION_JOINT Exact finite-pool cover and route contracts.
    tests=functiontests(localfunctions);
end

function testAllMinimumCoversAndCoreAreRetained(testCase)
    candidates=cover_fixture();
    [covers,cert]=enumerate_minimum_task_covers(candidates,[10 20 30]);
    expected=[1 2;1 3;1 5;2 3;2 4];
    verifyTrue(testCase,cert.feasible);
    verifyTrue(testCase,cert.exact);
    verifyEqual(testCase,cert.minimum_dock_count,2);
    verifyEqual(testCase,sortrows(covers),expected);
    verifyEqual(testCase,cert.cover_count,5);
    verifyEqual(testCase,sort(cert.core_candidate_indices(:)),(1:5)');
end

function testFullCoverCandidateWinsCardinality(testCase)
    candidates=struct('covered_tasks',{[1 2],1,2,[1 2],[]});
    [covers,cert]=enumerate_minimum_task_covers(candidates,[1 2]);
    verifyEqual(testCase,covers,[1;4]);
    verifyEqual(testCase,cert.minimum_dock_count,1);
    verifyEqual(testCase,sort(cert.core_candidate_indices(:)),[1;4]);
end

function testMissingTaskReturnsInfeasibilityCertificate(testCase)
    candidates=struct('covered_tasks',{1,1,[]});
    [covers,cert]=enumerate_minimum_task_covers(candidates,[1 2]);
    verifyFalse(testCase,cert.feasible);
    verifyEmpty(testCase,covers);
    verifyTrue(testCase,isinf(cert.minimum_dock_count));
end

function testCoverBudgetsFailWithoutTruncatedExactClaim(testCase)
    candidates=cover_fixture();
    verifyError(testCase,@()enumerate_minimum_task_covers(candidates,[10 20 30], ...
        struct('max_task_count',2)),'enumerate_minimum_task_covers:BudgetExceeded');
    verifyError(testCase,@()enumerate_minimum_task_covers(candidates,[10 20 30], ...
        struct('max_enumerated_combinations',1)),'enumerate_minimum_task_covers:BudgetExceeded');
    verifyError(testCase,@()enumerate_minimum_task_covers(candidates,[10 20 30], ...
        struct('max_covers',1)),'enumerate_minimum_task_covers:BudgetExceeded');
end

function testJointSetAndOrderMatchIndependentExhaustiveOracle(testCase)
    groups={[2;3],[4 5;4 6]};
    D=20*ones(6);D(1:7:end)=0;
    D(1,3)=1;D(3,6)=1;D(6,4)=1;D(4,1)=1;
    solution=solve_minimum_cover_route(groups,D,1);
    oracle=independent_two_group_route_cost(groups,D);
    verifyTrue(testCase,solution.feasible);
    verifyTrue(testCase,solution.exact);
    verifyEqual(testCase,solution.minimum_dock_count,3);
    verifyEqual(testCase,solution.route_cost,oracle,'AbsTol',1e-12);
    verifyEqual(testCase,solution.route_cost,4,'AbsTol',1e-12);
    verifyEqual(testCase,sort(solution.selected_indices(:)),[3;4;6]);
    verifyEqual(testCase,solution.visit_order(:)',[1 3 6 4]);
    verifyEqual(testCase,solution.closed_route(:)',[1 3 6 4 1]);
end

function testDirectedInfiniteEdgesAreSkipped(testCase)
    D=inf(4);D(1:5:end)=0;
    D(1,3)=1;D(3,4)=2;D(4,1)=3;
    solution=solve_minimum_cover_route({[2;3],4},D,1);
    verifyTrue(testCase,solution.feasible);
    verifyEqual(testCase,sort(solution.selected_indices(:)),[3;4]);
    verifyEqual(testCase,solution.route_cost,6,'AbsTol',1e-12);
    verifyEqual(testCase,solution.closed_route(:)',[1 3 4 1]);
end

function testNoFiniteTourIsReportedWithoutApproximation(testCase)
    D=inf(3);D(1:4:end)=0;
    solution=solve_minimum_cover_route({[2;3]},D,1);
    verifyFalse(testCase,solution.feasible);
    verifyTrue(testCase,isinf(solution.route_cost));
end

function testRouteEnumerationBudgetsFailExplicitly(testCase)
    D=ones(4);D(1:5:end)=0;
    verifyError(testCase,@()solve_minimum_cover_route({[2 3]},D,1, ...
        struct('max_route_evaluations',1)),'solve_minimum_cover_route:BudgetExceeded');
    verifyError(testCase,@()solve_minimum_cover_route({[2;3]},D,1, ...
        struct('max_cover_products',1)),'solve_minimum_cover_route:BudgetExceeded');
    verifyError(testCase,@()solve_minimum_cover_route({[2 3]},D,1, ...
        struct('max_selected_docks',1)),'solve_minimum_cover_route:BudgetExceeded');
end

function testTieHandlingIsDeterministic(testCase)
    D=ones(5);D(1:6:end)=0;
    groups={[2;3],[4;5]};
    first=solve_minimum_cover_route(groups,D,1);
    second=solve_minimum_cover_route(groups,D,1);
    verifyEqual(testCase,first,second);
    verifyEqual(testCase,first.route_cost,3,'AbsTol',1e-12);
end

function testProductionEntryKeepsCardinalityAheadOfNavigationCost(testCase)
    [lib,regions,scene,robot,cfg,D]=production_fixture();
    plan=plan_task_navigation_joint(lib,regions,scene,robot,cfg,struct('cost_matrix',D));
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.covered_tasks_predicted,[1 2]);
    % The two partial-cover candidates have a cheaper tour, but a full-cover
    % one-dock candidate exists. The lexicographic solver must select it.
    verifyEqual(testCase,size(plan.selected_poses,1),1);
    verifyEqual(testCase,plan.selected_poses,lib.regions(1).candidates(1).pose);
    verifyEqual(testCase,plan.preferred_visit_order(:)',[1 2]);
end

function testReorderControlPreservesTheSuppliedDockSet(testCase)
    [lib,regions,scene,robot,cfg,D]=production_fixture();
    opts=struct('cost_matrix',D,'control','reorder_only','fixed_candidate_indices',[2 3]);
    plan=plan_task_navigation_joint(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.covered_tasks_predicted,[1 2]);
    verifyEqual(testCase,size(plan.selected_poses,1),2);
    expected=reshape([lib.regions(1).candidates(2:3).pose],3,[])';
    verifyEqual(testCase,sortrows(plan.selected_poses),sortrows(expected));
    verifyEqual(testCase,sort(plan.preferred_visit_order(:)),(1:3)');
end

function testEmptyTaskCoverHasZeroCostCertificate(testCase)
    [covers,cert]=enumerate_minimum_task_covers(struct('covered_tasks',{}),[]);
    verifyTrue(testCase,cert.feasible);
    verifyEqual(testCase,size(covers),[1 0]);
    verifyEqual(testCase,cert.minimum_dock_count,0);
    solution=solve_minimum_cover_route({covers},0,1);
    verifyTrue(testCase,solution.feasible);
    verifyEqual(testCase,solution.route_cost,0);
    verifyEqual(testCase,solution.closed_route,[1 1]);
end

function testMalformedCostMatricesFailClearly(testCase)
    bad={ones(2,3),[0 NaN;1 0],[0 -1;1 0],[1 1;1 0]};
    for k=1:numel(bad)
        verifyError(testCase,@()solve_minimum_cover_route({2},bad{k},1), ...
            'solve_minimum_cover_route:CostMatrix');
    end
end

function testProductionRejectsRegionsThatDoNotPartitionAllTasks(testCase)
    [lib,regions,scene,robot,cfg,D]=production_fixture();
    lib=rmfield(lib,'scene_signature');regions.task_ids=1;
    verifyError(testCase,@()plan_task_navigation_joint(lib,regions,scene,robot,cfg, ...
        struct('cost_matrix',D)),'plan_task_navigation_joint:RegionTasks');
end

function testProductionRejectsOverlappingRegionTaskOwnership(testCase)
    [lib,regions,scene,robot,cfg,D]=production_fixture();
    lib=rmfield(lib,'scene_signature');lib.regions=[lib.regions,lib.regions];
    regions=[regions,regions];regions(1).task_ids=1;regions(2).task_ids=[1 2];
    verifyError(testCase,@()plan_task_navigation_joint(lib,regions,scene,robot,cfg, ...
        struct('cost_matrix',D)),'plan_task_navigation_joint:RegionTasks');
end

function testDispatcherKeepsJointStrategyOptIn(testCase)
    [lib,regions,scene,robot,cfg,D]=production_fixture();
    verifyEqual(testCase,cfg.strategy.name,'bottom_up');
    cfg.strategy.name='task_navigation_joint';cfg.strategy.joint.cost_matrix=D;
    plan=run_planning_strategy(lib,regions,scene,robot,cfg);
    verifyEqual(testCase,plan.status,'READY');
    verifyTrue(testCase,plan.joint.selection_optimal);
    verifyEqual(testCase,plan.joint.lexicographic_value,[0 1 20]);
end

function testDefaultOracleQueriesActualDirectedNavigationEdges(testCase)
    [lib,regions,scene,robot,cfg,~]=production_fixture();
    plan=plan_task_navigation_joint(lib,regions,scene,robot,cfg);
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.joint.navigation_queries,2);
    verifyGreaterThan(testCase,plan.joint.route_cost,0);
    verifyEqual(testCase,plan.joint.cost_source,'directed_actual_paths_plus_wrapped_endpoint_yaw');
end

function candidates=cover_fixture()
    candidates=struct('covered_tasks',{[10 20],[20 30],[10 30],10,30,[]});
end

function best=independent_two_group_route_cost(groups,D)
    best=Inf;
    for a=1:size(groups{1},1)
        for b=1:size(groups{2},1)
            nodes=[groups{1}(a,:),groups{2}(b,:)];
            orders=perms(nodes);
            for k=1:size(orders,1)
                route=[1,orders(k,:),1];cost=0;
                for edge=1:numel(route)-1
                    cost=cost+D(route(edge),route(edge+1));
                end
                best=min(best,cost);
            end
        end
    end
end

function [lib,regions,scene,robot,cfg,D]=production_fixture()
    [scene,robot,cfg,regions]=create_test_fixture();
    cfg.execution.return_to_start=true;
    cfg.routing.mode='dense';
    poses=[1 1 0;1.1 1 0;1.2 1 0];
    supports={[1 2],1,2};candidates=struct([]);
    for k=1:3
        scores=zeros(2,3);scores(supports{k},:) = repmat([1 0 1],numel(supports{k}),1);
        arms={'none';'none'};arms(supports{k})={'left'};
        c=struct('pose',poses(k,:),'table_id',regions(1).table_id,'side_id',2, ...
            'covered_tasks',supports{k},'reach_scores',scores,'arm_assignment',{arms}, ...
            'nominal_covered_tasks',supports{k},'nominal_reach_scores',scores, ...
            'robust_covered_tasks',[],'robust_reachability_summary', ...
            repmat(struct('minimum_score',NaN,'success_fraction',NaN),1,2), ...
            'coverage_ratio',numel(supports{k})/2,'cost_breakdown',struct('total',0), ...
            'total_cost',0);
        if isempty(candidates),candidates=c;else,candidates(end+1)=c;end %#ok<AGROW>
    end
    entry=struct('region_id',1,'table_id',regions(1).table_id,'region',regions(1), ...
        'candidates',candidates,'num_candidates',3,'task_ids',regions(1).task_ids, ...
        'generation_time_s',0,'generation_diagnostics',struct());
    lib=struct('regions',entry,'total_candidates',3,'total_generation_time_s',0, ...
        'scene_signature',build_candidate_library_signature(regions,scene,robot,cfg));
    D=ones(4);D(1:5:end)=0;D(1,2)=10;D(2,1)=10;
end
