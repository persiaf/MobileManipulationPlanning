function tests=test_task_navigation_joint_local
    tests=functiontests(localfunctions);
end

function testReplacementStrictlyDecreasesActualCostAndPreservesCoverage(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg, ...
        struct('cost_matrix',D,'initial_candidate_indices',1));
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.covered_tasks_predicted,[1 2]);
    verifyEqual(testCase,plan.joint_local.candidate_indices,2);
    verifyEqual(testCase,plan.joint_local.input_seed_cost,20);
    verifyEqual(testCase,plan.joint_local.route_cost,2);
    verifyEqual(testCase,plan.joint_local.regional_minimum_dock_counts,1);
    verifyFalse(testCase,plan.joint_local.seed_repaired);
    verifyFalse(testCase,plan.joint_local.global_optimal);
    verifyTrue(testCase,all([plan.joint_local.accepted_moves.new_cost] < ...
        [plan.joint_local.accepted_moves.previous_cost]));
end

function testSeedRepairIsSeparatedFromSubsequentDescent(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    D(1,3)=8;D(3,1)=8;
    opts=struct('cost_matrix',D,'initial_candidate_indices',[3 4], ...
        'initial_matrix_visit_order',[1 4 5]);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyTrue(testCase,plan.joint_local.seed_repaired);
    verifyEqual(testCase,plan.joint_local.input_seed_cost,3);
    verifyEqual(testCase,plan.joint_local.minimum_cover_seed_cost,20);
    verifyEqual(testCase,plan.joint_local.route_cost,16);
    verifyGreaterThan(testCase,plan.joint_local.route_cost,plan.joint_local.input_seed_cost);
    verifyEqual(testCase,plan.num_docking_poses,1);
end

function testMinimumSeedAndReorderControlsKeepTheirDockSets(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    opts=struct('cost_matrix',D,'initial_candidate_indices',[3 4],'control','minimum_seed');
    seed=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    opts.control='reorder_only';
    reorder=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,seed.joint_local.candidate_indices,1);
    verifyEqual(testCase,reorder.joint_local.candidate_indices,1);
    verifyEqual(testCase,seed.joint_local.route_cost,20);
    verifyEqual(testCase,reorder.joint_local.route_cost,20);
    verifyEmpty(testCase,seed.joint_local.accepted_moves);
    verifyEqual(testCase,seed.joint_local.termination_reason,'minimum_seed_control');
    verifyFalse(testCase,seed.joint_local.local_optimum_certified);
end

function testDirectedTwoOptRepricesReversedInternalEdges(testCase)
    [lib,regions,scene,robot,cfg]=many_region_fixture(3);
    D=100*ones(7);D(1:8:end)=0;
    route=[1 2 4 6 1];
    for k=1:4,D(route(k),route(k+1))=10;end
    better=[1 4 2 6 1];
    for k=1:4,D(better(k),better(k+1))=1;end
    opts=struct('cost_matrix',D,'initial_candidate_indices',[1 3 5], ...
        'initial_matrix_visit_order',[1 2 4 6],'control','reorder_only');
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.joint_local.candidate_indices,[1 3 5]);
    verifyEqual(testCase,plan.joint_local.route_cost,4);
    verifyLessThan(testCase,plan.joint_local.route_cost,plan.joint_local.input_seed_cost);
    verifyTrue(testCase,all(strcmp({plan.joint_local.accepted_moves.operation},'directed_two_opt')));
end

function testOracleBudgetReturnsOnlyFullyPricedIncumbent(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    opts=struct('edge_cost_callback',@(a,b)D(a,b),'initial_candidate_indices',1, ...
        'max_navigation_queries',2,'return_cost_cache',true);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.joint_local.route_cost,20);
    verifyEqual(testCase,plan.joint_local.oracle_queries,2);
    verifyEqual(testCase,plan.joint_local.termination_reason,'navigation_query_budget_exhausted');
    verifyFalse(testCase,plan.joint_local.local_optimum_certified);
    verifyFalse(testCase,isfield(plan.joint_local,'cost_matrix'));
    verifyEqual(testCase,numel(plan.joint_local.edge_cache_keys),2);
end

function testInitialBudgetFailureCannotReturnUnpricedPlan(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    opts=struct('edge_cost_callback',@(a,b)D(a,b),'initial_candidate_indices',1, ...
        'max_navigation_queries',1);
    verifyError(testCase,@()plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts), ...
        'plan_task_navigation_joint_local:InitialBudgetExceeded');
end

function testPartialMatrixKnownInfAndDirectedMemoAreRespected(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    partial=inf(5);partial(1:6:end)=0;partial(1,2)=10;partial(2,1)=10;
    known=logical(eye(5));known(1,2)=true;known(2,1)=true;
    opts=struct('cost_matrix',partial,'known_edge_mask',known, ...
        'edge_cost_callback',@(a,b)D(a,b),'initial_candidate_indices',1, ...
        'max_navigation_queries',2,'return_cost_cache',true);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.joint_local.route_cost,2);
    verifyEqual(testCase,plan.joint_local.oracle_queries,2);
    verifyEqual(testCase,plan.joint_local.cost_matrix(1,3),1);
    verifyEqual(testCase,plan.joint_local.cost_matrix(3,1),1);
    verifyTrue(testCase,plan.joint_local.known_edge_mask(3,1));
end

function testSixteenRegionsDoNotRequireGlobalPermutationEnumeration(testCase)
    [lib,regions,scene,robot,cfg]=many_region_fixture(16);
    opts=struct('edge_cost_callback',@(a,b)double(a~=b), ...
        'initial_candidate_indices',1:2:32,'enable_two_opt',false, ...
        'max_navigation_queries',100,'max_passes',1);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.covered_tasks_predicted,1:16);
    verifyEqual(testCase,plan.num_docking_poses,16);
    verifyEqual(testCase,plan.joint_local.regional_minimum_dock_counts,ones(1,16));
    verifyEqual(testCase,plan.joint_local.route_cost,17);
    verifyLessThanOrEqual(testCase,plan.joint_local.oracle_queries,100);
    verifyFalse(testCase,plan.joint_local.global_optimal);
end

function testDispatcherIsOptInAndIncompleteSeedIsRejected(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    verifyEqual(testCase,cfg.strategy.name,'bottom_up');
    cfg.strategy.name='task_navigation_joint_local';
    cfg.strategy.joint_local.cost_matrix=D;cfg.strategy.joint_local.initial_candidate_indices=1;
    plan=run_planning_strategy(lib,regions,scene,robot,cfg);
    verifyEqual(testCase,plan.status,'READY');
    verifyEqual(testCase,plan.strategy_name,'task_navigation_joint_local');
    verifyError(testCase,@()plan_task_navigation_joint_local(lib,regions,scene,robot,cfg, ...
        struct('initial_candidate_indices',3)),'plan_task_navigation_joint_local:SeedCoverage');
end

function testRepairPreservesRelativePositionsOfRetainedBaselineNodes(testCase)
    [lib,regions,scene,robot,cfg]=many_region_fixture(2);
    D=ones(5);D(1:6:end)=0;
    opts=struct('cost_matrix',D,'control','minimum_seed', ...
        'initial_candidate_indices',[2 3 1],'initial_matrix_visit_order',[1 3 4 2]);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyTrue(testCase,plan.joint_local.seed_repaired);
    verifyEqual(testCase,plan.joint_local.matrix_visit_order,[1 4 2]);
    verifyEqual(testCase,plan.joint_local.seed_repair_regions,1);
end

function testNeighborBudgetKeepsAlreadyAcceptedStrictImprovement(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    opts=struct('cost_matrix',D,'initial_candidate_indices',1,'max_neighbor_evaluations',1);
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.joint_local.route_cost,2);
    verifyEqual(testCase,plan.joint_local.neighbor_evaluations,1);
    verifyEqual(testCase,plan.joint_local.termination_reason,'neighbor_evaluation_budget_exhausted');
    verifyFalse(testCase,plan.joint_local.local_optimum_certified);
end

function testDefaultOracleUsesActualNavigationWithoutDenseMatrix(testCase)
    [lib,regions,scene,robot,cfg,~]=single_fixture();
    plan=plan_task_navigation_joint_local(lib,regions,scene,robot,cfg, ...
        struct('control','minimum_seed','initial_candidate_indices',1,'return_cost_cache',true));
    verifyEqual(testCase,plan.status,'READY');
    verifyGreaterThan(testCase,plan.joint_local.route_cost,0);
    verifyEqual(testCase,plan.joint_local.oracle_queries,2);
    verifyEqual(testCase,plan.joint_local.oracle_source,'directed_actual_paths_plus_wrapped_endpoint_yaw');
    verifyFalse(testCase,isfield(plan.joint_local,'cost_matrix'));
end

function testBalancedSearchPreservesMinimumAndUsesPricedReplacement(testCase)
    [lib,regions,scene,robot,cfg,D]=single_fixture();
    opts=struct('cost_matrix',D,'known_edge_mask',true(size(D)), ...
        'edge_cost_callback',@(a,b)D(a,b),'initial_candidate_indices',1,'initial_matrix_visit_order',[1 2]);
    plan=plan_task_navigation_joint_budgeted(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.joint_local.route_cost,2);
    verifyEqual(testCase,plan.covered_tasks_predicted,[1 2]);
    verifyEqual(testCase,plan.num_docking_poses,1);
    verifyEqual(testCase,plan.joint_local.oracle_queries,0);
    verifyTrue(testCase,plan.joint_local.fixed_set_order_converged);
    verifyFalse(testCase,plan.joint_local.local_optimum_certified);
end

function testBalancedReserveCannotExceedLogicalQueryBudget(testCase)
    [lib,regions,scene,robot,cfg]=many_region_fixture(3);
    D=100*ones(7);D(1:8:end)=0;nodes=[1 2 4 6];D(nodes,nodes)=10;D(1:8:end)=0;
    full=D;full(1,3)=1;full(3,4)=1;
    known=logical(eye(7));known(nodes,nodes)=true;
    opts=struct('cost_matrix',D,'known_edge_mask',known,'edge_cost_callback',@(a,b)full(a,b), ...
        'initial_candidate_indices',[1 3 5],'initial_matrix_visit_order',nodes,'max_navigation_queries',8);
    plan=plan_task_navigation_joint_budgeted(lib,regions,scene,robot,cfg,opts);
    verifyLessThanOrEqual(testCase,plan.joint_local.oracle_queries,8);
    verifyEqual(testCase,plan.num_docking_poses,3);
    verifyLessThanOrEqual(testCase,plan.joint_local.route_cost,40);
    r=plan.joint_local.matrix_visit_order;
    verifyTrue(testCase,all(plan.joint_local.known_edge_mask(r,r),'all'));
end

function testBalancedOrderingRepricesDirectedInternalArcs(testCase)
    [lib,regions,scene,robot,cfg]=many_region_fixture(3);
    D=100*ones(7);D(1:8:end)=0;
    original=[1 2 4 6 1];better=[1 4 2 6 1];
    for k=1:4,D(original(k),original(k+1))=10;end
    for k=1:4,D(better(k),better(k+1))=1;end
    opts=struct('cost_matrix',D,'known_edge_mask',true(7),'edge_cost_callback',@(a,b)D(a,b), ...
        'initial_candidate_indices',[1 3 5],'initial_matrix_visit_order',original(1:end-1));
    plan=plan_task_navigation_joint_budgeted(lib,regions,scene,robot,cfg,opts);
    verifyEqual(testCase,plan.joint_local.route_cost,4);
    verifyEqual(testCase,sort(plan.joint_local.candidate_indices),[1 3 5]);
    verifyTrue(testCase,plan.joint_local.fixed_set_order_converged);
end

function [lib,regions,scene,robot,cfg,D]=single_fixture()
    [scene,robot,cfg,regions]=create_test_fixture();cfg.execution.return_to_start=true;
    supports={[1 2],[1 2],1,2};candidates=struct([]);
    for k=1:4
        c=make_candidate([0.8+k*.1,0.8,0],1,supports{k},[1 2]);
        if isempty(candidates),candidates=c;else,candidates(end+1)=c;end %#ok<AGROW>
    end
    lib=make_library(candidates,regions,scene,robot,cfg);
    D=10*ones(5);D(1:6:end)=0;D(1,3)=1;D(3,1)=1;
    D(1,4)=1;D(4,5)=1;D(5,1)=1;
end

function [lib,regions,scene,robot,cfg]=many_region_fixture(nr)
    [scene,robot,cfg,region]=create_test_fixture();cfg.execution.return_to_start=true;
    scene.tasks=repmat(scene.tasks(1),1,nr);scene.num_tasks=nr;
    scene.scene_cfg.table_positions=[(1:nr)',ones(nr,1)];
    scene.scene_cfg.table_orientations=zeros(nr,1);scene.scene_cfg.num_tables=nr;
    regions=repmat(region,1,nr);entries=struct([]);
    for ri=1:nr
        regions(ri).table_id=ri;regions(ri).task_ids=ri;scene.tasks(ri).id=ri;scene.tasks(ri).table_id=ri;
        candidates=[make_candidate([ri,1,0],ri,ri,ri),make_candidate([ri,.8,0],ri,ri,ri)];
        entry=library_entry(candidates,regions(ri),ri);
        if isempty(entries),entries=entry;else,entries(end+1)=entry;end %#ok<AGROW>
    end
    lib=struct('regions',entries,'total_candidates',2*nr,'total_generation_time_s',0, ...
        'scene_signature',build_candidate_library_signature(regions,scene,robot,cfg));
end

function c=make_candidate(pose,table_id,support,task_ids)
    scores=zeros(numel(task_ids),3);arms=repmat({'none'},numel(task_ids),1);
    for ti=1:numel(task_ids)
        if ismember(task_ids(ti),support),scores(ti,:)=[1 0 1];arms{ti}='left';end
    end
    c=struct('pose',pose,'table_id',table_id,'side_id',2,'covered_tasks',support, ...
        'reach_scores',scores,'nominal_reach_scores',scores,'arm_assignment',{arms}, ...
        'cost_breakdown',struct('total',0),'total_cost',0);
end
function lib=make_library(candidates,regions,scene,robot,cfg)
    lib=struct('regions',library_entry(candidates,regions,1),'total_candidates',numel(candidates), ...
        'total_generation_time_s',0,'scene_signature',build_candidate_library_signature(regions,scene,robot,cfg));
end
function entry=library_entry(candidates,region,ri)
    entry=struct('region_id',ri,'table_id',region.table_id,'region',region, ...
        'candidates',candidates,'num_candidates',numel(candidates),'task_ids',region.task_ids, ...
        'generation_time_s',0,'generation_diagnostics',struct());
end
