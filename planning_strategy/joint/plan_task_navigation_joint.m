function plan=plan_task_navigation_joint(library,regions,scene,robot,planner,options)
%PLAN_TASK_NAVIGATION_JOINT Lexicographic finite-pool cover and route search.
% Full task support -> minimum dock count -> minimum closed-tour edge cost.
% options.cost_matrix uses 1=start, then region/candidate flattening order;
% its entries already include lambda_theta and may be directed or Inf.
% control='reorder_only' fixes options.fixed_candidate_indices (1-based
% flattened candidates), optimizing only their visit order.
% Exactness is limited to the supplied support matrix and navigation oracle;
% it does not imply continuous-space, collision-aware arm or physical optimality.
% Search is restricted to the minimum-cover layer and Hamiltonian closed
% tours: each selected dock is visited once. If that layer has no finite
% tour, return BLOCKED; do not search larger dock counts or transit revisits.
    if nargin<6,options=struct();end
    if ~isstruct(options)||~isscalar(options)
        error('plan_task_navigation_joint:Options','options must be a scalar struct.');
    end
    validate_candidate_library(library,regions,scene,robot,planner);
    grouped_task_ids=[];
    for ri=1:numel(regions),grouped_task_ids=[grouped_task_ids,reshape(regions(ri).task_ids,1,[])];end %#ok<AGROW>
    if ~isequal(sort(grouped_task_ids),1:scene.num_tasks)
        error('plan_task_navigation_joint:RegionTasks','Regions must partition all scene task indices exactly once.');
    end
    if ~isfield(planner,'execution')||~isfield(planner.execution,'return_to_start')|| ...
            ~planner.execution.return_to_start
        error('plan_task_navigation_joint:ClosedTourRequired','This strategy requires return_to_start=true.');
    end
    started=tic;settings=struct('max_navigation_queries',10000);
    if isfield(planner,'strategy')&&isfield(planner.strategy,'joint')
        settings=merge_options(settings,planner.strategy.joint);
    end
    settings=merge_options(settings,options);
    control=field_or(settings,'control','joint');
    if ~any(strcmp(control,{'joint','reorder_only'}))
        error('plan_task_navigation_joint:Control','control must be joint or reorder_only.');
    end
    n=sum(arrayfun(@(r)numel(r.candidates),library.regions));
    flat=cell(1,n);poses=zeros(n,3);owner=zeros(1,n);offset=zeros(1,numel(regions));
    cursor=0;
    for ri=1:numel(regions)
        offset(ri)=cursor;
        for ci=1:numel(library.regions(ri).candidates)
            cursor=cursor+1;flat{cursor}=library.regions(ri).candidates(ci);
            poses(cursor,:)=flat{cursor}.pose;owner(cursor)=ri;
        end
    end
    covers=cell(1,numel(regions));certificates=cell(size(covers));
    supported=true;
    for ri=1:numel(regions)
        [local,certificates{ri}]=enumerate_minimum_task_covers( ...
            library.regions(ri).candidates,regions(ri).task_ids,settings);
        covers{ri}=local+offset(ri)+1;
        supported=supported&&certificates{ri}.feasible;
    end
    if ~supported
        plan=empty_plan('TASK_UNSUPPORTED_ON_FINITE_POOL');return;
    end
    if strcmp(control,'reorder_only')
        fixed=field_or(settings,'fixed_candidate_indices',[]);
        if ~isnumeric(fixed)||~isreal(fixed)||any(~isfinite(fixed(:)))|| ...
                any(fixed(:)~=fix(fixed(:)))||any(fixed(:)<1)||any(fixed(:)>n)|| ...
                numel(unique(fixed))~=numel(fixed)
            error('plan_task_navigation_joint:FixedCandidates','Fixed indices must be unique flattened candidate indices.');
        end
        fixed=sort(reshape(fixed,1,[]));
        for ri=1:numel(regions)
            chosen=fixed(owner(fixed)==ri);covered=[];
            for ci=chosen,covered=union(covered,flat{ci}.covered_tasks);end
            if ~all(ismember(regions(ri).task_ids,covered))
                error('plan_task_navigation_joint:FixedCoverage','The fixed candidate set does not cover every task.');
            end
            covers{ri}=chosen+1;
        end
    end
    required=1;
    for ri=1:numel(covers),required=union(required,covers{ri}(:)');end
    navigation_queries=0;
    if isfield(settings,'cost_matrix')
        D=settings.cost_matrix;
        if ~isequal(size(D),[n+1,n+1])
            error('plan_task_navigation_joint:CostMatrix','Injected cost_matrix must be (N+1)-by-(N+1).');
        end
        cost_source='injected_directed_cost_matrix';
    else
        validateattributes(settings.max_navigation_queries,{'numeric'}, ...
            {'real','finite','scalar','integer','positive'});
        query_count=numel(required)*(numel(required)-1);
        if query_count>settings.max_navigation_queries
            error('plan_task_navigation_joint:BudgetExceeded','Directed navigation-query budget exceeded.');
        end
        D=inf(n+1);D(1:n+2:end)=0;all_poses=[scene.start_pose;poses];
        lambda=0;if isfield(planner,'edge_cost'),lambda=field_or(planner.edge_cost,'lambda_theta',0);end
        validateattributes(lambda,{'numeric'},{'real','finite','scalar','nonnegative'});
        planner.robot_cfg=robot;
        for a=required
            for b=required
                if a==b,continue;end
                [~,length_m,ok]=query_path_with_shared_cache(scene.grid_map, ...
                    all_poses(a,:),all_poses(b,:),scene.map_info,planner);
                navigation_queries=navigation_queries+1;
                if ok&&isfinite(length_m)&&length_m<=1e6
                    D(a,b)=length_m+lambda*abs(wrapToPiLocal(all_poses(b,3)-all_poses(a,3)));
                end
            end
        end
        cost_source='directed_actual_paths_plus_wrapped_endpoint_yaw';
    end
    solution=solve_minimum_cover_route(covers,D,1,settings);
    if ~solution.feasible
        plan=empty_plan(solution.reason);plan.joint.route_solution=solution;return;
    end
    selected=solution.selected_indices-1;
    optimal=zeros(0,3);pose_to_region=[];summaries={};info_list={};
    for ri=1:numel(regions)
        chosen=selected(owner(selected)==ri);remaining=reshape(regions(ri).task_ids,1,[]);
        region_poses=poses(chosen,:);infos=struct([]);
        for ci=chosen
            candidate=flat{ci};assigned=reshape(intersect(remaining,candidate.covered_tasks,'stable'),1,[]);
            remaining=setdiff(remaining,assigned,'stable');
            map=containers.Map('KeyType','double','ValueType','char');
            for tid=assigned
                ti=find(regions(ri).task_ids==tid,1);
                arm=candidate.arm_assignment{ti};
                if ~any(strcmp(arm,{'left','right','either'}))
                    error('plan_task_navigation_joint:CandidateAssignment','Supported task has no valid arm assignment.');
                end
                map(tid)=arm;
            end
            scores=candidate.reach_scores;
            if isfield(candidate,'nominal_reach_scores'),scores=candidate.nominal_reach_scores;end
            score=scores(ismember(regions(ri).task_ids,assigned),3);
            mean_score=NaN;min_score=NaN;
            if ~isempty(score),mean_score=mean(score);min_score=min(score);end
            entry=struct('pose',candidate.pose,'covered_tasks',assigned, ...
                'covered_count',numel(assigned),'arm_assign_map',map,'table_id',regions(ri).table_id, ...
                'side_id',candidate.side_id,'cost_breakdown',candidate.cost_breakdown, ...
                'remaining_after',numel(remaining),'nominal_reachability_summary', ...
                struct('mean',mean_score,'minimum',min_score), ...
                'robust_reachability_summary',struct('mean_success_fraction',NaN,'minimum_success_fraction',NaN));
            if isempty(infos),infos=entry;else,infos(end+1)=entry;end %#ok<AGROW>
        end
        if ~isempty(remaining)
            error('plan_task_navigation_joint:InternalCoverage','Selected cover lost task coverage.');
        end
        [optimal,pose_to_region,summaries,info_list]=append_region_plan( ...
            optimal,pose_to_region,summaries,info_list,ri,regions(ri),region_poses,infos,planner);
    end
    [present,preferred]=ismember(solution.visit_order,[1,selected+1]);
    assert(all(present),'Joint route nodes do not match selected poses.');
    timing=struct('local_selection_time_s',toc(started),'strategy_total_time_s',toc(started));
    diagnostics=struct('route_context_queries',navigation_queries);
    plan=build_unified_plan('task_navigation_joint',optimal,pose_to_region,summaries, ...
        info_list,1:numel(regions),preferred,library,scene,planner,timing,diagnostics);
    plan.covered_tasks_predicted=reshape(plan.covered_tasks_predicted,1,[]);
    plan.uncovered_tasks_predicted=reshape(plan.uncovered_tasks_predicted,1,[]);
    plan.candidate_library=library;
    plan.joint=joint_metadata();
    plan.joint.candidate_indices=selected;
    plan.joint.matrix_node_indices=solution.selected_indices;
    plan.joint.matrix_visit_order=solution.visit_order;
    plan.joint.route_cost=solution.route_cost;
    plan.joint.minimum_dock_count=sum(cellfun(@(c)c.minimum_dock_count,certificates));
    plan.joint.selected_dock_count=numel(selected);
    plan.joint.lexicographic_value=[0,numel(selected),solution.route_cost];
    plan.joint.route_solution=solution;
    plan.joint.cost_source=cost_source;
    plan.joint.navigation_queries=navigation_queries;
    plan.joint.selection_optimal=strcmp(control,'joint');
    plan.joint.route_order_optimal=true;
    validate_plan_consistency(plan,scene);

    function out=empty_plan(reason)
        out=build_unified_plan('task_navigation_joint',zeros(0,3),[],{}, {}, ...
            [],[],library,scene,planner,struct(),struct());
        out.status='BLOCKED';out.reason=reason;out.candidate_library=library;
        out.joint=joint_metadata();out.joint.candidate_indices=[];out.joint.route_cost=Inf;
    end
    function metadata=joint_metadata()
        metadata=struct('control',control,'region_certificates',{certificates}, ...
            'scope',['Hamiltonian closed tours on the finite minimum-cover layer; each selected dock visited once; ' ...
                'no finite tour means BLOCKED without larger-cover or repeated-dock search; ' ...
                'not continuous-space or physical optimality'], ...
            'objective_order',{{'full_task_support','minimum_dock_count','closed_tour_navigation_edge_cost'}});
        if strcmp(control,'reorder_only')
            metadata.objective_order={'fixed_candidate_set','closed_tour_navigation_edge_cost'};
        end
    end
end

function out=merge_options(out,in)
    names=fieldnames(in);for k=1:numel(names),out.(names{k})=in.(names{k});end
end
function value=field_or(s,name,fallback)
    if isfield(s,name),value=s.(name);else,value=fallback;end
end
