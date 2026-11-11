function plan=plan_task_navigation_joint_local(library,regions,scene,robot,planner,options)
%PLAN_TASK_NAVIGATION_JOINT_LOCAL Budgeted descent on regional minimum covers.
% Retain exact regional minimum dock counts. Replace one complete regional
% cover in its current visit slots, then try directed tour segment reversals.
% Only strictly lower actual oracle costs are accepted. Proxy geometry ranks
% trials only; it never supplies an accepted cost or prunes a neighborhood.
% This is a heuristic local search, not a global tour/selection certificate.
%
% Matrix node 1 is start; nodes 2..N+1 flatten library regions/candidates.
% options.initial_candidate_indices are flattened indices 1..N.
% options.initial_matrix_visit_order includes start but omits final return.
% control is minimum_seed (repair only), reorder_only (fixed repaired set),
% or joint_local (regional replacement plus reversal, the default).
% options.edge_cost_callback(a,b) returns an actual weighted directed edge.
% A full cost_matrix may instead be supplied. A partial matrix plus callback
% requires known_edge_mask; Inf on a known edge means genuinely unreachable.
% Query/neighbor/time limits return the last fully priced feasible incumbent
% with an explicit stop reason. Exhaustion before a minimum-cover seed is
% priced fails explicitly. No exact/optimal label is inferred from a budget.
    if nargin<6,options=struct();end
    if ~isstruct(options)||~isscalar(options)
        error('plan_task_navigation_joint_local:Options','options must be a scalar struct.');
    end
    validate_candidate_library(library,regions,scene,robot,planner);
    ids=[];
    for ri=1:numel(regions),ids=[ids,reshape(regions(ri).task_ids,1,[])];end %#ok<AGROW>
    if ~isequal(sort(ids),1:scene.num_tasks)
        error('plan_task_navigation_joint_local:RegionTasks','Regions must partition all scene tasks exactly once.');
    end
    if ~isfield(planner,'execution')||~isfield(planner.execution,'return_to_start')||~planner.execution.return_to_start
        error('plan_task_navigation_joint_local:ClosedTourRequired','return_to_start=true is required.');
    end
    settings=struct('max_passes',3,'max_navigation_queries',1500, ...
        'max_neighbor_evaluations',10000,'max_wall_time_s',120, ...
        'max_region_docks',3,'improvement_tolerance',1e-9,'enable_two_opt',true);
    if isfield(planner,'strategy')&&isfield(planner.strategy,'joint_local')
        settings=merge_options(settings,planner.strategy.joint_local);
    end
    settings=merge_options(settings,options);
    control=field_or(settings,'control','joint_local');
    if ~any(strcmp(control,{'minimum_seed','reorder_only','joint_local'}))
        error('plan_task_navigation_joint_local:Control','Unknown local-search control.');
    end
    for name={'max_passes','max_navigation_queries','max_neighbor_evaluations','max_region_docks'}
        validateattributes(settings.(name{1}),{'numeric'},{'real','finite','scalar','integer','positive'});
    end
    validateattributes(settings.max_wall_time_s,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(settings.improvement_tolerance,{'numeric'},{'real','finite','scalar','nonnegative'});
    if ~isscalar(settings.enable_two_opt)||~any(settings.enable_two_opt==[0 1])
        error('plan_task_navigation_joint_local:Options','enable_two_opt must be logical.');
    end
    started=tic;nr=numel(regions);n=sum(arrayfun(@(r)numel(r.candidates),library.regions));
    flat=cell(1,n);poses=zeros(n,3);owner=zeros(1,n);offset=zeros(1,nr);cursor=0;
    for ri=1:nr
        offset(ri)=cursor;
        for ci=1:numel(library.regions(ri).candidates)
            cursor=cursor+1;flat{cursor}=library.regions(ri).candidates(ci);
            poses(cursor,:)=flat{cursor}.pose;owner(cursor)=ri;
        end
    end
    all_poses=[scene.start_pose;poses];
    covers=cell(1,nr);certificates=cell(1,nr);minimum=zeros(1,nr);
    for ri=1:nr
        [local,certificates{ri}]=enumerate_minimum_task_covers( ...
            library.regions(ri).candidates,regions(ri).task_ids,settings);
        if ~certificates{ri}.feasible
            plan=blocked_plan('TASK_UNSUPPORTED_ON_FINITE_POOL');return;
        end
        minimum(ri)=certificates{ri}.minimum_dock_count;
        if minimum(ri)>settings.max_region_docks
            error('plan_task_navigation_joint_local:BudgetExceeded','Regional permutation budget exceeded.');
        end
        covers{ri}=local+offset(ri)+1;
    end
    lambda=0;if isfield(planner,'edge_cost'),lambda=field_or(planner.edge_cost,'lambda_theta',0);end
    validateattributes(lambda,{'numeric'},{'real','finite','scalar','nonnegative'});
    callback=field_or(settings,'edge_cost_callback',[]);
    if ~isempty(callback)&&~isa(callback,'function_handle')
        error('plan_task_navigation_joint_local:Oracle','edge_cost_callback must be a function handle.');
    end
    if isfield(settings,'known_edge_mask')&&~isfield(settings,'cost_matrix')
        error('plan_task_navigation_joint_local:CostMatrix','known_edge_mask requires cost_matrix.');
    end
    if isfield(settings,'cost_matrix')
        matrix_cache=true;
        D=settings.cost_matrix;
        if ~isnumeric(D)||~isreal(D)||~isequal(size(D),[n+1,n+1])||any(isnan(D(:)))|| ...
                any(D(:)<0)||any(diag(D)~=0)
            error('plan_task_navigation_joint_local:CostMatrix','cost_matrix must be nonnegative, NaN-free and zero-diagonal.');
        end
        known=true(n+1);
        if isfield(settings,'known_edge_mask')
            known=settings.known_edge_mask;
            if ~isequal(size(known),size(D))||~isreal(known)|| ...
                    any(~isfinite(known(:)))||any(known(:)~=0&known(:)~=1)
                error('plan_task_navigation_joint_local:CostMatrix','known_edge_mask must match the matrix and contain only 0/1.');
            end
            known=logical(known);known(1:n+2:end)=true;
        end
        if ~all(known(:))&&isempty(callback)
            error('plan_task_navigation_joint_local:Oracle','A partial matrix requires an edge_cost_callback.');
        end
        oracle_source='injected_directed_matrix';
        if ~isempty(callback),oracle_source='injected_matrix_and_directed_callback';end
    else
        % Callback/default-oracle mode has O(queried edges) cache storage;
        % never allocate a full N-by-N matrix for a large candidate pool.
        matrix_cache=false;D=[];known=[];
        oracle_source='directed_actual_paths_plus_wrapped_endpoint_yaw';
        if ~isempty(callback),oracle_source='directed_edge_cost_callback';end
    end
    edge_memo=containers.Map('KeyType','char','ValueType','double');
    oracle_queries=0;cache_hits=0;neighbors=0;passes_completed=0;
    accepted=struct('operation',{},'region_id',{},'previous_cost',{},'new_cost',{}, ...
        'previous_matrix_visit_order',{},'matrix_visit_order',{});
    if isfield(settings,'initial_candidate_indices')
        initial=reshape(settings.initial_candidate_indices,1,[]);seed_source='supplied_baseline';
    else
        initial=[];
        for ri=1:nr,initial=[initial,covers{ri}(1,:)-1];end %#ok<AGROW>
        seed_source='first_regional_minimum_cover';
    end
    if ~isnumeric(initial)||~isreal(initial)||any(~isfinite(initial))|| ...
            any(initial~=fix(initial))||any(initial<1)||any(initial>n)||numel(unique(initial))~=numel(initial)
        error('plan_task_navigation_joint_local:Seed','Initial candidates must be unique flattened indices.');
    end
    for ri=1:nr
        selected=initial(owner(initial)==ri);covered=[];
        for ci=selected,covered=union(covered,flat{ci}.covered_tasks);end
        if ~all(ismember(regions(ri).task_ids,covered))
            error('plan_task_navigation_joint_local:SeedCoverage','Initial candidates must cover all tasks.');
        end
    end
    route=[1,initial+1];
    if isfield(settings,'initial_matrix_visit_order')
        route=reshape(settings.initial_matrix_visit_order,1,[]);
        if ~isnumeric(route)||~isreal(route)||any(~isfinite(route))|| ...
                numel(route)~=numel(initial)+1||isempty(route)||route(1)~=1|| ...
                ~isequal(sort(route),sort([1,initial+1]))
            error('plan_task_navigation_joint_local:SeedOrder','Initial order must start at 1 and contain exactly the supplied candidates.');
        end
    end
    input_route=route;input_candidate_indices=initial;
    try
        input_cost=tour_cost(route);
        if ~isfinite(input_cost),plan=blocked_plan('NO_FINITE_INITIAL_TOUR');return;end
        repaired=[];
        for ri=1:nr
            slots=find(node_owners(route)==ri);
            if numel(slots)==minimum(ri),continue;end
            original=route(slots);C=covers{ri};
            overlap=sum(ismember(C,original),2);
            rows=find(overlap==max(overlap));replacement=C(rows(1),:);
            % Retained baseline nodes stay at their original relative tour
            % positions. New nodes occupy the earliest discarded regional
            % slots; only excess discarded slots are removed.
            discarded=slots(~ismember(original,replacement));
            added=replacement(~ismember(replacement,original));
            route(discarded(1:numel(added)))=added;
            route(discarded(numel(added)+1:end))=[];
            repaired(end+1)=ri; %#ok<AGROW>
        end
        cost=tour_cost(route);
        if ~isfinite(cost),plan=blocked_plan('NO_FINITE_REPAIRED_MINIMUM_COVER_SEED');return;end
    catch err
        if strcmp(err.identifier,'plan_task_navigation_joint_local:SearchBudgetReached')
            error('plan_task_navigation_joint_local:InitialBudgetExceeded', ...
                'No fully priced regional-minimum seed before budget exhaustion: %s.',err.message);
        end
        rethrow(err);
    end
    seed_route=route;seed_cost=cost;termination='max_passes';local_optimum=false;
    if strcmp(control,'minimum_seed')
        termination='minimum_seed_control';
    else
    try
        for pass=1:settings.max_passes
            improved=false;
            if strcmp(control,'joint_local')
            for ri=1:nr
                check_time();slots=find(node_owners(route)==ri);C=covers{ri};k=minimum(ri);
                if k==0,continue;end
                orders=sortrows(perms(1:k));variants=zeros(size(C,1)*size(orders,1),k);at=0;
                for row=1:size(C,1)
                    check_time();
                    for oi=1:size(orders,1),at=at+1;variants(at,:)=C(row,orders(oi,:));end
                end
                proxy=zeros(size(variants,1),1);
                for vi=1:size(variants,1)
                    trial=route;trial(slots)=variants(vi,:);proxy(vi)=proxy_cost(trial);
                end
                [~,trial_order]=sortrows([proxy,(1:size(variants,1))'],[1 2]);
                for vi=reshape(trial_order,1,[])
                    trial=route;trial(slots)=variants(vi,:);
                    if isequal(trial,route),continue;end
                    candidate_cost=neighbor_cost(trial);
                    if candidate_cost<cost-settings.improvement_tolerance
                        accept_move(trial,candidate_cost,'regional_minimum_cover_replacement',ri);
                        improved=true;break;
                    end
                end
            end
            end
            if settings.enable_two_opt&&numel(route)>2
                pairs=nchoosek(2:numel(route),2);proxy=zeros(size(pairs,1),1);
                for pi=1:size(pairs,1)
                    trial=route;segment=pairs(pi,1):pairs(pi,2);trial(segment)=fliplr(trial(segment));
                    proxy(pi)=proxy_cost(trial);
                end
                [~,trial_order]=sortrows([proxy,(1:size(pairs,1))'],[1 2]);
                for pi=reshape(trial_order,1,[])
                    trial=route;segment=pairs(pi,1):pairs(pi,2);trial(segment)=fliplr(trial(segment));
                    candidate_cost=neighbor_cost(trial);
                    if candidate_cost<cost-settings.improvement_tolerance
                        accept_move(trial,candidate_cost,'directed_two_opt',0);
                        improved=true;break;
                    end
                end
            end
            passes_completed=pass;
            if ~improved,termination='complete_neighborhood_pass_without_improvement';local_optimum=true;break;end
        end
    catch err
        if strcmp(err.identifier,'plan_task_navigation_joint_local:SearchBudgetReached')
            termination=err.message;local_optimum=false;
        else,rethrow(err);end
    end
    end
    selected=sort(route(2:end)-1);optimal=zeros(0,3);pose_to_region=[];summaries={};info_list={};
    for ri=1:nr
        chosen=selected(owner(selected)==ri);remaining=reshape(regions(ri).task_ids,1,[]);infos=struct([]);
        if numel(chosen)~=minimum(ri)
            error('plan_task_navigation_joint_local:InternalInvariant','Regional minimum cardinality changed.');
        end
        for ci=chosen
            c=flat{ci};assigned=reshape(intersect(remaining,c.covered_tasks,'stable'),1,[]);
            remaining=setdiff(remaining,assigned,'stable');map=containers.Map('KeyType','double','ValueType','char');
            for tid=assigned
                ti=find(regions(ri).task_ids==tid,1);arm=c.arm_assignment{ti};
                if ~any(strcmp(arm,{'left','right','either'}))
                    error('plan_task_navigation_joint_local:CandidateAssignment','Supported task lacks an arm assignment.');
                end
                map(tid)=arm;
            end
            scores=c.reach_scores;if isfield(c,'nominal_reach_scores'),scores=c.nominal_reach_scores;end
            s=scores(ismember(regions(ri).task_ids,assigned),3);average=NaN;lowest=NaN;
            if ~isempty(s),average=mean(s);lowest=min(s);end
            entry=struct('pose',c.pose,'covered_tasks',assigned,'covered_count',numel(assigned), ...
                'arm_assign_map',map,'table_id',regions(ri).table_id,'side_id',c.side_id, ...
                'cost_breakdown',c.cost_breakdown,'remaining_after',numel(remaining), ...
                'nominal_reachability_summary',struct('mean',average,'minimum',lowest), ...
                'robust_reachability_summary',struct('mean_success_fraction',NaN,'minimum_success_fraction',NaN));
            if isempty(infos),infos=entry;else,infos(end+1)=entry;end %#ok<AGROW>
        end
        if ~isempty(remaining),error('plan_task_navigation_joint_local:InternalInvariant','Task coverage was lost.');end
        [optimal,pose_to_region,summaries,info_list]=append_region_plan(optimal,pose_to_region,summaries, ...
            info_list,ri,regions(ri),poses(chosen,:),infos,planner);
    end
    [present,preferred]=ismember(route,[1,selected+1]);assert(all(present));
    timing=struct('local_selection_time_s',toc(started),'strategy_total_time_s',toc(started));
    plan=build_unified_plan('task_navigation_joint_local',optimal,pose_to_region,summaries,info_list, ...
        1:nr,preferred,library,scene,planner,timing,struct('route_context_queries',oracle_queries,'cache_hits',cache_hits));
    plan.covered_tasks_predicted=reshape(plan.covered_tasks_predicted,1,[]);
    plan.uncovered_tasks_predicted=reshape(plan.uncovered_tasks_predicted,1,[]);
    plan.candidate_library=library;
    plan.joint_local=struct('control',control,'candidate_indices',selected,'matrix_visit_order',route,'route_cost',cost, ...
        'input_candidate_indices',input_candidate_indices,'input_matrix_visit_order',input_route,'input_seed_cost',input_cost, ...
        'minimum_cover_seed_matrix_visit_order',seed_route,'minimum_cover_seed_cost',seed_cost, ...
        'seed_source',seed_source,'seed_repaired',~isempty(repaired),'seed_repair_regions',repaired, ...
        'minimum_dock_count',sum(minimum),'regional_minimum_dock_counts',minimum, ...
        'region_certificates',{certificates},'lexicographic_value',[0,numel(selected),cost], ...
        'accepted_moves',accepted,'oracle_queries',oracle_queries,'cache_hits',cache_hits, ...
        'neighbor_evaluations',neighbors,'passes_completed',passes_completed,'termination_reason',termination, ...
        'global_optimal',false,'regional_minimum_cardinality_exact',true, ...
        'local_optimum_certified',local_optimum,'oracle_source',oracle_source, ...
        'oracle_queries_semantics','new directed edge requests; a supplied callback may itself hit an external cache', ...
        'enabled_two_opt',logical(settings.enable_two_opt),'wall_time_limit_semantics','cooperative check between oracle calls and trial generation', ...
        'scope',['heuristic strict descent on regional minimum-cover selections and directed tours; ' ...
            'local-optimum flag applies only to a complete tested replacement/reversal pass; ' ...
            'no global, continuous-space, arm-collision or physical optimality claim']);
    if field_or(settings,'return_cost_cache',false)
        if matrix_cache
            plan.joint_local.cost_matrix=D;plan.joint_local.known_edge_mask=known;
        else
            plan.joint_local.edge_cache_keys=keys(edge_memo);
            plan.joint_local.edge_cache_values=cell2mat(values(edge_memo));
        end
    end
    validate_plan_consistency(plan,scene);

    function owners=node_owners(nodes)
        owners=zeros(size(nodes));mask=nodes>1;owners(mask)=owner(nodes(mask)-1);
    end
    function check_time()
        if toc(started)>=settings.max_wall_time_s
            error('plan_task_navigation_joint_local:SearchBudgetReached','wall_time_budget_exhausted');
        end
    end
    function value=edge_cost(a,b)
        check_time();
        if a==b,cache_hits=cache_hits+1;value=0;return;end
        key=sprintf('%d>%d',a,b);
        if matrix_cache&&known(a,b),cache_hits=cache_hits+1;value=D(a,b);return;end
        if ~matrix_cache&&isKey(edge_memo,key),cache_hits=cache_hits+1;value=edge_memo(key);return;end
        if oracle_queries>=settings.max_navigation_queries
            error('plan_task_navigation_joint_local:SearchBudgetReached','navigation_query_budget_exhausted');
        end
        oracle_queries=oracle_queries+1;
        if ~isempty(callback)
            value=callback(a,b);
        else
            nav=planner;nav.robot_cfg=robot;
            [~,distance,ok]=query_path_with_shared_cache(scene.grid_map,all_poses(a,:),all_poses(b,:),scene.map_info,nav);
            value=Inf;
            if ok&&isfinite(distance)&&distance<=1e6
                value=distance+lambda*abs(wrapToPiLocal(all_poses(b,3)-all_poses(a,3)));
            end
        end
        if ~isnumeric(value)||~isreal(value)||~isscalar(value)||isnan(value)||value<0
            error('plan_task_navigation_joint_local:Oracle','Oracle must return a nonnegative scalar cost or Inf.');
        end
        if matrix_cache,D(a,b)=value;known(a,b)=true;
        else,edge_memo(key)=value;end
    end
    function value=tour_cost(nodes)
        closed=[nodes,1];value=0;
        for ei=1:numel(closed)-1
            value=value+edge_cost(closed(ei),closed(ei+1));
            if ~isfinite(value),return;end
        end
    end
    function value=neighbor_cost(nodes)
        check_time();
        if neighbors>=settings.max_neighbor_evaluations
            error('plan_task_navigation_joint_local:SearchBudgetReached','neighbor_evaluation_budget_exhausted');
        end
        neighbors=neighbors+1;value=tour_cost(nodes);
    end
    function value=proxy_cost(nodes)
        closed=[nodes,1];delta=diff(all_poses(closed,:));
        value=sum(hypot(delta(:,1),delta(:,2)))+lambda*sum(abs(wrapToPiLocal(delta(:,3))));
    end
    function accept_move(nodes,value,operation,ri)
        event=struct('operation',operation,'region_id',ri,'previous_cost',cost,'new_cost',value, ...
            'previous_matrix_visit_order',route,'matrix_visit_order',nodes);
        accepted(end+1)=event;route=nodes;cost=value;
    end
    function out=blocked_plan(reason)
        out=build_unified_plan('task_navigation_joint_local',zeros(0,3),[],{}, {},[],[], ...
            library,scene,planner,struct(),struct());
        out.status='BLOCKED';out.reason=reason;out.candidate_library=library;
        out.joint_local=struct('global_optimal',false,'termination_reason',reason, ...
            'region_certificates',{certificates});
    end
end

function out=merge_options(out,in)
    names=fieldnames(in);for k=1:numel(names),out.(names{k})=in.(names{k});end
end
function value=field_or(s,name,fallback)
    if isfield(s,name),value=s.(name);else,value=fallback;end
end
