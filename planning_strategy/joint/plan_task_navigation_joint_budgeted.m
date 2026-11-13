function plan=plan_task_navigation_joint_budgeted(library,regions,scene,robot,planner,options)
%PLAN_TASK_NAVIGATION_JOINT_BUDGETED Fair regional scans with a priced-tour reserve.
% Same finite minimum covers; opt-in alternative to the historical planner.
% Initial complete fixed-dock graph is a common input. Each method counts all
% its other directed requests, even if an external oracle reuses a cache.
    if nargin<6,options=struct();end
    defaults=struct('max_passes',3,'max_navigation_queries',1500, ...
        'max_neighbor_evaluations',10000,'max_wall_time_s',120, ...
        'max_variants_per_region',12,'improvement_tolerance',1e-9);
    names=fieldnames(options);s=defaults;
    for k=1:numel(names),s.(names{k})=options.(names{k});end
    for name={'max_passes','max_navigation_queries','max_neighbor_evaluations','max_variants_per_region'}
        validateattributes(s.(name{1}),{'numeric'},{'scalar','integer','positive','finite'});
    end
    validateattributes(s.max_wall_time_s,{'numeric'},{'scalar','positive','finite'});
    start=tic;D=s.cost_matrix;known=logical(s.known_edge_mask);route=reshape(s.initial_matrix_visit_order,1,[]);
    % Reuse the established validator/plan builder; this never calls IK.
    seed_opts=s;seed_opts.control='minimum_seed';seed_opts.max_wall_time_s=s.max_wall_time_s;
    seed=plan_task_navigation_joint_local(library,regions,scene,robot,planner,seed_opts);
    route=seed.joint_local.matrix_visit_order;cost=seed.joint_local.route_cost;
    initial=route;initial_cost=cost;n=size(D,1);owner=zeros(1,n);poses=zeros(n,3);poses(1,:)=scene.start_pose;
    covers=cell(1,numel(regions));cursor=1;
    for ri=1:numel(regions)
        candidates=library.regions(ri).candidates;idx=cursor+(1:numel(candidates));
        owner(idx)=ri;poses(idx,:)=reshape([candidates.pose],3,[])';
        local=enumerate_minimum_task_covers(candidates,regions(ri).task_ids);
        k=size(local,2);orders=sortrows(perms(1:k));C=zeros(size(local,1)*size(orders,1),k);at=0;
        for row=1:size(local,1)
            for oi=1:size(orders,1),at=at+1;C(at,:)=local(row,orders(oi,:))+cursor;end
        end
        covers{ri}=C;cursor=cursor+numel(candidates);
    end
    queries=0;neighbors=0;accepted=struct('operation',{},'region_id',{},'previous_cost',{},'new_cost',{}, ...
        'previous_matrix_visit_order',{},'matrix_visit_order',{});
    log=struct('pass',{},'region_id',{},'query_allowance',{},'queries',{},'neighbors',{},'reserve_edges',{});
    stop='regional_pass_budget';order_converged=false;selection_queries=0;passes=0;
    % Leave at least one full ordering-neighborhood evaluation round.
    m=numel(route);order_round=(m-1)*(m-2)/2+(m-1)*(m-2);
    selection_neighbor_limit=max(0,s.max_neighbor_evaluations-order_round);
    selection_deadline=.75*s.max_wall_time_s;
    lambda=planner.edge_cost.lambda_theta;
    try
    for pass=1:s.max_passes
        if toc(start)>=selection_deadline,stop='selection_time_reserve';break;end
        for ri=1:numel(regions)
            if toc(start)>=selection_deadline||neighbors>=selection_neighbor_limit,break;end
            slots=find(owner(route)==ri);C=covers{ri};proxy=zeros(size(C,1),1);
            for vi=1:size(C,1)
                trial=route;trial(slots)=C(vi,:);closed=[trial,1];delta=diff(poses(closed,:));
                proxy(vi)=sum(hypot(delta(:,1),delta(:,2)))+lambda*sum(abs(wrapToPiLocal(delta(:,3))));
            end
            [~,rank]=sortrows([proxy,(1:numel(proxy))'],[1 2]);
            visits_left=(s.max_passes-pass)*numel(regions)+numel(regions)-ri+1;
            reserve=missing_graph(route);allowance=max(0,floor((s.max_navigation_queries-queries-reserve)/visits_left));
            before_q=queries;before_n=neighbors;best_route=route;best_cost=cost;tested=0;
            for vi=reshape(rank,1,[])
                if toc(start)>=selection_deadline||neighbors>=selection_neighbor_limit||tested>=s.max_variants_per_region,break;end
                trial=route;trial(slots)=C(vi,:);if isequal(trial,route),continue;end
                required=missing_tour(trial);
                % Reserve enough to finish either current or winning selected
                % graph after this trial, including failed trial edge requests.
                if required+queries-before_q>allowance,continue;end
                reserve=max([missing_graph(route),missing_graph(trial),missing_graph(best_route)]);
                if queries+required+reserve>s.max_navigation_queries,continue;end
                tested=tested+1;neighbors=neighbors+1;value=price(trial);
                if value<best_cost-s.improvement_tolerance,best_route=trial;best_cost=value;end
            end
            if best_cost<cost-s.improvement_tolerance,accept(best_route,best_cost,'balanced_regional_replacement',ri);end
            assert(queries+missing_graph(route)<=s.max_navigation_queries,'JointBudget:ReserveInvariant');
            log(end+1)=struct('pass',pass,'region_id',ri,'query_allowance',allowance, ...
                'queries',queries-before_q,'neighbors',neighbors-before_n,'reserve_edges',missing_graph(route)); %#ok<AGROW>
        end
        passes=pass;
    end
    catch err
        if strcmp(err.identifier,'JointBudget:Limit'),stop=err.message;else,rethrow(err);end
    end
    selection_queries=queries;preorder_cost=cost;
    try
        % Complete the final directed graph once, then order search has no
        % navigation callbacks and cannot be starved by regional enumeration.
        for a=route
            for b=route
                check_budget();edge(a,b);
            end
        end
        while true
            best_route=route;best_cost=cost;m=numel(route);
            for a=2:m-1
                for b=a+1:m
                    trial=route;trial(a:b)=fliplr(trial(a:b));evaluate_order(trial);
                end
            end
            for a=2:m
                reduced=route;v=reduced(a);reduced(a)=[];
                for after=1:numel(reduced)
                    trial=[reduced(1:after),v,reduced(after+1:end)];
                    if ~isequal(trial,route),evaluate_order(trial);end
                end
            end
            if best_cost>=cost-s.improvement_tolerance,order_converged=true;stop='fixed_set_order_neighborhood_converged';break;end
            accept(best_route,best_cost,'final_directed_order_descent',0);
        end
    catch err
        if strcmp(err.identifier,'JointBudget:Limit'),stop=err.message;else,rethrow(err);end
    end
    final_opts=struct('control','minimum_seed','cost_matrix',D,'known_edge_mask',known, ...
        'edge_cost_callback',s.edge_cost_callback,'initial_candidate_indices',route(2:end)-1, ...
        'initial_matrix_visit_order',route,'max_wall_time_s',120);
    plan=plan_task_navigation_joint_local(library,regions,scene,robot,planner,final_opts);
    j=plan.joint_local;j.control='joint_budgeted';j.input_matrix_visit_order=initial;j.input_candidate_indices=initial(2:end)-1;
    j.input_seed_cost=initial_cost;j.minimum_cover_seed_cost=initial_cost;j.minimum_cover_seed_matrix_visit_order=initial;
    j.oracle_queries=queries;j.neighbor_evaluations=neighbors;j.accepted_moves=accepted;
    j.termination_reason=stop;j.local_optimum_certified=false;j.fixed_set_order_converged=order_converged;
    j.passes_completed=passes;j.selection_queries=selection_queries;j.final_order_queries=queries-selection_queries;
    j.preorder_cost=preorder_cost;j.regional_budget_log=log;j.elapsed_s=toc(start);
    j.cost_matrix=D;j.known_edge_mask=known;j.global_optimal=false;
    plan.joint_local=j;plan.strategy_name='task_navigation_joint_budgeted';

    function count=missing_graph(nodes)
        count=nnz(~known(nodes,nodes));
    end
    function count=missing_tour(nodes)
        closed=[nodes,1];ids=sub2ind(size(known),closed(1:end-1),closed(2:end));count=nnz(~known(ids));
    end
    function check_budget()
        if toc(start)>=s.max_wall_time_s,error('JointBudget:Limit','wall_time_budget_exhausted');end
    end
    function value=edge(a,b)
        check_budget();
        if known(a,b),value=D(a,b);return;end
        if queries>=s.max_navigation_queries,error('JointBudget:Limit','navigation_query_budget_exhausted');end
        queries=queries+1;value=s.edge_cost_callback(a,b);
        if ~isnumeric(value)||~isscalar(value)||isnan(value)||value<0,error('JointBudget:Oracle','Invalid edge value.');end
        known(a,b)=true;D(a,b)=value;
    end
    function value=price(nodes)
        closed=[nodes,1];value=0;
        for ei=1:numel(nodes),value=value+edge(closed(ei),closed(ei+1));end
    end
    function evaluate_order(trial)
        check_budget();if neighbors>=s.max_neighbor_evaluations,error('JointBudget:Limit','neighbor_evaluation_budget_exhausted');end
        neighbors=neighbors+1;closed=[trial,1];value=sum(D(sub2ind(size(D),closed(1:end-1),closed(2:end))));
        if value<best_cost-s.improvement_tolerance,best_route=trial;best_cost=value;end
    end
    function accept(nodes,value,operation,ri)
        accepted(end+1)=struct('operation',operation,'region_id',ri,'previous_cost',cost,'new_cost',value, ...
            'previous_matrix_visit_order',route,'matrix_visit_order',nodes);
        route=nodes;cost=value;
    end
end
