function [result,next_state,diagnostics] = execute_replanning_action(original,changed,event,impact,action)
%EXECUTE_REPLANNING_ACTION Validate a detached candidate before atomic commit.
    if nargin<4, impact=[]; end
    if nargin<5, action=''; end
    original=normalize_planning_state(original);
    changed=normalize_planning_state(changed);
    [mask,md]=evaluate_replanning_action_mask(original,changed,event,impact);
    diagnostics=struct('action_mask',md,'low_level',struct(),'state_validation',struct(),'committed',false);
    result=struct('requested_action',upper(char(action)),'executed_action','', ...
        'status','ACTION_REJECTED','fallback',true,'fallback_reason','', ...
        'action_legal',false,'path_safe',false,'task_coverage_valid',false, ...
        'state_transition_valid',false,'metric_provenance','validated_candidate_state');
    next_state=paused_state(original,changed,event);
    idx=find(strcmp(md.actions,result.requested_action),1);
    if strcmp(md.event_type,'ADD_TASKS')
        result.status='UNSUPPORTED_EVENT'; result.fallback_reason='ADD_TASKS is a legacy placeholder'; return;
    end
    if ~any(mask)
        result.status='NO_LEGAL_ACTION'; result.fallback_reason=strjoin(md.reasons,'; '); return;
    end
    if isempty(idx)||~mask(idx)
        result.fallback_reason='requested action is not legal'; return;
    end
    result.action_legal=true; result.executed_action=result.requested_action;
    try
        if idx==1
            candidate=clone_value(original); candidate.scene=changed.scene;
            candidate.tasks=changed.scene.tasks;
            candidate.invalidated_coverage_pairs=next_state.invalidated_coverage_pairs;
            low=struct('status','NOOP','fallback',false,'fallback_reason','');
        else
            % containers.Map is a handle: detach before low-level arm merges.
            low_input=clone_value(original); low_input.unified_action_validation=true;
            [low,candidate,ld]=local_replan(low_input,clone_value(next_state),event,md.impact);
            diagnostics.low_level=ld;
        end
        diagnostics.low_level_result=rm_nested(low);
        result.status=low.status; result.fallback=low.fallback; result.fallback_reason=low.fallback_reason;
        if ~any(strcmp(low.status,{'SUCCESS','NOOP'})), return; end
        assert(isequaln(candidate.scene,changed.scene),'candidate changed event facts');
        if idx~=1, candidate=complete_replanning_route(candidate); end
        candidate.invalidated_coverage_pairs=unique([next_state.invalidated_coverage_pairs; ...
            pairs_of(candidate)],'rows','stable');
        if ~isfield(candidate,'current_plan')
            candidate=publish_aliases(candidate);
        end
        % Dock/task changes and KEEP cannot attest old search states to a new route.
        if idx~=2, candidate=clear_search(candidate); end
        v=validate_replanning_state(candidate,original);
        diagnostics.state_validation=v;
        if idx==2&&v.valid
            validate_search(candidate,diagnostics.low_level);
        end
        if ~v.valid
            result.status='FALLBACK_REQUIRED'; result.fallback=true;
            result.fallback_reason=['candidate validation: ' v.reason]; return;
        end
        candidate=publish_aliases(candidate);
        candidate.execution_allowed=true; candidate.replanning_status='READY';
        candidate.path_validation=v;
        candidate.map_signature=build_struct_signature(candidate.scene.grid_map);
        candidate.current_plan.map_signature=candidate.map_signature;
        candidate.current_plan.path_validation=v;
        next_state=candidate; diagnostics.committed=true;
        result.path_safe=v.path_safe; result.task_coverage_valid=v.task_coverage_valid;
        result.state_transition_valid=true;
    catch err
        result.status='FALLBACK_REQUIRED'; result.fallback=true; result.fallback_reason=err.message;
    end
end

function s=paused_state(original,changed,event)
    s=clone_value(original); s.scene=changed.scene; s.tasks=changed.scene.tasks;
    pairs=[pairs_of(original);pairs_of(changed)];
    if isfield(event,'invalidated_coverage_pairs'), pairs=[pairs;event.invalidated_coverage_pairs]; end
    s.invalidated_coverage_pairs=unique(pairs,'rows','stable');
    s.execution_allowed=false; s.replanning_status='PAUSED_REPAIR_REQUIRED';
    s.path_validation=struct('valid',false,'reason','not committed on changed environment');
    s=clear_search(s);
    if isfield(s,'current_plan'), s.current_plan.execution_allowed=false; end
end

function s=publish_aliases(s)
    s.current_plan=s.plan;
    s.current_plan.global_path=s.stats.global_path;
    s.current_plan.path_segments=s.stats.segment_stats;
    s.current_plan.route_order=s.stats.route_order;
    s.current_plan.sparse_graph=s.stats.sparse_graph;
    s.global_path=s.stats.global_path; s.path_segments=s.stats.segment_stats;
    s.route_order=s.stats.route_order; s.sparse_graph=s.stats.sparse_graph;
    s.stats.path_len=compute_path_length(s.stats.global_path);
end

function s=clear_search(s)
    if isfield(s.stats,'path_search_states'), s.stats=rmfield(s.stats,'path_search_states'); end
    if isfield(s.stats.sparse_graph,'path_search_states')
        s.stats.sparse_graph=rmfield(s.stats.sparse_graph,'path_search_states');
    end
    s.sparse_graph=s.stats.sparse_graph;
    if isfield(s,'current_plan'), s.current_plan.sparse_graph=s.sparse_graph; end
end

function validate_search(s,d)
    if ~isfield(d,'incremental_dstar'), return; end
    edges=d.incremental_dstar.edge_results;
    for k=1:numel(edges)
        e=edges(k); idx=[];
        if isfield(s.stats,'path_search_states')
            idx=find(strcmp(s.stats.path_search_states.keys,e.key),1);
        end
        if e.fallback_used
            assert(isempty(idx)&&~e.state_valid,'fallback retained D* state');
        elseif e.state_valid
            assert(~isempty(idx),'missing committed D* state');
            q=s.stats.path_search_states.states{idx};
            assert(q.search_state_valid&&q.validated_path_safe&&q.state_version==e.state_version,'invalid D* provenance');
            map=s.scene.grid_map;
            if ~isfield(s.planner_cfg,'footprint_aware_navigation')||s.planner_cfg.footprint_aware_navigation
                map=build_footprint_inflated_map(map,s.robot_cfg.footprint,s.scene.map_info);
            end
            % prepare_start_goal opens endpoint cells, matching the existing planner.
            map(q.start(1),q.start(2))=0; map(q.goal(1),q.goal(2))=0;
            assert(isequal(q.traversable,map==0),'search state belongs to stale map');
        end
    end
end

function p=pairs_of(s)
    p=zeros(0,2);
    if isfield(s,'invalidated_coverage_pairs'), p=s.invalidated_coverage_pairs; end
end

function x=clone_value(x)
    if isa(x,'containers.Map')
        old=x; x=containers.Map('KeyType',old.KeyType,'ValueType',old.ValueType);
        keys=old.keys;
        for k=1:numel(keys), x(keys{k})=clone_value(old(keys{k})); end
    elseif isstruct(x)
        names=fieldnames(x);
        for j=1:numel(x)
            for k=1:numel(names), x(j).(names{k})=clone_value(x(j).(names{k})); end
        end
    elseif iscell(x)
        for k=1:numel(x), x{k}=clone_value(x{k}); end
    end
end

function s=rm_nested(s)
    names=intersect(fieldnames(s),{'next_state','diagnostics'});
    if ~isempty(names), s=rmfield(s,names); end
end
