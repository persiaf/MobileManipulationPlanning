function result = demo_dynamic_replanning(event_type, seed)
%DEMO_DYNAMIC_REPLANNING Repair the actual current V3 planner state.
    if nargin < 1 || isempty(event_type), event_type='ADD_LOCAL_OBSTACLE'; end
    if nargin < 2 || isempty(seed), seed=2026; end
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    override.planner.strategy.name='bottom_up';
    override.planner.routing.mode='sparse_knn';
    override.planner.routing.sparse_knn.k=4;
    override.planner.candidate_pair_prefilter.enable=false;
    effective=resolve_effective_config('profile','V3','seed',seed,'experiment_override',override);
    state=load_current_demo_state('V3',seed,effective.planner);
    if ~strcmpi(state.plan.status,'READY') || isempty(state.stats.global_path)
        error('demo_dynamic_replanning:BlockedStaticPlan', ...
            'Current V3 scene has no complete static route; inspect candidate/task coverage before replanning.');
    end
    [changed_state,event]=apply_dynamic_event(state,event_type,1);
    impact=analyze_change_impact(state,event);
    local_result=local_replan(state,changed_state,event,impact);
    figures=visualize_dynamic_replan(state,event,local_result,effective.visualization, ...
        'title','V3 Dynamic Local Replanning','prefix','V3_dynamic');
    if isfield(local_result,'next_state'), after=local_result.next_state; else, after=changed_state; end
    result=struct('state_before',state,'state_after',after,'event',event, ...
        'impact',impact,'local_result',local_result,'figures',figures);
end
