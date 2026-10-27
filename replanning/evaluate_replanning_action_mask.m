function [mask,d] = evaluate_replanning_action_mask(original,changed,event,impact)
%EVALUATE_REPLANNING_ACTION_MASK Legality permits an attempt, not success.
    actions={'KEEP','REPLAN_PATH','SWITCH_DOCK','REALLOCATE_TASK'};
    mask=false(1,4); applicable=false(1,4);
    reasons=repmat({'unsupported or incomplete event/state'},1,4);
    d=struct('actions',{actions},'applicable',applicable,'legal',mask,'reasons',{reasons}, ...
        'event_type','','impact',struct(),'keep_validated',false,'validation',struct());
    if nargin<4, impact=[]; end
    try
        original=normalize_planning_state(original); changed=normalize_planning_state(changed);
        type=upper(char(event.type)); d.event_type=type;
        if ~any(strcmp(type,{'ADD_LOCAL_OBSTACLE','INVALIDATE_DOCK','TASK_COVERAGE_FAILURE'})), return; end
        d.validation=validate_replanning_state(changed,original);
        applicable(1)=true;
        keep=d.validation.valid;
        if strcmp(type,'TASK_COVERAGE_FAILURE') && isfield(event,'invalidated_coverage_pairs')
            for k=1:size(event.invalidated_coverage_pairs,1)
                pair=event.invalidated_coverage_pairs(k,:);
                if pair(2)>=1&&pair(2)<=numel(original.plan.pose_info_list)&& ...
                        any(original.plan.pose_info_list{pair(2)}.covered_tasks==pair(1))&& ...
                        ~ismember(pair(1),original.task_state.verified_tasks), keep=false; end
            end
        elseif strcmp(type,'INVALIDATE_DOCK')
            keep=false; % explicit invalidation is a fact even before map rasterization
        end
        mask(1)=keep; reasons{1}=d.validation.reason;
        if ~keep&&d.validation.valid, reasons{1}='event invalidates active dock/coverage'; end
        assert(isequaln(original.rc2_plan,changed.rc2_plan)&&isequaln(original.task_state,changed.task_state),'input execution state mismatch');
        assert(isfield(original.robot_cfg,'footprint')&&isstruct(original.planner_cfg),'missing effective config');
        map=changed.scene.grid_map; mi=changed.scene.map_info;
        assert(isequal(size(map),[mi.rows mi.cols])&&mi.resolution>0&&all(isfinite(map(:))),'invalid map');
        assert(isfield(original.stats.sparse_graph,'direct_paths')&& ...
            isfield(original.stats.sparse_graph,'adjacency')&&size(original.all_poses,2)==3,'missing graph');
        if isempty(impact), impact=analyze_change_impact(original,event); end
        d.impact=impact;
        switch type
            case 'ADD_LOCAL_OBSTACLE'
                applicable(2)=true;
                assert(numel(event.center)==2&&all(isfinite(event.center))&&isscalar(event.radius_m)&&event.radius_m>0,'invalid obstacle');
                mask(2)=~isempty(impact.invalid_sparse_edges);
                reasons{2}='requires affected graph edges; feasibility tested during execution';
            case 'INVALIDATE_DOCK'
                applicable(3)=true; k=event.target_dock_index;
                assert(isscalar(k)&&k==floor(k)&&k>=1&&k<=numel(original.plan.pose_info_list),'invalid dock ID');
                pending=setdiff(original.plan.pose_info_list{k}.covered_tasks,original.task_state.verified_tasks);
                assert(isfield(original.candidate_library,'regions')&&isfield(original.planner_cfg,'reachability'),'missing candidate/config');
                mask(3)=~isempty(pending)&&ismember(k,impact.invalid_docks);
                reasons{3}='requires an invalid dock with pending tasks and candidate library';
            case 'TASK_COVERAGE_FAILURE'
                applicable(4)=true; tid=event.target_task_id; k=event.target_dock_index;
                assert(isscalar(tid)&&ismember(tid,[original.scene.tasks.id]),'unknown task ID');
                assert(isscalar(k)&&k==floor(k)&&k>=1&&k<=numel(original.plan.pose_info_list),'invalid dock ID');
                assert(isfield(original.candidate_library,'regions')&&isfield(original.planner_cfg,'reachability'),'missing candidate/config');
                mask(4)=~ismember(tid,original.task_state.verified_tasks)&& ...
                    ismember(tid,original.plan.pose_info_list{k}.covered_tasks)&& ...
                    ismember([tid k],event.invalidated_coverage_pairs,'rows')&& ...
                    ismember([tid k],impact.invalidated_coverage_pairs,'rows');
                reasons{4}='requires an existing pending task and its invalidated owner pair';
        end
    catch err
        % A partially checked event must not retain a tentative KEEP bit.
        mask(:)=false;
        reasons(:)={err.message};
    end
    d.applicable=applicable; d.legal=mask; d.reasons=reasons; d.keep_validated=mask(1);
end
