function v = validate_replanning_state(state, original)
%VALIDATE_REPLANNING_STATE Read-only safety gate; never optimizes a plan.
    v = struct('valid',false,'path_safe',false,'dock_valid',false, ...
        'task_coverage_valid',false,'state_transition_valid',false,'reason','');
    try
        state = normalize_planning_state(state);
        assert(isequaln(state.rc2_plan,original.rc2_plan),'rc2_plan changed');
        assert(isequaln(state.task_state,original.task_state),'task execution state changed');
        assert(isequaln([state.scene.tasks.id],[original.scene.tasks.id]),'task IDs changed');
        assert(isequaln([state.scene.tasks.status],[original.scene.tasks.status]),'task completion changed');
        map=state.scene.grid_map; info=state.scene.map_info;
        assert(isnumeric(map)&&ismatrix(map)&&all(isfinite(map(:)))&& ...
            isequal(size(map),[info.rows info.cols])&&info.resolution>0,'invalid map');
        fp=state.robot_cfg.footprint;
        poses=state.plan.selected_poses; regions=state.plan.pose_to_region_map;
        summaries=state.plan.task_sequence_summary; poseinfo=state.plan.pose_info_list;
        n=size(poses,1); assert(n>0&&size(poses,2)==3&&all(isfinite(poses(:))),'invalid docks');
        assert(numel(regions)==n&&numel(summaries)==n&&numel(poseinfo)==n,'dock mapping size');
        assert(all(ismember(regions,state.scene.region_metadata.region_ids)),'unknown region');
        assert(isequaln(state.all_poses,[state.scene.start_pose;poses]),'pose aliases disagree');
        for k=1:n
            assert(pose_safe(poses(k,:),fp,state.scene),'dock footprint unsafe');
            assert(isequal(sort(summaries{k}.covered_tasks),sort(poseinfo{k}.covered_tasks)),'coverage aliases disagree');
            if isfield(summaries{k},'region_id')
                assert(summaries{k}.region_id==regions(k),'dock region mismatch');
            end
        end
        v.dock_valid=true;
        path=state.stats.global_path; segments=state.stats.segment_stats; order=state.stats.route_order;
        assert(size(path,1)>=2&&size(path,2)==3&&all(isfinite(path(:))),'missing finite path');
        assert(all(order>=1 & order<=n+1 & order==floor(order))&&order(1)==1,'invalid route');
        assert(all(ismember(2:n+1,order)),'route omits selected dock');
        assert(numel(segments)==numel(order)-1,'segment count mismatch');
        joined=[]; tolerance=info.resolution*sqrt(2)/2+1e-8;
        for k=1:numel(segments)
            s=segments(k); q=s.path;
            assert(s.from_id==order(k)&&s.to_id==order(k+1)&&size(q,1)>=2&&s.num_points==size(q,1),'segment/order mismatch');
            assert(norm(q(1,1:2)-state.all_poses(order(k),1:2))<=tolerance&& ...
                norm(q(end,1:2)-state.all_poses(order(k+1),1:2))<=tolerance,'segment endpoints mismatch');
            if isempty(joined), joined=q;
            else
                assert(norm(joined(end,1:2)-q(1,1:2))<1e-8,'discontinuous segments');
                joined=[joined;q(2:end,:)]; %#ok<AGROW>
            end
        end
        assert(isequaln(joined,path),'segments do not produce global path');
        [stitched,~]=stitch_sparse_graph_path(state.all_poses,order,state.stats.sparse_graph);
        assert(isequaln(stitched,path),'graph and path disagree');
        [a,b]=find(triu(state.stats.sparse_graph.adjacency,1));
        for e=1:numel(a)
            q=state.stats.sparse_graph.direct_paths{a(e),b(e)};
            assert(~isempty(q)&&abs(compute_path_length(q)-state.stats.sparse_graph.cost_matrix(a(e),b(e)))<1e-7,'edge cost mismatch');
        end
        q=path(1,:);
        for k=2:size(path,1)
            a=path(k-1,:);b=path(k,:);
            count=max([1,ceil(norm(b(1:2)-a(1:2))/(info.resolution/3)),ceil(abs(b(3)-a(3))/(pi/36))]);
            q=[q;a+((1:count)'/count).*(b-a)]; %#ok<AGROW>
        end
        assert(validate_world_path_collision(map,q,info,false),'path map collision');
        for k=1:size(q,1), assert(pose_safe(q(k,:),fp,state.scene),'path footprint unsafe'); end
        if isfield(state.planner_cfg,'execution') && state.planner_cfg.execution.return_to_start
            assert(order(end)==1,'return route missing');
        end
        if isfield(state.stats,'path_len')
            assert(abs(state.stats.path_len-compute_path_length(path))<1e-7,'path length mismatch');
        end
        v.path_safe=true;
        ids=[state.scene.tasks.id]; verified=state.task_state.verified_tasks;
        assert(numel(unique(ids))==numel(ids)&&all(ismember(verified,ids)),'invalid task IDs');
        pairs=zeros(0,2);
        if isfield(state,'invalidated_coverage_pairs'), pairs=state.invalidated_coverage_pairs; end
        if isfield(original,'invalidated_coverage_pairs')
            assert(all(ismember(original.invalidated_coverage_pairs,pairs,'rows')),'lost historical invalidation');
        end
        owners=[];
        for d=1:n
            ts=reshape(summaries{d}.covered_tasks,1,[]);
            assert(all(ismember(ts,ids)),'assignment contains unknown task');
            owners=[owners,ts]; %#ok<AGROW>
            for tid=setdiff(ts,verified)
                assert(~ismember([tid d],pairs,'rows'),'invalidated coverage reused');
                cfg=state.planner_cfg; threshold=cfg.reachability.threshold;
                target=state.scene.tasks(find(ids==tid,1)); xyz=[target.x target.y target.z];
                [ls,rs,score,~]=evaluate_dual_arm_reachability(poses(d,:),xyz,state.robot_cfg,threshold);
                assert(score>0,'pending task unreachable');
                arm=summaries{d}.arm_assign_map(tid);
                assert((strcmp(arm,'left')&&ls>0)||(strcmp(arm,'right')&&rs>0),'assigned arm unreachable');
                if isfield(cfg.reachability,'mode')&&strcmp(cfg.reachability.mode,'robust')
                    rr=evaluate_robust_reachability(poses(d,:),xyz,state.robot_cfg,threshold,cfg.docking_error_model);
                    assert(rr.success_fraction>=cfg.reachability.robust_success_fraction_min,'robust coverage failed');
                end
                if isfield(summaries{d},'task_actions')
                    actions=summaries{d}.task_actions; ii=find([actions.task_id]==tid);
                    assert(numel(ii)==1&&strcmp(actions(ii).assigned_arm,arm),'task action mismatch');
                end
            end
        end
        for tid=setdiff(ids,verified), assert(sum(owners==tid)==1,'pending task must have one owner'); end
        v.task_coverage_valid=true;
        if isfield(state,'current_plan')
            c=state.current_plan;
            if isfield(c,'selected_poses'), assert(isequaln(c.selected_poses,poses),'current docks mismatch'); end
            if isfield(c,'selected_docks'), assert(isequaln(c.selected_docks,poses),'current docks mismatch'); end
            if isfield(c,'task_sequence_summary'), assert(isequaln(c.task_sequence_summary,summaries),'current assignments mismatch'); end
            if isfield(c,'task_assignment'), assert(isequaln(c.task_assignment,summaries),'current assignments mismatch'); end
            assert(isequaln(c.global_path,path)&&isequaln(c.route_order,order),'current path mismatch');
        end
        v.state_transition_valid=true; v.valid=true; v.reason='validated';
    catch err
        v.reason=err.message;
    end
end

function ok=pose_safe(p,fp,scene)
    corners=oriented_rectangle_corners(p(1:2),p(3),fp.length_m,fp.width_m);
    info=scene.map_info;
    ok=all(corners(:,1)>=0 & corners(:,1)<=info.cols*info.resolution & ...
        corners(:,2)>=0 & corners(:,2)<=info.height_m) && ...
        ~check_robot_footprint_collision(p,fp,scene);
end

