function raw = run_protocol_dynamic_comparison(options)
%RUN_PROTOCOL_DYNAMIC_COMPARISON Actual local/full replans from one fresh state.
% Event generation is outside both timers. Both modes use the changed map,
% same tasks/robot/threshold/return and current path validation; no estimates.
% Internal planner validation stays inside each replan timer. The identical
% external route certification runs after BOTH timers and has its own time.
    if nargin<1,options=struct();end
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    out=field_or(options,'output_dir',fullfile(root,'results','repair_20261027','dynamic'));
    if ~isfolder(out),mkdir(out);end
    seeds=field_or(options,'seeds',[2026 2027 2028]);
    scene_id=field_or(options,'scene_id','MAIN4');
    events=field_or(options,'events',{'ADD_LOCAL_OBSTACLE','INVALIDATE_DOCK','ADD_TASKS'});
    rows=struct([]);
    for seed=seeds
        cfg=apply_experiment_protocol(field_or(options,'planner_cfg',planner_config()));
        if isfield(options,'scene_cfg')
            scene_cfg=options.scene_cfg; scene_cfg.random_seed=seed;
            scene_cfg.repair_study_id=scene_id;
        else,scene_cfg=repair_study_scene_config(scene_id,seed);end
        [base,state,audit]=run_protocol_planning_case(scene_cfg,cfg, ...
            struct('run_audit',field_or(options,'run_base_audit',true)));
        write_repair_case_artifacts(state,base,audit,fullfile(out,'base'),sprintf('seed%d',seed));
        if ~strcmp(base.status,'SUCCESS')
            row=empty_dynamic_row(); row.seed=seed; row.scene_id=scene_id;
            row.status='BASE_BLOCKED'; row.failure_reason=base.failure_reason;
            rows=append(rows,row); writetable(struct2table(rows,'AsArray',true),fullfile(out,'dynamic_raw.csv')); continue;
        end
        for ei=1:numel(events)
            [changed,event]=apply_dynamic_event(state,events{ei},ei);
            local_row=base_row(base,event,'LOCAL_REPLAN');
            local_result=struct(); next=changed; impact=struct();
            t=tic;
            try
                impact=analyze_change_impact(state,event);
                [local_result,next]=local_replan(state,changed,event,impact);
                local_row.wall_time_s=toc(t);
                local_row.status=local_result.status;
                local_row.failure_reason=local_result.fallback_reason;
                local_row.geometric_coverage_percent=local_result.raw_coverage;
                local_row.path_queries=field_or(local_result,'path_query_count_total',NaN);
                local_row.path_queries_known=field_or(local_result,'path_query_count_known',false) && ...
                    valid_count(local_row.path_queries);
                if ~local_row.path_queries_known,local_row.path_queries=NaN;end
                local_row.path_query_count_source='local_replan_all_attempted_graphs';
                local_row.candidate_trials=field_or(field_or(local_result,'diagnostics',struct()),'candidate_trials',NaN);
            catch ME
                local_row.wall_time_s=toc(t); local_row.status='ERROR';
                local_row.failure_reason=[ME.identifier ': ' ME.message];
            end
            local_row.execution_sim_time_s_excluded=0;
            local_row.execution_sim_time_source='not_performed_by_local_replan';
            local_row.replanning_time_s=local_row.wall_time_s;
            full_row=base_row(base,event,'FULL_REPLAN'); t=tic;
            [full_case,full_state,~]=run_protocol_planning_case(changed.scene.scene_cfg,cfg, ...
                struct('scene',changed.scene,'run_audit',false,'compute_identity',false));
            full_row.wall_time_s=toc(t);
            excluded=field_or(full_case,'execution_sim_time_s',NaN);
            if isfield(full_state,'stats')
                excluded=field_or(field_or(full_state.stats,'timing',struct()),'execution_sim_time_s',NaN);
                full_row.execution_sim_time_source='measured_stage3_execution_simulation';
            elseif any(strcmp(full_case.status,{'BLOCKED_COVERAGE','BLOCKED_ROUTING'}))
                full_row.execution_sim_time_source='not_reached_before_coverage_or_route_block';
            end
            full_row.execution_sim_time_s_excluded=excluded;
            if isfinite(excluded) && excluded>=0 && excluded<=full_row.wall_time_s
                full_row.replanning_time_s=full_row.wall_time_s-excluded;
            end
            full_row.status=full_case.status; full_row.failure_reason=full_case.failure_reason;
            full_row.geometric_coverage_percent=full_case.geometric_coverage_percent;
            full_row.path_queries=full_case.path_queries;
            full_row.path_queries_known=valid_count(full_row.path_queries);
            full_row.path_query_count_source='full_replan_route_graph_actual_path_queries';
            full_row.path_safe=full_case.path_safe;
            full_row.path_length_m=full_case.path_length_m;
            full_row.selected_docks=full_case.selected_docks;
            full_row.endpoint_error_m=full_case.endpoint_error_m;

            % Paired external certification is deliberately outside BOTH
            % timers; neither method is charged an extra validation pass.
            [local_row,local_certification]=certify_result(local_row,next,changed.scene,state.robot_cfg);
            [full_row,full_certification]=certify_result(full_row,full_state,changed.scene,state.robot_cfg);
            pair=struct('event',event,'local_row',local_row,'full_row',full_row, ...
                'local_result',local_result,'impact',impact,'state',state,'changed',changed, ...
                'next',next,'full_case',full_case,'full_state',full_state, ...
                'local_certification',local_certification,'full_certification',full_certification);
            % Commit a reload-validated pair before advertising its rows in
            % the CSV. Mutable caches and recursive duplicate states are not
            % part of the scientific evidence snapshot.
            save_repair_dynamic_pair(fullfile(out,sprintf('seed%d_%s_pair.mat',seed,event.type)),pair);
            rows=append(rows,local_row); rows=append(rows,full_row);
            raw=struct2table(rows,'AsArray',true); writetable(raw,fullfile(out,'dynamic_raw.csv'));
            fprintf('DYNAMIC seed=%d event=%s local=%s %.3fs full=%s %.3fs coverage=%.2f/%.2f\n', ...
                seed,event.type,local_row.status,local_row.replanning_time_s,full_row.status, ...
                full_row.replanning_time_s,local_row.geometric_coverage_percent,full_row.geometric_coverage_percent);
        end
    end
    raw=struct2table(rows,'AsArray',true);
end

function r=base_row(base,event,mode)
    r=empty_dynamic_row(); r.scene_id=base.scene_id; r.seed=base.seed;
    r.event_type=event.type; r.mode=mode; r.threshold=base.threshold;
    r.return_to_start=base.return_to_start; r.global_planner=base.global_planner;
    r.routing_mode=base.routing_mode; r.sparse_k=base.sparse_k;
    r.implementation_signature=base.implementation_signature;
    r.event_geometry_json=jsonencode(event);
end

function r=empty_dynamic_row()
    r=struct('scene_id','','seed',0,'event_type','','mode','','status','','failure_reason','', ...
        'threshold',0.30,'return_to_start',true,'global_planner','','routing_mode','','sparse_k',0, ...
        'geometric_coverage_percent',NaN,'actual_completion_percent',NaN,'ik_feasibility_percent',NaN, ...
        'selected_docks',NaN,'path_queries',NaN,'path_queries_known',false, ...
        'path_query_count_source','','path_query_scope','route_graph_requests_across_all_attempted_candidate_graphs_including_cache_hits', ...
        'candidate_trials',NaN,'path_safe',false,'endpoint_error_m',NaN, ...
        'path_length_m',NaN,'replanning_time_s',NaN,'wall_time_s',NaN,'execution_sim_time_s_excluded',NaN, ...
        'execution_sim_time_source','unknown','certification_time_s',0,'certification_checked',false, ...
        'certification_scope','same_validate_planned_route_after_both_replan_timers', ...
        'timing_scope','post_event_replanning_wall_includes_internal_validation_excludes_execution_simulation_and_external_certification', ...
        'excluded_time_scope','base_planning+event_generation+external_certification+execution_simulation+oracle+file_io', ...
        'reference_type','actual_current_code_replan','event_geometry_json','','implementation_signature','');
end

function [row,validation]=certify_result(row,candidate,scene,robot_cfg)
    validation=struct('valid',false,'reason','NO_SUCCESSFUL_ROUTE_TO_CERTIFY');
    if ~any(strcmp(row.status,{'SUCCESS','NOOP'})),return;end
    t=tic;
    row.certification_checked=true;
    row.path_safe=false;
    try
        validation=validate_planned_route(scene,candidate.stats.global_path,candidate.all_poses, ...
            candidate.stats.route_order,candidate.stats.segment_stats,robot_cfg);
        row.certification_time_s=toc(t);
        row.path_safe=validation.valid;
        row.endpoint_error_m=validation.endpoint_max_position_error_m;
        row.path_length_m=compute_path_length(candidate.stats.global_path);
        row.selected_docks=size(candidate.plan.selected_poses,1);
        if ~validation.valid
            row.status='UNSAFE_CERTIFIED_ROUTE'; row.failure_reason=validation.reason;
        end
    catch ME
        row.certification_time_s=toc(t);
        validation=struct('valid',false,'reason',[ME.identifier ': ' ME.message]);
        row.status='CERTIFICATION_ERROR'; row.failure_reason=validation.reason;
    end
end

function tf=valid_count(value)
    tf=isnumeric(value)&&isscalar(value)&&isfinite(value)&&value>=0&&value==fix(value);
end

function rows=append(rows,row)
    if isempty(rows),rows=row;else,rows(end+1)=row;end
end

function v=field_or(s,n,d)
    if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end
end
