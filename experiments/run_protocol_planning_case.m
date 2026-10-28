function [row,state,audit_rows] = run_protocol_planning_case(scene_cfg,cfg,options)
%RUN_PROTOCOL_PLANNING_CASE Fresh measured planning with explicit evidence.
% No candidate/task is removed from the coverage denominator. Independent
% audit and disk output are outside the reported planning-time scope.
    if nargin<3,options=struct();end
    workflow_started=tic;
    cfg=apply_experiment_protocol(cfg);
    cfg.runtime.headless=true; cfg.runtime.disable_visualization=true;
    cfg.runtime.disable_video=true; cfg.runtime.quiet=true; cfg.runtime.quiet_stage2=true;
    cfg.benchmark.enable=false;
    row=empty_row(); row.scene_id=field_or(scene_cfg,'repair_study_id','CUSTOM');
    row.seed=scene_cfg.random_seed; row.variant=field_or(options,'variant','baseline');
    row.protocol_id=cfg.experiment_protocol.id;
    row.threshold=cfg.reachability.threshold; row.return_to_start=cfg.execution.return_to_start;
    row.global_planner=cfg.global_planner; row.routing_mode=cfg.routing.mode;
    row.sparse_k=cfg.routing.sparse_knn.k; row.grid_resolution_m=scene_cfg.grid_resolution_m;
    row.lateral_step_m=scene_cfg.docking_lateral_step_m;
    row.distance_step_m=scene_cfg.docking_distance_step_m;
    row.orientation_step_deg=rad2deg(field_or(scene_cfg,'docking_orientation_step_rad',pi/12));
    row.cost_weights_json=jsonencode(cfg.cost_weights);
    row.selection_weights_json=jsonencode(cfg.strategy.bottom_up.weights);
    row.augmentation=cfg.candidate_augmentation.enable;
    row.timing_scope=cfg.experiment_protocol.planning_time_scope;
    state=struct(); audit_rows=struct([]);
    try
        t=tic;
        if isfield(options,'scene'),scene=options.scene;
        else,scene=build_scene_from_cfg(scene_cfg);end
        robot=robot_config(); task_map=assign_tasks_to_tables(scene);
        regions=build_task_regions(scene,task_map); row.scene_time_s=toc(t);
        row.table_count=size(scene.scene_cfg.table_positions,1); row.task_count=scene.num_tasks;
        geometry=struct('tables',scene.scene_cfg.table_positions,'yaws',scene.scene_cfg.table_orientations, ...
            'table_dimensions',scene.scene_cfg.table,'start_pose',scene.start_pose, ...
            'tasks',[[scene.tasks.id]' [scene.tasks.table_id]' [scene.tasks.x]' [scene.tasks.y]' [scene.tasks.z]'], ...
            'static_obstacles',field_or(scene.scene_cfg,'static_obstacles',[]), ...
            'visual_static_obstacles',field_or(scene.scene_cfg,'visual_static_obstacles',[]));
        row.task_geometry_signature=build_struct_signature(geometry);
        if field_or(options,'compute_identity',true)
            identity=demo_state_cache_identity(scene,robot,cfg,row.scene_id,row.seed);
            row.implementation_signature=identity.implementation_signature;
        end
        t=tic; library=build_candidate_library(regions,scene,robot,cfg); row.candidate_time_s=toc(t);
        row.candidate_count=library.total_candidates;
        covered=[];
        for ri=1:numel(library.regions)
            for ci=1:numel(library.regions(ri).candidates)
                covered=union(covered,library.regions(ri).candidates(ci).covered_tasks);
            end
        end
        row.candidate_coverage_percent=100*numel(covered)/max(scene.num_tasks,1);
        t=tic; [~,~,~,~,plan]=stage2_docking_planning(regions,scene,robot,cfg,library);
        row.selection_time_s=toc(t);
        row.selected_docks=size(plan.selected_poses,1);
        row.geometric_coverage_percent=100*numel(plan.covered_tasks_predicted)/max(scene.num_tasks,1);
        row.planning_time_s=row.scene_time_s+row.candidate_time_s+row.selection_time_s;
        state=struct('scene',scene,'robot_cfg',robot,'planner_cfg',cfg,'candidate_library',library, ...
            'regions',regions,'task_table_map',{task_map},'plan',plan,'scale',row.scene_id,'seed',row.seed);
        state.all_poses=[scene.start_pose;plan.selected_poses];
        if strcmpi(plan.status,'BLOCKED')
            row.status='BLOCKED_COVERAGE'; row.failure_reason=plan.reason;
            row.timing_scope='scene_candidate_selection_until_coverage_blocked_no_routing';
        elseif ~field_or(options,'run_routing',true)
            row.status='CANDIDATE_SELECTION_ONLY';
            row.timing_scope='scene_candidate_selection_only';
        else
            timing=struct('scene_time_s',row.scene_time_s,'candidate_generation_time_s',row.candidate_time_s, ...
                'strategy_total_time_s',row.selection_time_s,'docking_time_s',row.candidate_time_s+row.selection_time_s);
            routing_started=tic;
            try
            [~,path,all_poses,~,~,distance,stats]=stage3_global_planning(scene,plan.selected_poses, ...
                plan.task_sequence_summary,task_map,robot,cfg,plan.pose_info_list,timing,plan);
            row.routing_time_s=stats.timing.stage3_time_s;
            row.path_validation_time_s=stats.timing.path_validation_time_s;
            row.planning_time_s=row.planning_time_s+row.routing_time_s;
            row.path_length_m=distance; row.path_safe=stats.path_validation.valid;
            row.path_safety_checked=true;
            row.endpoint_error_m=stats.path_validation.endpoint_max_position_error_m;
            row.simulated_geometric_completion_percent=stats.evidence.simulated_geometric_completion_rate;
            row.actual_completion_percent=stats.evidence.actual_completion_rate;
            row.ik_feasibility_percent=stats.evidence.ik_feasibility_rate;
            if strcmpi(cfg.routing.mode,'sparse_knn')
                row.path_queries=stats.sparse_graph.stats.actual_path_queries;
            else,row.path_queries=(size(all_poses,1)-1)*size(all_poses,1)/2;end
            row.status='SUCCESS';
            state.stats=stats; state.stats.global_path=path; state.all_poses=all_poses;
            state.rc2_plan=stats.rc2_plan;
            % Dynamic tests start before mission execution; no completed IDs.
            state.task_state=separate_execution_status(plan.covered_tasks_predicted,[],[],[],struct('source','geometric_simulation'));
            state.path_validation=stats.path_validation;
            row.execution_sim_time_s=stats.timing.execution_sim_time_s;
            catch route_error
                row.routing_time_s=toc(routing_started);
                row.planning_time_s=row.planning_time_s+row.routing_time_s;
                row.status='ERROR_ROUTING';
                row.failure_reason=[route_error.identifier ': ' route_error.message];
                row.timing_scope='scene+candidate+selection+routing_attempt_until_failure';
                pre_execution_errors={'stage3_global_planning:SparseGraphDisconnected', ...
                    'stage3_global_planning:UnsafeRoute','stitch_global_path:MissingSegment', ...
                    'stitch_sparse_graph_path:MissingEdgePath','stitch_sparse_graph_path:DisconnectedRoute'};
                if ismember(route_error.identifier,pre_execution_errors)
                    row.status='BLOCKED_ROUTING'; row.execution_sim_time_s=0;
                else
                    row.execution_sim_time_s=NaN;
                end
            end
        end
        if field_or(options,'run_audit',true)
            try
                [audit,audit_rows]=audit_independent_reachability(scene,robot,cfg,library,struct());
                row.audit_status='PASS'; row.oracle_time_s=audit.audit_time_s;
                row.independent_coverage_percent=audit.audited_geometric_coverage_percent;
                row.independent_mismatch_count=audit.mismatch_count;
                row.independent_pair_mismatch_count=audit.candidate_pair_mismatch_count;
                row.sampling_gap_count=audit.sampling_gap_count;
                row.unresolved_task_count=audit.unresolved_count;
            catch audit_error
                row.audit_status='ERROR'; row.audit_failure_reason=[audit_error.identifier ': ' audit_error.message];
            end
        end
    catch ME
        row.status='ERROR'; row.failure_reason=[ME.identifier ': ' ME.message];
    end
    row.workflow_wall_time_s=toc(workflow_started);
end

function r=empty_row()
    r=struct('scene_id','','seed',0,'variant','','status','','failure_reason','', ...
        'protocol_id','','global_planner','','routing_mode','','sparse_k',0,'threshold',NaN, ...
        'return_to_start',true,'grid_resolution_m',NaN,'lateral_step_m',NaN,'distance_step_m',NaN, ...
        'orientation_step_deg',NaN,'cost_weights_json','','selection_weights_json','','augmentation',false, ...
        'table_count',0,'task_count',0,'candidate_count',0,'selected_docks',0, ...
        'candidate_coverage_percent',NaN,'geometric_coverage_percent',NaN, ...
        'simulated_geometric_completion_percent',NaN,'ik_feasibility_percent',NaN, ...
        'actual_completion_percent',NaN,'independent_coverage_percent',NaN,'independent_mismatch_count',NaN, ...
        'independent_pair_mismatch_count',NaN,'audit_status','NOT_RUN','audit_failure_reason','', ...
        'sampling_gap_count',NaN,'unresolved_task_count',NaN,'path_length_m',NaN,'path_safe',false,'path_safety_checked',false, ...
        'endpoint_error_m',NaN,'path_queries',NaN,'scene_time_s',NaN,'candidate_time_s',NaN, ...
        'selection_time_s',NaN,'routing_time_s',NaN,'path_validation_time_s',NaN, ...
        'planning_time_s',NaN,'execution_sim_time_s',0,'oracle_time_s',NaN,'workflow_wall_time_s',NaN,'timing_scope','', ...
        'task_geometry_signature','','implementation_signature','');
end

function v=field_or(s,n,d)
    if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end
end
