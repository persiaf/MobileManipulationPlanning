function result=exp_task_constraint_comparison(output_dir)
%EXP_TASK_CONSTRAINT_COMPARISON Same arm, scene, tasks, candidates and routing.
%   Run: addpath('pipeline'); init_paths; result=exp_task_constraint_comparison;
%   Outputs are numerical IK support and footprint-validated base routes.
    if nargin<1 || isempty(output_dir)
        output_dir=fullfile('results','task_constraint_comparison_20261111');
    end
    if ~isfolder(output_dir),mkdir(output_dir);end
    setup=task_constraint_comparison_setup();
    scene0=setup.scene;robot=setup.robot;planner=setup.planner;pool=setup.pool;
    modes=setup.mode_order;
    task_map=assign_tasks_to_tables(scene0);regions=build_task_regions(scene0,task_map);
    xyz=[[scene0.tasks.x]',[scene0.tasks.y]',[scene0.tasks.z]'];
    tasks=table([scene0.tasks.id]',[scene0.tasks.table_id]',xyz(:,1),xyz(:,2),xyz(:,3), ...
        'VariableNames',{'task_id','table_id','x_m','y_m','z_m'});
    writetable(tasks,fullfile(output_dir,'tasks.csv'));
    geometry=struct('poses',reshape([pool.pose],3,[])','table_ids',[pool.table_id], ...
        'tasks',xyz,'grid_map',scene0.grid_map,'model',robot.manipulator);
    metadata=struct('experiment_id','task_constraint_comparison_20261111', ...
        'date','2026-11-11','mode_order',{modes},'model_revision',robot.manipulator.model.revision, ...
        'geometry_signature',build_struct_signature(geometry), ...
        'raw_candidate_count',setup.raw_candidate_count,'candidate_count',numel(pool), ...
        'reference_dock',setup.reference_dock,'wipe_length_m',setup.wipe_length_m, ...
        'scene',struct('table_centers',scene0.scene_cfg.table_positions, ...
            'table_length_m',scene0.scene_cfg.table.length,'table_width_m',scene0.scene_cfg.table.width, ...
            'table_height_m',scene0.scene_cfg.table.height,'map_width_m',scene0.scene_cfg.map_width_m, ...
            'map_height_m',scene0.scene_cfg.map_height_m,'start_pose',scene0.start_pose), ...
        'robot_footprint',robot.footprint,'ik_budget',robot.manipulator.model.ik, ...
        'route_rule','same deterministic nearest-neighbor; A*; dense matrix; return to start', ...
        'arm_collision_checked',false,'hardware_calibrated',false, ...
        'scope','single frozen two-table scene; finite IK search; no physical execution');
    write_json(fullfile(output_dir,'metadata.json'),metadata);
    write_json(fullfile(output_dir,'effective_task_settings.json'),task_constraint_effective_settings(setup));
    candidate_rows={};pair_rows={};dock_rows={};path_rows={};summary_rows={};
    mode_results=cell(1,numel(modes));
    for mi=1:numel(modes)
        mode=modes{mi};scene=scene0;
        for t=1:scene.num_tasks,scene.tasks(t)=comparison_manipulation_task(scene.tasks(t),mode);end
        fprintf('\nMODE %s: scoring %d frozen candidates\n',mode,numel(pool));
        scoring=tic;pair_matrix=false(numel(pool),scene.num_tasks);
        library=struct('regions',struct([]),'total_candidates',numel(pool), ...
            'scene_signature',build_candidate_library_signature(regions,scene,robot,planner));
        for ri=1:numel(regions)
            ids=find([pool.table_id]==regions(ri).table_id);
            tids=regions(ri).task_ids;cs=struct([]);
            for ci=1:numel(ids)
                pi=ids(ci);entry=pool(pi);reach=zeros(numel(tids),3);arms=cell(numel(tids),1);
                for ti=1:numel(tids)
                    tid=tids(ti);
                    [ls,rs,s,arm,d]=evaluate_task_reachability(entry.pose,scene.tasks(tid),robot,planner.reachability.threshold);
                    reach(ti,:)=[ls,rs,s];arms{ti}=arm;pair_matrix(pi,tid)=s>0;
                    [pe,oe,step]=quality(d,arm);
                    pair_rows(end+1,:)={mode,entry.candidate_id,tid,s>0,arm, ...
                        d.left.reason,d.right.reason,d.left.failed_sample,d.right.failed_sample,pe,oe,step}; %#ok<AGROW>
                end
                covered=tids(reach(:,3)>0);
                c=struct('pose',entry.pose,'table_id',entry.table_id,'side_id',entry.side_id, ...
                    'covered_tasks',covered,'reach_scores',reach,'arm_assignment',{arms}, ...
                    'nominal_covered_tasks',covered,'nominal_reach_scores',reach, ...
                    'robust_covered_tasks',[],'robust_reachability_summary', ...
                    repmat(struct('success_fraction',NaN,'minimum_score',NaN),1,numel(tids)), ...
                    'coverage_ratio',numel(covered)/numel(tids),'cost_breakdown',struct(), ...
                    'total_cost',Inf,'candidate_id',entry.candidate_id);
                c.cost_breakdown=evaluate_docking_candidate(c,regions(ri),scene,robot,planner);
                c.total_cost=c.cost_breakdown.total;
                if isempty(cs),cs=c;else,cs(end+1)=c;end %#ok<AGROW>
                candidate_rows(end+1,:)={mode,entry.candidate_id,entry.table_id,entry.pose(1), ...
                    entry.pose(2),entry.pose(3),numel(covered),numel(tids)}; %#ok<AGROW>
                if mod(ci,12)==0,fprintf('  region %d: %d/%d candidates\n',ri,ci,numel(ids));end
            end
            lib_entry=struct('region_id',ri,'table_id',regions(ri).table_id, ...
                'region',regions(ri),'candidates',cs,'num_candidates',numel(cs), ...
                'task_ids',tids,'generation_time_s',0,'generation_diagnostics',struct());
            if isempty(library.regions),library.regions=lib_entry;
            else,library.regions(end+1)=lib_entry;end
        end
        scoring_time=toc(scoring);library.total_generation_time_s=scoring_time;
        selection=tic;
        [poses,~,summaries,info,plan]=stage2_docking_planning(regions,scene,robot,planner,library);
        selection_time=toc(selection);path=zeros(0,3);stats=struct();route_length=NaN;nav_time=NaN;
        nav_estimate=NaN;valid=false;verified=0;
        if strcmp(plan.status,'READY')
            nav_clock=tic;
            [~,path,~,~,~,route_length,stats]=stage3_global_planning(scene,poses,summaries, ...
                task_map,robot,planner,info,struct('scene_time_s',0,'docking_time_s',scoring_time),plan);
            nav_time=toc(nav_clock);valid=stats.path_validation.valid;
            verified=numel(stats.execution_status.verified_tasks);
            nav_estimate=stats.exec_stats.navigation_time_s;
        end
        selected_pool_ids=zeros(size(poses,1),1);
        for di=1:size(poses,1)
            deltas=vecnorm(reshape([pool.pose],3,[])'-poses(di,:),2,2);
            [delta,pi]=min(deltas);assert(delta<1e-10);
            selected_pool_ids(di)=pi;
            dock_rows(end+1,:)={mode,di,pool(pi).candidate_id,poses(di,1),poses(di,2),poses(di,3), ...
                strjoin(string(summaries{di}.covered_tasks),'|')}; %#ok<AGROW>
        end
        for k=1:size(path,1)
            path_rows(end+1,:)={mode,k,path(k,1),path(k,2),path(k,3)}; %#ok<AGROW>
        end
        pair_count=sum(arrayfun(@(r)sum([pool.table_id]==r.table_id)*numel(r.task_ids),regions));
        union_count=sum(any(pair_matrix,1));counts=sum(pair_matrix,1);
        summary_rows(end+1,:)={mode,scene.num_tasks,union_count,sum(pair_matrix(:)),pair_count, ...
            size(poses,1),verified,valid,route_length,nav_estimate,scoring_time,selection_time,nav_time, ...
            min(counts),median(counts),max(counts)}; %#ok<AGROW>
        mode_results{mi}=struct('mode',mode,'scene',scene,'pair_matrix',pair_matrix, ...
            'selected_pool_ids',selected_pool_ids,'plan',plan,'path',path,'stats',stats);
        write_tables();
        save(fullfile(output_dir,['checkpoint_' mode '.mat']),'metadata','mode_results','setup','-v7');
    end
    workspace=evaluate_workspace(false,setup.workspace_nx,setup.workspace_ny);
    writetable(workspace,fullfile(output_dir,'workspace.csv'));
    matched=evaluate_workspace(true,setup.matched_nx,setup.matched_ny);
    writetable(matched,fullfile(output_dir,'matched_path_workspace.csv'));
    transfer_rows={};
    for a=1:3
        selected=mode_results{a}.selected_pool_ids;
        for b=1:3
            supported=sum(any(mode_results{b}.pair_matrix(selected,:),1));
            transfer_rows(end+1,:)={modes{a},modes{b},supported,scene0.num_tasks}; %#ok<AGROW>
        end
    end
    writetable(cell2table(transfer_rows,'VariableNames',{'planned_mode','evaluated_mode','supported_tasks','target_count'}), ...
        fullfile(output_dir,'plan_transfer.csv'));
    diagnostics=struct('workspace_count',height(workspace),'matched_path_count',height(matched), ...
        'workspace_supported_counts',sum(workspace{:,4:6},1), ...
        'matched_path_supported_counts',sum(matched{:,4:6},1), ...
        'matched_axis_success_position_failure',sum(matched.top_down_grasp & ~matched.position_only), ...
        'matched_full_success_axis_failure',sum(matched.table_wipe & ~matched.top_down_grasp), ...
        'pair_constrained_success_position_failure',[sum(mode_results{2}.pair_matrix(:)&~mode_results{1}.pair_matrix(:)), ...
            sum(mode_results{3}.pair_matrix(:)&~mode_results{1}.pair_matrix(:))]);
    write_json(fullfile(output_dir,'diagnostics.json'),diagnostics);
    result=struct('metadata',metadata,'mode_results',{mode_results},'diagnostics',diagnostics);
    save(fullfile(output_dir,'comparison_result.mat'),'result','-v7');
    fprintf('\nCOMPARISON COMPLETE\n');disp(readtable(fullfile(output_dir,'summary.csv')));

    function write_tables()
        writetable(cell2table(candidate_rows,'VariableNames',{'mode','candidate_id','table_id','x_m','y_m','yaw_rad','support_count','task_count'}),fullfile(output_dir,'candidate_support.csv'));
        writetable(cell2table(pair_rows,'VariableNames',{'mode','candidate_id','task_id','feasible','assigned_arm','reason_left','reason_right','failed_sample_left','failed_sample_right','max_position_error_m','max_orientation_error_rad','max_joint_step_rad'}),fullfile(output_dir,'pair_support.csv'));
        writetable(cell2table(dock_rows,'VariableNames',{'mode','dock_id','candidate_id','x_m','y_m','yaw_rad','covered_task_ids'}),fullfile(output_dir,'docking_poses.csv'));
        writetable(cell2table(path_rows,'VariableNames',{'mode','point_id','x_m','y_m','yaw_rad'}),fullfile(output_dir,'navigation_paths.csv'));
        writetable(cell2table(summary_rows,'VariableNames',{'mode','target_count','union_supported_count','supported_pairs','pair_count','selected_docks','verified_tasks','route_valid','route_length_m','nav_estimated_time_s','scoring_time_s','selection_time_s','navigation_planning_time_s','min_support_per_task','median_support_per_task','max_support_per_task'}),fullfile(output_dir,'summary.csv'));
    end

    function tab=evaluate_workspace(matched_path,nx,ny)
        [X,Y]=meshgrid(linspace(1.48,2.52,nx),linspace(2.04,2.36,ny));
        support=false(numel(X),3);
        for wi=1:numel(X)
            anchor=struct('x',X(wi),'y',Y(wi),'z',scene0.scene_cfg.table.height);
            for mj=1:3
                task=comparison_manipulation_task(anchor,modes{mj},matched_path);
                [~,~,s]=evaluate_task_reachability(setup.reference_dock,task,robot);
                support(wi,mj)=s>0;
            end
            if mod(wi,15)==0,fprintf('WORKSPACE matched=%d: %d/%d\n',matched_path,wi,numel(X));end
        end
        tab=table(X(:),Y(:),repmat(scene0.scene_cfg.table.height,numel(X),1),support(:,1),support(:,2),support(:,3), ...
            'VariableNames',{'x_m','y_m','z_m','position_only','top_down_grasp','table_wipe'});
    end
end

function [pe,oe,step]=quality(details,arm)
    pe=NaN;oe=NaN;step=NaN;
    if strcmp(arm,'none'),return;end
    d=details.(arm);pe=max(d.position_errors_m);oe=max(d.orientation_errors_rad);
    if size(d.q_path,1)>1,step=max(abs(diff(d.q_path)),[],'all');else,step=0;end
end

function write_json(path,data)
    fid=fopen(path,'w','n','UTF-8');
    if fid<0,error('exp_task_constraint_comparison:WriteFailed','Cannot write %s.',path);end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(data,'PrettyPrint',true));
end
