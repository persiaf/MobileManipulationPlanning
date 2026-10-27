function output=run_stage_progress_review_v1(output,mode)
%RUN_STAGE_PROGRESS_REVIEW_V1 Actual small-model run, never a benchmark.
% Invoke only after the safety regression gate passes. Output must be new
% and outside the project. Every figure is rendered from saved MAT data.
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
    if nargin<1,error('Provide a new external output directory');end
    out=char(java.io.File(output).getCanonicalPath());
    project=char(java.io.File(root).getCanonicalPath());
    assert(~strcmpi(out,project)&&~startsWith(lower(out),[lower(project) filesep]),'Output must be external');
    if nargin>1&&strcmp(mode,'render_saved')
        diary(fullfile(out,'logs','render_saved.log')); cleanup=onCleanup(@()diary('off'));
        raw=load(fullfile(out,'raw_data','stage_run.mat'));
        write_tables(raw,out);render_figures(raw,out);return;
    end
    assert(~exist(out,'dir'),'Output already exists');
    dirs={'config','raw_data','tables','logs','diagnostics','reports','figures/RC1','figures/RC2','figures/RC3'};
    for k=1:numel(dirs),mkdir(fullfile(out,dirs{k}));end
    diary(fullfile(out,'logs','matlab_run.log')); finish=onCleanup(@()diary('off'));
    rng(2026,'twister'); timer=tic; [initial,model]=create_c2_5_1_fixture(); pipeline_time=toc(timer);
    validation=validate_replanning_state(initial,initial); assert(validation.valid,validation.reason);
    metadata=struct('matlab_version',version,'seed',2026,'scene_id','single_table_two_tasks', ...
        'geometry_only',true,'hardware_verified',false,'pipeline_time_s',pipeline_time);
    [~,metadata.git_head]=system('git rev-parse HEAD'); [~,metadata.git_branch]=system('git branch --show-current');
    config=struct('robot',initial.robot_cfg,'planner',initial.planner_cfg,'scene',initial.scene.scene_cfg, ...
        'map_info',initial.scene.map_info,'tasks',initial.scene.tasks);
    save(fullfile(out,'config','effective_config.mat'),'config','metadata');
    % All four actions use the actual production interface from one baseline.
    actions={'KEEP','REPLAN_PATH','SWITCH_DOCK','REALLOCATE_TASK'};
    types={'ADD_LOCAL_OBSTACLE','ADD_LOCAL_OBSTACLE','INVALIDATE_DOCK','TASK_COVERAGE_FAILURE'};
    cases=cell(1,4);
    for k=1:4
        [changed,event]=apply_dynamic_event(initial,types{k},1);
        if k==1
            changed.scene=initial.scene;event.center=[3.8 3.8];
            [i,j]=world2grid(3.8,3.8,initial.scene.map_info.resolution,initial.scene.map_info.height_m);
            changed.scene.grid_map(i,j)=1;event.rasterization='single occupied cell, distant KEEP fixture';
        elseif k==3
            changed.scene=initial.scene;
            [i,j]=world2grid(event.center(1),event.center(2),initial.scene.map_info.resolution,initial.scene.map_info.height_m);
            changed.scene.grid_map(i,j)=1;
            event.rasterization='single occupied dock cell (C2-2 fixture); radius is impact envelope';
        end
        [mask,mask_detail]=evaluate_replanning_action_mask(initial,changed,event,[]);
        timer=tic;[result,next,diagnostics]=execute_replanning_action(initial,changed,event,[],actions{k});elapsed=toc(timer);
        assert(diagnostics.committed&&result.path_safe,result.fallback_reason);
        if k>1,assert(strcmp(result.status,'SUCCESS'));else,assert(strcmp(result.status,'NOOP'));end
        cases{k}=struct('original',initial,'changed',changed,'event',event,'mask',mask,'mask_detail',mask_detail, ...
            'result',result,'next',next,'diagnostics',diagnostics,'runtime_s',elapsed);
    end
    ds=normalize_planning_state(create_c2_5_1_dstar_fixture());dstar=cell(1,2);
    for k=1:2
        [changed,event]=apply_dynamic_event(ds,'ADD_LOCAL_OBSTACLE',5);
        q=ds.stats.sparse_graph.direct_paths{1,2};event.center=q(floor(size(q,1)/2),1:2);
        if k==1,event.center=event.center+[.25 0];event.radius_m=.15;else,event.radius_m=.25;end
        changed.scene=ds.scene;
        for i=1:changed.scene.map_info.rows
            for j=1:changed.scene.map_info.cols
                [x,y]=grid2world(i,j,changed.scene.map_info.resolution,changed.scene.map_info.height_m);
                if hypot(x-event.center(1),y-event.center(2))<=event.radius_m,changed.scene.grid_map(i,j)=1;end
            end
        end
        timer=tic;[result,next,diagnostics]=execute_replanning_action(ds,changed,event,[],'REPLAN_PATH');elapsed=toc(timer);
        assert(strcmp(result.status,'SUCCESS')&&diagnostics.committed,result.fallback_reason);
        edge=diagnostics.low_level.incremental_dstar.edge_results(1);
        assert(~edge.fallback_used&&result.path_safe);
        if k==2
            assert(edge.state_loaded&&edge.incremental_reuse&&~edge.reinitialized);
            assert(strcmp(edge.key,dstar{1}.edge.key)&&edge.state_version>dstar{1}.edge.state_version);
        end
        dstar{k}=struct('original',ds,'changed',changed,'event',event,'result',result,'next',next, ...
            'diagnostics',diagnostics,'edge',edge,'runtime_s',elapsed);ds=next;
    end
    % Candidate metrics use the saved production library, not a new sampler.
    candidates=model.library.regions(1).candidates;nc=numel(candidates);nt=numel(initial.scene.tasks);
    coverage=false(nc,nt);scores=zeros(nc,2);nominal=[];robust=[];
    for k=1:nc
        coverage(k,:)=ismember([initial.scene.tasks.id],candidates(k).covered_tasks);
        scores(k,:)=max(candidates(k).nominal_reach_scores(:,1:2),[],1);
        nominal=union(nominal,candidates(k).nominal_covered_tasks);
    end
    for d=1:size(initial.plan.selected_poses,1)
        for tid=initial.plan.task_sequence_summary{d}.covered_tasks
            task=initial.scene.tasks(find([initial.scene.tasks.id]==tid,1));
            rr=evaluate_robust_reachability(initial.plan.selected_poses(d,:),[task.x task.y task.z], ...
                initial.robot_cfg,initial.planner_cfg.reachability.threshold,initial.planner_cfg.docking_error_model);
            if rr.success_fraction>=initial.planner_cfg.reachability.robust_success_fraction_min,robust=union(robust,tid);end
        end
    end
    generation=model.library.regions(1).generation_diagnostics;
    metrics=struct('candidate_count',nc,'nominal_task_ids',nominal,'robust_selected_task_ids',robust, ...
        'coverage',coverage,'arm_scores',scores,'generation',generation);
    save(fullfile(out,'raw_data','stage_run.mat'),'initial','model','validation','metadata','cases','dstar','metrics','-v7.3');
    % Reload saved evidence for tables and figures.
    raw=load(fullfile(out,'raw_data','stage_run.mat'));
    write_tables(raw,out); render_figures(raw,out);
    fprintf('STAGE_RUN_COMPLETE: %s\n',out);
end

function write_tables(r,out)
    s=r.initial;m=r.metrics;g=m.generation;
    row=struct('scene_id',r.metadata.scene_id,'task_count',numel(s.scene.tasks), ...
        'candidate_count',g.raw_candidate_count,'library_candidate_count',m.candidate_count, ...
        'collision_valid_candidate_count',g.collision_valid_candidate_count, ...
        'collision_rejected_candidate_count',g.collision_rejected_candidate_count, ...
        'valid_candidate_count',g.reachability_valid_candidate_count, ...
        'invalid_candidate_count',g.raw_candidate_count-g.reachability_valid_candidate_count, ...
        'zero_coverage_library_candidates',sum(sum(m.coverage,2)==0), ...
        'one_task_library_candidates',sum(sum(m.coverage,2)==1), ...
        'two_task_library_candidates',sum(sum(m.coverage,2)==2), ...
        'nominal_covered_tasks',numel(m.nominal_task_ids),'robust_covered_tasks',numel(m.robust_selected_task_ids), ...
        'robust_scope','selected assignment only','reachability_mode',s.planner_cfg.reachability.mode, ...
        'error_source','configured_bound','measured',false,'runtime_s',r.model.candidate_time_s,'status','PASSED');
    writetable(struct2table(row),fullfile(out,'tables','RC1_REACHABILITY_SUMMARY.csv'));
    assigned=[];for d=1:numel(s.plan.task_sequence_summary),assigned=union(assigned,s.plan.task_sequence_summary{d}.covered_tasks);end
    row=struct('scene_id',r.metadata.scene_id,'strategy',s.planner_cfg.strategy.name,'task_count',numel(s.scene.tasks), ...
        'selected_dock_count',size(s.plan.selected_poses,1),'assigned_task_count',numel(assigned), ...
        'covered_task_count',numel(s.plan.covered_tasks_predicted),'uncovered_task_count',numel(setdiff([s.scene.tasks.id],assigned)), ...
        'route_order',mat2str(s.stats.route_order),'route_length_m',compute_path_length(s.stats.global_path), ...
        'runtime_s',r.metadata.pipeline_time_s,'path_search_count',NaN,'footprint_safe',r.validation.path_safe, ...
        'status','PASSED','hardware_verified_tasks',NaN);
    writetable(struct2table(row),fullfile(out,'tables','RC2_PLANNING_SUMMARY.csv'));
    rows=[];
    for k=1:numel(r.cases)
        c=r.cases{k};a=c.original.plan.selected_poses;b=c.next.plan.selected_poses;
        changed=size(b,1)-size(a,1)+sum(any(a~=b(1:size(a,1),:),2));
        affected=c.mask_detail.impact.affected_task_count;
        row=struct('scenario_id',k,'event_type',c.event.type,'requested_action',c.result.requested_action, ...
            'executed_action',c.result.executed_action,'result_status',c.result.status,'action_mask',mat2str(c.mask), ...
            'affected_task_count',affected,'changed_dock_count',changed, ...
            'old_path_length_m',compute_path_length(c.original.stats.global_path),'new_path_length_m',compute_path_length(c.next.stats.global_path), ...
            'replan_time_s',c.runtime_s,'fallback_used',c.result.fallback,'path_safe',c.result.path_safe, ...
            'committed',c.diagnostics.committed,'verified_tasks_before',numel(c.original.task_state.verified_tasks), ...
            'verified_tasks_after',numel(c.next.task_state.verified_tasks));
        rows=[rows;row]; %#ok<AGROW>
    end
    writetable(struct2table(rows),fullfile(out,'tables','RC3_REPLANNING_SUMMARY.csv'));
    rows=[];
    for k=1:2
        c=r.dstar{k};e=c.edge;
        row=struct('event_id',k,'edge_key',e.key,'state_version',e.state_version,'previous_state_loaded',e.state_loaded, ...
            'incremental_reuse',e.incremental_reuse,'reinitialized',e.reinitialized,'fallback_used',e.fallback_used, ...
            'actual_planner',e.actual_planner,'changed_cell_count',e.changed_cell_count, ...
            'expanded_node_count',c.diagnostics.low_level.incremental_dstar.expanded_node_count, ...
            'path_length_m',compute_path_length(c.next.stats.global_path),'path_safe',c.result.path_safe);
        rows=[rows;row]; %#ok<AGROW>
    end
    writetable(struct2table(rows),fullfile(out,'tables','RC3_DSTAR_INCREMENTAL_SUMMARY.csv'));
end

function render_figures(r,out)
    s=r.initial;cs=r.model.library.regions(1).candidates;poses=vertcat(cs.pose);
    f=newfig();scene_axes(s.scene);title('RC1 | deterministic two-task scene');
    savefigs(f,out,'RC1','RC1_01_SCENE_AND_TASKS');
    f=newfig();tiledlayout(1,2);
    for arm=1:2
        nexttile;scene_axes(s.scene);scatter(poses(:,1),poses(:,2),18,r.metrics.arm_scores(:,arm),'filled');
        colormap(gca,parula(256));colorbar;caxis([0 1]);names={'Left','Right'};title([names{arm} ' arm | max task reach score']);
    end
    savefigs(f,out,'RC1','RC1_02_DUAL_ARM_REACHABILITY');
    f=newfig();scene_axes(s.scene);scatter(poses(:,1),poses(:,2),18,sum(r.metrics.coverage,2),'filled');colormap(gca,parula(256));colorbar;caxis([0 numel(s.scene.tasks)]);
    title(sprintf('RC1 | %d collision-free candidates | coverage count',size(poses,1)));
    subtitle(sprintf('%d candidates cover at least one task',sum(any(r.metrics.coverage,2))));
    savefigs(f,out,'RC1','RC1_03_DOCKING_CANDIDATES');
    f=newfig();imagesc(r.metrics.coverage);colormap(f,[.95 .95 .95;.08 .43 .65]);colorbar;
    xlabel('Task index (IDs in saved scene)');ylabel('Candidate index');xticks(1:numel(s.scene.tasks));
    title('RC1 | actual candidate-task coverage matrix');savefigs(f,out,'RC1','RC1_04_TASK_COVERAGE');
    names={'RC2_01_SELECTED_DOCKS','RC2_02_GLOBAL_PATH','RC2_03_PATH_AND_TASK_COVERAGE'};
    for k=1:3
        f=newfig();scene_axes(s.scene);docks(s.plan.selected_poses,[.1 .35 .8]);
        if k~=1,pathplot(s.stats.global_path,[.05 .55 .35],'-','Validated path');end
        if k~=2,assignments(s,[.65 .25 .7]);end
        title(sprintf('RC2 | %s | safe | %.3f m',s.planner_cfg.strategy.name,compute_path_length(s.stats.global_path)),'Interpreter','none');
        subtitle(['Visit order (start=0): ' mat2str(s.stats.route_order-1)]);
        savefigs(f,out,'RC2',names{k});
    end
    names={'RC3_01_PATH_REPLANNING','RC3_02_DOCK_REPLACEMENT','RC3_03_TASK_REALLOCATION'};
    for k=2:4
        c=r.cases{k};f=newfig();tiledlayout(1,2);
        nexttile;scene_axes(c.changed.scene);pathplot(c.original.stats.global_path,[.7 .3 .2],'--','Original');
        docks(c.original.plan.selected_poses,[.7 .3 .2]);assignments(c.original,[.7 .3 .2]);event_overlay(c);title('Before repair | changed map');
        nexttile;scene_axes(c.next.scene);pathplot(c.original.stats.global_path,[.7 .7 .7],'--','Original');
        pathplot(c.next.stats.global_path,[.05 .55 .35],'-','Repaired');docks(c.next.plan.selected_poses,[.1 .35 .8]);assignments(c.next,[.25 .35 .7]);
        event_overlay(c);title([c.result.executed_action ' | ' c.result.status],'Interpreter','none');
        if isfinite(c.event.target_task_id),subtitle(sprintf('Failed task ID: %d',c.event.target_task_id));
        elseif k==3,subtitle('Red cross: invalid dock; red cell: actual new obstacle');
        else,subtitle('Dashed gray: old path; green: validated repair');end
        savefigs(f,out,'RC3',names{k-1});
    end
    f=newfig();tiledlayout(1,2);
    for k=1:2
        c=r.dstar{k};nexttile;scene_axes(c.next.scene);
        pathplot(c.original.stats.global_path,[.7 .4 .3],'--','Previous');pathplot(c.next.stats.global_path,[.05 .55 .35],'-','Updated');
        title(sprintf('Event %d | reuse=%d | v%d | no fallback',k,c.edge.incremental_reuse,c.edge.state_version));
        subtitle('Deterministic algorithm fixture; tables not rasterized');
    end
    savefigs(f,out,'RC3','RC3_04_DSTAR_INCREMENTAL_UPDATE');
    f=newfig();m=zeros(4);labels=cell(4,1);
    for k=1:4,m(k,:)=r.cases{k}.mask;labels{k}=[r.cases{k}.result.executed_action ' / ' r.cases{k}.result.status];end
    imagesc(m);colormap(f,[.94 .94 .94;.12 .58 .38]);caxis([0 1]);
    xticks(1:4);xticklabels({'KEEP','REPLAN_PATH','SWITCH_DOCK','REALLOCATE_TASK'});xtickangle(15);
    yticks(1:4);yticklabels(labels);set(gca,'TickLabelInterpreter','none');
    for i=1:4,for j=1:4,text(j,i,num2str(m(i,j)),'HorizontalAlignment','center','FontSize',15);end,end
    title('RC3 | actual masks | all committed and footprint safe');
    savefigs(f,out,'RC3','RC3_05_ACTION_MASK_AND_EXECUTION');
end
function f=newfig()
    f=figure('Visible','off','Color','w','Position',[80 80 1200 620]);
end
function scene_axes(s)
    mi=s.map_info;x=((1:mi.cols)-.5)*mi.resolution;y=mi.height_m-((1:mi.rows)-.5)*mi.resolution;
    occupied=double(flipud(s.grid_map)~=0);rgb=ones(mi.rows,mi.cols,3);
    shade=[.2 .23 .27];for k=1:3,rgb(:,:,k)=1-occupied*(1-shade(k));end
    image(x,fliplr(y),rgb);set(gca,'YDir','normal');hold on;
    scatter([s.tasks.x],[s.tasks.y],65,[.85 .35 .05],'filled');
    for k=1:numel(s.tasks),text(s.tasks(k).x+.04,s.tasks(k).y+.04,sprintf('T%d',s.tasks(k).id),'FontWeight','bold');end
    plot(s.start_pose(1),s.start_pose(2),'kp','MarkerSize',12,'MarkerFaceColor',[1 .8 .1]);
    axis equal;xlim([0 mi.width_m]);ylim([0 mi.height_m]);xlabel('World x (m)');ylabel('World y (m)');grid on;
end
function event_overlay(c)
    mi=c.changed.scene.map_info;[ii,jj]=find(c.changed.scene.grid_map~=c.original.scene.grid_map);
    if ~isempty(ii)
        [x,y]=grid2world(ii,jj,mi.resolution,mi.height_m);
        plot(x,y,'s','Color',[.9 .15 .1],'MarkerSize',5,'MarkerFaceColor',[.9 .15 .1]);
    end
    if strcmp(c.event.type,'INVALIDATE_DOCK')
        p=c.original.plan.selected_poses(c.event.target_dock_index,:);
        plot(p(1),p(2),'rx','MarkerSize',16,'LineWidth',2);
    end
end
function pathplot(p,color,style,label)
    plot(p(:,1),p(:,2),style,'Color',color,'LineWidth',1.8,'DisplayName',label);
end
function docks(p,color)
    scatter(p(:,1),p(:,2),80,color,'s','filled');
    quiver(p(:,1),p(:,2),.18*cos(p(:,3)),.18*sin(p(:,3)),0,'Color',color,'LineWidth',1.4);
    for k=1:size(p,1),text(p(k,1)+.06,p(k,2),sprintf('D%d',k),'Color',color);end
end
function assignments(s,color)
    for d=1:numel(s.plan.task_sequence_summary)
        for tid=s.plan.task_sequence_summary{d}.covered_tasks
            t=s.scene.tasks(find([s.scene.tasks.id]==tid,1));p=s.plan.selected_poses(d,:);
            plot([p(1) t.x],[p(2) t.y],':','Color',color,'LineWidth',1.6);
        end
    end
end
function savefigs(f,out,rc,name)
    annotation(f,'textbox',[.01 .001 .98 .035],'String','Source: raw_data/stage_run.mat | model demonstration; no hardware execution', ...
        'EdgeColor','none','HorizontalAlignment','center','Interpreter','none','FontSize',9);
    drawnow;
    exportgraphics(f,fullfile(out,'figures',rc,[name '.png']),'Resolution',160);
    exportgraphics(f,fullfile(out,'figures',rc,[name '.pdf']),'ContentType','vector');close(f);
end
