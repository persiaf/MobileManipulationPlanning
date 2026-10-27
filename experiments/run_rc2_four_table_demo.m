function result = run_rc2_four_table_demo(output_root)
%RUN_RC2_FOUR_TABLE_DEMO Execute the real RC1/RC2/Stage3 pipeline externally.
    root = fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
    if nargin < 1 || isempty(output_root)
        error('Provide an external result directory.');
    end
    output_root = char(java.io.File(output_root).getCanonicalPath());
    if startsWith(lower(output_root), [lower(root) filesep])
        error('Results must be outside the project tree.');
    end
    dirs = {'config','raw_data','figures','tables','logs','diagnostics','reports'};
    for i=1:numel(dirs), mkdir(fullfile(output_root,dirs{i})); end
    diary(fullfile(output_root,'logs','matlab_run.log')); cleanup = onCleanup(@() diary('off')); %#ok<NASGU>
    scene_cfg = rc2_four_table_scene_config();
    effective = resolve_effective_config('user_override', struct('scene', scene_cfg, ...
        'planner', struct('runtime', struct('headless',true,'disable_visualization',true, ...
        'figure_visible','off','close_figures_after_save',true))));
    t = tic; [robot_cfg, scene_cfg, planner_cfg, scene, task_table_map, regions] = stage1_build_scene(effective); stage1_s = toc(t);
    t = tic; library = build_candidate_library(regions, scene, robot_cfg, planner_cfg); rc1_s = toc(t);
    t = tic; [~,~,~,~,plan] = stage2_docking_planning(regions, scene, robot_cfg, planner_cfg, library); rc2_s = toc(t);
    stage3_ok = false; stage3_error = '';
    if isfield(plan,'status') && strcmpi(plan.status,'BLOCKED')
        scene3=scene; global_path=[]; all_poses=[scene.start_pose; plan.selected_poses];
        visit_order=[]; executed_tasks=[]; path_len=NaN; stats=struct(); stage3_s=NaN;
        stage3_error=sprintf('RC2 returned BLOCKED: %s',plan.reason);
    else
    try
        t = tic;
        [scene3, global_path, all_poses, visit_order, executed_tasks, path_len, stats] = ...
            stage3_global_planning(scene, plan.selected_poses, plan.task_sequence_summary, ...
            task_table_map, robot_cfg, planner_cfg, plan.pose_info_list, struct(), plan);
        stage3_s = toc(t); stage3_ok = true;
    catch err
        scene3 = scene; global_path = []; all_poses = [scene.start_pose; zeros(0,3)];
        visit_order = []; executed_tasks = []; path_len = NaN; stats = struct(); stage3_s = NaN;
        stage3_error = getReport(err,'basic','hyperlinks','off');
        warning('run_rc2_four_table_demo:Stage3Failed','%s',stage3_error);
    end
    end
    candidate_count = library.total_candidates;
    if isfield(plan,'covered_tasks_predicted'), covered=unique(plan.covered_tasks_predicted(:))'; else, covered=[]; end
    all_tasks = [scene.tasks.id]; uncovered = setdiff(all_tasks, covered);
    assigned = [];
    summaries={}; if isfield(plan,'task_sequence_summary'), summaries=plan.task_sequence_summary; end
    for i=1:numel(summaries)
        if isfield(summaries{i},'covered_tasks'), assigned=union(assigned,summaries{i}.covered_tasks); end
    end
    region_rows = struct([]);
    for ri=1:numel(library.regions)
        cs=library.regions(ri).candidates; ids=[];
        for ci=1:numel(cs), if isfield(cs(ci),'covered_tasks'), ids=union(ids,cs(ci).covered_tasks); end, end
        region_tasks=scene.tasks(ismember([scene.tasks.table_id],regions(ri).table_id)); region_task_ids=[region_tasks.id];
        row=struct('region_id',ri,'table_id',regions(ri).table_id,'raw_candidate_count',numel(cs), ...
            'coverage_candidate_count',sum(arrayfun(@(x)isfield(x,'covered_tasks') && ~isempty(x.covered_tasks),cs)), ...
            'covered_task_ids',mat2str(intersect(ids,region_task_ids)),'uncovered_task_ids',mat2str(setdiff(region_task_ids,ids))); %#ok<AGROW>
        if isempty(region_rows), region_rows=row; else, region_rows(end+1)=row; end %#ok<AGROW>
    end
    safety = struct('checked',false,'safe',false,'reason','stage3 did not produce a path');
    if stage3_ok && ~isempty(global_path)
        safety.checked = true;
        [safety.safe, safety.bad_segments] = validate_world_path_collision(scene.grid_map,global_path,scene.map_info,false);
        safety.reason = 'center path validation only; full footprint validation is recorded separately';
        safety.footprint_checked = false;
        if isfield(stats,'path_validation'), safety=stats.path_validation; end
    end
    metadata = struct('matlab_version',version,'seed',2026,'git_head',system_string('git rev-parse HEAD'), ...
        'git_branch',system_string('git branch --show-current'),'stage1_s',stage1_s,'rc1_s',rc1_s,'rc2_s',rc2_s,'stage3_s',stage3_s,'stage3_error',stage3_error);
    config = struct('scene',scene_cfg,'robot',robot_cfg,'planner',planner_cfg,'effective',effective);
    save(fullfile(output_root,'raw_data','rc2_plan.mat'),'scene','scene3','regions','library','plan','stats','global_path','all_poses','visit_order','executed_tasks','safety','metadata','config','-v7.3');
    writetable(struct2table(region_rows),fullfile(output_root,'tables','RC2_4TABLE_REGION_COVERAGE.csv'));
    summary=table(size(scene_cfg.table_positions,1),numel(scene.tasks),candidate_count,numel(covered),size(plan.selected_poses,1),numel(assigned),numel(uncovered),path_len,safety.safe, ...
        'VariableNames',{'table_count','task_count','candidate_count','covered_task_count','selected_dock_count','assigned_task_count','uncovered_task_count','path_length_m','full_footprint_safe'});
    writetable(summary,fullfile(output_root,'tables','RC2_4TABLE_SUMMARY.csv'));
    task_rows=struct('task_id',num2cell([scene.tasks.id]'),'table_id',num2cell([scene.tasks.table_id]'),'x',num2cell([scene.tasks.x]'),'y',num2cell([scene.tasks.y]'),'z',num2cell([scene.tasks.z]'),'covered',num2cell(ismember([scene.tasks.id],covered)')); %#ok<NASGU>
    writetable(struct2table(task_rows),fullfile(output_root,'tables','RC2_4TABLE_TASK_ASSIGNMENT.csv'));
    save_scene_figures(scene,library,plan,global_path,output_root);
    result=struct('output_root',output_root,'scene',scene,'regions',regions,'library',library,'plan',plan,'global_path',global_path,'safety',safety,'summary',summary,'metadata',metadata);
end
function s=system_string(cmd), [~,s]=system(cmd); s=strtrim(s); end
function save_scene_figures(scene,library,plan,path,out)
    names={'RC2_4TABLE_01_SCENE','RC2_4TABLE_02_CANDIDATES','RC2_4TABLE_03_DOCK_ASSIGNMENT','RC2_4TABLE_04_GLOBAL_PATH'};
    for k=1:4
        f=figure('Visible','off','Color','w'); imagesc([0 scene.map_info.width_m],[0 scene.map_info.height_m],flipud(scene.grid_map)); set(gca,'YDir','normal'); hold on; axis equal tight;
        plot([scene.tasks.x],[scene.tasks.y],'ro','MarkerFaceColor','r'); for i=1:numel(scene.tasks), text(scene.tasks(i).x+.04,scene.tasks(i).y,sprintf('T%d',scene.tasks(i).id)); end; plot(scene.start_pose(1),scene.start_pose(2),'kp','MarkerFaceColor','y');
        if k>=2, for ri=1:numel(library.regions), cs=library.regions(ri).candidates; for ci=1:numel(cs), if isfield(cs(ci),'pose'), plot(cs(ci).pose(1),cs(ci).pose(2),'.','Color',[.1 .4 .8]); end, end, end; end
        if k>=3 && isfield(plan,'selected_poses'), plot(plan.selected_poses(:,1),plan.selected_poses(:,2),'ks','MarkerFaceColor','g'); end
        if k==4 && ~isempty(path), plot(path(:,1),path(:,2),'m-','LineWidth',2); end
        title(names{k},'Interpreter','none'); xlabel('x (m)'); ylabel('y (m)'); exportgraphics(f,fullfile(out,'figures',[names{k} '.png']),'Resolution',150); close(f);
    end
end
