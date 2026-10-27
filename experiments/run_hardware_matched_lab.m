function result = run_hardware_matched_lab()
%RUN_HARDWARE_MATCHED_LAB Execute the reproducible six-table lab experiment.
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    cfg=hardware_matched_scene_profile_config(); robot=robot_config();
    % Frozen hardware parameters for this round.
    robot.arm_base_mount_z = 0.400;
    robot.arm_base_mount_x = 0.060;
    robot.dual_arm.base_spacing = 0.400;
    robot.single_arm.reach_radius = 0.278;
    robot.single_arm.reach_forward = 0.278;
    robot.single_arm.reach_backward = 0.197;
    robot.single_arm.reach_lateral = 0.278;
    planner=planner_config();
    % Keep geometric limits fixed; use the configured rho threshold for this
    % hardware profile so valid near-boundary supports are not discarded.
    planner.reachability.threshold = 0.25;
    planner.runtime.headless=true; planner.runtime.disable_visualization=true; planner.runtime.disable_video=true;
    stamp=datestr(now,'yyyymmdd_HHMMSS'); out=fullfile(root,'results','hardware_matched_lab',stamp); if ~isfolder(out), mkdir(out); end
    result=struct('status','NOT_AVAILABLE','map_size',[cfg.map_width_m cfg.map_height_m], 'table_count',6,'task_count',8,...
        'candidate_count',NaN,'selected_docking_count',NaN,'task_coverage',NaN,'planner',planner.global_planner,...
        'path_length_m',NaN,'planning_time_s',NaN,'collision_count',NaN,'minimum_clearance_m',NaN,'geometry_feasible',false,...
        'reachability_feasible',false,'overall_feasible',false,'output_dir',out,'unreachable_tasks',[]);
    scene=build_scene_from_cfg(cfg); result.scene=scene;
    runtime_task_coordinates = build_runtime_task_coordinates(scene, cfg);
    print_runtime_task_coordinates(runtime_task_coordinates);
    task_map=assign_tasks_to_tables(scene); regions=build_task_regions(scene,task_map);
    t=tic; library=build_candidate_library(regions,scene,robot,planner); result.candidate_count=library.total_candidates;
    task_reachability = build_task_reachability_table(library, scene, robot, planner);
    print_task_reachability_table(task_reachability);
    covered=library_covered(library); true_covered = [task_reachability([task_reachability.status_pass]).task_id];
    result.task_coverage=numel(true_covered)/scene.num_tasks; result.reachability_feasible=result.task_coverage>=1;
    result.geometric_task_coverage=numel(covered); result.task_assignment_coverage=numel(covered); result.manipulation_sequence_count=0;
    result.true_covered_task_ids = true_covered;
    result.task_reachability = task_reachability;
    result.runtime_task_coordinates = runtime_task_coordinates;
    if ~result.reachability_feasible, result.unreachable_tasks=setdiff(1:scene.num_tasks,true_covered); result.status='BLOCKED'; write_summary(result,out,cfg); return; end
    [~,~,~,~,plan]=stage2_docking_planning(regions,scene,robot,planner,library);
    result.selected_docking_count=size(plan.selected_poses,1);
    result.task_coverage=numel(true_covered)/scene.num_tasks;
    result.reachability_feasible=result.task_coverage>=1;
    result.docking=plan; result.planning_time_s=toc(t);
    result.geometric_task_coverage = numel(plan.covered_tasks_predicted);
    result.task_assignment_ids = unique(cell2mat(cellfun(@(s) reshape(s.covered_tasks,1,[]), plan.task_sequence_summary, 'UniformOutput', false)));
    result.task_assignment_coverage = numel(result.task_assignment_ids);
    result.manipulation_sequence = [];
    for di=1:numel(plan.task_sequence_summary)
        s=plan.task_sequence_summary{di};
        for ai=1:numel(s.task_actions)
            a=s.task_actions(ai); a.docking_index=di; result.manipulation_sequence=[result.manipulation_sequence; a]; %#ok<AGROW>
        end
    end
    result.manipulation_sequence_count = numel(result.manipulation_sequence);
    fprintf('GEOMETRIC_TASK_COVERAGE = %d/8\nTASK_ASSIGNMENT_COVERAGE = %d/8\nMANIPULATION_SEQUENCE_COUNT = %d\nDOCKING_POSE_COUNT = %d\n', ...
        result.geometric_task_coverage,result.task_assignment_coverage,result.manipulation_sequence_count,size(plan.selected_poses,1));
    result.stage2_figure = save_stage2_docking_figure(scene, plan, robot, out);
    if result.task_coverage < 1 || numel(plan.covered_tasks_predicted) < scene.num_tasks
        result.status='BLOCKED'; result.unreachable_tasks=setdiff(1:scene.num_tasks,true_covered);
        write_summary(result,out,cfg); return;
    end
    result.status='PASSED'; result.overall_feasible=true;
    print_round11_summary(result, plan);
    % Stage 3/global planning is intentionally not entered this round.
    write_summary(result,out,cfg);
end

function c=library_covered(lib), c=[]; for i=1:numel(lib.regions), for j=1:numel(lib.regions(i).candidates), c=union(c,lib.regions(i).candidates(j).covered_tasks,'stable'); end,end,end
function rows = build_runtime_task_coordinates(scene,cfg)
    rows = repmat(struct('task_id',0,'table_id',0,'local_xy',[NaN NaN],'world_xyz',[NaN NaN NaN]),scene.num_tasks,1);
    for i=1:scene.num_tasks
        tid=scene.tasks(i).table_id; world=[scene.tasks(i).x scene.tasks(i).y scene.tasks(i).z];
        theta=cfg.table_orientations(tid); local=(rot2(-theta)*(world(1:2)-cfg.table_positions(tid,:))')';
        rows(i)=struct('task_id',i,'table_id',tid,'local_xy',local,'world_xyz',world);
    end
end
function print_runtime_task_coordinates(rows)
    fprintf('\nRUNTIME_TASK_COORDINATES\n');
    fprintf('%-8s %-8s %-22s %-22s\n','Task ID','Table ID','local coordinate','world coordinate');
    for i=1:numel(rows)
        fprintf('Task%-4d T%-7d [%.2f, %.2f]          [%.2f, %.2f, %.2f]\n',rows(i).task_id,rows(i).table_id,rows(i).local_xy(1),rows(i).local_xy(2),rows(i).world_xyz(1),rows(i).world_xyz(2),rows(i).world_xyz(3));
    end
end
function rows = build_task_reachability_table(library,scene,robot,planner)
    rows=repmat(struct('task_id',0,'table_id',0,'world_xy',[NaN NaN],'candidate_count',0,'best_pose',[NaN NaN NaN],'assigned_arm','none','radial',NaN,'rho',0,'status','FAIL','status_pass',false),scene.num_tasks,1);
    for tid=1:scene.num_tasks
        rows(tid).task_id=tid; rows(tid).table_id=scene.tasks(tid).table_id; rows(tid).world_xy=[scene.tasks(tid).x scene.tasks(tid).y];
        best_score=-Inf; best_arm='none'; best_pose=[NaN NaN NaN]; best_radial=NaN; count=0;
        for ri=1:numel(library.regions)
            local_idx=find(library.regions(ri).task_ids==tid,1);
            if isempty(local_idx), continue; end
            candidates=library.regions(ri).candidates;
            for ci=1:numel(candidates)
                if size(candidates(ci).reach_scores,1)<local_idx, continue; end
                score=candidates(ci).reach_scores(local_idx,3);
                if score<=0, continue; end
                count=count+1;
                if score>best_score
                    best_score=score; best_pose=candidates(ci).pose; best_arm=candidates(ci).arm_assignment{local_idx};
                    best_radial=arm_radial_distance(best_pose,[scene.tasks(tid).x scene.tasks(tid).y scene.tasks(tid).z],best_arm,robot);
                end
            end
        end
        rows(tid).candidate_count=count; rows(tid).best_pose=best_pose; rows(tid).assigned_arm=best_arm; rows(tid).radial=best_radial;
        if isfinite(best_score) && best_score>=planner.reachability.threshold
            rows(tid).rho=best_score; rows(tid).status='PASS'; rows(tid).status_pass=true;
        else
            rows(tid).rho=max(0,best_score); rows(tid).status='FAIL';
        end
    end
end
function d=arm_radial_distance(pose,target_xyz,arm_side,robot)
    if strcmp(arm_side,'left')
        offset=[robot.arm_base_mount_x robot.dual_arm.base_spacing/2];
    elseif strcmp(arm_side,'right')
        offset=[robot.arm_base_mount_x -robot.dual_arm.base_spacing/2];
    else
        d=NaN; return;
    end
    base=pose(1:2)+(rot2(pose(3))*offset')';
    d=norm(target_xyz(1:2)-base);
end
function print_task_reachability_table(rows)
    fprintf('\nTASK_LEVEL_REACHABILITY_TABLE\n');
    fprintf('%-8s %-8s %-17s %-12s %-25s %-8s %-8s %-8s %-8s\n','Task ID','Table','World','Candidates','Best docking pose','Arm','Radial','rho','STATUS');
    for i=1:numel(rows)
        fprintf('Task%-4d T%-7d [%.2f, %.2f]   %-12d [%.3f, %.3f, %.1fdeg]   %-8s %.3f    %.3f    %s\n',rows(i).task_id,rows(i).table_id,rows(i).world_xy(1),rows(i).world_xy(2),rows(i).candidate_count,rows(i).best_pose(1),rows(i).best_pose(2),rad2deg(rows(i).best_pose(3)),rows(i).assigned_arm,rows(i).radial,rows(i).rho,rows(i).status);
    end
end
function print_round11_summary(result,plan)
    covered=result.true_covered_task_ids;
    multi=sum(cellfun(@(s) numel(s.covered_tasks)>=2,plan.task_sequence_summary));
    fprintf('\nTRUE_COVERED_TASK_IDS = %s\n',mat2str(covered));
    fprintf('TRUE_TASK_COVERAGE = %d/%d\n',numel(covered),result.scene.num_tasks);
    fprintf('SELECTED_DOCKING_POSE_COUNT = %d\n',size(plan.selected_poses,1));
    fprintf('MULTI_TASK_SHARED_DOCK_COUNT = %d\n',multi);
    fprintf('STAGE2_STATUS = %s\n',result.status);
    fprintf('ROUND11_STAGE2_STATUS = %s\n',result.status);
end
function filename = save_stage2_docking_figure(scene,plan,robot,out)
    filename=fullfile(out,'stage2_docking_result.png');
    old_visibility=get(0,'DefaultFigureVisible');
    set(0,'DefaultFigureVisible','off');
    cleanup=onCleanup(@() set(0,'DefaultFigureVisible',old_visibility)); %#ok<NASGU>
    fig=figure('Visible','off','Color','w','Position',[80 80 1000 780]);
    draw_scene(scene,fig);
    draw_docking_result(plan.selected_poses,scene,fig,[],plan.pose_info_list,legacy_visualization_config());
    title(sprintf('Stage 2 docking poses - %d/%d true covered tasks', ...
        numel(plan.covered_tasks_predicted),scene.num_tasks),'FontSize',12);
    exportgraphics(fig,filename,'Resolution',180);
    close(fig);
    fprintf('STAGE2_DOCKING_FIGURE = %s\n',filename);
end
function export_files(r,cfg,out)
    p=r.path; n=size(p,1); rows=table((1:n)',p(:,1),p(:,2),zeros(n,1),repmat("route",n,1),repmat(0,n,1),repmat(0,n,1),false(n,1),'VariableNames',{'index','x_m','y_m','theta_rad','segment_type','target_table','target_task','is_docking_pose'}); writetable(rows,fullfile(out,'planned_path.csv'));
    d=r.docking; m=size(d.selected_poses,1); order=(1:m)'; covered=cell(m,1); arms=repmat("unknown",m,1); for i=1:m, covered{i}=mat2str(d.task_sequence_summary{i}.covered_tasks); end; T=table(order,(1:m)',d.selected_poses(:,1),d.selected_poses(:,2),d.selected_poses(:,3),covered,arms,'VariableNames',{'order','table_id','dock_id','x_m','y_m','theta_rad','covered_tasks','assigned_arm'}); writetable(T,fullfile(out,'docking_sequence.csv'));
end
function make_figures(r,cfg,out)
    names={'01_physical_scene_reconstruction','02_tasks_and_candidates','03_matched_planned_route','04_clearance_map'}; for k=1:4, f=figure('Visible','off','Color','w'); hold on; axis equal; xlim([0 cfg.map_width_m]); ylim([0 cfg.map_height_m]); for i=1:cfg.num_tables, rectangle('Position',cfg.table_left_bottom(i,:),'FaceColor',[.65 .45 .25]); text(cfg.table_positions(i,1),cfg.table_positions(i,2),sprintf('T%d',i),'Color','w','HorizontalAlignment','center'); end; for i=1:8, plot(cfg.manual_tasks(i,2),cfg.manual_tasks(i,3),'rx','LineWidth',2); end; if k>=3 && isfield(r,'path'), plot(r.path(:,1),r.path(:,2),'b-','LineWidth',1.5); end; saveas(f,fullfile(out,[names{k} '.png'])); close(f); end
end
function write_summary(r,out,cfg)
    true_count=numel(r.true_covered_task_ids);
    s=struct('map_width',cfg.map_width_m,'map_height',cfg.map_height_m,'table_count',6,'task_count',8,'candidate_count',r.candidate_count,'selected_docking_count',r.selected_docking_count,'true_covered_task_ids',r.true_covered_task_ids,'true_task_coverage',sprintf('%d/8',true_count),'geometric_task_coverage',r.geometric_task_coverage,'task_assignment_coverage',r.task_assignment_coverage,'manipulation_sequence_count',r.manipulation_sequence_count,'stage2_status',r.status,'round11_stage2_status',r.status);
    fid=fopen(fullfile(out,'matched_scene_summary.json'),'w'); fwrite(fid,jsonencode(s,'PrettyPrint',true)); fclose(fid);
    fid=fopen(fullfile(out,'trajectory_feasibility_report.txt'),'w');
    fprintf(fid,'MAP_SIZE = %.1f x %.1f m\nTABLE_COUNT = 6\nTASK_COUNT = 8\n',cfg.map_width_m,cfg.map_height_m);
    fprintf(fid,'RUNTIME_TASK_COORDINATES\n');
    for i=1:numel(r.runtime_task_coordinates)
        q=r.runtime_task_coordinates(i); fprintf(fid,'Task%d T%d local=[%.2f, %.2f] world=[%.2f, %.2f, %.2f]\n',q.task_id,q.table_id,q.local_xy(1),q.local_xy(2),q.world_xyz(1),q.world_xyz(2),q.world_xyz(3));
    end
    fprintf(fid,'TASK_LEVEL_REACHABILITY_TABLE\n');
    for i=1:numel(r.task_reachability)
        q=r.task_reachability(i); fprintf(fid,'Task%d T%d world=[%.2f, %.2f] candidates=%d dock=[%.3f, %.3f, %.1fdeg] arm=%s radial=%.3f rho=%.3f status=%s\n',q.task_id,q.table_id,q.world_xy(1),q.world_xy(2),q.candidate_count,q.best_pose(1),q.best_pose(2),rad2deg(q.best_pose(3)),q.assigned_arm,q.radial,q.rho,q.status);
    end
    fprintf(fid,'TRUE_COVERED_TASK_IDS = %s\nTRUE_TASK_COVERAGE = %d/8\nGEOMETRIC_TASK_COVERAGE = %d/8\nTASK_ASSIGNMENT_COVERAGE = %d/8\nMANIPULATION_SEQUENCE_COUNT = %d\nDOCKING_POSE_COUNT = %d\nSTAGE2_STATUS = %s\nROUND11_STAGE2_STATUS = %s\n',mat2str(r.true_covered_task_ids),true_count,r.geometric_task_coverage,r.task_assignment_coverage,r.manipulation_sequence_count,r.selected_docking_count,r.status,r.status);
    fclose(fid);
end
