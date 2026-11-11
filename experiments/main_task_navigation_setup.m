function setup=main_task_navigation_setup(scene_cfg)
%MAIN_TASK_NAVIGATION_SETUP Exact default scene/targets, geometric budget pool.
% Main map/tables/tasks are never edited. Sampling is independent of task
% types, IK scores and planner outcomes: <=8 XY x 2 yaw per table side.
    if nargin<1||isempty(scene_cfg),scene_cfg=scene_config();end
    scene=build_scene_from_cfg(scene_cfg);
    robot=robot_config();robot.manipulator=task_manipulation_config(robot);
    planner=planner_config();planner.robot_cfg=robot;
    planner.global_planner='astar';planner.routing.mode='dense';
    planner.routing.directed_costs=true;
    planner.routing.finite_nn_restarts=true;
    planner.execution.return_to_start=true;planner.enable_2opt=false;
    planner.benchmark.enable=false;planner.candidate_augmentation.enable=false;
    planner.reachability.mode='nominal';
    planner.runtime.headless=true;planner.runtime.disable_visualization=true;planner.runtime.disable_video=true;
    planner.visualization.show_scene=false;planner.visualization.show_docking=false;
    planner.visualization.show_global_path=false;
    task_map=assign_tasks_to_tables(scene);regions=build_task_regions(scene,task_map);
    pool=struct('candidate_id',{},'dense_candidate_id',{},'table_id',{},'side_id',{},'pose',{});
    dense_rows={};dense_id=0;table_rows={};
    for ri=1:numel(regions)
        table_id=regions(ri).table_id;center=scene_cfg.table_positions(table_id,:);
        theta=scene_cfg.table_orientations(table_id);
        tids=regions(ri).task_ids;targets=[[scene.tasks(tids).x]',[scene.tasks(tids).y]'];
        field=compute_reachability_field(center,theta,scene_cfg,robot,scene.map_info,mean(targets,1),targets);
        P=field.candidates;side_ids=field.side_ids;
        valid=false(size(P,1),1);
        for ci=1:size(P,1),valid(ci)=~check_footprint_collision(P(ci,:),scene.grid_map,scene.map_info,robot);end
        selected=[];
        for side=1:4
            indices=find(valid&side_ids(:)==side);
            if isempty(indices),continue;end
            xy_local=(rot2(-theta)*(P(indices,1:2)-center)')';
            rounded=round(xy_local*1e10)/1e10;
            [xy,~,groups]=unique(rounded,'rows','sorted');
            switch side
                case 1,seed=[0,scene_cfg.table.width/2+.30];normal=theta-pi/2;
                case 2,seed=[0,-scene_cfg.table.width/2-.30];normal=theta+pi/2;
                case 3,seed=[-scene_cfg.table.length/2-.30,0];normal=theta;
                case 4,seed=[scene_cfg.table.length/2+.30,0];normal=theta+pi;
            end
            [~,first]=min(vecnorm(xy-seed,2,2));chosen=first;
            while numel(chosen)<min(8,size(xy,1))
                distances=inf(size(xy,1),1);
                for j=chosen(:)',distances=min(distances,vecnorm(xy-xy(j,:),2,2));end
                distances(chosen)=-Inf;[~,next]=max(distances);chosen(end+1)=next; %#ok<AGROW>
            end
            for g=chosen
                members=indices(groups==g);used=[];
                for desired=[normal,normal+pi/12]
                    available=setdiff(members,used,'stable');
                    if isempty(available),break;end
                    [~,index]=min(abs(wrapToPiLocal(P(available,3)-desired)));
                    used(end+1)=available(index); %#ok<AGROW>
                end
                selected=[selected,used]; %#ok<AGROW>
            end
        end
        selected=sort(unique(selected));sub_ids=zeros(size(P,1),1);
        for ci=selected
            id=numel(pool)+1;sub_ids(ci)=id;
            pool(end+1)=struct('candidate_id',id,'dense_candidate_id',dense_id+ci, ...
                'table_id',table_id,'side_id',side_ids(ci),'pose',P(ci,:)); %#ok<AGROW>
        end
        for ci=1:size(P,1)
            dense_rows(end+1,:)={dense_id+ci,table_id,side_ids(ci),P(ci,1),P(ci,2),P(ci,3), ...
                valid(ci),sub_ids(ci)>0,sub_ids(ci)}; %#ok<AGROW>
        end
        table_rows(end+1,:)={table_id,numel(tids),size(P,1),sum(valid),numel(selected)}; %#ok<AGROW>
        dense_id=dense_id+size(P,1);
    end
    setup=struct('scene',scene,'source_scene_cfg',scene_cfg,'robot',robot,'planner',planner, ...
        'regions',regions,'task_map',{task_map},'pool',pool, ...
        'dense_mapping',cell2table(dense_rows,'VariableNames',{'dense_candidate_id','table_id','side_id', ...
            'x_m','y_m','yaw_rad','collision_valid','selected','candidate_id'}), ...
        'table_counts',cell2table(table_rows,'VariableNames',{'table_id','task_count','dense_candidates', ...
            'footprint_valid_candidates','retained_candidates'}), ...
        'mode_order',{{'position_only','top_down_grasp','table_wipe'}}, ...
        'sampling_rule','per side farthest-point 8 XY seeded at side midpoint/d=.30; two original yaw nearest normal and normal+15deg', ...
        'candidate_budget_per_table',64);
end
