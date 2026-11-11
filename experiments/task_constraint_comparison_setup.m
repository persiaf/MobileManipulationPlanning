function setup=task_constraint_comparison_setup()
%TASK_CONSTRAINT_COMPARISON_SETUP Frozen geometry and budgets for all modes.
    cfg=scene_config();
    cfg.map_width_m=7;cfg.map_height_m=4.4;
    cfg.table_positions=[2,2.2;4.8,2.2];
    cfg.table_orientations=[0;0];cfg.num_tables=2;
    cfg.start_pose=[0.6,0.6,0];cfg.random_seed=2026;
    cfg.task_mode='manual';
    local=[-.4,-.08;-.2,.08;0,0;.2,-.08;.4,.08];
    cfg.manual_tasks=zeros(10,4);
    for t=1:2
        xy=table_local_to_world(cfg.table_positions(t,:),0,local);
        cfg.manual_tasks((t-1)*5+(1:5),:)=[repmat(t,5,1),xy,repmat(cfg.table.height,5,1)];
    end
    scene=build_scene_from_cfg(cfg);
    scene.tasks=normalize_task_fields(scene.tasks);
    robot=robot_config();robot.manipulator=task_manipulation_config(robot);
    planner=planner_config();planner.robot_cfg=robot;
    planner.global_planner='astar';planner.routing.mode='dense';
    planner.enable_2opt=false;planner.execution.return_to_start=true;
    planner.benchmark.enable=false;
    planner.candidate_augmentation.enable=false;
    planner.reachability.mode='nominal';
    planner.runtime.headless=true;planner.runtime.disable_visualization=true;
    planner.runtime.disable_video=true;
    planner.visualization.show_scene=false;planner.visualization.show_docking=false;
    planner.visualization.show_global_path=false;
    pool=struct('candidate_id',{},'table_id',{},'side_id',{},'pose',{});
    raw_count=0;
    for t=1:2
        center=cfg.table_positions(t,:);
        for side=1:4
            if side<=2,lat=[-.4,0,.4];else,lat=[-.12,0,.12];end
            for distance=[.4,.6]
                for tangent=lat
                    switch side
                        case 1,xy=center+[tangent,cfg.table.width/2+distance];yaw=-pi/2;
                        case 2,xy=center+[tangent,-cfg.table.width/2-distance];yaw=pi/2;
                        case 3,xy=center+[-cfg.table.length/2-distance,tangent];yaw=0;
                        case 4,xy=center+[cfg.table.length/2+distance,tangent];yaw=pi;
                    end
                    for offset=deg2rad([-20,0,20])
                        raw_count=raw_count+1;pose=[xy,wrapToPiLocal(yaw+offset)];
                        if check_footprint_collision(pose,scene.grid_map,scene.map_info,robot),continue;end
                        pool(end+1)=struct('candidate_id',raw_count,'table_id',t, ...
                            'side_id',side,'pose',pose); %#ok<AGROW>
                    end
                end
            end
        end
    end
    setup=struct('scene',scene,'robot',robot,'planner',planner,'pool',pool, ...
        'raw_candidate_count',raw_count,'mode_order',{{'position_only','top_down_grasp','table_wipe'}}, ...
        'reference_dock',[2,1.45,pi/2],'workspace_nx',21,'workspace_ny',9, ...
        'matched_nx',11,'matched_ny',5,'wipe_length_m',.12);
end
