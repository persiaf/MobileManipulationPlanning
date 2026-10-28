function state = p5_fixture_state(purpose)
%P5_FIXTURE_STATE Three-table, physically checked main-robot planning state.
% A narrow test table admits genuine opposite-side backup service poses.
% Named experiment scenes and every robot parameter remain unchanged.
    if nargin<1, purpose='local_repair'; end
    if ~any(strcmp(purpose,{'local_repair','incremental_navigation'}))
        error('p5_fixture_state:Purpose','Unknown fixture purpose.');
    end
    navigation_fixture=strcmp(purpose,'incremental_navigation');
    cfg=scene_config();
    cfg.map_width_m=7.0; cfg.map_height_m=6.5;
    cfg.table_mode='custom';
    cfg.table_positions=[2.025 2.025;4.525 2.025;3.275 4.525];
    cfg.num_tables=3; cfg.table_orientations=zeros(3,1);
    cfg.table.width=0.30;
    cfg.table_region_id=(1:3)';
    cfg.task_mode='manual'; cfg.task.difficulty='normal';
    cfg.start_pose=[0.625 0.625 0]; cfg.random_seed=1;
    cfg.docking_min_distance_m=0.30; cfg.docking_max_distance_m=0.40;
    cfg.manual_tasks=zeros(6,4);
    for table_id=1:3
        for arm=1:2
            task_id=2*(table_id-1)+arm;
            cfg.manual_tasks(task_id,:)=[table_id, ...
                cfg.table_positions(table_id,:)+[0.125*(2*arm-3),0],cfg.table.height];
        end
    end
    if navigation_fixture
        % Incremental grid-search tests need grid-traversable terminal cells.
        % These edge tasks remain truly reachable from the farther docks.
        cfg.manual_tasks(:,3)=cfg.manual_tasks(:,3)+0.12;
        cfg.docking_min_distance_m=0.40; cfg.docking_max_distance_m=0.40;
    end
    scene=build_scene_from_cfg(cfg); robot=robot_config(); planner=planner_config();
    planner.global_planner='astar'; planner.benchmark.enable=false;
    planner.runtime.headless=true; planner.runtime.disable_visualization=true;
    planner.runtime.disable_video=true; planner.runtime.quiet=true; planner.runtime.quiet_stage2=true;
    planner.execution.return_to_start=true; planner.routing.mode='sparse_knn';
    planner.routing.sparse_knn.k=3; planner.routing.sparse_knn.k_local=2;
    planner.routing.sparse_knn.k_cross=1; planner.robot_cfg=robot;
    if navigation_fixture
        planner.global_planner='dstar_lite'; planner.incremental_replanning_enabled=true;
    end
    task_map=assign_tasks_to_tables(scene); regions=build_task_regions(scene,task_map);
    library=build_candidate_library(regions,scene,robot,planner);
    % Retain real, fully covering candidates from the two opposing sides.
    % Keeping one per side bounds D2 trials without inventing any coverage.
    library.total_candidates=0;
    for ri=1:numel(regions)
        candidates=library.regions(ri).candidates; selected=[];
        if navigation_fixture, sides=1; else, sides=1:2; end
        for side=sides
            indices=find(arrayfun(@(c)c.side_id==side && ...
                all(ismember(regions(ri).task_ids,c.covered_tasks)),candidates));
            assert(~isempty(indices),'p5_fixture_state:NoBackup', ...
                'The test table must provide an independently reachable dock on each side.');
            [~,best]=min([candidates(indices).total_cost]); selected(end+1)=indices(best); %#ok<AGROW>
        end
        library.regions(ri).candidates=candidates(selected);
        library.regions(ri).num_candidates=numel(selected);
        library.regions(ri).generation_diagnostics.fixture_shortlist_count=numel(selected);
        library.total_candidates=library.total_candidates+numel(selected);
    end
    evalc('[~,~,~,~,plan]=stage2_docking_planning(regions,scene,robot,planner,library);');
    assert(strcmp(plan.status,'READY'),'p5_fixture_state:Blocked','Fixture tasks must be covered.');
    all_poses=[scene.start_pose;plan.selected_poses]; t=tic;
    graph=build_sparse_docking_graph(all_poses,scene.grid_map,scene.map_info, ...
        planner,[0;plan.pose_to_region_map(:)]);
    assert(graph.connectivity.ok,'p5_fixture_state:Disconnected','Fixture route must be connected.');
    visit_order=nearest_neighbor_order(graph.shortest_cost_matrix,1,false);
    route_order=[visit_order,1];
    [path,segments]=stitch_sparse_graph_path(all_poses,route_order,graph);
    stats=struct('sparse_graph',graph,'global_path',path,'segment_stats',segments, ...
        'path_len',compute_path_length(path),'route_order',route_order, ...
        'timing',struct('stage3_time_s',toc(t),'expensive_planner_calls',graph.stats.actual_astar_calls));
    task_state=struct('predicted_tasks',plan.covered_tasks_predicted, ...
        'commanded_tasks',[],'executed_tasks',[],'verified_tasks',[]);
    rc2=build_rc2_plan(plan,visit_order,route_order,path,segments,graph,scene,robot,planner);
    state=normalize_planning_state(build_planning_state(scene,robot,planner,library,rc2,task_state,struct()));
    state.scale='fixture'; state.seed=1;
    state.scene=scene; state.tasks=scene.tasks; state.plan=plan;
    state.regions=regions; state.task_table_map=task_map; state.candidate_library=library;
    state.all_poses=[scene.start_pose;plan.selected_poses]; state.stats=stats;
    state.task_state=task_state;
    % Explicit short safe endpoint motions connect grid routes to exact docks.
    state=complete_replanning_route(state);
    state.rc2_plan=build_rc2_plan(plan,visit_order,route_order,state.stats.global_path, ...
        state.stats.segment_stats,state.stats.sparse_graph,scene,robot,planner);
    validation=validate_replanning_state(state,state);
    assert(validation.valid,'p5_fixture_state:InvalidGeometry','%s',validation.reason);
    state.path_validation=validation;
end
