function data=prepare_joint_task_navigation(source_dir)
%PREPARE_JOINT_TASK_NAVIGATION Reuse frozen IK evidence after identity checks.
    if nargin<1,source_dir=fullfile('results','task_constraint_comparison_20261111');end
    frozen=load(fullfile(source_dir,'checkpoint_table_wipe.mat'),'setup','mode_results','metadata');
    fresh=task_constraint_comparison_setup();
    assert(isequaln(frozen.setup.robot,fresh.robot),'JointComparison:RobotChanged');
    assert(isequaln(frozen.setup.pool,fresh.pool),'JointComparison:CandidatesChanged');
    assert(isequaln(frozen.setup.scene.grid_map,fresh.scene.grid_map),'JointComparison:MapChanged');
    assert(isequaln(frozen.setup.scene.tasks,fresh.scene.tasks),'JointComparison:TasksChanged');
    setup=frozen.setup;planner=setup.planner;
    current=planner_config();planner.strategy.joint=current.strategy.joint;
    task_map=assign_tasks_to_tables(setup.scene);
    regions=build_task_regions(setup.scene,task_map);
    all_poses=[setup.scene.start_pose;reshape([setup.pool.pose],3,[])'];
    n=size(all_poses,1);needed=false(n);mode_needed=cell(1,3);
    covers=cell(3,numel(regions));certificates=cell(size(covers));
    libraries=cell(1,3);scenes=cell(1,3);
    for mi=1:3
        old=frozen.mode_results{mi};library=old.plan.candidate_library;scenes{mi}=old.scene;
        validate_candidate_library(library,regions,old.scene,setup.robot,planner);
        mode_needed{mi}=false(n);offset=0;cores=cell(1,numel(regions));
        for ri=1:numel(regions)
            cs=library.regions(ri).candidates;
            assert(isequal(reshape([cs.pose],3,[])',all_poses(offset+(2:numel(cs)+1),:)));
            [local,cert]=enumerate_minimum_task_covers(cs,regions(ri).task_ids,planner.strategy.joint);
            assert(cert.feasible,'JointComparison:TaskUnsupported');
            covers{mi,ri}=local+offset+1;certificates{mi,ri}=cert;
            cores{ri}=cert.core_candidate_indices+offset+1;
            mode_needed{mi}(1,cores{ri})=true;mode_needed{mi}(cores{ri},1)=true;
            for row=1:size(local,1)
                nodes=covers{mi,ri}(row,:);mode_needed{mi}(nodes,nodes)=true;
            end
            offset=offset+numel(cs);
        end
        for a=1:numel(regions)
            for b=a+1:numel(regions)
                mode_needed{mi}(cores{a},cores{b})=true;
                mode_needed{mi}(cores{b},cores{a})=true;
            end
        end
        fixed=[1;old.selected_pool_ids(:)+1];mode_needed{mi}(fixed,fixed)=true;
        mode_needed{mi}(1:n+1:end)=false;needed=needed|mode_needed{mi};
        libraries{mi}=library;
        minimum=sum(cellfun(@(c)c.minimum_dock_count,certificates(mi,:)));
        assert(size(old.plan.selected_poses,1)==minimum, ...
            'JointComparison:BaselineNotMinimum','The controlled baseline must already use the certified minimum dock count.');
    end
    signature=build_struct_signature(struct('poses',all_poses,'needed',needed, ...
        'path_semantics',build_path_cache_signature(setup.scene.grid_map,setup.scene.map_info,planner), ...
        'edge_cost',planner.edge_cost));
    data=struct('setup',setup,'planner',planner,'regions',regions,'task_map',{task_map}, ...
        'all_poses',all_poses,'needed',needed,'mode_needed',{mode_needed}, ...
        'libraries',{libraries},'scenes',{scenes},'covers',{covers}, ...
        'certificates',{certificates},'baseline',{frozen.mode_results}, ...
        'source_metadata',frozen.metadata,'source_dir',source_dir,'graph_signature',signature);
end
