function tests=test_main_task_navigation_protocol
%TEST_MAIN_TASK_NAVIGATION_PROTOCOL Main geometry, frozen subpool and cache IO.
% Does not score the full main-scene IK matrix. Cache IO uses one tiny pair.
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    cached=fullfile('results','cache','main_task_navigation_setup_20261111.mat');
    if isfile(cached)
        data=load(cached,'s');testCase.TestData.main=data.s;
    else
        testCase.TestData.main=main_task_navigation_setup();
    end
end

function testMainSceneAndTasksMatchAnIndependentDefaultBuild(testCase)
    setup=testCase.TestData.main;source=scene_config();fresh=build_scene_from_cfg(source);
    verifyEqual(testCase,setup.source_scene_cfg,source);
    verifyEqual(testCase,setup.scene.grid_map,fresh.grid_map);
    verifyEqual(testCase,setup.scene.map_info,fresh.map_info);
    verifyEqual(testCase,setup.scene.scene_cfg,fresh.scene_cfg);
    verifyEqual(testCase,setup.scene.start_pose,fresh.start_pose);
    verifyEqual(testCase,setup.scene.tasks,fresh.tasks);
    verifyEqual(testCase,setup.scene.num_tasks,36);
    verifyEqual(testCase,numel(setup.regions),16);
    verifyEqual(testCase,source.random_seed,2026);
end

function testSubpoolMatchesTheExportedOriginalDenseMembers(testCase)
    setup=testCase.TestData.main;mapping=setup.dense_mapping;pool=setup.pool;
    verifyEqual(testCase,height(mapping),26660);
    verifyEqual(testCase,numel(pool),906);
    verifyEqual(testCase,[pool.candidate_id],1:numel(pool));
    verifyEqual(testCase,numel(unique([pool.dense_candidate_id])),numel(pool));
    for k=1:numel(pool)
        row=find(mapping.dense_candidate_id==pool(k).dense_candidate_id);
        verifyEqual(testCase,numel(row),1);
        verifyTrue(testCase,logical(mapping.collision_valid(row)));
        verifyTrue(testCase,logical(mapping.selected(row)));
        verifyEqual(testCase,mapping.candidate_id(row),pool(k).candidate_id);
        verifyEqual(testCase,mapping.table_id(row),pool(k).table_id);
        verifyEqual(testCase,mapping.side_id(row),pool(k).side_id);
        verifyEqual(testCase,[mapping.x_m(row),mapping.y_m(row),mapping.yaw_rad(row)],pool(k).pose);
        verifyFalse(testCase,check_footprint_collision(pool(k).pose,setup.scene.grid_map, ...
            setup.scene.map_info,setup.robot));
    end
end

function testRetainedPosesAndYawsReallyOccurInRegeneratedDenseFields(testCase)
    setup=testCase.TestData.main;scene=setup.scene;cfg=scene.scene_cfg;
    for ri=1:numel(setup.regions)
        region=setup.regions(ri);t=region.table_id;tids=region.task_ids;
        targets=[[scene.tasks(tids).x]',[scene.tasks(tids).y]'];
        dense=compute_reachability_field(cfg.table_positions(t,:),cfg.table_orientations(t), ...
            cfg,setup.robot,scene.map_info,mean(targets,1),targets);
        selected=setup.pool([setup.pool.table_id]==t);
        for ci=1:numel(selected)
            delta=vecnorm(dense.candidates-selected(ci).pose,2,2);
            same=find(delta<1e-12&dense.side_ids(:)==selected(ci).side_id);
            verifyNotEmpty(testCase,same,'A selected yaw or XY was invented outside the dense source.');
        end
    end
end

function testPerSideGeometryBudgetsAndFlatteningOrder(testCase)
    setup=testCase.TestData.main;pool=setup.pool;flattened=[];
    for ri=1:numel(setup.regions)
        table_id=setup.regions(ri).table_id;indices=find([pool.table_id]==table_id);
        flattened=[flattened,indices]; %#ok<AGROW>
        verifyLessThanOrEqual(testCase,numel(indices),64);
        for side=1:4
            selected=pool([pool.table_id]==table_id&[pool.side_id]==side);
            if isempty(selected),continue;end
            P=reshape([selected.pose],3,[])';
            xy=round(P(:,1:2)*1e10)/1e10;[unique_xy,~,groups]=unique(xy,'rows');
            verifyLessThanOrEqual(testCase,size(unique_xy,1),8);
            verifyLessThanOrEqual(testCase,max(accumarray(groups,1)),2);
        end
    end
    verifyEqual(testCase,flattened,1:numel(pool));
    verifyEqual(testCase,sum(setup.table_counts.retained_candidates),numel(pool));
end

function testThreeProfilesKeepAllMainAnchorsAndTCP(testCase)
    setup=testCase.TestData.main;first_tcp=[];
    for k=1:setup.scene.num_tasks
        anchor=setup.scene.tasks(k);
        for mode=setup.mode_order
            task=comparison_manipulation_task(anchor,mode{1});cfg=resolve_manipulation_task(task);
            verifyEqual(testCase,[task.x task.y task.z],[anchor.x anchor.y anchor.z]);
            verifyEqual(testCase,task.id,anchor.id);verifyEqual(testCase,task.table_id,anchor.table_id);
            if isempty(first_tcp),first_tcp=cfg.T_ee_tcp;end
            verifyEqual(testCase,cfg.T_ee_tcp,first_tcp);
            if strcmp(mode{1},'table_wipe')
                verifyEqual(testCase,norm(diff(cfg.waypoints_world,1,1)),0.12,'AbsTol',1e-12);
            end
        end
    end
end

function testDefaultPipelineParametersRemainUnchanged(testCase)
    setup=testCase.TestData.main;baseline=planner_config();canonical=robot_config();
    verifyEqual(testCase,baseline.strategy.name,'bottom_up');
    verifyEqual(testCase,baseline.global_planner,'convex_astar');
    verifyEqual(testCase,baseline.robot_cfg,canonical);
    verifyEqual(testCase,canonical.single_arm.reach_radius,0.278);
    verifyEqual(testCase,rmfield(setup.robot,'manipulator'),canonical);
    verifyEqual(testCase,setup.planner.robot_cfg,setup.robot);
    verifyEqual(testCase,setup.planner.global_planner,'astar');
    verifyEqual(testCase,setup.planner.routing.mode,'dense');
    verifyTrue(testCase,setup.planner.execution.return_to_start);
    verifyEqual(testCase,setup.planner.cost_weights,baseline.cost_weights);
    verifyEqual(testCase,setup.planner.edge_cost,baseline.edge_cost);
end

function testTinyLibraryPreservesPairsAndReusesOnlyItsSignedArtifacts(testCase)
    mini=tiny_setup(testCase.TestData.main);folder=tempname;mkdir(folder);
    cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    [lib,scene,proof]=build_main_task_navigation_library(mini,'position_only',folder);
    verifyEqual(testCase,lib.total_candidates,1);verifyEqual(testCase,proof.pair_count,1);
    verifyEqual(testCase,proof.expected_pair_count,1);verifyTrue(testCase,proof.all_pairs_preserved);
    verifyFalse(testCase,proof.reused_from_disk);verifyEqual(testCase,scene.tasks.task_type,'position_only');
    pairs=readtable(fullfile(folder,'pair_support_position_only.csv'));
    verifyEqual(testCase,height(pairs),1);
    verifyTrue(testCase,ismember('cache_kind',pairs.Properties.VariableNames));
    contents=whos('-file',fullfile(folder,'candidate_library_position_only.mat'));
    verifyFalse(testCase,ismember('cache',{contents.name}));
    [again,~,reused]=build_main_task_navigation_library(mini,'position_only',folder);
    verifyEqual(testCase,again,lib);verifyTrue(testCase,reused.reused_from_disk);
    changed=mini;changed.pool.pose(1)=changed.pool.pose(1)+0.01;
    verifyError(testCase,@()build_main_task_navigation_library(changed,'position_only',folder), ...
        'build_main_task_navigation_library:DirectorySignatureMismatch');
end

function setup=tiny_setup(source)
    setup=source;task=source.scene.tasks(source.regions(1).task_ids(1));task.id=1;
    setup.scene.tasks=task;setup.scene.num_tasks=1;
    setup.regions=source.regions(1);setup.regions.task_ids=1;
    setup.pool=source.pool(1);setup.pool.candidate_id=1;
end
