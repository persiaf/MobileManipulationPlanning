function tests = test_frozen_scene_identity_repair
%TEST_FROZEN_SCENE_IDENTITY_REPAIR Cache reuse must preserve scene semantics.
    tests=functiontests(localfunctions);
end

function testEachVisualProfileKeepsItsOwnScene(testCase)
    expected=[28 40 28]; cfg=planner_config();
    for k=1:3
        [state,info]=load_current_demo_state(sprintf('V%d',k),2026,cfg, ...
            'build_if_missing',false,'cache_dir',tempname);
        verifyEqual(testCase,size(state.scene.scene_cfg.table_positions,1),expected(k));
        verifyEqual(testCase,state.scene.scene_cfg.scene_complexity_case,sprintf('V%d',k));
        verifyEqual(testCase,info.status,'miss');
        verifyFalse(testCase,isfield(state,'plan'));
    end
end

function testIdentityIncludesMapTasksThresholdAndReturn(testCase)
    [state,info]=inspection(); a=info.identity;
    scene=state.scene; scene.grid_map(2,2)=1-scene.grid_map(2,2);
    b=demo_state_cache_identity(scene,state.robot_cfg,state.planner_cfg,'V1',2026);
    verifyNotEqual(testCase,a.key,b.key);
    scene=state.scene; scene.tasks(1).x=scene.tasks(1).x+0.001;
    b=demo_state_cache_identity(scene,state.robot_cfg,state.planner_cfg,'V1',2026);
    verifyNotEqual(testCase,a.key,b.key);
    planner=state.planner_cfg; planner.reachability.threshold=planner.reachability.threshold+0.01;
    b=demo_state_cache_identity(state.scene,state.robot_cfg,planner,'V1',2026);
    verifyNotEqual(testCase,a.key,b.key);
    planner=state.planner_cfg; planner.execution.return_to_start=~planner.execution.return_to_start;
    b=demo_state_cache_identity(state.scene,state.robot_cfg,planner,'V1',2026);
    verifyNotEqual(testCase,a.key,b.key);
end

function testRuntimeControlsDoNotInvalidatePlanningIdentity(testCase)
    [state,info]=inspection(); planner=state.planner_cfg;
    planner.runtime.figure_visible='off'; planner.runtime.quiet=true;
    planner.visualization.show_scene=~planner.visualization.show_scene;
    b=demo_state_cache_identity(state.scene,state.robot_cfg,planner,'V1',2026);
    verifyEqual(testCase,info.identity.key,b.key);
end

function testExactBlockedCacheCanBeInspectedWithoutPlanning(testCase)
    folder=tempname; mkdir(folder); cleanup=onCleanup(@()remove_temp_folder(folder)); %#ok<NASGU>
    [state,info]=inspection(folder);
    state.plan=struct('status','BLOCKED','selected_poses',zeros(0,3));
    state.stats=struct('global_path',[]);
    cache_identity=info.identity; save(info.cache_file,'state','cache_identity');
    [loaded,hit]=load_current_demo_state('V1',2026,planner_config(), ...
        'cache_dir',folder,'build_if_missing',false);
    verifyEqual(testCase,hit.status,'hit'); verifyTrue(testCase,hit.reused);
    verifyEqual(testCase,loaded.plan.status,'BLOCKED');
end

function testL1SceneCannotMasqueradeAsV1Cache(testCase)
    folder=tempname; mkdir(folder); cleanup=onCleanup(@()remove_temp_folder(folder)); %#ok<NASGU>
    [state,info]=inspection(folder);
    state.scene=build_scene_from_cfg(large_scale_profile_config('L1',2026));
    state.plan=struct('status','BLOCKED','selected_poses',zeros(0,3));
    state.stats=struct('global_path',[]);
    cache_identity=info.identity; save(info.cache_file,'state','cache_identity');
    [loaded,hit]=load_current_demo_state('V1',2026,planner_config(), ...
        'cache_dir',folder,'build_if_missing',false);
    verifyEqual(testCase,hit.status,'miss'); verifyFalse(testCase,hit.reused);
    verifyEqual(testCase,size(loaded.scene.scene_cfg.table_positions,1),28);
end

function testOldImplementationTagAndUnprovenReadyPathAreRejected(testCase)
    folder=tempname; mkdir(folder); cleanup=onCleanup(@()remove_temp_folder(folder)); %#ok<NASGU>
    [state,info]=inspection(folder);
    state.plan=struct('status','BLOCKED','selected_poses',zeros(0,3));
    state.stats=struct('global_path',[]);
    cache_identity=info.identity; cache_identity.implementation_signature='old_geometry';
    save(info.cache_file,'state','cache_identity');
    [~,hit]=load_current_demo_state('V1',2026,planner_config(), ...
        'cache_dir',folder,'build_if_missing',false);
    verifyEqual(testCase,hit.status,'miss');
    cache_identity=info.identity; state.plan.status='READY';
    save(info.cache_file,'state','cache_identity');
    [~,hit]=load_current_demo_state('V1',2026,planner_config(), ...
        'cache_dir',folder,'build_if_missing',false);
    verifyEqual(testCase,hit.status,'miss');
end

function testAfterFigureDoesNotRelabelOldRouteAsRepaired(testCase)
    state=p5_fixture_state(); state.stats.global_path=[state.scene.start_pose;state.plan.selected_poses];
    [~,event]=apply_dynamic_event(state,'ADD_LOCAL_OBSTACLE',1);
    cfg=visualization_config(); cfg.figure_visible='off'; cfg.save_png=false;
    result=struct('no_collision_invalid_route',true,'fallback',false,'local_astar_calls',0);
    figs=visualize_dynamic_replan(state,event,result,cfg,'prefix','');
    cleanup=onCleanup(@()close_figures(figs)); %#ok<NASGU>
    verifyFalse(testCase,figs.after.UserData.repaired_route_validated);
    lines=findobj(figs.after,'Type','line');
    labels=get(lines,'DisplayName');
    if ischar(labels), labels={labels}; end
    verifyFalse(testCase,any(strcmp(labels,'Collision-free route')));
end

function [state,info] = inspection(folder)
    if nargin<1, folder=tempname; end
    [state,info]=load_current_demo_state('V1',2026,planner_config(), ...
        'cache_dir',folder,'build_if_missing',false);
end

function close_figures(figs)
    if isvalid(figs.before),close(figs.before);end
    if isvalid(figs.after),close(figs.after);end
end

function remove_temp_folder(folder)
    resolved=char(java.io.File(folder).getCanonicalPath());
    temp_root=char(java.io.File(tempdir).getCanonicalPath());
    assert(startsWith(lower(resolved),[lower(temp_root) filesep]), ...
        'test_frozen_scene_identity_repair:CleanupPath','Cleanup must stay inside tempdir.');
    if isfolder(resolved), rmdir(resolved,'s'); end
end
