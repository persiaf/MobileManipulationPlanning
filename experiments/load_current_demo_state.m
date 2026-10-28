function [state, info] = load_current_demo_state(profile_id, seed, planner_cfg, varargin)
%LOAD_CURRENT_DEMO_STATE Build the requested V scene or reuse an exact cache.
% Old L1/L2/L3 files are never substituted for presentation profiles.
% build_if_missing=false performs cheap scene/cache inspection only.
    profile_id=upper(char(profile_id));
    if ~any(strcmp(profile_id,{'V1','V2','V3'}))
        error('load_current_demo_state:UnknownProfile','Expected V1, V2, or V3.');
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    opts=struct('cache_dir',fullfile(root,'results','cache','current_demo_states'), ...
        'build_if_missing',true,'save_cache',true,'force_rebuild',false);
    if mod(numel(varargin),2)~=0
        error('load_current_demo_state:Options','Options must be name/value pairs.');
    end
    for k=1:2:numel(varargin)
        name=lower(char(varargin{k}));
        if ~isfield(opts,name), error('load_current_demo_state:Options','Unknown option %s.',name); end
        opts.(name)=varargin{k+1};
    end
    robot_cfg=robot_config(); planner_cfg.robot_cfg=robot_cfg;
    scene=build_complex_visual_scene(profile_id,seed);
    identity=demo_state_cache_identity(scene,robot_cfg,planner_cfg,profile_id,seed);
    cache_file=fullfile(opts.cache_dir,sprintf('demo_state_%s_seed%d_%s.mat', ...
        lower(profile_id),seed,identity.key));
    info=struct('status','miss','reason','no cache for current inputs', ...
        'cache_file',cache_file,'identity',identity,'reused',false);
    state=struct('scale',profile_id,'seed',seed,'scene',scene, ...
        'robot_cfg',robot_cfg,'planner_cfg',planner_cfg,'cache_info',info);
    if ~opts.force_rebuild && isfile(cache_file)
        try
            data=load(cache_file,'state','cache_identity');
            [ok,reason]=cache_matches(data,identity);
            if ok
                state=data.state; state.planner_cfg=planner_cfg;
                info.status='hit'; info.reason='exact scene/model/planner/source identity'; info.reused=true;
                state.cache_info=info; return;
            end
            info.reason=reason;
        catch ME
            info.reason=['unreadable or invalid cache: ' ME.identifier];
        end
    end
    state.cache_info=info;
    if ~opts.build_if_missing, return; end
    t=tic;
    task_table_map=assign_tasks_to_tables(scene);
    regions=build_task_regions(scene,task_table_map);
    library=build_candidate_library(regions,scene,robot_cfg,planner_cfg);
    [~,~,~,~,plan]=stage2_docking_planning(regions,scene,robot_cfg,planner_cfg,library);
    state.task_table_map=task_table_map; state.regions=regions;
    state.candidate_library=library; state.plan=plan;
    state.all_poses=[scene.start_pose;plan.selected_poses];
    state.stats=struct('global_path',[],'path_len',NaN,'route_order',[], ...
        'path_validation',struct('valid',false,'reason','stage 2 did not produce a READY plan'));
    if strcmpi(plan.status,'READY')
        [~,path,all_poses,visit_order,~,path_len,stats]=stage3_global_planning( ...
            scene,plan.selected_poses,plan.task_sequence_summary,task_table_map, ...
            robot_cfg,planner_cfg,plan.pose_info_list,struct(),plan);
        stats.global_path=path; stats.path_len=path_len;
        % visit_order omits optional return; stats.route_order is the actual route.
        stats.visit_order=visit_order;
        state.all_poses=all_poses; state.stats=stats;
        if isfield(stats,'rc2_plan'), state.rc2_plan=stats.rc2_plan; end
        info.status='built';
    else
        info.status='built_blocked';
    end
    info.reason='regenerated requested scene with current implementation';
    info.build_time_s=toc(t); state.cache_info=info;
    if opts.save_cache
        if ~isfolder(opts.cache_dir), mkdir(opts.cache_dir); end
        cache_identity=identity; %#ok<NASGU>
        save(cache_file,'state','cache_identity','-v7.3');
    end
end

function [ok,reason] = cache_matches(data,expected)
    ok=false; reason='missing cache identity or planning result';
    if ~isfield(data,'cache_identity') || ~isequaln(data.cache_identity,expected) || ...
            ~isfield(data,'state') || ~isfield(data.state,'plan') || ~isfield(data.state,'stats')
        return;
    end
    state=data.state;
    if ~all(isfield(state,{'scene','robot_cfg','planner_cfg','scale','seed'})), return; end
    actual=demo_state_cache_identity(state.scene,state.robot_cfg,state.planner_cfg,state.scale,state.seed);
    if ~isequaln(actual,expected)
        reason='cached scene/model/planner content does not match requested identity'; return;
    end
    if ~isfield(state.plan,'status') || ~any(strcmpi(state.plan.status,{'READY','BLOCKED'}))
        reason='cached plan lacks a supported status'; return;
    end
    if strcmpi(state.plan.status,'READY') && ...
            (~isfield(state.stats,'global_path') || size(state.stats.global_path,1)<2)
        reason='READY cache lacks planner waypoints'; return;
    end
    ok=true; reason='';
end
