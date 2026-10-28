function identity = demo_state_cache_identity(scene, robot_cfg, planner_cfg, profile_id, seed)
%DEMO_STATE_CACHE_IDENTITY Bind a demo state to actual inputs and sources.
% Runtime/plot options do not affect identity. Source hashes invalidate cached
% geometry, candidates and routes after an implementation change.
    robot_cfg = validate_main_robot_config(robot_cfg);
    identity.schema = 'validated_demo_state_v1';
    identity.profile = upper(char(profile_id));
    identity.seed = seed;
    payload = struct('grid_map',scene.grid_map,'map_info',scene.map_info, ...
        'scene_cfg',scene.scene_cfg,'tasks',scene.tasks,'start_pose',scene.start_pose);
    identity.scene_signature = sha256(jsonencode(payload));
    identity.robot_signature = sha256(jsonencode(robot_cfg));
    for name = {'runtime','visualization','timing'}
        if isfield(planner_cfg,name{1}), planner_cfg=rmfield(planner_cfg,name{1}); end
    end
    planner_cfg.robot_cfg = robot_cfg;
    identity.planner_signature = sha256(jsonencode(planner_cfg));
    identity.implementation_signature = implementation_signature();
    identity.key = sha256(jsonencode(identity));
end

function signature = implementation_signature()
    root=fileparts(fileparts(mfilename('fullpath')));
    folders={'config','environment','collision','docking','manipulator', ...
        'planning_strategy','global_planner','execution','replanning','utils','pipeline','experiments'};
    paths={};
    for k=1:numel(folders)
        files=dir(fullfile(root,folders{k},'**','*.m'));
        for j=1:numel(files)
            paths{end+1}=fullfile(files(j).folder,files(j).name); %#ok<AGROW>
        end
    end
    paths=sort(paths); pieces=cell(size(paths));
    for k=1:numel(paths)
        relative=strrep(paths{k}(numel(root)+2:end),'\','/');
        pieces{k}=[relative ':' sha256(fileread(paths{k}))];
    end
    signature=sha256(strjoin(pieces,newline));
end

function signature = sha256(value)
    digest=java.security.MessageDigest.getInstance('SHA-256');
    digest.update(unicode2native(char(value),'UTF-8'));
    bytes=typecast(digest.digest(),'uint8');
    signature=lower(reshape(dec2hex(bytes,2)',1,[]));
end
