function [summary, task_rows] = audit_independent_reachability(scene, robot_cfg, planner_cfg, library, options)
%AUDIT_INDEPENDENT_REACHABILITY Audit coverage without changing the planner.
%   One row is returned for every scene task. Candidate poses are reevaluated
%   with independent_geometric_reachability. Tasks without independent library
%   support receive a deterministic, denser table-frame pose search. A search
%   failure means not_found_in_audited_samples, never physical infeasibility.
%   The only infeasibility certificate used here is a violated z hard bound,
%   whose value cannot change with planar docking pose in the current model.
%
%   Explicit options: tangential_step_m (0.025), normal_step_m (0.025),
%   heading_step_deg (5), heading_span_deg (90), tangential_half_window_m
%   (0.35), corner_extension_m (0.20), normal_offsets_m ([] means scene min:max),
%   max_samples_per_task (40000), stop_after_first_support (true).
%   A sample cap violation raises an error rather than dropping tasks/poses.

    if nargin < 5 || isempty(options), options = struct(); end
    started = tic;
    robot_cfg = validate_main_robot_config(robot_cfg);
    options = audit_options(options,scene.scene_cfg);
    threshold = planner_cfg.reachability.threshold;
    mode = field_or(planner_cfg.reachability,'mode','nominal');
    if ~strcmpi(mode,'nominal')
        error('audit_independent_reachability:NominalOnly', ...
            'This independent audit compares nominal coverage; robust coverage needs a separate perturbation audit.');
    end
    if scene.num_tasks ~= numel(scene.tasks)
        error('audit_independent_reachability:TaskCount','scene.num_tasks must match all scene.tasks rows.');
    end
    [regions,claimed,library_collision_checks] = prepare_library(library,scene,robot_cfg);
    task_rows = repmat(empty_task_row(),scene.num_tasks,1);
    pair_mismatches = 0;
    for ti = 1:scene.num_tasks
        task_started = tic;
        task = scene.tasks(ti);
        row = empty_task_row(); row.task_id = task.id; row.table_id = task.table_id;
        row.target_x_m = task.x; row.target_y_m = task.y; row.target_z_m = task.z;
        row.production_library_covered = ismember(task.id,claimed);
        [poses,free,claims] = task_library(regions,task);
        row.library_pose_count = size(poses,1); row.library_collision_free_count = sum(free);
        if ~isempty(poses)
            oracle = independent_geometric_reachability(poses,[task.x,task.y,task.z],robot_cfg,threshold);
            accepted = oracle.score_accepted & free;
            row.independent_library_covered = any(accepted);
            pair_mismatches = pair_mismatches + sum(accepted ~= claims);
            if any(accepted)
                scores = oracle.combined_score; scores(~accepted) = -Inf;
                [row.best_independent_score,index] = max(scores);
                row = set_witness(row,poses(index,:));
            end
        end
        row.library_coverage_mismatch = row.production_library_covered ~= row.independent_library_covered;
        relative_z = task.z-robot_cfg.arm_base_mount_z;
        row.hard_vertical_certificate = relative_z < robot_cfg.single_arm.reach_z_min || ...
            relative_z > robot_cfg.single_arm.reach_z_max;
        if row.hard_vertical_certificate
            row.classification = 'hard_vertical_certificate';
        elseif row.independent_library_covered
            if row.library_coverage_mismatch, row.classification = 'library_coverage_mismatch';
            else, row.classification = 'covered_in_library'; end
        else
            dense = sample_poses(scene,task,options);
            row.dense_sample_count = size(dense,1);
            oracle = independent_geometric_reachability(dense,[task.x,task.y,task.z],robot_cfg,threshold);
            possible = find(oracle.score_accepted);
            row.dense_score_accepted_count = numel(possible);
            for pi = reshape(possible,1,[])
                row.dense_collision_checks = row.dense_collision_checks+1;
                if ~check_footprint_collision(dense(pi,:),scene.grid_map,scene.map_info,robot_cfg)
                    row.independent_dense_support_found = true;
                    if oracle.combined_score(pi) > row.best_independent_score
                        row.best_independent_score = oracle.combined_score(pi);
                        row = set_witness(row,dense(pi,:));
                    end
                    if options.stop_after_first_support
                        row.dense_search_stopped_at_witness = true; break;
                    end
                end
            end
            row.candidate_sampling_gap = ~row.production_library_covered && ...
                row.independent_dense_support_found;
            if row.library_coverage_mismatch
                row.classification = 'library_coverage_mismatch';
            elseif row.candidate_sampling_gap
                row.classification = 'candidate_sampling_gap';
            else
                row.classification = 'not_found_in_audited_samples';
            end
        end
        row.audited_geometric_supported = row.independent_library_covered || row.independent_dense_support_found;
        row.audit_time_s = toc(task_started);
        task_rows(ti) = row;
    end
    n = scene.num_tasks;
    summary = struct('task_count',n,'threshold',threshold, ...
        'candidate_coverage_percent',100*sum([task_rows.production_library_covered])/max(n,1), ...
        'independent_library_coverage_percent',100*sum([task_rows.independent_library_covered])/max(n,1), ...
        'audited_geometric_coverage_percent',100*sum([task_rows.audited_geometric_supported])/max(n,1), ...
        'mismatch_count',sum([task_rows.library_coverage_mismatch]), ...
        'candidate_pair_mismatch_count',pair_mismatches, ...
        'sampling_gap_count',sum([task_rows.candidate_sampling_gap]), ...
        'hard_vertical_certificate_count',sum([task_rows.hard_vertical_certificate]), ...
        'unresolved_count',sum(~[task_rows.audited_geometric_supported] & ~[task_rows.hard_vertical_certificate]), ...
        'dense_sample_count',sum([task_rows.dense_sample_count]), ...
        'dense_collision_checks',sum([task_rows.dense_collision_checks]), ...
        'library_collision_checks',library_collision_checks, ...
        'ik_feasibility_percent',NaN,'actual_completion_percent',NaN, ...
        'ik_verified',false,'measured_execution_available',false, ...
        'audit_scope','independent_configured_geometric_model_with_finite_pose_search', ...
        'library_comparison_scope','task_region_candidate_poses', ...
        'collision_checker','check_footprint_collision', ...
        'audit_options_json',jsonencode(options),'audit_time_s',toc(started));
end

function [out,claimed,checks] = prepare_library(library,scene,robot)
    out = struct('table_id',{},'task_ids',{},'poses',{},'free',{},'claims',{});
    claimed = []; checks = 0;
    if ~isfield(library,'regions'), return; end
    for ri = 1:numel(library.regions)
        entry = library.regions(ri); candidates = entry.candidates;
        poses = zeros(numel(candidates),3); free = false(numel(candidates),1);
        claims = cell(numel(candidates),1);
        for ci = 1:numel(candidates)
            poses(ci,:) = candidates(ci).pose;
            claims{ci} = reshape(candidates(ci).covered_tasks,1,[]);
            claimed = union(claimed,claims{ci},'stable');
            free(ci) = ~check_footprint_collision(poses(ci,:),scene.grid_map,scene.map_info,robot);
            checks = checks+1;
        end
        out(ri) = struct('table_id',field_or(entry,'table_id',0), ...
            'task_ids',field_or(entry,'task_ids',[]),'poses',poses,'free',free,'claims',{claims}); %#ok<AGROW>
    end
end

function [poses,free,claims] = task_library(regions,task)
    poses = zeros(0,3); free = false(0,1); claims = false(0,1);
    for ri = 1:numel(regions)
        entry = regions(ri);
        if entry.table_id ~= task.table_id && ~ismember(task.id,entry.task_ids), continue; end
        poses = [poses;entry.poses]; free = [free;entry.free]; %#ok<AGROW>
        c = cellfun(@(ids) ismember(task.id,ids),entry.claims);
        claims = [claims;reshape(c,[],1)]; %#ok<AGROW>
    end
end

function poses = sample_poses(scene,task,options)
    cfg = scene.scene_cfg; tid = task.table_id;
    if tid < 1 || tid > size(cfg.table_positions,1) || tid ~= fix(tid)
        error('audit_independent_reachability:TableId','Every task must identify a valid table.');
    end
    center = cfg.table_positions(tid,:); angle = cfg.table_orientations(tid);
    c = cos(angle); s = sin(angle);
    delta = [task.x,task.y]-center;
    local_target = [delta(1)*c+delta(2)*s,-delta(1)*s+delta(2)*c];
    offsets = unique([(-options.heading_span_deg:options.heading_step_deg:options.heading_span_deg),0]);
    positions = zeros(0,2);
    for side = 1:4
        if side <= 2, half = cfg.table.length/2; projection = local_target(1);
        else, half = cfg.table.width/2; projection = local_target(2); end
        lower = max(-half-options.corner_extension_m,projection-options.tangential_half_window_m);
        upper = min(half+options.corner_extension_m,projection+options.tangential_half_window_m);
        if lower > upper, continue; end
        tangent = endpoint_grid(lower,upper,options.tangential_step_m);
        tangent = unique([tangent,min(upper,max(lower,projection))]);
        for normal = reshape(options.normal_offsets_m,1,[])
            switch side
                case 1, local = [tangent',repmat(cfg.table.width/2+normal,numel(tangent),1)];
                case 2, local = [tangent',repmat(-cfg.table.width/2-normal,numel(tangent),1)];
                case 3, local = [repmat(-cfg.table.length/2-normal,numel(tangent),1),tangent'];
                case 4, local = [repmat(cfg.table.length/2+normal,numel(tangent),1),tangent'];
            end
            positions = [positions;center+[c*local(:,1)-s*local(:,2),s*local(:,1)+c*local(:,2)]]; %#ok<AGROW>
        end
    end
    count = size(positions,1)*numel(offsets);
    if count > options.max_samples_per_task
        error('audit_independent_reachability:SampleCap', ...
            'Task %d requires %d poses, exceeding explicit max_samples_per_task=%d. Increase the cap or coarsen explicit audit options.', ...
            task.id,count,options.max_samples_per_task);
    end
    face = atan2(task.y-positions(:,2),task.x-positions(:,1));
    poses = [repelem(positions,numel(offsets),1), ...
        reshape((face+deg2rad(offsets))',[],1)];
    poses(:,3) = atan2(sin(poses(:,3)),cos(poses(:,3)));
end

function options = audit_options(input,cfg)
    options = struct('tangential_step_m',0.025,'normal_step_m',0.025, ...
        'heading_step_deg',5,'heading_span_deg',90,'tangential_half_window_m',0.35, ...
        'corner_extension_m',0.20,'normal_offsets_m',[], ...
        'max_samples_per_task',40000,'stop_after_first_support',true);
    names = fieldnames(input);
    for k = 1:numel(names)
        if ~isfield(options,names{k})
            error('audit_independent_reachability:UnknownOption','Unsupported option "%s".',names{k});
        end
        options.(names{k}) = input.(names{k});
    end
    positive = {'tangential_step_m','normal_step_m','heading_step_deg','tangential_half_window_m','max_samples_per_task'};
    for k = 1:numel(positive)
        value = options.(positive{k});
        if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
            error('audit_independent_reachability:InvalidOption','%s must be positive and finite.',positive{k});
        end
    end
    if ~isscalar(options.heading_span_deg) || ~isfinite(options.heading_span_deg) || ...
            options.heading_span_deg < 0 || options.heading_span_deg > 180 || ...
            ~isscalar(options.corner_extension_m) || ~isfinite(options.corner_extension_m) || options.corner_extension_m < 0
        error('audit_independent_reachability:InvalidOption','Heading span and corner extension are invalid.');
    end
    if options.max_samples_per_task ~= fix(options.max_samples_per_task) || ...
            ~isscalar(options.stop_after_first_support) || ~any(options.stop_after_first_support == [false,true])
        error('audit_independent_reachability:InvalidOption','Sample cap must be an integer and stop_after_first_support must be logical.');
    end
    if isempty(options.normal_offsets_m)
        options.normal_offsets_m = endpoint_grid(cfg.docking_min_distance_m,cfg.docking_max_distance_m,options.normal_step_m);
    end
    normals = options.normal_offsets_m;
    if ~isnumeric(normals) || any(~isfinite(normals(:))) || ...
            any(normals(:) < cfg.docking_min_distance_m-1e-12 | normals(:) > cfg.docking_max_distance_m+1e-12)
        error('audit_independent_reachability:InvalidOption','normal_offsets_m must stay within the scene docking min/max.');
    end
    options.normal_offsets_m = unique(reshape(normals,1,[]));
end

function values = endpoint_grid(lower,upper,step)
    values = lower:step:upper;
    if isempty(values) || abs(values(end)-upper) > 1e-12, values = [values,upper]; end
end

function row = set_witness(row,pose)
    row.support_x_m = pose(1); row.support_y_m = pose(2); row.support_yaw_rad = pose(3);
end

function row = empty_task_row()
    row = struct('task_id',0,'table_id',0,'target_x_m',NaN,'target_y_m',NaN,'target_z_m',NaN, ...
        'production_library_covered',false,'independent_library_covered',false, ...
        'library_coverage_mismatch',false,'independent_dense_support_found',false, ...
        'audited_geometric_supported',false,'candidate_sampling_gap',false, ...
        'hard_vertical_certificate',false,'classification','not_audited', ...
        'library_pose_count',0,'library_collision_free_count',0, ...
        'dense_sample_count',0,'dense_score_accepted_count',0,'dense_collision_checks',0, ...
        'dense_search_stopped_at_witness',false,'best_independent_score',0, ...
        'support_x_m',NaN,'support_y_m',NaN,'support_yaw_rad',NaN, ...
        'ik_feasible',NaN,'actual_completed',NaN,'audit_time_s',0);
end

function value = field_or(s,name,fallback)
    if isstruct(s) && isfield(s,name), value = s.(name); else, value = fallback; end
end
