function candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg)
%BUILD_CANDIDATE_LIBRARY Generate docking candidates once per region.

    t_all = tic;
    empty_region = struct('region_id', {}, 'table_id', {}, 'region', {}, ...
        'candidates', {}, 'num_candidates', {}, 'task_ids', {}, ...
        'generation_time_s', {}, 'generation_diagnostics', {});
    candidate_library = struct();
    candidate_library.regions = empty_region;
    candidate_library.total_candidates = 0;
    candidate_library.scene_signature = build_scene_signature(regions, scene, robot_cfg, planner_cfg);

    for ri = 1:numel(regions)
        t_region = tic;
        [candidates, generation_diagnostics] = generate_docking_candidates( ...
            regions(ri), scene, robot_cfg, planner_cfg);
        entry.region_id = ri;
        entry.table_id = regions(ri).table_id;
        entry.region = regions(ri);
        entry.candidates = candidates;
        entry.num_candidates = numel(candidates);
        entry.task_ids = regions(ri).task_ids;
        entry.generation_time_s = toc(t_region);
        entry.generation_diagnostics = generation_diagnostics;
        candidate_library.regions(end + 1) = entry;
        candidate_library.total_candidates = candidate_library.total_candidates + numel(candidates);
    end
    candidate_library.baseline_generation_time_s = toc(t_all);
    [candidate_library, augmentation_diagnostics] = augment_candidate_library( ...
        candidate_library, regions, scene, robot_cfg, planner_cfg);
    candidate_library.augmentation_diagnostics = augmentation_diagnostics;
    candidate_library.total_generation_time_s = toc(t_all);
end

function signature = build_scene_signature(regions, scene, robot_cfg, planner_cfg)
    s = struct();
    s.seed = getfield_default(scene.scene_cfg, 'random_seed', []);
    s.map = scene.map_info;
    s.table_positions = scene.scene_cfg.table_positions;
    s.table_orientations = scene.scene_cfg.table_orientations;
    s.tasks = zeros(scene.num_tasks, 3);
    for i = 1:scene.num_tasks
        s.tasks(i, :) = [scene.tasks(i).x, scene.tasks(i).y, scene.tasks(i).z];
    end
    s.region_task_ids = arrayfun(@(r) r.task_ids, regions, 'UniformOutput', false);
    s.robot_footprint = getfield_default(robot_cfg, 'footprint', struct());
    s.reachability_threshold = planner_cfg.reachability.threshold;
    s.candidate_augmentation = getfield_default(planner_cfg, ...
        'candidate_augmentation', struct('enable', false));
    s.global_planner = planner_cfg.global_planner;
    encoded = jsonencode(s);
    signature = char(matlab.lang.makeValidName(sprintf('sig_%d_%d', ...
        numel(encoded), sum(double(encoded)))));
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
