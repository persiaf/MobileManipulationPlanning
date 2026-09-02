function [raw_table, summary_table] = exp_scene_complexity_factor_isolation(options)
%EXP_SCENE_COMPLEXITY_FACTOR_ISOLATION Candidate-layer P1.5 factor audit.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_row(), 0, 1);
    for fi = 1:numel(options.factor_ids)
        for seed = options.seeds
            cfg = scene_complexity_factor_config(options.factor_ids{fi}, seed);
            scene = build_scene_from_cfg(cfg);
            task_table_map = assign_tasks_to_tables(scene);
            regions = build_task_regions(scene, task_table_map);
            for aug = {'off', 'position_only'}
                planner_cfg = configure_planner(aug{1});
                robot_cfg = robot_geometry_profile('legacy_validated');
                t_candidate = tic;
                library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
                row = row_from_library(options.factor_ids{fi}, seed, aug{1}, ...
                    scene, regions, library, toc(t_candidate));
                rows(end + 1, 1) = row; %#ok<AGROW>
            end
        end
    end

    raw_table = struct2table(rows, 'AsArray', true);
    summary_table = summarize_rows(raw_table);
    ensure_result_dir();
    writetable(raw_table, fullfile(project_root, 'results', 'tables', ...
        'scene_complexity_factor_isolation_raw.csv'));
    writetable(summary_table, fullfile(project_root, 'results', 'tables', ...
        'scene_complexity_factor_isolation_summary.csv'));
end

function planner_cfg = configure_planner(augmentation_mode)
    planner_cfg = planner_config();
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.runtime.quiet = true;
    planner_cfg.runtime.quiet_stage2 = true;
    planner_cfg.benchmark.enable = false;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    planner_cfg.candidate_augmentation.enable = false;
    if strcmpi(augmentation_mode, 'position_only')
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'position_only';
    end
end

function row = row_from_library(factor_id, seed, aug, scene, regions, library, t_candidate)
    row = empty_row();
    row.factor_id = factor_id;
    row.seed = seed;
    row.augmentation_mode = aug;
    row.layout_pattern = scene.scene_cfg.layout.pattern;
    row.orientation_mode = scene.scene_cfg.layout.orientation_mode;
    row.task_position_mode = scene.scene_cfg.task.position_mode;
    row.table_count = size(scene.scene_cfg.table_positions, 1);
    row.task_count = scene.num_tasks;
    row.region_count = numel(regions);
    row.T_candidate = t_candidate;
    diag = [library.regions.generation_diagnostics];
    row.raw_candidate_count = sum([diag.raw_candidate_count]);
    row.collision_valid_candidate_count = sum([diag.collision_valid_candidate_count]);
    row.reachability_valid_candidate_count = sum([diag.reachability_valid_candidate_count]);
    row.library_candidate_count = library.total_candidates;
    row.supplementary_raw_count = getfield_default(library.augmentation_diagnostics, 'raw_supplementary_count', 0);
    row.supplementary_collision_valid_count = getfield_default(library.augmentation_diagnostics, 'collision_valid_supplementary_count', 0);
    row.supplementary_reachable_count = getfield_default(library.augmentation_diagnostics, 'reachable_supplementary_count', 0);
    row.supplementary_selected_count = getfield_default(library.augmentation_diagnostics, 'selected_supplementary_count', 0);
    covered = library_covered_tasks(library);
    row.library_coverage = 100 * numel(intersect(1:scene.num_tasks, covered)) / max(scene.num_tasks, 1);
    row.uncovered_task_count = scene.num_tasks - numel(intersect(1:scene.num_tasks, covered));
end

function summary = summarize_rows(raw)
    groups = unique(raw(:, {'factor_id','layout_pattern','orientation_mode','task_position_mode','augmentation_mode'}), 'rows');
    summary = groups;
    metrics = {'task_count','raw_candidate_count','collision_valid_candidate_count', ...
        'reachability_valid_candidate_count','library_candidate_count', ...
        'supplementary_selected_count','library_coverage','uncovered_task_count','T_candidate'};
    summary.run_count = zeros(height(groups), 1);
    for mi = 1:numel(metrics)
        summary.([metrics{mi} '_mean']) = NaN(height(groups), 1);
    end
    for gi = 1:height(groups)
        mask = strcmp(raw.factor_id, groups.factor_id{gi}) & ...
            strcmp(raw.augmentation_mode, groups.augmentation_mode{gi});
        summary.run_count(gi) = sum(mask);
        for mi = 1:numel(metrics)
            summary.([metrics{mi} '_mean'])(gi) = mean(raw.(metrics{mi})(mask), 'omitnan');
        end
    end
end

function row = empty_row()
    row = struct('factor_id','','seed',0,'augmentation_mode','', ...
        'layout_pattern','','orientation_mode','','task_position_mode','', ...
        'table_count',0,'task_count',0,'region_count',0,'raw_candidate_count',0, ...
        'collision_valid_candidate_count',0,'reachability_valid_candidate_count',0, ...
        'library_candidate_count',0,'supplementary_raw_count',0, ...
        'supplementary_collision_valid_count',0,'supplementary_reachable_count',0, ...
        'supplementary_selected_count',0,'library_coverage',NaN, ...
        'uncovered_task_count',0,'T_candidate',NaN);
end

function options = default_options(options)
    if ~isfield(options, 'factor_ids')
        options.factor_ids = {'F000','F001','F010','F011','F100','F101','F110','F111'};
    end
    if ~isfield(options, 'seeds'), options.seeds = [2026, 2027, 2028]; end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), v = s.(name); else, v = default_value; end
end
