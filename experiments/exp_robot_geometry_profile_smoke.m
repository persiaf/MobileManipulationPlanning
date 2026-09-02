function summary_table = exp_robot_geometry_profile_smoke(options)
%EXP_ROBOT_GEOMETRY_PROFILE_SMOKE Compare legacy vs physical candidate coverage.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_row(), 0, 1);
    for ci = 1:numel(options.cases)
        scene_cfg = scene_complexity_profile_config(options.cases{ci}, options.seed);
        scene = build_scene_from_cfg(scene_cfg);
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        for pi = 1:numel(options.profiles)
            robot_cfg = robot_geometry_profile(options.profiles{pi});
            planner_cfg = planner_config();
            planner_cfg.runtime.disable_visualization = true;
            planner_cfg.runtime.disable_video = true;
            planner_cfg.benchmark.enable = false;
            planner_cfg.visualization.show_scene = false;
            planner_cfg.visualization.show_docking = false;
            planner_cfg.visualization.show_global_path = false;
            planner_cfg.candidate_augmentation.enable = false;
            t_candidate = tic;
            library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
            rows(end + 1, 1) = row_from_library(options.cases{ci}, options.seed, ...
                options.profiles{pi}, scene, library, toc(t_candidate)); %#ok<AGROW>
        end
    end
    summary_table = struct2table(rows, 'AsArray', true);
    ensure_result_dir();
    writetable(summary_table, fullfile(project_root, 'results', 'tables', ...
        'robot_geometry_profile_smoke_summary.csv'));
end

function row = row_from_library(case_id, seed, profile, scene, library, t_candidate)
    row = empty_row();
    row.case_id = case_id;
    row.seed = seed;
    row.robot_geometry_profile = profile;
    row.task_count = scene.num_tasks;
    row.library_candidate_count = library.total_candidates;
    covered = library_covered_tasks(library);
    row.library_coverage = 100 * numel(intersect(1:scene.num_tasks, covered)) / max(scene.num_tasks, 1);
    row.uncovered_task_count = scene.num_tasks - numel(intersect(1:scene.num_tasks, covered));
    row.T_candidate = t_candidate;
end

function row = empty_row()
    row = struct('case_id','','seed',0,'robot_geometry_profile','', ...
        'task_count',0,'library_candidate_count',0,'library_coverage',NaN, ...
        'uncovered_task_count',0,'T_candidate',NaN);
end

function options = default_options(options)
    if ~isfield(options, 'cases'), options.cases = {'C0','C2'}; end
    if ~isfield(options, 'seed'), options.seed = 2026; end
    if ~isfield(options, 'profiles'), options.profiles = {'legacy_validated','physical_robot'}; end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end
