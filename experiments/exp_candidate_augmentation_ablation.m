function outputs = exp_candidate_augmentation_ablation(options)
%EXP_CANDIDATE_AUGMENTATION_ABLATION Compare candidate-library variants.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    rows = repmat(empty_row(), 0, 1);
    task_rows = repmat(empty_task_row(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config(options.scale, seed));
        robot_cfg = robot_config(); cfg = experiment_config();
        regions = build_task_regions(scene, assign_tasks_to_tables(scene));
        baseline = build_candidate_library(regions, scene, robot_cfg, cfg);

        for vi = 1:numel(options.variants)
            variant = options.variants{vi};
            if strcmpi(variant, 'baseline')
                library = baseline;
                diagnostics = baseline.augmentation_diagnostics;
            else
                cfg.candidate_augmentation.enable = true;
                cfg.candidate_augmentation.variant = variant;
                [library, diagnostics] = augment_candidate_library( ...
                    baseline, regions, scene, robot_cfg, cfg);
            end
            row = make_row(options.scale, seed, variant, baseline, library, diagnostics);
            rows(end + 1, 1) = row; %#ok<AGROW>
            task_rows = [task_rows; make_task_rows(options.scale, seed, variant, diagnostics)]; %#ok<AGROW>
            fprintf(['AUGMENT_ABLATION scale=%s seed=%d variant=%s ' ...
                'coverage=%.4f added=%d time=%.3f\n'], options.scale, seed, ...
                variant, row.library_coverage_percent, row.added_candidates, ...
                row.supplementation_time_s);
        end
    end

    outputs.raw = struct2table(rows, 'AsArray', true);
    outputs.task = struct2table(task_rows, 'AsArray', true);
    outputs.summary = summarize(outputs.raw);
    if options.write_files, write_outputs(outputs, root); end
end

function cfg = experiment_config()
    cfg = planner_config();
    cfg.candidate_augmentation.enable = false;
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.visualization.show_scene = false; cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false; cfg.benchmark.enable = false;
end

function row = make_row(scale, seed, variant, baseline, library, d)
    row = empty_row(); row.scale = scale; row.seed = seed; row.variant = variant;
    row.baseline_candidate_count = baseline.total_candidates;
    row.candidate_count_after = library.total_candidates;
    row.added_candidates = library.total_candidates - baseline.total_candidates;
    row.uncovered_tasks_before = d.uncovered_tasks_before;
    row.uncovered_tasks_after = d.uncovered_tasks_after;
    row.library_coverage_percent = 100 * d.library_coverage_after;
    row.raw_supplementary_count = d.raw_supplementary_count;
    row.collision_valid_supplementary_count = d.collision_valid_supplementary_count;
    row.reachable_supplementary_count = d.reachable_supplementary_count;
    row.coverage_positive_supplementary_count = d.coverage_positive_supplementary_count;
    row.selected_supplementary_count = d.selected_supplementary_count;
    row.recovery_rate_percent = 100 * d.recovery_rate;
    row.baseline_candidate_time_s = baseline.baseline_generation_time_s;
    row.supplementation_time_s = d.supplementation_time_s;
    row.candidate_time_total_s = baseline.baseline_generation_time_s + d.supplementation_time_s;
end

function rows = make_task_rows(scale, seed, variant, d)
    rows = repmat(empty_task_row(), numel(d.task), 1);
    for i = 1:numel(d.task)
        rows(i).scale = scale; rows(i).seed = seed; rows(i).variant = variant;
        rows(i).task_id = d.task(i).task_id; rows(i).region_id = d.task(i).region_id;
        rows(i).support_count_before = d.task(i).support_count_before;
        rows(i).support_count_after = d.task(i).support_count_after;
        rows(i).supplemental_support_count = d.task(i).supplemental_support_count;
        rows(i).recovered = d.task(i).recovered;
    end
end

function out = summarize(raw)
    variants = unique(raw.variant, 'stable');
    rows = repmat(empty_summary_row(), numel(variants), 1);
    for i = 1:numel(variants)
        subset = raw(strcmp(raw.variant, variants{i}), :);
        row = empty_summary_row(); row.variant = variants{i}; row.runs = height(subset);
        row.library_coverage_mean_percent = mean(subset.library_coverage_percent);
        row.library_coverage_min_percent = min(subset.library_coverage_percent);
        row.added_candidates_mean = mean(subset.added_candidates);
        row.added_candidates_max = max(subset.added_candidates);
        row.supplementation_time_mean_s = mean(subset.supplementation_time_s);
        row.candidate_time_total_mean_s = mean(subset.candidate_time_total_s);
        row.recovery_rate_mean_percent = mean(subset.recovery_rate_percent);
        rows(i) = row;
    end
    out = struct2table(rows, 'AsArray', true);
end

function write_outputs(outputs, root)
    out_dir = fullfile(root, 'results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
    writetable(outputs.raw, fullfile(out_dir, 'candidate_augmentation_ablation_raw.csv'));
    writetable(outputs.task, fullfile(out_dir, 'candidate_augmentation_ablation_task.csv'));
    writetable(outputs.summary, fullfile(out_dir, 'candidate_augmentation_ablation_summary.csv'));
    save(fullfile(out_dir, 'candidate_augmentation_ablation_20260901.mat'), 'outputs');
end

function options = defaults(options)
    if ~isfield(options, 'scale'), options.scale = 's4'; end
    if ~isfield(options, 'seeds'), options.seeds = 1:5; end
    if ~isfield(options, 'variants')
        options.variants = {'baseline','position_only','orientation_only','joint'};
    end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function row = empty_row()
    row = struct('scale','','seed',0,'variant','', ...
        'baseline_candidate_count',0,'candidate_count_after',0,'added_candidates',0, ...
        'uncovered_tasks_before',0,'uncovered_tasks_after',0, ...
        'library_coverage_percent',0,'raw_supplementary_count',0, ...
        'collision_valid_supplementary_count',0,'reachable_supplementary_count',0, ...
        'coverage_positive_supplementary_count',0,'selected_supplementary_count',0, ...
        'recovery_rate_percent',0,'baseline_candidate_time_s',0, ...
        'supplementation_time_s',0,'candidate_time_total_s',0);
end

function row = empty_task_row()
    row = struct('scale','','seed',0,'variant','','task_id',0,'region_id',0, ...
        'support_count_before',0,'support_count_after',0, ...
        'supplemental_support_count',0,'recovered',false);
end

function row = empty_summary_row()
    row = struct('variant','','runs',0,'library_coverage_mean_percent',0, ...
        'library_coverage_min_percent',0,'added_candidates_mean',0, ...
        'added_candidates_max',0,'supplementation_time_mean_s',0, ...
        'candidate_time_total_mean_s',0,'recovery_rate_mean_percent',0);
end
