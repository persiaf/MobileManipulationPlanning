function outputs = exp_candidate_efficiency_p2(options)
%EXP_CANDIDATE_EFFICIENCY_P2 Candidate-level P2 efficiency benchmark.
%
% E0 is the current pipeline. E1 enables the conservative pair-level quick
% filter. Heading-shortlist variants are reported as not enabled in this fast
% round because the existing production generator already uses task-facing
% headings and no independent safe switch is introduced here.

    if nargin < 1 || isempty(options), options = struct(); end
    options = default_options(options);
    project_root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(project_root);
    cd(project_root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>

    rows = repmat(empty_row(), 0, 1);
    for ci = 1:numel(options.cases)
        case_id = options.cases{ci};
        for seed = options.seeds
            baseline_row = [];
            for vi = 1:numel(options.variants)
                variant = options.variants{vi};
                scene_cfg = make_scene_config(case_id, seed);
                scene = build_scene_from_cfg(scene_cfg);
                regions = build_task_regions(scene, assign_tasks_to_tables(scene));
                robot_cfg = robot_geometry_profile('legacy_validated');
                planner_cfg = configure_planner(variant);

                t_candidate = tic;
                library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
                elapsed = toc(t_candidate);
                row = row_from_library(case_id, seed, variant, scene, regions, ...
                    library, robot_cfg, elapsed, project_root);
                if strcmpi(variant, 'E0_current')
                    row.false_negative_count = audit_quick_filter_false_negatives( ...
                        library, scene, robot_cfg);
                    baseline_row = row;
                else
                    row.false_negative_count = audit_quick_filter_false_negatives( ...
                        library, scene, robot_cfg);
                    if ~isempty(baseline_row)
                        row.raw_coverage_delta_pp = ...
                            row.raw_task_coverage_percent - baseline_row.raw_task_coverage_percent;
                        row.feasible_coverage_delta_pp = ...
                            row.feasible_task_coverage_percent - baseline_row.feasible_task_coverage_percent;
                        row.formal_reachability_reduction_percent = ...
                            100 * (baseline_row.formal_reachability_calls - row.formal_reachability_calls) / ...
                            max(baseline_row.formal_reachability_calls, 1);
                        row.candidate_time_reduction_percent = ...
                            100 * (baseline_row.T_candidate_s - row.T_candidate_s) / ...
                            max(baseline_row.T_candidate_s, eps);
                    end
                end
                rows(end + 1, 1) = row; %#ok<AGROW>
                if options.write_outputs && options.write_partial
                    write_outputs(project_root, rows);
                end
            end
        end
    end

    raw = struct2table(rows, 'AsArray', true);
    summary = summarize_rows(raw);
    outputs.raw = raw;
    outputs.summary = summary;
    if options.write_outputs
        ensure_result_dir();
        writetable(raw, fullfile(project_root, 'results', 'tables', ...
            'candidate_efficiency_p2_raw.csv'));
        writetable(summary, fullfile(project_root, 'results', 'tables', ...
            'candidate_efficiency_p2_summary.csv'));
    end
end

function planner_cfg = configure_planner(variant)
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
    planner_cfg.candidate_pair_prefilter.enable = strcmpi(variant, 'E1_quick_filter');
end

function row = row_from_library(case_id, seed, variant, scene, regions, library, ...
        robot_cfg, elapsed, project_root)
    row = empty_row();
    row.case_id = case_id;
    row.seed = seed;
    row.variant = variant;
    row.heading_shortlist_enabled = false;
    row.task_count = scene.num_tasks;
    diag = [library.regions.generation_diagnostics];
    row.raw_candidate_count = sum([diag.raw_candidate_count]);
    row.collision_valid_candidate_count = sum([diag.collision_valid_candidate_count]);
    row.total_candidate_task_pairs = sum([diag.candidate_task_pair_count]);
    row.quick_rejected_pairs = sum([diag.quick_rejected_pair_count]);
    row.formal_reachability_calls = sum([diag.formal_reachability_call_count]);
    row.formal_valid_pairs = sum_reachable_support(diag);
    row.reachability_valid_candidate_count = sum([diag.reachability_valid_candidate_count]);
    row.final_candidate_count = library.total_candidates;
    row.quick_filter_rejection_ratio = ...
        row.quick_rejected_pairs / max(row.total_candidate_task_pairs, 1);
    row.T_candidate_s = elapsed;

    covered = library_covered_tasks(library);
    model_infeasible = model_infeasible_tasks_for_case(project_root, case_id, seed);
    accounting = task_operability_accounting(1:scene.num_tasks, covered, model_infeasible);
    row.raw_task_coverage_percent = accounting.raw_task_coverage_percent;
    row.model_feasible_task_count = accounting.model_feasible_task_count;
    row.feasible_task_coverage_percent = accounting.feasible_task_coverage_percent;
    row.candidate_generation_residual = ...
        accounting.model_feasible_task_count - accounting.covered_model_feasible_tasks;
end

function count = audit_quick_filter_false_negatives(library, scene, robot_cfg)
    count = 0;
    for ri = 1:numel(library.regions)
        task_ids = library.regions(ri).task_ids;
        candidates = library.regions(ri).candidates;
        for ci = 1:numel(candidates)
            pose = candidates(ci).pose;
            for ti = 1:numel(task_ids)
                if size(candidates(ci).reach_scores, 1) < ti || candidates(ci).reach_scores(ti, 3) <= 0
                    continue;
                end
                tid = task_ids(ti);
                target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
                quick_reject = candidate_reachability_quick_filter(pose, target, robot_cfg);
                if quick_reject
                    count = count + 1;
                end
            end
        end
    end
end

function total = sum_reachable_support(diagnostics)
    total = 0;
    for i = 1:numel(diagnostics)
        total = total + sum(diagnostics(i).reachable_support_count);
    end
end

function summary = summarize_rows(raw)
    variants = unique(raw.variant, 'stable');
    summary = table();
    for vi = 1:numel(variants)
        mask = strcmp(raw.variant, variants{vi});
        row = table();
        row.variant = variants(vi);
        row.run_count = sum(mask);
        metrics = {'total_candidate_task_pairs','quick_rejected_pairs', ...
            'formal_reachability_calls','formal_valid_pairs','final_candidate_count', ...
            'raw_task_coverage_percent','feasible_task_coverage_percent', ...
            'candidate_generation_residual','T_candidate_s', ...
            'false_negative_count','formal_reachability_reduction_percent', ...
            'candidate_time_reduction_percent'};
        for mi = 1:numel(metrics)
            vals = raw.(metrics{mi})(mask);
            vals = vals(isfinite(vals));
            if isempty(vals)
                row.([metrics{mi} '_mean']) = NaN;
                row.([metrics{mi} '_max']) = NaN;
            else
                row.([metrics{mi} '_mean']) = mean(vals);
                row.([metrics{mi} '_max']) = max(vals);
            end
        end
        summary = [summary; row]; %#ok<AGROW>
    end
end

function model_infeasible = model_infeasible_tasks_for_case(project_root, case_id, seed)
    model_infeasible = [];
    if ~(strcmpi(case_id, 'F010') && seed == 2026)
        return;
    end
    p = fullfile(project_root, 'results', 'tables', 'radial_operability_audit_task.csv');
    if ~isfile(p), return; end
    t = readtable(p);
    if all(ismember({'task_id','final_classification'}, t.Properties.VariableNames))
        model_infeasible = t.task_id(strcmp(t.final_classification, 'TASK_GEOMETRY_REACH_LIMIT'));
    end
end

function scene_cfg = make_scene_config(case_id, seed)
    if startsWith(upper(case_id), 'F')
        scene_cfg = scene_complexity_factor_config(case_id, seed);
    else
        scene_cfg = scene_complexity_profile_config(case_id, seed);
    end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function write_outputs(project_root, rows)
    ensure_result_dir();
    writetable(struct2table(rows, 'AsArray', true), fullfile(project_root, ...
        'results', 'tables', 'candidate_efficiency_p2_raw_partial.csv'));
end

function row = empty_row()
    row = struct('case_id', '', 'seed', 0, 'variant', '', ...
        'heading_shortlist_enabled', false, 'task_count', 0, ...
        'model_feasible_task_count', 0, 'raw_candidate_count', 0, ...
        'collision_valid_candidate_count', 0, 'total_candidate_task_pairs', 0, ...
        'quick_rejected_pairs', 0, 'formal_reachability_calls', 0, ...
        'formal_valid_pairs', 0, 'reachability_valid_candidate_count', 0, ...
        'final_candidate_count', 0, 'quick_filter_rejection_ratio', 0, ...
        'false_negative_count', 0, 'raw_task_coverage_percent', NaN, ...
        'feasible_task_coverage_percent', NaN, ...
        'candidate_generation_residual', 0, 'T_candidate_s', NaN, ...
        'raw_coverage_delta_pp', NaN, 'feasible_coverage_delta_pp', NaN, ...
        'formal_reachability_reduction_percent', NaN, ...
        'candidate_time_reduction_percent', NaN);
end

function options = default_options(options)
    if ~isfield(options, 'cases')
        options.cases = {'C0','C1','C2','C3','F010','F110','F111'};
    end
    if ~isfield(options, 'seeds'), options.seeds = [2026, 2027, 2028]; end
    if ~isfield(options, 'variants')
        options.variants = {'E0_current','E1_quick_filter'};
    end
    if ~isfield(options, 'write_partial'), options.write_partial = true; end
    if ~isfield(options, 'write_outputs'), options.write_outputs = true; end
end

function ensure_result_dir()
    out_dir = fullfile('results', 'tables');
    if ~isfolder(out_dir), mkdir(out_dir); end
end
