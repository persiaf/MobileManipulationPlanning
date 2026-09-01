function outputs = exp_position_only_heldout_coverage(options)
%EXP_POSITION_ONLY_HELDOUT_COVERAGE Evaluate frozen augmentation on new scenes.

    if nargin < 1, options = struct(); end
    options = defaults(options);
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'pipeline')); init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    rows = repmat(empty_row(), 0, 1); failures = repmat(empty_failure(), 0, 1);

    for seed = options.seeds
        scene = build_scene_from_cfg(large_scale_profile_config('s4', seed));
        robot_cfg = robot_config(); cfg = experiment_config();
        regions = build_task_regions(scene, assign_tasks_to_tables(scene));
        t_baseline = tic;
        baseline = build_candidate_library(regions, scene, robot_cfg, cfg);
        baseline_time = toc(t_baseline);
        cfg.candidate_augmentation.enable = true;
        cfg.candidate_augmentation.variant = 'position_only';
        [position, diagnostics] = augment_candidate_library( ...
            baseline, regions, scene, robot_cfg, cfg);

        row = empty_row(); row.seed = seed; row.tasks = scene.num_tasks;
        row.baseline_coverage_percent = 100 * diagnostics.library_coverage_before;
        row.position_coverage_percent = 100 * diagnostics.library_coverage_after;
        row.uncovered_before = diagnostics.uncovered_tasks_before;
        row.uncovered_after = diagnostics.uncovered_tasks_after;
        row.recovered_tasks = diagnostics.uncovered_tasks_before - diagnostics.uncovered_tasks_after;
        row.added_candidates = diagnostics.selected_supplementary_count;
        row.baseline_candidate_time_s = baseline_time;
        row.supplementation_time_s = diagnostics.supplementation_time_s;
        row.position_candidate_time_s = baseline_time + diagnostics.supplementation_time_s;
        rows(end + 1, 1) = row; %#ok<AGROW>

        if diagnostics.uncovered_tasks_after > 0
            failures = [failures; diagnose_failures(seed, diagnostics.remaining_task_ids, ...
                position, regions, scene)]; %#ok<AGROW>
        end
        fprintf(['POSITION_HELDOUT seed=%d baseline=%.3f position=%.3f ' ...
            'recovered=%d/%d added=%d\n'], seed, row.baseline_coverage_percent, ...
            row.position_coverage_percent, row.recovered_tasks, row.uncovered_before, ...
            row.added_candidates);
        if options.write_files, write_checkpoint(rows, failures, root); end
    end

    outputs.raw = struct2table(rows, 'AsArray', true);
    outputs.failures = struct2table(failures, 'AsArray', true);
    outputs.summary = table(mean(outputs.raw.baseline_coverage_percent), ...
        mean(outputs.raw.position_coverage_percent), ...
        min(outputs.raw.position_coverage_percent), ...
        sum(abs(outputs.raw.position_coverage_percent - 100) < 1e-9), ...
        mean(outputs.raw.added_candidates), mean(outputs.raw.position_candidate_time_s), ...
        'VariableNames', {'baseline_coverage_mean_percent', ...
        'position_coverage_mean_percent','position_coverage_min_percent', ...
        'full_recovery_scene_count','added_candidates_mean','candidate_time_mean_s'});
    if options.write_files, write_outputs(outputs, root); end
end

function failures = diagnose_failures(seed, task_ids, library, regions, scene)
    failures = repmat(empty_failure(), numel(task_ids), 1);
    for i = 1:numel(task_ids)
        tid = task_ids(i); ri = find(arrayfun(@(r) ismember(tid, r.task_ids), regions), 1);
        local_idx = find(regions(ri).task_ids == tid, 1);
        candidates = library.regions(ri).candidates;
        scores = zeros(0, 1); offsets = zeros(0, 1);
        for ci = 1:numel(candidates)
            if size(candidates(ci).reach_scores, 1) >= local_idx
                scores(end + 1, 1) = candidates(ci).reach_scores(local_idx, 3); %#ok<AGROW>
            end
            offsets(end + 1, 1) = normal_offset(candidates(ci).pose, ...
                regions(ri).table_id, scene); %#ok<AGROW>
        end
        f = empty_failure(); f.seed = seed; f.task_id = tid; f.region_id = ri;
        f.table_id = regions(ri).table_id; f.x = scene.tasks(tid).x;
        f.y = scene.tasks(tid).y; f.z = scene.tasks(tid).z;
        [f.table_local_x, f.table_local_y] = table_local_position( ...
            f.x, f.y, f.table_id, scene);
        if isempty(candidates)
            f.failure_category = 'NO_COLLISION_VALID_CANDIDATE';
            f.max_reachable_score = 0; f.nearest_collision_valid_offset_m = NaN;
        else
            f.failure_category = 'COLLISION_VALID_BUT_UNREACHABLE';
            f.max_reachable_score = max(scores);
            f.nearest_collision_valid_offset_m = min(offsets);
        end
        failures(i) = f;
    end
end

function d = normal_offset(pose, table_id, scene)
    [x, y] = table_local_position(pose(1), pose(2), table_id, scene);
    table = scene.scene_cfg.table;
    d = max([abs(x) - table.length / 2, abs(y) - table.width / 2, 0]);
end

function [x, y] = table_local_position(wx, wy, table_id, scene)
    center = scene.scene_cfg.table_positions(table_id, :);
    theta = scene.scene_cfg.table_orientations(table_id);
    local = rot2(-theta) * ([wx, wy] - center)'; x = local(1); y = local(2);
end

function write_checkpoint(rows, failures, root)
    d = output_dir(root);
    writetable(struct2table(rows, 'AsArray', true), ...
        fullfile(d, 'position_only_heldout_coverage_partial.csv'));
    writetable(struct2table(failures, 'AsArray', true), ...
        fullfile(d, 'position_only_heldout_failures_partial.csv'));
end

function write_outputs(outputs, root)
    d = output_dir(root);
    writetable(outputs.raw, fullfile(d, 'position_only_heldout_coverage_raw.csv'));
    writetable(outputs.summary, fullfile(d, 'position_only_heldout_coverage_summary.csv'));
    writetable(outputs.failures, fullfile(d, 'position_only_heldout_failures.csv'));
    save(fullfile(d, 'position_only_heldout_coverage_20260901.mat'), 'outputs');
end

function d = output_dir(root)
    d = fullfile(root, 'results', 'tables'); if ~isfolder(d), mkdir(d); end
end

function cfg = experiment_config()
    cfg = planner_config(); cfg.candidate_augmentation.enable = false;
    cfg.runtime.disable_visualization = true; cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true; cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false; cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false; cfg.visualization.show_global_path = false;
end

function options = defaults(options)
    if ~isfield(options, 'seeds'), options.seeds = 6:20; end
    if ~isfield(options, 'write_files'), options.write_files = true; end
end

function r = empty_row()
    r = struct('seed',0,'tasks',0,'baseline_coverage_percent',0, ...
        'position_coverage_percent',0,'uncovered_before',0,'uncovered_after',0, ...
        'recovered_tasks',0,'added_candidates',0,'baseline_candidate_time_s',0, ...
        'supplementation_time_s',0,'position_candidate_time_s',0);
end

function r = empty_failure()
    r = struct('seed',0,'task_id',0,'region_id',0,'table_id',0, ...
        'x',NaN,'y',NaN,'z',NaN,'table_local_x',NaN,'table_local_y',NaN, ...
        'failure_category','','max_reachable_score',NaN, ...
        'nearest_collision_valid_offset_m',NaN);
end
