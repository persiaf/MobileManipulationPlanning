% Focused read-only verification for the 2026-10-30 user assessment.
project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(project_root, 'pipeline'));
init_paths(project_root);
out_dir = fileparts(mfilename('fullpath'));
diary(fullfile(out_dir, 'verification.log'));
log_cleanup = onCleanup(@() diary('off'));
fprintf('MATLAB_VERSION=%s\n', version);
test_files = {'test_rotated_table_task_transform.m', ...
    'test_scene_layout_determinism.m', 'test_scene_layout_inside_map.m', ...
    'test_scene_layout_no_overlap.m', 'test_scene_pattern_smoke.m', ...
    'test_independent_geometric_reachability.m', 'test_planned_execution_consistency.m'};
test_files = cellfun(@(f) fullfile(project_root, 'tests', f), test_files, 'UniformOutput', false);
results = runtests(test_files);
for k = 1:numel(results)
    fprintf('TEST=%s PASSED=%d FAILED=%d INCOMPLETE=%d DURATION_S=%.6f\n', ...
        results(k).Name, results(k).Passed, results(k).Failed, ...
        results(k).Incomplete, results(k).Duration);
end
assert(all([results.Passed]), 'Assessment:Regression', 'A focused regression test failed.');

base = scene_config();
for table_count = [4, 8, 16]
    cfg = base;
    cfg.table_positions = base.table_positions(1:table_count, :);
    cfg.num_tables = 999; % custom mode must derive the real count from positions.
    degrees = repmat([0; 30; -45; 90], ceil(table_count/4), 1);
    cfg.table_orientations = deg2rad(degrees(1:table_count));
    cfg.tasks_per_table_range = [1, 1];
    cfg.task_candidate_local = [0.1, 0.1];
    scene = build_scene_from_cfg(cfg);
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    assert(scene.scene_cfg.num_tables == table_count);
    assert(scene.num_tasks == table_count);
    assert(numel(regions) == table_count);
    for t = 1:table_count
        expected = cfg.table_positions(t,:) + (rot2(cfg.table_orientations(t))*[0.1;0.1])';
        actual = [scene.tasks(t).x, scene.tasks(t).y];
        assert(norm(actual-expected) < 1e-12);
    end
    fprintf('CUSTOM_TABLE_COUNT=%d TASK_COUNT=%d REGION_COUNT=%d ANGLES_DEG=%s LAYOUT_VALID=%d TASK_ROTATION_VALID=1\n', ...
        table_count, scene.num_tasks, numel(regions), mat2str(degrees(1:table_count)'), ...
        scene.scene_cfg.layout_validation.is_valid);
end

cfg = base;
cfg.table_positions = base.table_positions(1:4,:);
cfg.table_orientations = pi/6;
scene = build_scene_from_cfg(cfg);
assert(isequal(scene.scene_cfg.table_orientations(:), [pi/6;0;0;0]));
fprintf('SHORT_ORIENTATION_VECTOR=%s RESOLVED=%s\n', mat2str(cfg.table_orientations), mat2str(scene.scene_cfg.table_orientations(:)'));
cfg.table_orientations = [0;pi/6;-pi/4;pi/2;0.7];
scene = build_scene_from_cfg(cfg);
assert(numel(scene.scene_cfg.table_orientations) == 5);
fprintf('LONG_ORIENTATION_VECTOR_LENGTH=%d TABLE_COUNT=%d EXTRA_ORIENTATION_PRESERVED=1\n', ...
    numel(scene.scene_cfg.table_orientations), scene.scene_cfg.num_tables);
fprintf('ASSESSMENT_VERIFICATION=PASS TEST_COUNT=%d FULL_PATH_PLANNING_RUN=0\n', numel(results));
diary('off');
