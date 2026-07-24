%% save_publication_figures.m
% SCI publication figure export dispatcher.
%
% Run:
%   run('visualization/save_publication_figures.m')

project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(project_root));
init_paths(project_root);

out_dir = fullfile(project_root, 'results', 'figures');
table_dir = fullfile(project_root, 'results', 'tables');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fprintf('===== SCI Publication Figures Export =====\n');
fprintf('Output: %s\n\n', out_dir);

fprintf('[1/8] Framework pipeline ...\n');
plot_framework_pipeline(out_dir);

fprintf('[2/8] Main scenario publication view ...\n');
plot_main_scenario_publication(out_dir);

fprintf('[3/8] Ablation completion/uncovered ...\n');
plot_ablation_bar(fullfile(table_dir, 'ablation_summary.csv'), out_dir);

fprintf('[4/8] Scalability summary ...\n');
plot_scalability_curves(fullfile(table_dir, 'random_batch_summary.csv'), out_dir);

fprintf('[5/8] Hard scenario summary ...\n');
plot_hard_scenario_comparison(fullfile(table_dir, 'hard_scenario_summary.csv'), out_dir);

fprintf('[6/8] Normal vs hard task distribution ...\n');
plot_normal_vs_hard_task_distribution(out_dir, 2026);

fprintf('[7/8] Hard ablation comparison ...\n');
plot_hard_ablation_comparison(fullfile(table_dir, 'hard_ablation_summary.csv'), out_dir);

fprintf('[8/8] Legacy normal-vs-hard scene helper skipped by default.\n');

fprintf('\n===== Publication figure export finished =====\n');
fprintf('If a CSV was missing, its figure was skipped with a warning.\n');
