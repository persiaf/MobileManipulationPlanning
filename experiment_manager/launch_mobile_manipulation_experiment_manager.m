function app = launch_mobile_manipulation_experiment_manager()
%LAUNCH_MOBILE_MANIPULATION_EXPERIMENT_MANAGER Open the configuration app.
%   The app is an App Designer-compatible MATLAB class and delegates all
%   experiment execution to run_selected_experiment.
    manager_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(manager_dir);
    addpath(manager_dir);
    addpath(fullfile(project_root, 'pipeline'));
    init_paths(project_root);
    app = MobileManipulationExperimentManager();
end
