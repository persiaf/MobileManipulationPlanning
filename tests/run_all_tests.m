function results = run_all_tests()
%RUN_ALL_TESTS Initialize the project and execute the MATLAB test suite.

    tests_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(tests_dir);
    addpath(fullfile(project_root, 'pipeline'));
    init_paths(project_root);
    addpath(tests_dir);
    results = runtests(tests_dir, 'IncludeSubfolders', false);
    assert(all([results.Passed]), 'One or more project tests failed.');
end
