function tests = test_no_removed_learning_references
    tests = functiontests(localfunctions);
end

function testSourceTreeIsClean(testCase)
    tests_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(tests_dir);
    tokens = {char([99, 102, 103, 46, 108, 110, 110]), ...
        char([112, 121, 116, 104, 111, 110, 95, 108, 110, 110]), ...
        char([108, 116, 114, 95, 114, 97, 110, 107, 101, 114]), ...
        char([114, 101, 115, 101, 114, 118, 111, 105, 114])};
    files = files_to_scan(project_root);
    violations = {};
    for i = 1:numel(files)
        path_value = fullfile(files(i).folder, files(i).name);
        if strcmp(files(i).name, 'CHANGELOG.md') || startsWith(files(i).name, 'Codex_')
            continue;
        end
        content = lower(fileread(path_value));
        for k = 1:numel(tokens)
            if contains(content, tokens{k})
                violations{end + 1} = path_value; %#ok<AGROW>
                break;
            end
        end
    end
    verifyEmpty(testCase, unique(violations));
end

function files = files_to_scan(project_root)
%FILES_TO_SCAN Limit the audit to executable source and maintained project docs.
% Generated results, paper builds, caches, archives, and historical artifacts
% are not implementation inputs and must not affect this source-tree check.
    source_roots = { ...
        'config', 'environment', 'collision', 'manipulator', 'docking', ...
        'planning_strategy', 'global_planner', 'execution', 'integration', ...
        'calibration', 'utils', 'pipeline', 'experiments', 'visualization', 'tests'};
    files = dir(fullfile(project_root, 'README.md'));
    for i = 1:numel(source_roots)
        root = fullfile(project_root, source_roots{i});
        files = [files; dir(fullfile(root, '**', '*.m'))]; %#ok<AGROW>
        files = [files; dir(fullfile(root, '**', '*.md'))]; %#ok<AGROW>
        files = [files; dir(fullfile(root, '**', '*.json'))]; %#ok<AGROW>
    end
    docs_root = fullfile(project_root, 'docs');
    files = [files; dir(fullfile(docs_root, '**', '*.md'))]; %#ok<AGROW>
    files = unique_file_entries(files);
end

function files = unique_file_entries(files)
    if isempty(files)
        return;
    end
    keys = arrayfun(@(f) fullfile(f.folder, f.name), files, 'UniformOutput', false);
    [~, keep] = unique(keys, 'stable');
    files = files(keep);
end
