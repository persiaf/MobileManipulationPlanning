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
    files = [dir(fullfile(project_root, '**', '*.m')); ...
             dir(fullfile(project_root, '**', '*.md')); ...
             dir(fullfile(project_root, '**', '*.json'))];
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
