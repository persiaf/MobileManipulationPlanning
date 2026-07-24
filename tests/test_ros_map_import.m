function tests = test_ros_map_import
    tests = functiontests(localfunctions);
end

function testYamlPgmImport(testCase)
    temp_dir = tempname;
    mkdir(temp_dir);
    cleanup = onCleanup(@() remove_temp_dir(temp_dir)); %#ok<NASGU>
    image_path = fullfile(temp_dir, 'map.pgm');
    yaml_path = fullfile(temp_dir, 'map.yaml');
    imwrite(uint8([255, 0; 205, 255]), image_path);
    fid = fopen(yaml_path, 'w');
    file_cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'image: map.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0.0]\n');
    fprintf(fid, 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n');
    clear file_cleanup;

    [grid_map, map_info] = load_ros_map(yaml_path);
    verifyEqual(testCase, size(grid_map), [2, 2]);
    verifyEqual(testCase, grid_map(1, 1), 0);
    verifyEqual(testCase, grid_map(1, 2), 1);
    verifyEqual(testCase, map_info.origin_xy, [1.0, 2.0]);
end

function remove_temp_dir(path_value)
    if isfolder(path_value), rmdir(path_value, 's'); end
end
