function validate_candidate_library(candidate_library, regions, scene, robot_cfg, planner_cfg)
%VALIDATE_CANDIDATE_LIBRARY Check that a library matches the current regions.

    if ~isstruct(candidate_library) || ~isfield(candidate_library, 'regions')
        error('validate_candidate_library:InvalidLibrary', 'Candidate library is missing regions.');
    end
    if numel(candidate_library.regions) ~= numel(regions)
        error('validate_candidate_library:RegionCountMismatch', ...
            'Candidate library has %d regions; expected %d.', ...
            numel(candidate_library.regions), numel(regions));
    end
    for ri = 1:numel(regions)
        if candidate_library.regions(ri).table_id ~= regions(ri).table_id
            error('validate_candidate_library:TableMismatch', ...
                'Region %d table mismatch.', ri);
        end
    end
    if nargin >= 5 && isfield(candidate_library, 'scene_signature')
        expected = build_candidate_library_signature(regions, scene, robot_cfg, planner_cfg);
        if ~strcmp(candidate_library.scene_signature, expected)
            error('validate_candidate_library:StaleLibrary', ...
                'Candidate library does not match the scene, robot, or reachability configuration.');
        end
    end
end
