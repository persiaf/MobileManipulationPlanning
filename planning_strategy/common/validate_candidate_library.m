function validate_candidate_library(candidate_library, regions)
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
end
