function metadata = summarize_region_metadata(scene, candidate_library, selected_pose_region_ids)
%SUMMARIZE_REGION_METADATA Build compact region statistics for experiments.

    if nargin < 3
        selected_pose_region_ids = [];
    end
    if isfield(scene, 'region_metadata')
        metadata = scene.region_metadata;
    else
        metadata = struct();
        metadata.table_region_id = (1:size(scene.scene_cfg.table_positions, 1))';
        metadata.task_region_id = zeros(scene.num_tasks, 1);
        metadata.region_ids = unique(metadata.table_region_id, 'stable');
        metadata.region_count = numel(metadata.region_ids);
        metadata.tables_per_region = ones(metadata.region_count, 1);
        metadata.tasks_per_region = zeros(metadata.region_count, 1);
    end

    metadata.candidate_count_per_region = zeros(numel(metadata.region_ids), 1);
    if nargin >= 2 && isstruct(candidate_library) && isfield(candidate_library, 'regions')
        for ri = 1:numel(candidate_library.regions)
            region_id = candidate_library.regions(ri).region_id;
            idx = find(metadata.region_ids == region_id, 1);
            if isempty(idx) && isfield(candidate_library.regions(ri), 'table_id')
                table_id = candidate_library.regions(ri).table_id;
                if table_id <= numel(metadata.table_region_id)
                    idx = find(metadata.region_ids == metadata.table_region_id(table_id), 1);
                end
            end
            if ~isempty(idx)
                metadata.candidate_count_per_region(idx) = ...
                    metadata.candidate_count_per_region(idx) + candidate_library.regions(ri).num_candidates;
            end
        end
    end

    if ~isempty(selected_pose_region_ids)
        metadata.dock_region_id = reshape(selected_pose_region_ids, [], 1);
        metadata.selected_docks_per_region = zeros(numel(metadata.region_ids), 1);
        for i = 1:numel(metadata.region_ids)
            metadata.selected_docks_per_region(i) = sum(metadata.dock_region_id == metadata.region_ids(i));
        end
    end
end
