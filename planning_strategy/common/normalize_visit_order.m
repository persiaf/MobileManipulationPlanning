function normalized = normalize_visit_order(order, num_docks, options)
%NORMALIZE_VISIT_ORDER Convert route orders to local dock indices.
% The returned dock_order uses 1:num_docks and excludes the start/return node.
% Global route nodes are assumed to index [start; selected_poses], where start
% is node 1 and dock k is node k+1.

    if nargin < 3 || isempty(options)
        options = struct();
    end
    normalized = struct('dock_order', [], 'includes_start', false, ...
        'includes_return', false, 'is_valid', true, 'error_message', '');

    if isempty(order)
        normalized.is_valid = num_docks == 0;
        if ~normalized.is_valid
            normalized.error_message = 'Empty order for non-empty dock set.';
        end
        return;
    end

    order = reshape(double(order), 1, []);
    mode = get_option(options, 'node_indexing', 'auto');
    if strcmp(mode, 'auto')
        mode = infer_mode(order, num_docks);
    end

    switch mode
        case 'global'
            normalized.includes_start = any(order == 1);
            if numel(order) >= 2 && order(end) == 1
                normalized.includes_return = true;
            end
            dock_order = order(order ~= 1) - 1;
        case 'local'
            dock_order = order;
        otherwise
            normalized.is_valid = false;
            normalized.error_message = sprintf('Unknown node_indexing mode: %s', mode);
            return;
    end

    if any(dock_order < 1) || any(dock_order > num_docks) || ...
            numel(unique(dock_order)) ~= numel(dock_order)
        normalized.is_valid = false;
        normalized.error_message = 'Order contains invalid or repeated dock indices.';
        normalized.dock_order = dock_order;
        return;
    end
    if numel(dock_order) ~= num_docks
        normalized.is_valid = false;
        normalized.error_message = 'Order does not contain every selected dock exactly once.';
    end
    normalized.dock_order = dock_order;
end

function mode = infer_mode(order, num_docks)
    if any(order == 0) || any(order > num_docks + 1)
        mode = 'invalid';
    elseif any(order == num_docks + 1) || numel(order) == num_docks + 1 || ...
            (numel(order) == num_docks + 2 && order(1) == 1 && order(end) == 1)
        mode = 'global';
    else
        mode = 'local';
    end
end

function v = get_option(options, field_name, default_value)
    if isstruct(options) && isfield(options, field_name)
        v = options.(field_name);
    else
        v = default_value;
    end
end
