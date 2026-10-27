function cfg = legacy_visualization_config(overrides)
%LEGACY_VISUALIZATION_CONFIG Adapt canonical settings to the stage API.

    canonical = visualization_config();
    cfg = canonical.legacy;
    if nargin >= 1 && ~isempty(overrides)
        cfg = merge_struct_recursive(cfg, overrides);
    end
end
