function expanded = expand_lift_candidates(service_config, real_cfg)
%EXPAND_LIFT_CANDIDATES Expand a base pose over configured discrete heights.

    if nargin < 2 || isempty(real_cfg)
        real_cfg = real_platform_config();
    end
    lift = real_cfg.lift;
    if ~lift.enabled
        heights = lift.default_height_m;
    else
        heights = lift.candidate_heights_m;
        heights = heights(heights >= lift.min_height_m & heights <= lift.max_height_m);
        if isempty(heights)
            error('expand_lift_candidates:NoValidHeight', ...
                'No lift candidate lies inside the configured height limits.');
        end
    end

    expanded = repmat(service_config, 1, numel(heights));
    for k = 1:numel(heights)
        expanded(k).lift_height_m = heights(k);
    end
end
