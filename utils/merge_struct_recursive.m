function output = merge_struct_recursive(base, override)
%MERGE_STRUCT_RECURSIVE Recursively overlay scalar configuration structs.

    output = base;
    if isempty(override)
        return;
    end
    if ~isstruct(base) || ~isstruct(override) || ~isscalar(base) || ~isscalar(override)
        output = override;
        return;
    end
    names = fieldnames(override);
    for i = 1:numel(names)
        name = names{i};
        if isfield(output, name) && isstruct(output.(name)) && ...
                isscalar(output.(name)) && isstruct(override.(name)) && ...
                isscalar(override.(name))
            output.(name) = merge_struct_recursive(output.(name), override.(name));
        else
            output.(name) = override.(name);
        end
    end
end
