function h = build_scene_legend(ax, location)
%BUILD_SCENE_LEGEND Build one legend entry per nonempty display name.

    if nargin < 2 || isempty(location), location = 'eastoutside'; end
    objects = findobj(ax, '-property', 'DisplayName');
    objects = flipud(objects(:));
    keep = gobjects(0);
    names = {};
    for i = 1:numel(objects)
        name = get(objects(i), 'DisplayName');
        if isempty(name) || any(strcmp(names, name)), continue; end
        names{end + 1} = name; %#ok<AGROW>
        keep(end + 1, 1) = objects(i); %#ok<AGROW>
    end
    if isempty(keep)
        h = gobjects(0);
    else
        h = legend(ax, keep, names, 'Location', location, 'Interpreter', 'none');
    end
end
