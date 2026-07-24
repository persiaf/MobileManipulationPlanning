function d = compute_path_length(path)
    if isempty(path) || size(path, 1) < 2
        d = 0;
        return;
    end
    d = sum(sqrt(diff(path(:,1)).^2 + diff(path(:,2)).^2));
end