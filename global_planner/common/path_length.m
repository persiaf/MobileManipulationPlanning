function d = path_length(path)
    d = 0;
    for k = 2:size(path, 1)
        d = d + sqrt((path(k,1)-path(k-1,1))^2 + (path(k,2)-path(k-1,2))^2);
    end
end