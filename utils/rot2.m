function R = rot2(theta)
    c = cos(theta);
    s = sin(theta);
    R = [c, -s; s, c];
end