function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2*pi);
    if angle < 0
        angle = angle + 2*pi;
    end
    angle = angle - pi;
end