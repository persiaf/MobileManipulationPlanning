function signature = build_struct_signature(value)
%BUILD_STRUCT_SIGNATURE Deterministic two-lane hash of JSON-serializable data.
    bytes = double(uint8(jsonencode(value)));
    p1 = 4294967291; p2 = 4294967279; multiplier = 65599;
    h1 = 2166136261; h2 = 2246822519;
    for i = 1:numel(bytes)
        h1 = mod(h1 * multiplier + bytes(i) + 1, p1);
        h2 = mod(h2 * multiplier + bytes(i) + 1, p2);
    end
    signature = lower([dec2hex(uint32(h1), 8), dec2hex(uint32(h2), 8)]);
end
