function tests = test_world_grid_roundtrip
    tests = functiontests(localfunctions);
end

function testRoundtripWithinOneCell(testCase)
    resolution = 0.05;
    height_m = 8.0;
    origin = [-1.5, 2.0];
    points = [-1.49, 2.01; 0.17, 5.33; 4.42, 9.91];
    for i = 1:size(points, 1)
        [row, col] = world2grid(points(i, 1), points(i, 2), resolution, height_m, origin);
        [x, y] = grid2world(row, col, resolution, height_m, origin);
        verifyLessThanOrEqual(testCase, norm([x, y] - points(i, :), inf), resolution);
    end
end
