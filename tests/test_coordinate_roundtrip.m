function tests = test_coordinate_roundtrip
    tests = functiontests(localfunctions);
end

function testWorldGridWorld(testCase)
    resolution = 0.05;
    height_m = 4.0;
    origin = [-1.0, 2.0];
    points = [-0.75, 2.25; 0.10, 3.10; 1.75, 5.75];
    for k = 1:size(points, 1)
        [gi, gj] = world2grid(points(k, 1), points(k, 2), resolution, height_m, origin);
        [wx, wy] = grid2world(gi, gj, resolution, height_m, origin);
        verifyLessThanOrEqual(testCase, norm([wx, wy] - points(k, :), inf), resolution / 2 + 1e-12);
    end
end
