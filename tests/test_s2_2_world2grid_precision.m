function tests = test_s2_2_world2grid_precision
    tests = functiontests(localfunctions);
end

function testFourTableCentersUseTranslationConsistentCells(testCase)
    resolution = 0.05;
    height_m = 8.0;
    [r22, c22] = world2grid(2.2, 2.2, resolution, height_m);
    [r56, c56] = world2grid(5.6, 5.6, resolution, height_m);
    verifyEqual(testCase, [r22 c22], [117 45]);
    verifyEqual(testCase, [r56 c56], [49 113]);
    verifyEqual(testCase, abs(r22 - r56), 68);
    verifyEqual(testCase, c56 - c22, 68);
end

function testTranslatedDockCentersKeepTheirTableOffset(testCase)
    resolution = 0.05;
    height_m = 8.0;
    [table1_row, ~] = world2grid(2.2, 2.2, resolution, height_m);
    [table3_row, ~] = world2grid(2.2, 5.6, resolution, height_m);
    [dock1_row, dock_col] = world2grid(1.9, 1.75, resolution, height_m);
    [dock3_row, translated_col] = world2grid(1.9, 5.15, resolution, height_m);
    verifyEqual(testCase, table1_row - table3_row, 68);
    verifyEqual(testCase, dock1_row - dock3_row, 68);
    verifyEqual(testCase, dock_col, translated_col);
    verifyEqual(testCase, dock1_row - table1_row, dock3_row - table3_row);
end

function testGridLinesAndPointsOutsideSnapTolerance(testCase)
    resolution = 0.05;
    height_m = 8.0;
    [~, on_line] = world2grid(1.0, 4.0, resolution, height_m);
    [~, just_left] = world2grid(1.0 - 1e-10, 4.0, resolution, height_m);
    [~, just_right] = world2grid(1.0 + 1e-10, 4.0, resolution, height_m);
    verifyEqual(testCase, [just_left on_line just_right], [20 21 21]);
end

function testMapCornersAndOutsideIndicesKeepExistingConvention(testCase)
    resolution = 0.05;
    height_m = 8.0;
    [top_left_row, top_left_col] = world2grid(0, 8, resolution, height_m);
    [bottom_right_row, bottom_right_col] = world2grid(8, 0, resolution, height_m);
    [outside_row, outside_col] = world2grid(-0.1, 8.1, resolution, height_m);
    verifyEqual(testCase, [top_left_row top_left_col], [1 1]);
    verifyEqual(testCase, [bottom_right_row bottom_right_col], [161 161]);
    verifyEqual(testCase, [outside_row outside_col], [-1 -1]);
end

function testDifferentResolutionAndOrigin(testCase)
    [row, col] = world2grid(1.0, 1.0, 0.1, 4.0, [-1.0 -1.0]);
    verifyEqual(testCase, [row col], [21 21]);
end

function testCellCenterRoundTrip(testCase)
    resolution = 0.05;
    height_m = 8.0;
    origin = [-1.5 2.0];
    rows = [1 40 160];
    cols = [1 72 160];
    for k = 1:numel(rows)
        [x, y] = grid2world(rows(k), cols(k), resolution, height_m, origin);
        [actual_row, actual_col] = world2grid(x, y, resolution, height_m, origin);
        verifyEqual(testCase, [actual_row actual_col], [rows(k) cols(k)]);
    end
end

function testOrdinaryCoordinatesRetainFloorBehavior(testCase)
    resolution = 0.05;
    height_m = 8.0;
    x = 2.237;
    y = 5.413;
    [row, col] = world2grid(x, y, resolution, height_m);
    verifyEqual(testCase, col, floor(x / resolution) + 1);
    verifyEqual(testCase, row, floor((height_m - y) / resolution) + 1);
end

function testNonFiniteInputsKeepExistingResults(testCase)
    [row, col] = world2grid(NaN, Inf, 0.05, 8.0);
    verifyEqual(testCase, row, -Inf);
    verifyTrue(testCase, isnan(col));
end
