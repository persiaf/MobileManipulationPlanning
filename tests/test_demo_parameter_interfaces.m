function tests = test_demo_parameter_interfaces
    tests = functiontests(localfunctions);
end

function testMaintainedDemosAcceptDocumentedArguments(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    cleanup = onCleanup(@() rmpath(root)); %#ok<NASGU>
    verifyEqual(testCase, nargin('demo_complex_scene'), 2);
    verifyEqual(testCase, nargin('demo_complex_planning'), 2);
    verifyEqual(testCase, nargin('demo_dynamic_replanning'), 2);
    verifyEqual(testCase, nargin('demo_cafeteria_scene'), 2);
    verifyEqual(testCase, nargin('demo_cafeteria_planning'), 3);
    verifyEqual(testCase, nargin('main_cafeteria_mobile_manipulation_planning'), 3);
end
