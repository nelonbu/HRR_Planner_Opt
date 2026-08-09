function tests = test_polyline_high_precision2d
%TEST_POLYLINE_HIGH_PRECISION2D Native-polyline fixed-chord regression tests.
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    initCSSCProjectPath;
end

function testFirstForwardRootAcrossCorner(testCase)
    path = [0,0; 0.10,0; 0.10,0.20];
    L = 0.15;
    env = fixedChordSetPolyline2D(path, L, struct('nU', 31));

    expected = [0.10, sqrt(L^2 - 0.10^2)];
    verifyTrue(testCase, env.validChord(1));
    verifyEqual(testCase, env.NChord(1,:), expected, 'AbsTol', 1e-10);
    verifyLessThanOrEqual(testCase, ...
        abs(env.chordResidual(1)), 1e-10);
end

function testRootIsNotRejectedByEndpointDistance(testCase)
    path = [0,0; 0.20,0; 0.20,0.10; 0.05,0.10];
    L = 0.15;
    verifyLessThan(testCase, norm(path(end,:) - path(1,:)), L);

    env = fixedChordSetPolyline2D(path, L, struct('nU', 41));
    verifyTrue(testCase, env.validChord(1));
    verifyEqual(testCase, env.NChord(1,:), [L,0], 'AbsTol', 1e-10);
end

function testJaggedPolylineHasCompleteExactCoverage(testCase)
    path = [0,0; 0.08,0.03; 0.15,-0.02; 0.23,0.04; ...
        0.31,-0.01; 0.40,0.02];
    opts = struct('L',0.15, 'dMin',0.02, ...
        'envOpts',struct('nU',120), 'pathSampleN',300);
    metrics = evaluatePolylineHighPrecision2D( ...
        path, struct([]), opts);

    verifyTrue(testCase, metrics.chordConstructionSuccess);
    verifyFalse(testCase, metrics.feasibilityNumericalFailure);
    verifyEqual(testCase, metrics.chordCoverage, 1, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, ...
        metrics.maxChordLengthResidual, 1e-10);
end
