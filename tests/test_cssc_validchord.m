function tests = test_cssc_validchord
%TEST_CSSC_VALIDCHORD Regression tests for chord/envelope validity handling.

    tests = functiontests(localfunctions);
end

function testStraightPathUsesAllValidChords(testCase)
    P = [linspace(0.05, 0.95, 13).', zeros(13, 1)];
    obstacle = obsCircle2D([0.50, 0.0], 0.05);
    params = baseParams(P);

    state = evaluateCSSCGlobal(P, obstacle, params);
    metrics = evaluateCSSCHighPrecision(P, obstacle, params);

    verifyGreaterThan(testCase, nnz(state.env.validChord), 0);
    verifyEqual(testCase, nnz(state.env.validLine), 0);
    verifyEqual(testCase, state.validLine, state.env.validChord);
    verifyEqual(testCase, state.clearanceValiditySource, 'validChord');
    verifyLessThan(testCase, state.minClear, 0);
    verifyEqual(testCase, metrics.numEvaluatedChords, ...
        metrics.numValidChords);
    verifyEqual(testCase, metrics.chordCoverage, 1, 'AbsTol', 1e-12);
end

function testEmptyActivePenaltyIsConsistent(testCase)
    P = [linspace(0.00, 0.10, 6).', zeros(6, 1)];
    obstacle = obsCircle2D([0.05, 0.20], 0.02);
    params = baseParams(P);
    params.L = 0.15;

    [Jvalue, valueDetails] = ...
        objectiveCSSC2D_Value(P, P, obstacle, params);
    [Jsemi, semiDetails] = ...
        objectiveCSSC2D_SemiGrad(P, P, obstacle, params);

    verifyEqual(testCase, valueDetails.Jobs, 1e6);
    verifyEqual(testCase, valueDetails.Jclear, 1e6);
    verifyEqual(testCase, semiDetails.Jobs, valueDetails.Jobs);
    verifyEqual(testCase, semiDetails.Jclear, valueDetails.Jclear);
    verifyEqual(testCase, Jsemi, Jvalue, 'AbsTol', 1e-10);
end

function testNaNClearanceStopsAsFailure(testCase)
    P = [linspace(0.00, 0.10, 6).', zeros(6, 1)];
    obstacle = obsCircle2D([0.05, 0.20], 0.02);
    params = baseParams(P);
    params.L = 0.15;
    params.numIter = 3;
    params.printInterval = 100;

    [~, info] = optimizeCSSC2D(P, P, obstacle, params);

    verifyTrue(testCase, info.failed);
    verifyFalse(testCase, info.converged);
    verifyEqual(testCase, info.stopReason, 'invalid-clearance');
    verifyEqual(testCase, info.numIterActual, 1);
end

function params = baseParams(P)
    params = struct();
    params.L = 0.15;
    params.dMin = 0.02;
    params.dPref = 0.03;
    params.degree = 3;
    params.knot = makeClampedUniformKnot(size(P, 1), params.degree);
    params.clearanceMode = 'segment';
    params.envOpts = struct( ...
        'uRange', [0, 1], ...
        'vSearchRange', [0, 1], ...
        'nU', 120, ...
        'epsV', 1e-8, ...
        'tolDen', 1e-8);
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
    params.printEvalTiming = false;

    params.wObs = 600;
    params.wClear = 100;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0;
    params.wCurv = 0;
    params.kappaMax = 2;
    params.solver = struct('gradMode', 'semi-analytic');
    params.stop = struct('requireSafe', true);
end
