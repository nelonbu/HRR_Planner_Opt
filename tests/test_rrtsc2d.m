function tests = test_rrtsc2d
%TEST_RRTSC2D Minimal regression coverage for the formal RRTSC-2D baseline.

    tests = functiontests(localfunctions);
end

function testObstacleFreeSuccess(testCase)
    obstacles = emptyObstacles();
    opts = fastPlannerOpts(7);

    [P, path, info] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], obstacles, opts);

    verifyTrue(testCase, info.success);
    verifyEqual(testCase, info.terminationReason, 'success');
    verifyLessThanOrEqual(testCase, info.attemptCount, 2);
    verifyNotEmpty(testCase, P);
    verifyNotEmpty(testCase, path);
end

function testSmoothingCollisionRejected(testCase)
    path = [
        0.05, 0.00
        0.32, 0.00
        0.40, 0.22
        0.60, 0.22
        0.68, 0.00
        0.95, 0.00
    ];
    obstacle = obsCircle2D([0.50, 0.08], 0.11);
    [P, splineInfo] = polylineToBSplineInit2D( ...
        path, struct('degree', 3, 'nCtrl', 6));

    opts = defaultRRTSC2DParams();
    opts.safetyMargin = 0;
    opts.knot = splineInfo.knot;
    rawClear = sampledPolylineClearance(path, obstacle, 0.001);
    validation = validateRRTSCCenterline2D(P, obstacle, opts);

    verifyGreaterThan(testCase, rawClear, 0);
    verifyFalse(testCase, validation.safe);
    verifyLessThan(testCase, validation.minClear, 0);
end

function testChordSweepCollisionRejected(testCase)
    P = [
        0.05, 0.00
        0.28, 0.00
        0.40, 0.30
        0.50, 0.34
        0.60, 0.30
        0.72, 0.00
        0.95, 0.00
    ];
    obstacle = obsCircle2D([0.50, 0.21], 0.08);
    opts = defaultRRTSC2DParams();
    opts.safetyMargin = 0;
    opts.chord.nU = 180;
    opts.chord.maxRefinement = 0;

    centerValidation = validateRRTSCCenterline2D(P, obstacle, opts);
    chordValidation = validateRRTSCChordSweep2D(P, obstacle, opts);

    verifyTrue(testCase, centerValidation.safe);
    verifyGreaterThan(testCase, centerValidation.minClear, 0);
    verifyFalse(testCase, chordValidation.safe);
    verifyFalse(testCase, chordValidation.numericalFailure);
    verifyLessThan(testCase, chordValidation.minClear, 0);
end

function testSeedReproducibility(testCase)
    obstacles = emptyObstacles();
    opts = fastPlannerOpts(19);

    [P1, path1, info1] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], obstacles, opts);
    [P2, path2, info2] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], obstacles, opts);

    verifyEqual(testCase, P1, P2, 'AbsTol', 1e-13);
    verifyEqual(testCase, path1, path2, 'AbsTol', 1e-13);
    verifyEqual(testCase, info1.success, info2.success);
    verifyEqual(testCase, info1.attemptCount, info2.attemptCount);
    verifyEqual(testCase, info1.rrtFailureCount, info2.rrtFailureCount);
    verifyEqual(testCase, info1.centerlineRejectCount, ...
        info2.centerlineRejectCount);
    verifyEqual(testCase, info1.chordRejectCount, info2.chordRejectCount);
    verifyEqual(testCase, info1.attempts, info2.attempts);
    verifyEqual(testCase, [info1.attemptLog.seed], ...
        [info2.attemptLog.seed]);
    verifyEqual(testCase, {info1.attemptLog.outcome}, ...
        {info2.attemptLog.outcome});
end

function testAttemptAndTimeLimits(testCase)
    blockingObstacle = obsCircle2D([0.05, 0], 0.04);
    opts = fastPlannerOpts(23);
    opts.maxAttempts = 2;

    [P, path, info] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], blockingObstacle, opts);
    verifyFalse(testCase, info.success);
    verifyEmpty(testCase, P);
    verifyEmpty(testCase, path);
    verifyEqual(testCase, info.attemptCount, 2);
    verifyEqual(testCase, info.terminationReason, 'noRRTPath');

    timeOpts = fastPlannerOpts(23);
    timeOpts.maxTotalTime = 1e-12;
    [~, ~, timeInfo] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], emptyObstacles(), timeOpts);
    verifyFalse(testCase, timeInfo.success);
    verifyEqual(testCase, timeInfo.terminationReason, 'maxTotalTime');
end

function testBestRejectedCandidateReturnedAsFallback(testCase)
    obstacle = obsCircle2D([0.50, 2.00], 0.10);
    opts = fastPlannerOpts(31);
    opts.maxAttempts = 1;
    opts.safetyMargin = 10.0;

    [P, path, info] = planRRTSC2D( ...
        [0.05, 0], [0.95, 0], obstacle, opts);

    verifyFalse(testCase, info.success);
    verifyFalse(testCase, info.strictAccepted);
    verifyTrue(testCase, info.outputAvailable);
    verifyTrue(testCase, info.fallbackReturned);
    verifyEqual(testCase, info.returnedAttempt, 1);
    verifyNotEmpty(testCase, P);
    verifyNotEmpty(testCase, path);
    verifyNotEqual(testCase, info.solutionStatus, 'strict-safe');
end

function opts = fastPlannerOpts(seed)
    opts = defaultRRTSC2DParams();
    opts.seed = seed;
    opts.maxAttempts = 3;
    opts.maxTotalTime = 5;
    opts.rrt.maxIter = 500;
    opts.centerline.minSamples = 300;
    opts.chord.nU = 100;
    opts.chord.maxRefinement = 1;
end

function obstacles = emptyObstacles()
    obstacles = repmat(obsCircle2D([0, 0], 0), 0, 1);
end

function minClear = sampledPolylineClearance(path, obstacles, resolution)
    minClear = inf;
    for i = 1:size(path,1)-1
        a = path(i,:);
        b = path(i+1,:);
        n = max(2, ceil(norm(b-a) / resolution) + 1);
        t = linspace(0, 1, n).';
        samples = (1-t).*a + t.*b;
        for j = 1:size(samples,1)
            minClear = min(minClear, ...
                queryObstaclePointSDF2D(samples(j,:), obstacles));
        end
    end
end
