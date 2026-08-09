function tests = test_sprrt2d
%TEST_SPRRT2D Regression coverage for Adaptive Sp-RRT-2D.

    tests = functiontests(localfunctions);
end

function testObstacleFreeReachableAndReproducible(testCase)
    opts = deterministicOpts();
    obstacles = emptyObstacles();
    startPt = [0.00, 0.00];
    goalPt = [0.60, 0.00];

    [path1, info1] = planAdaptiveSpRRT2D( ...
        startPt, goalPt, obstacles, opts);
    [path2, info2] = planAdaptiveSpRRT2D( ...
        startPt, goalPt, obstacles, opts);

    verifyTrue(testCase, info1.success);
    verifyEqual(testCase, path1(1,:), startPt, 'AbsTol', 1e-12);
    verifyEqual(testCase, path1(end,:), goalPt, 'AbsTol', 1e-12);
    verifyLessThanOrEqual(testCase, info1.rawMaxTurn, ...
        40*pi/180 + opts.angleTolerance);
    verifyEqual(testCase, path1, path2, 'AbsTol', 1e-13);
    verifyEqual(testCase, info1.numIter, info2.numIter);
    verifyEqual(testCase, info1.abandonedCandidateCount, ...
        info2.abandonedCandidateCount);
    verifyEqual(testCase, info1.finalRawSegmentCount, ...
        info2.finalRawSegmentCount);
    verifyEqual(testCase, info1.maxDepthReached, ...
        info2.maxDepthReached);
    verifyLessThanOrEqual(testCase, ...
        info1.finalRawSegmentCount, info1.nominalLinkCount);
    verifyFalse(testCase, info1.usedExtraSegments);
end

function testRootHasNoDirectionButSecondEdgeHasTurnLimit(testCase)
    opts = deterministicOpts();
    obstacles = emptyObstacles();
    tree = makeTree([0,0], 0, 0);

    [rootAccepted, rootDetail] = connectSpRRT2D( ...
        tree, 1, [0.15,0], 0.15, false, obstacles, opts);
    verifyTrue(testCase, rootAccepted);
    verifyTrue(testCase, isnan(rootDetail.theta));

    tree.position(2,:) = [0.15,0];
    tree.parent(2,1) = 1;
    tree.depth(2,1) = 1;
    tree.cost(2,1) = 0.15;
    [secondAccepted, secondDetail] = connectSpRRT2D( ...
        tree, 2, [0.15,0.15], 0.15, false, obstacles, opts);
    verifyFalse(testCase, secondAccepted);
    verifyEqual(testCase, secondDetail.reason, 'angle');
end

function testSmallerThetaLimitRejectsMoreStrictly(testCase)
    opts = deterministicOpts();
    obstacles = emptyObstacles();
    tree = makeTree([0,0], 0, 0);
    tree.position(2,:) = [0.15,0];
    tree.parent(2,1) = 1;
    tree.depth(2,1) = 1;
    tree.cost(2,1) = 0.15;
    q2 = tree.position(2,:) + 0.15 * [cosd(30), sind(30)];

    [accepted40, detail40] = connectSpRRT2D( ...
        tree, 2, q2, 0.15, false, obstacles, opts);
    opts.thetaMax = deg2rad(20);
    [accepted20, detail20] = connectSpRRT2D( ...
        tree, 2, q2, 0.15, false, obstacles, opts);

    verifyTrue(testCase, accepted40);
    verifyEqual(testCase, detail40.theta, deg2rad(30), ...
        'AbsTol', 1e-12);
    verifyFalse(testCase, accepted20);
    verifyEqual(testCase, detail20.reason, 'angle');
end

function testExpandTriesAlternativeNearestCandidates(testCase)
    opts = deterministicOpts();
    opts.bounds = [-0.1,0.5; -0.2,0.3];
    tree.position = [0,0; 0.15,0; 0,0.15];
    tree.parent = [0;1;1];
    tree.depth = [0;1;1];
    tree.cost = [0;0.15;0.15];
    qRand = [0.30,0.10];
    blockedMidpoint = [0.2124,0.0416];
    obstacle = obsCircle2D(blockedMidpoint, 0.015);

    [treeOut, newIdx, stats] = ...
        expandSpRRT2D(tree, qRand, obstacle, opts);

    verifyGreaterThan(testCase, newIdx, 0);
    verifyEqual(testCase, treeOut.parent(newIdx), 1);
    verifyGreaterThanOrEqual(testCase, stats.candidateAttemptCount, 2);
    verifyGreaterThanOrEqual(testCase, stats.abandonedCandidateCount, 1);
    verifyEqual(testCase, stats.expandSuccessCount, 1);
end

function testAdaptiveSegmentReachabilityDiagnostics(testCase)
    opts = deterministicOpts();
    obstacles = emptyObstacles();

    [pathFar, infoFar] = planAdaptiveSpRRT2D( ...
        [0,0], [1.90,0], obstacles, opts);
    verifyEmpty(testCase, pathFar);
    verifyEqual(testCase, infoFar.terminationReason, ...
        'structurallyUnreachable');
    verifyEqual(testCase, infoFar.maximumAllowedLength, 1.8, ...
        'AbsTol', 1e-12);

    obstacle = obsCircle2D([0.90,0], 0.04);
    [pathBlocked, infoBlocked] = planAdaptiveSpRRT2D( ...
        [0,0], [1.8,0], obstacle, opts);
    verifyEmpty(testCase, pathBlocked);
    verifyTrue(testCase, infoBlocked.structuralWarning);
    verifyEqual(testCase, infoBlocked.terminationReason, ...
        'zeroMarginStraightPathBlocked');
end

function testDistanceBeyondNominalUsesExtraSegments(testCase)
    opts = deterministicOpts();
    startPt = [0.00,0.00];
    goalPt = [1.20,0.00];

    [path, info] = planAdaptiveSpRRT2D( ...
        startPt, goalPt, emptyObstacles(), opts);

    verifyTrue(testCase, info.success);
    verifyEqual(testCase, path(1,:), startPt, 'AbsTol', 1e-12);
    verifyEqual(testCase, path(end,:), goalPt, 'AbsTol', 1e-12);
    verifyEqual(testCase, info.finalRawSegmentCount, 8);
    verifyTrue(testCase, info.usedExtraSegments);
    verifyEqual(testCase, info.numberOfExtraSegments, 2);
    verifyEqual(testCase, info.maxDepthReached, 8);
end

function testTwelfthSegmentAcceptedWithoutThirteenth(testCase)
    opts = deterministicOpts();
    [path, info] = planAdaptiveSpRRT2D( ...
        [0,0], [1.80,0], emptyObstacles(), opts);

    verifyTrue(testCase, info.success);
    verifyEqual(testCase, info.finalRawSegmentCount, 12);
    verifyEqual(testCase, info.maxDepthReached, 12);
    verifyLessThanOrEqual(testCase, ...
        info.finalRawSegmentCount, info.maxSegmentCount);
    verifyEqual(testCase, path(1,:), [0,0], 'AbsTol', 1e-12);
    verifyEqual(testCase, path(end,:), [1.8,0], 'AbsTol', 1e-12);
end

function testDepthSixBranchCanExtendAroundObstacle(testCase)
    opts = deterministicOpts();
    headings = deg2rad([180,160,140,180,220,200,180]);
    points = zeros(numel(headings)+1, 2);
    points(1,:) = [0.90,0.00];
    for i = 1:numel(headings)
        points(i+1,:) = points(i,:) + ...
            opts.L * [cos(headings(i)), sin(headings(i))];
    end
    obstacle = obsCircle2D([0.45,0.00], 0.10);

    tree.position = points(1:7,:);
    tree.parent = [0;(1:6).'];
    tree.depth = (0:6).';
    tree.cost = tree.depth * opts.L;
    [treeOut, newIdx, stats] = expandSpRRT2D( ...
        tree, points(8,:), obstacle, opts);

    verifyGreaterThan(testCase, newIdx, 0);
    verifyEqual(testCase, treeOut.depth(newIdx), 7);
    verifyEqual(testCase, treeOut.parent(newIdx), 7);
    verifyEqual(testCase, stats.depthRejectCount, 0);
    path = flipud(treeOut.position(1:newIdx,:));
    validation = validateSpRRTPolyline2D(path, obstacle, opts);
    verifyTrue(testCase, validation.valid);
    verifyGreaterThan(testCase, size(path,1)-1, ...
        opts.nominalLinkCount);
end

function testDeterministicPathOptimization(testCase)
    opts = deterministicOpts();
    rawPath = [
        0.00, 0.00
        0.10, 0.02
        0.20, 0.00
        0.30, 0.02
        0.40, 0.00
    ];

    [optimized, info] = optimizeSpRRTPath2D( ...
        rawPath, emptyObstacles(), opts);

    verifyTrue(testCase, info.validationPassed);
    verifyGreaterThan(testCase, info.shortcutCandidateCount, 0);
    verifyGreaterThan(testCase, info.shortcutAcceptedCount, 0);
    verifyLessThanOrEqual(testCase, info.optimizedPathLength, ...
        info.rawPathLength + opts.lengthTolerance);
    verifyEqual(testCase, optimized(1,:), rawPath(1,:));
    verifyEqual(testCase, optimized(end,:), rawPath(end,:));
end

function testCollidingShortcutIsRejected(testCase)
    opts = deterministicOpts();
    rawPath = [
        0.00, 0.00
        0.10, 0.035
        0.20, 0.00
    ];
    obstacle = obsCircle2D([0.10,0.00], 0.02);

    [optimized, info] = optimizeSpRRTPath2D( ...
        rawPath, obstacle, opts);

    verifyTrue(testCase, info.validationPassed);
    verifyGreaterThan(testCase, size(optimized,1), 2);
    verifyGreaterThan(testCase, info.shortcutCandidateCount, 0);
    verifyFalse(testCase, info.lastCandidateDetail.collisionFree);
end

function testMaxIterReturnsNormally(testCase)
    opts = deterministicOpts();
    opts.entranceBias = 0;
    opts.maxIter = 1;
    opts.seed = 31;

    [path, info] = planAdaptiveSpRRT2D( ...
        [0,0], [0.60,0], emptyObstacles(), opts);

    verifyEmpty(testCase, path);
    verifyFalse(testCase, info.success);
    verifyEqual(testCase, info.terminationReason, 'maxIter');
    verifyEqual(testCase, info.numIter, 1);
end

function testPlannerAndWholeBodySuccessRemainSeparate(testCase)
    opts = deterministicOpts();
    startPt = [0.00,0.00];
    goalPt = [0.60,0.00];
    obstacle = obsCircle2D([0.30,0.06], 0.05);

    [path, info] = planAdaptiveSpRRT2D( ...
        startPt, goalPt, obstacle, opts);
    evalOpts = struct('L', opts.L, 'dMin', 0.02, ...
        'envOpts', struct('nU', 240), 'pathSampleN', 600);
    metrics = evaluateSpRRTPath2D(path, obstacle, evalOpts);

    verifyTrue(testCase, info.success);
    verifyTrue(testCase, metrics.success);
    verifyFalse(testCase, metrics.dMinSatisfied);
    verifyEqual(testCase, info.firstSolutionIter, info.numIter);
end

function testExtraSegmentPathUsesCommonFTLEvaluation(testCase)
    opts = deterministicOpts();
    [path, info] = planAdaptiveSpRRT2D( ...
        [0,0], [1.20,0], emptyObstacles(), opts);
    evalOpts = struct('L', opts.L, 'dMin', 0.02, ...
        'envOpts', struct('nU', 240), 'pathSampleN', 600);
    metrics = evaluateSpRRTPath2D(path, emptyObstacles(), evalOpts);

    verifyTrue(testCase, info.usedExtraSegments);
    verifyEqual(testCase, info.finalRawSegmentCount, 8);
    verifyTrue(testCase, isfinite(metrics.pathLength));
    verifyEqual(testCase, metrics.representation, ...
        'piecewise-linear-exact-chord');
end

function opts = deterministicOpts()
    opts = defaultSpRRT2DParams();
    opts.L = 0.15;
    opts.nominalLinkCount = 6;
    opts.maxSegmentCount = 12;
    opts.thetaMax = 40*pi/180;
    opts.bounds = [-0.1,2.1; -0.3,0.3];
    opts.seed = 17;
    opts.maxIter = 100;
    opts.maxTimeSec = inf;
    opts.entranceBias = 1.0;
    opts.collisionResolution = 0.001;
    opts.inflateRadius = 0;
    opts.emitWarnings = false;
end

function tree = makeTree(position, parent, depth)
    tree = struct();
    tree.position = position;
    tree.parent = parent;
    tree.depth = depth;
    tree.cost = 0;
end

function obstacles = emptyObstacles()
    obstacles = repmat(obsCircle2D([0,0], 0), 0, 1);
end
