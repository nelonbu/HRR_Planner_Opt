function tests = test_rrtconnect2d
%TEST_RRTCONNECT2D Regression tests for RRT-Connect and CSSC selection.

    tests = functiontests(localfunctions);
end

function testObstacleFreePathDirectionAndCollision(testCase)
    opts = deterministicOpts();
    [path, info] = planRRTConnect2D( ...
        [0,0], [1,0], emptyObstacles(), opts);

    verifyTrue(testCase, info.success);
    verifyEqual(testCase, path(1,:), [0,0], 'AbsTol', 1e-12);
    verifyEqual(testCase, path(end,:), [1,0], 'AbsTol', 1e-12);
    verifyGreaterThanOrEqual(testCase, info.numNodes, 2);
    verifyEqual(testCase, info.bestCost, info.pathLength, ...
        'AbsTol', 1e-12);

    for i = 1:size(path,1)-1
        verifyTrue(testCase, isSegmentCollisionFree2D( ...
            path(i,:), path(i+1,:), emptyObstacles(), opts));
    end
end

function testAlternatingTreesPreserveOutputDirection(testCase)
    opts = deterministicOpts();
    opts.goalBias = 1;
    opts.maxConnectSteps = 1;

    [path, info] = planRRTConnect2D( ...
        [0,0], [1,0], emptyObstacles(), opts);

    verifyTrue(testCase, info.success);
    verifyGreaterThan(testCase, info.treeSwapCount, 0);
    verifyEqual(testCase, path(1,:), [0,0], 'AbsTol', 1e-12);
    verifyEqual(testCase, path(end,:), [1,0], 'AbsTol', 1e-12);
end

function testSameSeedIsReproducible(testCase)
    opts = deterministicOpts();
    opts.goalBias = 0;
    [path1, info1] = planRRTConnect2D( ...
        [0,0], [1,0], emptyObstacles(), opts);
    [path2, info2] = planRRTConnect2D( ...
        [0,0], [1,0], emptyObstacles(), opts);

    verifyEqual(testCase, path1, path2, 'AbsTol', 1e-13);
    verifyEqual(testCase, info1.numIter, info2.numIter);
    verifyEqual(testCase, info1.numNodes, info2.numNodes);
    verifyEqual(testCase, info1.connectExtendCount, ...
        info2.connectExtendCount);
end

function testCSSCFrontendDefaultsToRRTConnect(testCase)
    opts = deterministicOpts();
    opts.goalBias = 1;
    [pathDefault, infoDefault] = planCSSCFrontend2D( ...
        [0,0], [1,0], emptyObstacles(), opts);

    opts.method = 'rrt';
    [pathRRT, infoRRT] = planCSSCFrontend2D( ...
        [0,0], [1,0], emptyObstacles(), opts);

    verifyTrue(testCase, infoDefault.success);
    verifyEqual(testCase, infoDefault.frontendMethod, 'rrtconnect');
    verifyEqual(testCase, pathDefault(1,:), [0,0], 'AbsTol', 1e-12);
    verifyTrue(testCase, infoRRT.success);
    verifyEqual(testCase, infoRRT.frontendMethod, 'rrt');
    verifyEqual(testCase, pathRRT(end,:), [1,0], 'AbsTol', 1e-12);
end

function testInvalidStartReturnsFailure(testCase)
    opts = deterministicOpts();
    obstacle = obsCircle2D([0,0], 0.05);
    [path, info] = planRRTConnect2D( ...
        [0,0], [1,0], obstacle, opts);

    verifyEmpty(testCase, path);
    verifyFalse(testCase, info.success);
end

function opts = deterministicOpts()
    opts = struct();
    opts.bounds = [-0.1,1.1; -0.3,0.3];
    opts.stepSize = 0.1;
    opts.goalBias = 0.2;
    opts.maxIter = 100;
    opts.collisionResolution = 0.005;
    opts.inflateRadius = 0;
    opts.seed = 73;
    opts.maxConnectSteps = inf;
end

function obstacles = emptyObstacles()
    obstacles = repmat(obsCircle2D([0,0], 0), 0, 1);
end
