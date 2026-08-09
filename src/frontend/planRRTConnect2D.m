function [path, info] = planRRTConnect2D( ...
        startPt, goalPt, obstacles, opts)
%PLANRRTCONNECT2D Bidirectional RRT-Connect point-path planner.
%
% The planner alternates two trees rooted at startPt and goalPt. Each
% iteration extends one tree once toward a sample, then greedily connects
% the opposite tree toward the new node. Collision checking, obstacle
% inflation, bounds, step size, iteration budget, and seed use the same
% conventions as planRRT2D.
%
% Key opts: bounds, stepSize, goalBias, maxIter,
%           collisionResolution, inflateRadius, seed, maxConnectSteps.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    if ~isempty(opts.seed)
        rng(opts.seed);
    end

    startPt = startPt(:).';
    goalPt = goalPt(:).';
    path = zeros(0, 2);
    info = initInfo();

    if ~isPointInsideBounds(startPt, opts.bounds) || ...
            ~isPointCollisionFree2D( ...
            startPt, obstacles, opts.inflateRadius)
        info.message = 'Start point is outside bounds or in collision.';
        return;
    end
    if ~isPointInsideBounds(goalPt, opts.bounds) || ...
            ~isPointCollisionFree2D( ...
            goalPt, obstacles, opts.inflateRadius)
        info.message = 'Goal point is outside bounds or in collision.';
        return;
    end

    treeA = makeTree(startPt, true);
    treeB = makeTree(goalPt, false);

    for iter = 1:opts.maxIter
        info.numIter = iter;

        if rand < opts.goalBias
            qRand = treeB.nodes(1,:);
        else
            qRand = sampleBounds(opts.bounds);
        end

        [treeA, newIdxA, statusA, nExtend] = ...
            extendTree(treeA, qRand, obstacles, opts);
        info.extendAttemptCount = ...
            info.extendAttemptCount + nExtend;

        if statusA ~= "trapped"
            qTarget = treeA.nodes(newIdxA,:);
            [treeB, newIdxB, statusB, nConnect] = ...
                connectTree(treeB, qTarget, obstacles, opts);
            info.connectCallCount = info.connectCallCount + 1;
            info.connectExtendCount = ...
                info.connectExtendCount + nConnect;

            if statusB == "reached"
                path = mergeTrees(treeA, newIdxA, treeB, newIdxB);
                info.success = true;
                info.numNodesStart = countRootTreeNodes( ...
                    treeA, treeB, true);
                info.numNodesGoal = countRootTreeNodes( ...
                    treeA, treeB, false);
                info.numNodes = info.numNodesStart + info.numNodesGoal;
                info.firstSolutionIter = iter;
                info.firstSolutionNodes = info.numNodes;
                info.pathLength = polylineLength(path);
                info.bestCost = info.pathLength;
                info.message = 'RRT-Connect joined the two trees.';
                if opts.returnTrees
                    info.treeA = treeA;
                    info.treeB = treeB;
                end
                return;
            end
        end

        temp = treeA;
        treeA = treeB;
        treeB = temp;
        info.treeSwapCount = info.treeSwapCount + 1;
    end

    info.numNodesStart = countRootTreeNodes(treeA, treeB, true);
    info.numNodesGoal = countRootTreeNodes(treeA, treeB, false);
    info.numNodes = info.numNodesStart + info.numNodesGoal;
    if opts.returnTrees
        info.treeA = treeA;
        info.treeB = treeB;
    end
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'bounds'); opts.bounds = [0,1; -0.4,0.4]; end
    if ~isfield(opts, 'stepSize'); opts.stepSize = 0.035; end
    if ~isfield(opts, 'goalBias'); opts.goalBias = 0.12; end
    if ~isfield(opts, 'maxIter'); opts.maxIter = 3000; end
    if ~isfield(opts, 'collisionResolution')
        opts.collisionResolution = 0.004;
    end
    if ~isfield(opts, 'inflateRadius'); opts.inflateRadius = 0.0; end
    if ~isfield(opts, 'seed'); opts.seed = []; end
    if ~isfield(opts, 'maxConnectSteps'); opts.maxConnectSteps = inf; end
    if ~isfield(opts, 'duplicateTolerance')
        opts.duplicateTolerance = 1e-12;
    end
    if ~isfield(opts, 'returnTrees'); opts.returnTrees = false; end

    if ~isscalar(opts.stepSize) || ~isfinite(opts.stepSize) || ...
            opts.stepSize <= 0
        error('opts.stepSize must be positive.');
    end
    if ~isscalar(opts.maxIter) || opts.maxIter < 1 || ...
            opts.maxIter ~= round(opts.maxIter)
        error('opts.maxIter must be a positive integer.');
    end
    if ~isscalar(opts.maxConnectSteps) || ...
            isnan(opts.maxConnectSteps) || opts.maxConnectSteps <= 0
        error('opts.maxConnectSteps must be positive or Inf.');
    end
end

function info = initInfo()
    info = struct();
    info.method = 'RRT-Connect';
    info.success = false;
    info.numIter = 0;
    info.numNodes = 2;
    info.numNodesStart = 1;
    info.numNodesGoal = 1;
    info.firstSolutionIter = nan;
    info.firstSolutionNodes = nan;
    info.bestCost = nan;
    info.pathLength = nan;
    info.extendAttemptCount = 0;
    info.connectCallCount = 0;
    info.connectExtendCount = 0;
    info.treeSwapCount = 0;
    info.message = 'RRT-Connect failed to join the two trees.';
    info.treeA = [];
    info.treeB = [];
end

function tree = makeTree(root, rootIsStart)
    tree = struct();
    tree.nodes = root;
    tree.parent = 0;
    tree.rootIsStart = rootIsStart;
end

function [tree, newIdx, status, nAttempt] = ...
        extendTree(tree, qTarget, obstacles, opts)
    nAttempt = 1;
    newIdx = 0;
    status = "trapped";

    [~, nearIdx] = min(sum((tree.nodes - qTarget).^2, 2));
    qNear = tree.nodes(nearIdx,:);
    qNew = steerPoint(qNear, qTarget, opts.stepSize);

    if norm(qNew - qNear) <= opts.duplicateTolerance || ...
            ~isPointInsideBounds(qNew, opts.bounds) || ...
            ~isSegmentCollisionFree2D( ...
            qNear, qNew, obstacles, opts)
        return;
    end

    tree.nodes(end+1,:) = qNew;
    tree.parent(end+1,1) = nearIdx;
    newIdx = size(tree.nodes, 1);
    if norm(qNew - qTarget) <= opts.duplicateTolerance
        status = "reached";
    else
        status = "advanced";
    end
end

function [tree, newIdx, status, nAttempt] = ...
        connectTree(tree, qTarget, obstacles, opts)
    newIdx = 0;
    status = "trapped";
    nAttempt = 0;

    [nearestDistance, nearestIdx] = min(vecnorm( ...
        tree.nodes - qTarget, 2, 2));
    if nearestDistance <= opts.duplicateTolerance
        newIdx = nearestIdx;
        status = "reached";
        return;
    end

    while nAttempt < opts.maxConnectSteps
        [tree, candidateIdx, extendStatus, count] = ...
            extendTree(tree, qTarget, obstacles, opts);
        nAttempt = nAttempt + count;
        if extendStatus == "trapped"
            return;
        end
        newIdx = candidateIdx;
        if extendStatus == "reached"
            status = "reached";
            return;
        end
        status = "advanced";
    end
end

function path = mergeTrees(treeA, idxA, treeB, idxB)
    pathA = backtrackRootToNode(treeA, idxA);
    pathB = backtrackRootToNode(treeB, idxB);

    if treeA.rootIsStart
        pathStart = pathA;
        pathGoal = pathB;
    else
        pathStart = pathB;
        pathGoal = pathA;
    end

    tail = flipud(pathGoal);
    path = [pathStart; tail(2:end,:)];
end

function path = backtrackRootToNode(tree, idx)
    ids = idx;
    while tree.parent(ids(1)) ~= 0
        ids = [tree.parent(ids(1)); ids]; %#ok<AGROW>
    end
    path = tree.nodes(ids,:);
end

function count = countRootTreeNodes(treeA, treeB, rootIsStart)
    if treeA.rootIsStart == rootIsStart
        count = size(treeA.nodes, 1);
    else
        count = size(treeB.nodes, 1);
    end
end

function q = sampleBounds(bounds)
    q = [
        bounds(1,1) + rand * (bounds(1,2) - bounds(1,1)), ...
        bounds(2,1) + rand * (bounds(2,2) - bounds(2,1))
    ];
end

function qNew = steerPoint(qFrom, qTo, stepSize)
    direction = qTo - qFrom;
    distance = norm(direction);
    if distance <= stepSize
        qNew = qTo;
    else
        qNew = qFrom + stepSize * direction / distance;
    end
end

function tf = isPointInsideBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
        q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end

function len = polylineLength(path)
    if size(path,1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end
