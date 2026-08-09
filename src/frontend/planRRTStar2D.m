function [path, info] = planRRTStar2D(startPt, goalPt, obstacles, opts)
%PLANRRTSTAR2D Build a collision-free point path with RRT* rewiring.
%
% The collision model matches planRRT2D: point-robot SDF with obstacle
% inflation and sampled segment collision checks.
%
% Key opts: bounds, stepSize, goalBias, goalTol, maxIter, maxTimeSec,
%           collisionResolution, inflateRadius, seed,
%           rewireRadius, minRewireRadius, rewireGamma,
%           terminateOnFirstSolution, maxNoImproveIter.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);
    tPlanning = tic;

    if ~isempty(opts.seed)
        rng(opts.seed);
    end

    startPt = startPt(:).';
    goalPt = goalPt(:).';

    nodes = startPt;
    parent = 0;
    cost = 0;

    bestGoalParent = nan;
    bestGoalCost = inf;
    firstSolutionIter = nan;
    firstSolutionNodes = nan;
    lastImproveIter = 0;
    timeToFirstSolutionSec = nan;
    timedOut = false;

    info = initInfo();

    if ~isPointCollisionFree2D(startPt, obstacles, opts.inflateRadius)
        info.message = 'Start point is in collision.';
        info.terminationReason = 'invalidStart';
        info.planningTimeSec = toc(tPlanning);
        path = zeros(0, 2);
        return;
    end
    if ~isPointCollisionFree2D(goalPt, obstacles, opts.inflateRadius)
        info.message = 'Goal point is in collision.';
        info.terminationReason = 'invalidGoal';
        info.planningTimeSec = toc(tPlanning);
        path = zeros(0, 2);
        return;
    end

    for iter = 1:opts.maxIter
        if toc(tPlanning) >= opts.maxTimeSec
            timedOut = true;
            break;
        end
        info.numIter = iter;

        if rand < opts.goalBias
            qRand = goalPt;
        else
            qRand = sampleBounds(opts.bounds);
        end

        [~, iNear] = min(sum((nodes - qRand).^2, 2));
        qNear = nodes(iNear,:);
        qNew = steerPoint(qNear, qRand, opts.stepSize);

        if ~isPointInsideBounds(qNew, opts.bounds)
            continue;
        end

        if ~isSegmentCollisionFree2D(qNear, qNew, obstacles, opts)
            continue;
        end

        nOld = size(nodes, 1);
        radius = computeRewireRadius(opts, nOld);
        nearIdx = find(sum((nodes - qNew).^2, 2) <= radius^2);
        if isempty(nearIdx)
            nearIdx = iNear;
        end

        [bestParent, bestCost] = chooseBestParent( ...
            nodes, cost, nearIdx, iNear, qNew, obstacles, opts);

        nodes(end+1,:) = qNew; %#ok<AGROW>
        parent(end+1,1) = bestParent; %#ok<AGROW>
        cost(end+1,1) = bestCost; %#ok<AGROW>
        newIdx = size(nodes, 1);

        ancestorMask = makeAncestorMask(parent, bestParent, nOld);
        [parent, cost] = rewireNearNodes( ...
            nodes, parent, cost, nearIdx, newIdx, ancestorMask, obstacles, opts);

        [bestGoalParent, bestGoalCost, improvedGoal] = tryUpdateGoalConnection( ...
            nodes, cost, newIdx, goalPt, bestGoalParent, bestGoalCost, obstacles, opts);

        if improvedGoal
            lastImproveIter = iter;
            if isnan(firstSolutionIter)
                firstSolutionIter = iter;
                firstSolutionNodes = size(nodes, 1);
                timeToFirstSolutionSec = toc(tPlanning);
            end

            if opts.terminateOnFirstSolution
                break;
            end
        end

        if isfinite(bestGoalCost) && isfinite(opts.maxNoImproveIter) && ...
           opts.maxNoImproveIter > 0 && iter - lastImproveIter >= opts.maxNoImproveIter
            break;
        end
    end

    info.numNodes = size(nodes, 1);
    info.firstSolutionIter = firstSolutionIter;
    info.firstSolutionNodes = firstSolutionNodes;
    info.bestGoalParent = bestGoalParent;
    info.bestCost = bestGoalCost;
    info.planningTimeSec = toc(tPlanning);
    info.timeToFirstSolutionSec = timeToFirstSolutionSec;

    if isfinite(bestGoalCost)
        path = backtrackPathToGoal(nodes, parent, bestGoalParent, goalPt);
        info.success = true;
        info.pathLength = polylineLength(path);
        info.message = 'RRT* connected to goal.';
        if timedOut
            info.terminationReason = 'maxTimeSec-bestAvailable';
        else
            info.terminationReason = 'success';
        end
    else
        path = zeros(0, 2);
        if timedOut
            info.message = 'RRT* reached maxTimeSec without a solution.';
            info.terminationReason = 'maxTimeSec';
        else
            info.message = 'RRT* failed to connect to goal.';
            info.terminationReason = 'maxIter';
        end
    end

    if opts.returnTree
        info.nodes = nodes;
        info.parent = parent;
        info.cost = cost;
    end
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'bounds'); opts.bounds = [0, 1; -0.4, 0.4]; end
    if ~isfield(opts, 'stepSize'); opts.stepSize = 0.035; end
    if ~isfield(opts, 'goalBias'); opts.goalBias = 0.12; end
    if ~isfield(opts, 'goalTol'); opts.goalTol = 0.04; end
    if ~isfield(opts, 'maxIter'); opts.maxIter = 3000; end
    if ~isfield(opts, 'maxTimeSec'); opts.maxTimeSec = inf; end
    if ~isfield(opts, 'collisionResolution'); opts.collisionResolution = 0.004; end
    if ~isfield(opts, 'inflateRadius'); opts.inflateRadius = 0.0; end
    if ~isfield(opts, 'seed'); opts.seed = []; end
    if ~isfield(opts, 'rewireRadius'); opts.rewireRadius = 0.12; end
    if ~isfield(opts, 'minRewireRadius'); opts.minRewireRadius = 2.0 * opts.stepSize; end
    if ~isfield(opts, 'rewireGamma'); opts.rewireGamma = 0.45; end
    if ~isfield(opts, 'rewireCostTol'); opts.rewireCostTol = 1e-9; end
    if ~isfield(opts, 'terminateOnFirstSolution'); opts.terminateOnFirstSolution = false; end
    if ~isfield(opts, 'maxNoImproveIter'); opts.maxNoImproveIter = inf; end
    if ~isfield(opts, 'returnTree'); opts.returnTree = false; end
end

function info = initInfo()
    info = struct();
    info.success = false;
    info.numIter = 0;
    info.numNodes = 1;
    info.message = 'RRT* failed to connect to goal.';
    info.goalNode = nan;
    info.bestGoalParent = nan;
    info.firstSolutionIter = nan;
    info.firstSolutionNodes = nan;
    info.bestCost = nan;
    info.pathLength = nan;
    info.terminationReason = 'maxIter';
    info.planningTimeSec = nan;
    info.timeToFirstSolutionSec = nan;
end

function q = sampleBounds(bounds)
    q = [
        bounds(1,1) + rand * (bounds(1,2) - bounds(1,1)), ...
        bounds(2,1) + rand * (bounds(2,2) - bounds(2,1))
    ];
end

function qNew = steerPoint(qFrom, qTo, stepSize)
    v = qTo - qFrom;
    d = norm(v);
    if d <= stepSize
        qNew = qTo;
    else
        qNew = qFrom + stepSize * v / d;
    end
end

function tf = isPointInsideBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
         q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end

function radius = computeRewireRadius(opts, nNode)
    if nNode <= 1
        radius = opts.rewireRadius;
        return;
    end

    asymptoticRadius = opts.rewireGamma * sqrt(log(nNode + 1) / (nNode + 1));
    radius = min(opts.rewireRadius, max(opts.minRewireRadius, asymptoticRadius));
end

function [bestParent, bestCost] = chooseBestParent(nodes, cost, nearIdx, iNear, qNew, obstacles, opts)
    bestParent = iNear;
    bestCost = cost(iNear) + norm(qNew - nodes(iNear,:));

    for k = 1:numel(nearIdx)
        i = nearIdx(k);
        edgeCost = norm(qNew - nodes(i,:));
        candidateCost = cost(i) + edgeCost;

        if candidateCost + opts.rewireCostTol >= bestCost
            continue;
        end

        if isSegmentCollisionFree2D(nodes(i,:), qNew, obstacles, opts)
            bestParent = i;
            bestCost = candidateCost;
        end
    end
end

function ancestorMask = makeAncestorMask(parent, idx, nOld)
    ancestorMask = false(nOld, 1);
    while idx > 0 && idx <= nOld
        ancestorMask(idx) = true;
        idx = parent(idx);
    end
end

function [parent, cost] = rewireNearNodes(nodes, parent, cost, nearIdx, newIdx, ancestorMask, obstacles, opts)
    qNew = nodes(newIdx,:);

    for k = 1:numel(nearIdx)
        i = nearIdx(k);
        if i == parent(newIdx) || ancestorMask(i)
            continue;
        end

        edgeCost = norm(nodes(i,:) - qNew);
        candidateCost = cost(newIdx) + edgeCost;

        if candidateCost + opts.rewireCostTol >= cost(i)
            continue;
        end

        if isSegmentCollisionFree2D(qNew, nodes(i,:), obstacles, opts)
            parent(i) = newIdx;
            cost(i) = candidateCost;
            cost = refreshDescendantCosts(nodes, parent, cost, i);
        end
    end
end

function cost = refreshDescendantCosts(nodes, parent, cost, rootIdx)
    queue = rootIdx;
    head = 1;

    while head <= numel(queue)
        i = queue(head);
        head = head + 1;

        children = find(parent == i);
        for k = 1:numel(children)
            child = children(k);
            cost(child) = cost(i) + norm(nodes(child,:) - nodes(i,:));
        end
        queue = [queue; children(:)]; %#ok<AGROW>
    end
end

function [bestGoalParent, bestGoalCost, improvedGoal] = tryUpdateGoalConnection( ...
    nodes, cost, newIdx, goalPt, bestGoalParent, bestGoalCost, obstacles, opts)

    improvedGoal = false;
    qNew = nodes(newIdx,:);
    if norm(qNew - goalPt) > opts.goalTol
        return;
    end

    edgeCost = norm(goalPt - qNew);
    candidateCost = cost(newIdx) + edgeCost;
    if candidateCost + opts.rewireCostTol >= bestGoalCost
        return;
    end

    if isSegmentCollisionFree2D(qNew, goalPt, obstacles, opts)
        bestGoalParent = newIdx;
        bestGoalCost = candidateCost;
        improvedGoal = true;
    end
end

function path = backtrackPathToGoal(nodes, parent, goalParent, goalPt)
    path = backtrackPath(nodes, parent, goalParent);
    path = [path; goalPt];
end

function path = backtrackPath(nodes, parent, idx)
    ids = idx;
    while parent(ids(1)) ~= 0
        ids = [parent(ids(1)); ids]; %#ok<AGROW>
    end
    path = nodes(ids,:);
end

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end
