function [path, info] = planRRT2D(startPt, goalPt, obstacles, opts)
%PLANRRT2D Build a simple collision-free point path with RRT.
%
% The collision model is point-robot SDF with obstacle inflation. This is
% intended as an initial-path generator before CSSC envelope optimization.
%
% Key opts: bounds, stepSize, goalBias, goalTol, maxIter,
%           collisionResolution, inflateRadius, seed.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    if ~isempty(opts.seed)
        rng(opts.seed);
    end

    startPt = startPt(:).';
    goalPt = goalPt(:).';

    nodes = startPt;
    parent = 0;

    info = struct();
    info.success = false;
    info.numIter = 0;
    info.numNodes = 1;
    info.message = 'RRT failed to connect to goal.';
    info.goalNode = nan;
    info.bestCost = nan;
    info.firstSolutionIter = nan;
    info.firstSolutionNodes = nan;
    info.pathLength = nan;

    if ~isPointCollisionFree2D(startPt, obstacles, opts.inflateRadius)
        info.message = 'Start point is in collision.';
        path = zeros(0, 2);
        return;
    end
    if ~isPointCollisionFree2D(goalPt, obstacles, opts.inflateRadius)
        info.message = 'Goal point is in collision.';
        path = zeros(0, 2);
        return;
    end

    for iter = 1:opts.maxIter
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

        nodes(end+1,:) = qNew; %#ok<AGROW>
        parent(end+1,1) = iNear; %#ok<AGROW>
        newIdx = size(nodes, 1);

        if norm(qNew - goalPt) <= opts.goalTol && ...
           isSegmentCollisionFree2D(qNew, goalPt, obstacles, opts)
            nodes(end+1,:) = goalPt; %#ok<AGROW>
            parent(end+1,1) = newIdx; %#ok<AGROW>
            goalIdx = size(nodes, 1);

            path = backtrackPath(nodes, parent, goalIdx);
            info.success = true;
            info.numNodes = size(nodes, 1);
            info.goalNode = goalIdx;
            info.pathLength = polylineLength(path);
            info.bestCost = info.pathLength;
            info.firstSolutionIter = iter;
            info.firstSolutionNodes = info.numNodes;
            info.message = 'RRT connected to goal.';
            return;
        end
    end

    info.numNodes = size(nodes, 1);
    path = zeros(0, 2);
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'bounds'); opts.bounds = [0, 1; -0.4, 0.4]; end
    if ~isfield(opts, 'stepSize'); opts.stepSize = 0.035; end
    if ~isfield(opts, 'goalBias'); opts.goalBias = 0.12; end
    if ~isfield(opts, 'goalTol'); opts.goalTol = 0.04; end
    if ~isfield(opts, 'maxIter'); opts.maxIter = 3000; end
    if ~isfield(opts, 'collisionResolution'); opts.collisionResolution = 0.004; end
    if ~isfield(opts, 'inflateRadius'); opts.inflateRadius = 0.0; end
    if ~isfield(opts, 'seed'); opts.seed = []; end
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
