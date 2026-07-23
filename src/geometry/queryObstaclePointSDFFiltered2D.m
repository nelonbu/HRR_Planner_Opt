function [d, grad, info, stats] = queryObstaclePointSDFFiltered2D(q, obstacles, bounds, opts)
%QUERYOBSTACLEPOINTSDFFILTERED2D Point SDF with bounding-circle broad phase.
%
% Obstacles are ordered by the point-to-bounding-circle lower bound. Exact
% SDF checks stop once the next lower bound is larger than the current best
% exact signed distance.

    if nargin < 3 || isempty(bounds)
        bounds = computeObstacleBoundingCircles2D(obstacles);
    end
    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    nObs = numel(obstacles);
    stats = emptyStats(nObs);

    if nObs == 0
        d = inf;
        grad = [1, 0];
        info = struct('id', nan, 'type', '', 'rawDistance', inf);
        return;
    end

    q = q(:).';
    lowerBound = vecnorm(bounds.centers - q, 2, 2) - bounds.radii;
    lowerBound(~isfinite(lowerBound)) = -inf;
    [sortedLB, order] = sort(lowerBound, 'ascend');
    stats.minLowerBound = sortedLB(1);

    minCandidates = min(max(1, round(opts.minCandidates)), nObs);
    d = inf;
    grad = [1, 0];
    info = struct('id', nan, 'type', '', 'rawDistance', inf);

    for ii = 1:nObs
        if stats.numChecked >= minCandidates ...
                && sortedLB(ii) > d + opts.stopTol
            break;
        end

        k = order(ii);
        [dk, gk, infoK] = queryObstaclePointSDF2D(q, obstacles(k));
        stats.numChecked = stats.numChecked + 1;

        if dk < d
            d = dk;
            grad = gk;
            info = infoK;
            info.id = k;
        end
    end

    stats.numSkipped = nObs - stats.numChecked;
    stats.checkFraction = stats.numChecked / max(1, nObs);
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'minCandidates') || isempty(opts.minCandidates)
        opts.minCandidates = 1;
    end
    if ~isfield(opts, 'stopTol') || isempty(opts.stopTol)
        opts.stopTol = 1e-12;
    end
end

function stats = emptyStats(nObs)
    stats = struct();
    stats.numObstacles = nObs;
    stats.numChecked = 0;
    stats.numSkipped = nObs;
    stats.checkFraction = 0;
    stats.minLowerBound = nan;
end
