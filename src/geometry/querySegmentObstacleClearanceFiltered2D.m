function [out, stats] = querySegmentObstacleClearanceFiltered2D(M, N, obstacles, bounds, opts)
%QUERYSEGMENTOBSTACLECLEARANCEFILTERED2D Segment clearance with broad phase.
%
% Obstacles are ordered by the distance lower bound from the segment to each
% obstacle bounding circle. Exact obstacle checks stop once the next lower
% bound is larger than the current best exact clearance.

    if nargin < 4 || isempty(bounds)
        bounds = computeObstacleBoundingCircles2D(obstacles);
    end
    if nargin < 5 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    nObs = numel(obstacles);
    stats = emptyStats(nObs);

    if nObs == 0
        out = emptyOutput();
        return;
    end

    lowerBound = segmentToBoundingCircleLowerBound(M, N, bounds);
    [sortedLB, order] = sort(lowerBound, 'ascend');
    stats.minLowerBound = sortedLB(1);

    minCandidates = min(max(1, round(opts.minCandidates)), nObs);
    bestClearance = inf;
    out = emptyOutput();

    for ii = 1:nObs
        if stats.numChecked >= minCandidates ...
                && sortedLB(ii) > bestClearance + opts.stopTol
            break;
        end

        k = order(ii);
        outK = querySegmentObstacleClearance2D(M, N, obstacles(k), false);
        outK.obsId = k;
        stats.numChecked = stats.numChecked + 1;

        if outK.clearance < bestClearance
            bestClearance = outK.clearance;
            out = outK;
        end
    end

    stats.numSkipped = nObs - stats.numChecked;
    stats.checkFraction = stats.numChecked / max(1, nObs);
end

function lowerBound = segmentToBoundingCircleLowerBound(M, N, bounds)
    M = M(:).';
    N = N(:).';
    e = N - M;
    len2 = dot(e, e);
    C = bounds.centers;

    if len2 < 1e-14
        closest = repmat(M, size(C, 1), 1);
    else
        t = ((C - M) * e.') / len2;
        t = max(0, min(1, t));
        closest = M + t .* e;
    end

    lowerBound = vecnorm(C - closest, 2, 2) - bounds.radii;
    lowerBound(~isfinite(lowerBound)) = -inf;
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'minCandidates') || isempty(opts.minCandidates)
        opts.minCandidates = 1;
    end
    if ~isfield(opts, 'stopTol') || isempty(opts.stopTol)
        opts.stopTol = 1e-12;
    end
end

function out = emptyOutput()
    out = struct();
    out.clearance = inf;
    out.closestPoint = [nan, nan];
    out.alpha = nan;
    out.normal = [1, 0];
    out.obsId = nan;
    out.obsType = '';
    out.rawDistance = inf;
end

function stats = emptyStats(nObs)
    stats = struct();
    stats.numObstacles = nObs;
    stats.numChecked = 0;
    stats.numSkipped = nObs;
    stats.checkFraction = 0;
    stats.minLowerBound = nan;
end
