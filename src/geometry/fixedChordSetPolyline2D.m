function env = fixedChordSetPolyline2D(path, L, opts)
%FIXEDCHORDSETPOLYLINE2D Construct first-forward fixed chords on a polyline.
%
% Each chord endpoint is found analytically by intersecting successive
% polyline segments with the circle centered at M and radius L. The first
% root in path order is selected. This avoids applying a derivative-based
% Newton solver at degree-1 B-spline knots, where the tangent is
% discontinuous.

    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    path = removeConsecutiveDuplicates(path, opts.pointTolerance);
    nU = opts.nU;
    env = emptyEnvironment(nU, opts);

    if size(path, 1) < 2 || ~isfinite(L) || L <= 0
        env.stats.message = 'Invalid polyline or chord length.';
        return;
    end

    segment = diff(path, 1, 1);
    segmentLength = vecnorm(segment, 2, 2);
    cumulativeLength = [0; cumsum(segmentLength)];
    totalLength = cumulativeLength(end);
    if ~isfinite(totalLength) || totalLength <= opts.pointTolerance
        env.stats.message = 'Polyline length is zero.';
        return;
    end

    sList = linspace(0, totalLength, nU).';
    uList = sList / totalLength;
    MList = nan(nU, 2);
    NList = nan(nU, 2);
    vList = nan(nU, 1);
    residual = nan(nU, 1);
    validChord = false(nU, 1);
    rootSegmentIndex = nan(nU, 1);
    rootSegmentAlpha = nan(nU, 1);

    for i = 1:nU
        [M, startSegment, startAlpha] = pointAtArcLength( ...
            path, segmentLength, cumulativeLength, sList(i), ...
            opts.pointTolerance);
        MList(i,:) = M;

        [found, N, sN, segmentIndex, segmentAlpha] = findFirstRoot( ...
            path, segmentLength, cumulativeLength, M, sList(i), ...
            startSegment, startAlpha, L, opts);
        if ~found
            continue;
        end

        NList(i,:) = N;
        vList(i) = sN / totalLength;
        residual(i) = norm(N - M) - L;
        validChord(i) = all(isfinite(N)) && isfinite(vList(i)) && ...
            vList(i) > uList(i) && ...
            abs(residual(i)) <= opts.rootAcceptTolerance;
        rootSegmentIndex(i) = segmentIndex;
        rootSegmentAlpha(i) = segmentAlpha;
    end

    env.u = uList;
    env.v = vList;
    env.vChord = vList;
    env.M = MList;
    env.N = NList;
    env.NChord = NList;
    env.G = nan(nU, 2);
    env.lambda = nan(nU, 1);
    env.vp = nan(nU, 1);
    env.validChord = validChord;
    env.validLine = false(nU, 1);
    env.validSegment = false(nU, 1);
    env.residual = residual;
    env.chordResidual = residual;
    env.s = sList;
    env.sChord = vList * totalLength;
    env.rootSegmentIndex = rootSegmentIndex;
    env.rootSegmentAlpha = rootSegmentAlpha;
    env.polyline = path;
    env.totalPathLength = totalLength;

    env.stats.numFeasible = nnz(validChord);
    env.stats.numValidChord = nnz(validChord);
    env.stats.numNewtonSuccess = 0;
    env.stats.numFallbackUsed = 0;
    env.stats.avgNewtonIters = 0;
    env.stats.numNumericalFailure = 0;
    env.stats.solver = 'polyline-segment-quadratic-first-forward-root';
    env.stats.message = 'ok';
end

function [found, N, sN, rootSegment, rootAlpha] = findFirstRoot( ...
        path, segmentLength, cumulativeLength, M, sM, ...
        startSegment, startAlpha, L, opts)

    found = false;
    N = [nan, nan];
    sN = nan;
    rootSegment = nan;
    rootAlpha = nan;
    nSegment = numel(segmentLength);

    for k = startSegment:nSegment
        A = path(k,:);
        D = path(k+1,:) - A;
        a = dot(D, D);
        if a <= opts.pointTolerance^2
            continue;
        end

        C = A - M;
        b = 2 * dot(C, D);
        c = dot(C, C) - L^2;
        discriminant = b^2 - 4*a*c;
        scale = max([b^2, abs(4*a*c), L^4, 1]);
        discTol = opts.discriminantTolerance * scale;
        if discriminant < -discTol
            continue;
        end
        discriminant = max(discriminant, 0);

        roots = sort([(-b - sqrt(discriminant)) / (2*a), ...
                      (-b + sqrt(discriminant)) / (2*a)]);
        lowerAlpha = 0;
        if k == startSegment
            lowerAlpha = startAlpha;
        end

        for j = 1:numel(roots)
            alpha = roots(j);
            if alpha < lowerAlpha - opts.rootParameterTolerance || ...
                    alpha > 1 + opts.rootParameterTolerance
                continue;
            end
            alpha = min(max(alpha, lowerAlpha), 1);
            candidateS = cumulativeLength(k) + alpha * segmentLength(k);
            if candidateS <= sM + opts.forwardArcTolerance
                continue;
            end

            candidate = A + alpha * D;
            if abs(norm(candidate - M) - L) > opts.rootAcceptTolerance
                continue;
            end

            found = true;
            N = candidate;
            sN = candidateS;
            rootSegment = k;
            rootAlpha = alpha;
            return;
        end
    end
end

function [point, segmentIndex, alpha] = pointAtArcLength( ...
        path, segmentLength, cumulativeLength, s, tol)

    nSegment = numel(segmentLength);
    segmentIndex = find(cumulativeLength <= s + tol, 1, 'last');
    segmentIndex = min(max(segmentIndex, 1), nSegment);
    alpha = (s - cumulativeLength(segmentIndex)) / ...
        segmentLength(segmentIndex);
    alpha = min(max(alpha, 0), 1);
    point = path(segmentIndex,:) + alpha * ...
        (path(segmentIndex+1,:) - path(segmentIndex,:));
end

function path = removeConsecutiveDuplicates(path, tol)
    if isempty(path)
        path = zeros(0, 2);
        return;
    end
    if size(path, 2) ~= 2 || any(~isfinite(path), 'all')
        path = zeros(0, 2);
        return;
    end
    keep = [true; vecnorm(diff(path, 1, 1), 2, 2) > tol];
    path = path(keep,:);
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'nU'); opts.nU = 480; end
    if ~isfield(opts, 'pointTolerance'); opts.pointTolerance = 1e-12; end
    if ~isfield(opts, 'forwardArcTolerance'); opts.forwardArcTolerance = 1e-12; end
    if ~isfield(opts, 'rootParameterTolerance'); opts.rootParameterTolerance = 1e-10; end
    if ~isfield(opts, 'discriminantTolerance'); opts.discriminantTolerance = 1e-13; end
    if ~isfield(opts, 'rootAcceptTolerance'); opts.rootAcceptTolerance = 1e-10; end
    opts.nU = max(2, round(opts.nU));
end

function env = emptyEnvironment(nU, opts)
    env = struct();
    env.u = linspace(0, 1, nU).';
    env.v = nan(nU, 1);
    env.vChord = nan(nU, 1);
    env.M = nan(nU, 2);
    env.N = nan(nU, 2);
    env.NChord = nan(nU, 2);
    env.G = nan(nU, 2);
    env.lambda = nan(nU, 1);
    env.vp = nan(nU, 1);
    env.validChord = false(nU, 1);
    env.validLine = false(nU, 1);
    env.validSegment = false(nU, 1);
    env.residual = nan(nU, 1);
    env.chordResidual = nan(nU, 1);
    env.s = nan(nU, 1);
    env.sChord = nan(nU, 1);
    env.rootSegmentIndex = nan(nU, 1);
    env.rootSegmentAlpha = nan(nU, 1);
    env.polyline = zeros(0, 2);
    env.totalPathLength = nan;
    env.opts = opts;
    env.opts.vAcceptTol = opts.rootAcceptTolerance;
    env.stats = struct( ...
        'numFeasible', 0, 'numValidChord', 0, ...
        'numNewtonSuccess', 0, 'numFallbackUsed', 0, ...
        'avgNewtonIters', 0, 'numNumericalFailure', 0, ...
        'solver', 'polyline-segment-quadratic-first-forward-root', ...
        'message', 'not evaluated');
end
