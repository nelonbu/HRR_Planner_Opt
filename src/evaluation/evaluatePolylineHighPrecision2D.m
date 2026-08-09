function metrics = evaluatePolylineHighPrecision2D(path, obstacles, opts)
%EVALUATEPOLYLINEHIGHPRECISION2D Evaluate native polyline FTL geometry.
%
% Fixed-length chord endpoints are solved exactly, segment by segment, from
% the first forward intersection between the polyline and a radius-L circle.
% This avoids the derivative discontinuity and root-selection ambiguity that
% arise when a polyline is passed through the smooth-curve Newton solver as a
% degree-1 B-spline. No smoothing is introduced.

    tAll = tic;
    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);
    metrics = emptyMetrics(opts.dMin);

    pathOriginal = path;
    path = removeConsecutiveDuplicates(path, 1e-12);
    metrics.polyline = pathOriginal;
    metrics.representation = 'piecewise-linear-exact-chord';
    if size(path, 1) < 2
        metrics.message = 'Invalid polyline.';
        metrics.evalTimeSec = toc(tAll);
        return;
    end

    tChord = tic;
    env = fixedChordSetPolyline2D(path, opts.L, opts.envOpts);
    chordConstructionTime = toc(tChord);

    paramsQuery = opts;
    paramsQuery.clearanceMode = 'segment';
    paramsQuery.enablePointClearance = false;
    paramsQuery.enableObstacleMetadata = true;
    paramsQuery.printEvalTiming = false;
    clearanceOut = queryChordClearanceSet2D(env, obstacles, paramsQuery);

    state = packState(path, env, clearanceOut, opts, ...
        chordConstructionTime, numel(obstacles));
    metrics.state = state;
    metrics.timing = state.timing;
    metrics.paramsUsed = state.paramsUsed;
    metrics.minClear = state.minClear;
    metrics.minPoint = state.minPoint;
    metrics.minU = state.minU;
    metrics.minIdx = state.minIdx;
    metrics.success = ~isnan(state.minClear) && state.minClear > 0;
    metrics.dMinSatisfied = ~isnan(state.minClear) && ...
        state.minClear >= opts.dMin;

    validChord = state.validChord;
    residual = env.chordResidual(validChord);
    finiteGeometry = all(isfinite(state.M(validChord,:)), 'all') && ...
        all(isfinite(state.N(validChord,:)), 'all');
    residualOK = ~isempty(residual) && all(isfinite(residual)) && ...
        all(abs(residual) <= opts.envOpts.rootAcceptTolerance);
    metrics.numValidChords = nnz(validChord);
    metrics.numEvaluatedChords = nnz(state.isExactSegment);
    metrics.numFeasibleChordSamples = nnz(validChord);
    metrics.chordCoverage = metrics.numEvaluatedChords / ...
        max(1, metrics.numValidChords);
    metrics.chordConstructionSuccess = metrics.numValidChords > 0 && ...
        metrics.numEvaluatedChords == metrics.numValidChords && ...
        finiteGeometry && residualOK;
    metrics.ftlGeometricRealizable = metrics.chordConstructionSuccess;
    metrics.feasibilityNumericalFailure = metrics.numValidChords > 0 && ...
        (~finiteGeometry || ~residualOK || ...
        metrics.numEvaluatedChords ~= metrics.numValidChords);
    if isempty(residual)
        metrics.maxChordLengthResidual = nan;
    else
        metrics.maxChordLengthResidual = max(abs(residual), [], 'omitnan');
    end

    metrics.pathSample = sampleByArcLength(path, opts.pathSampleN);
    metrics.numPathSamples = size(metrics.pathSample, 1);
    metrics.pathLength = sum(vecnorm(diff(path, 1, 1), 2, 2));
    [metrics.turnAbsSum, metrics.turnSqSum, metrics.meanAbsTurn] = ...
        pathTurnMetrics(path);
    [metrics.pointMinClear, metrics.pointMinPoint] = ...
        sampledPointClearance(path, obstacles, opts.pointClearanceResolution);
    metrics.pointSuccess = isfinite(metrics.pointMinClear) && ...
        metrics.pointMinClear > 0;
    metrics.evalTimeSec = toc(tAll);
    metrics.message = 'ok';
end

function state = packState(path, env, out, opts, chordTime, numObstacles)
    clearance = out.clearance;
    [minClear, minIdx] = min(clearance, [], 'omitnan');
    if isempty(minClear) || isnan(minClear)
        minClear = nan;
        minIdx = nan;
        minU = nan;
        minPoint = [nan, nan];
        minType = 'none';
    else
        minU = env.u(minIdx);
        minPoint = out.closestPoint(minIdx,:);
        minType = char(out.clearanceSource(minIdx));
    end

    state = struct();
    state.P = path;
    state.knot = [];
    state.env = env;
    state.clearanceMode = 'segment';
    state.u = env.u;
    state.v = out.vUsed;
    state.M = out.MUsed;
    state.N = out.NUsed;
    state.G = env.G;
    state.lambda = env.lambda;
    state.validLine = out.validLineMask;
    state.validSegment = out.validSegmentMask;
    state.validChord = env.validChord;
    state.validEnvelopeLine = env.validLine;
    state.validEnvelopeSegment = env.validSegment;
    state.clearanceValiditySource = out.validitySource;
    state.clearance = out.clearance;
    state.clearanceSegment = out.clearanceSegment;
    state.clearanceProbe = out.clearanceProbe;
    state.clearanceG = out.clearanceG;
    state.clearanceM = out.clearanceM;
    state.clearanceN = out.clearanceN;
    state.clearanceSource = out.clearanceSource;
    state.probeSource = out.probeSource;
    state.isExactSegment = out.isExactSegment;
    state.closestPoint = out.closestPoint;
    state.closestAlpha = out.closestAlpha;
    state.closestNormal = out.closestNormal;
    state.nearestObsId = out.nearestObsId;
    state.nearestObsType = out.nearestObsType;
    state.minClear = minClear;
    state.minIdx = minIdx;
    state.minU = minU;
    state.minPoint = minPoint;
    state.minType = minType;
    state.pathW = linspace(0, 1, opts.pathSampleN).';
    state.pathSample = sampleByArcLength(path, opts.pathSampleN);
    state.pathDeriv = [];
    state.paramsUsed = opts;
    state.paramsUsed.degree = 1;
    state.paramsUsed.knot = [];
    state.paramsUsed.polylineChordSolver = ...
        'piecewise-quadratic-first-forward-root';

    timing = struct();
    timing.total = chordTime + out.timing.total;
    timing.defaultParams = 0;
    timing.setup = 0;
    timing.envelope = chordTime;
    timing.allocate = max(0, out.timing.total - ...
        out.timing.segmentClearance - out.timing.pointMN - out.timing.pointG);
    timing.segmentClearance = out.timing.segmentClearance;
    timing.pointMN = out.timing.pointMN;
    timing.pointG = out.timing.pointG;
    timing.clearanceBackend = out.timing.total;
    timing.clearanceOther = timing.allocate;
    timing.minExtraction = 0;
    timing.pathSample = 0;
    timing.packState = 0;
    timing.numU = numel(env.u);
    timing.numValidLine = nnz(out.validLineMask);
    timing.numValidSegment = nnz(out.validSegmentMask);
    timing.numExactSegment = out.timing.numExactSegment;
    timing.exactFraction = out.timing.exactFraction;
    timing.numProbe = out.timing.numProbe;
    timing.clearanceMode = 'segment';
    timing.hybridTriggerThreshold = nan;
    timing.obstacleFilter = out.timing.obstacleFilter;
    timing.numObstacles = numObstacles;
    timing.hasEnvelopeStats = true;
    timing.envelopeStats = env.stats;
    timing.chordSolver = env.stats.solver;
    state.timing = timing;
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'L'); opts.L = 0.15; end
    if ~isfield(opts, 'dMin'); opts.dMin = 0.02; end
    if ~isfield(opts, 'envOpts') || isempty(opts.envOpts)
        opts.envOpts = struct();
    end
    if ~isfield(opts.envOpts, 'nU'); opts.envOpts.nU = 480; end
    if ~isfield(opts.envOpts, 'rootAcceptTolerance')
        opts.envOpts.rootAcceptTolerance = 1e-10;
    end
    if ~isfield(opts, 'pathSampleN'); opts.pathSampleN = 1200; end
    if ~isfield(opts, 'pointClearanceResolution')
        opts.pointClearanceResolution = 0.001;
    end
end

function metrics = emptyMetrics(dMin)
    metrics = struct( ...
        'success', false, 'dMinSatisfied', false, 'pointSuccess', false, ...
        'dMin', dMin, 'minClear', nan, 'minPoint', [nan, nan], ...
        'minU', nan, 'minIdx', nan, 'pointMinClear', nan, ...
        'pointMinPoint', [nan, nan], 'pathLength', nan, ...
        'turnAbsSum', nan, 'turnSqSum', nan, 'meanAbsTurn', nan, ...
        'numPathSamples', 0, 'numValidChords', 0, ...
        'numEvaluatedChords', 0, 'chordCoverage', nan, ...
        'numFeasibleChordSamples', 0, ...
        'chordConstructionSuccess', false, ...
        'ftlGeometricRealizable', false, ...
        'feasibilityNumericalFailure', false, ...
        'maxChordLengthResidual', nan, 'pathSample', zeros(0,2), ...
        'state', [], 'timing', [], 'paramsUsed', [], ...
        'evalTimeSec', nan, 'message', '', ...
        'representation', 'piecewise-linear-exact-chord', ...
        'polyline', zeros(0,2));
end

function path = removeConsecutiveDuplicates(path, tol)
    if isempty(path) || size(path, 2) ~= 2 || any(~isfinite(path), 'all')
        path = zeros(0, 2);
        return;
    end
    keep = [true; vecnorm(diff(path, 1, 1), 2, 2) > tol];
    path = path(keep,:);
end

function samples = sampleByArcLength(path, n)
    segmentLength = vecnorm(diff(path, 1, 1), 2, 2);
    cumulative = [0; cumsum(segmentLength)];
    total = cumulative(end);
    if total <= 0
        samples = path(1,:);
        return;
    end
    sList = linspace(0, total, max(2, round(n))).';
    samples = nan(numel(sList), 2);
    segmentIndex = 1;
    for i = 1:numel(sList)
        while segmentIndex < numel(segmentLength) && ...
                sList(i) > cumulative(segmentIndex + 1)
            segmentIndex = segmentIndex + 1;
        end
        alpha = (sList(i) - cumulative(segmentIndex)) / ...
            segmentLength(segmentIndex);
        alpha = min(max(alpha, 0), 1);
        samples(i,:) = path(segmentIndex,:) + alpha * ...
            (path(segmentIndex+1,:) - path(segmentIndex,:));
    end
end

function [minClear, minPoint] = sampledPointClearance(path, obstacles, resolution)
    total = sum(vecnorm(diff(path, 1, 1), 2, 2));
    samples = sampleByArcLength(path, max(2, ceil(total / resolution) + 1));
    minClear = inf;
    minPoint = [nan, nan];
    for i = 1:size(samples, 1)
        d = queryObstaclePointSDF2D(samples(i,:), obstacles);
        if d < minClear
            minClear = d;
            minPoint = samples(i,:);
        end
    end
end

function [turnAbsSum, turnSqSum, meanAbsTurn] = pathTurnMetrics(path)
    if size(path, 1) < 3
        turnAbsSum = 0;
        turnSqSum = 0;
        meanAbsTurn = 0;
        return;
    end
    v1 = diff(path(1:end-1,:), 1, 1);
    v2 = diff(path(2:end,:), 1, 1);
    angle = atan2(v1(:,1).*v2(:,2) - v1(:,2).*v2(:,1), ...
        sum(v1 .* v2, 2));
    turnAbsSum = sum(abs(angle));
    turnSqSum = sum(angle.^2);
    meanAbsTurn = mean(abs(angle));
end
