function metrics = evaluateCSSCHighPrecision(P, obstacles, opts)
%EVALUATECSSCHIGHPRECISION Final high-resolution evaluator for CSSC paths.
%
% This function is for final statistics only. It is intentionally separate
% from optimization-time sampling. Demos and experiments should use the
% fields returned here for success rate, minClear, and dMin satisfaction.
%
% Inputs:
%   P         : B-spline control points, nCtrl-by-2
%   obstacles : obstacle struct array, supported types: circle, rect, polygon
%   opts      : L, dMin, degree, knot, envOpts, pathSampleN,
%               pointClearanceResolution
%
% Key outputs:
%   metrics.minClear, metrics.success, metrics.dMinSatisfied
%   metrics.ftlGeometricRealizable, metrics.chordConstructionSuccess
%   metrics.pointMinClear, metrics.pathLength, metrics.turnAbsSum
%   metrics.state, metrics.timing, metrics.paramsUsed

    tAll = tic;

    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts, size(P, 1));

    metrics = emptyMetrics();
    metrics.dMin = opts.dMin;

    if isempty(P) || size(P, 2) ~= 2 || size(P, 1) < opts.degree + 1
        metrics.message = 'Invalid B-spline control points.';
        metrics.evalTimeSec = toc(tAll);
        return;
    end

    paramsEval = opts;
    paramsEval.printEvalTiming = false;
    paramsEval.enablePathSample = true;
    paramsEval.enablePointClearance = false;
    paramsEval.enableObstacleMetadata = true;
    paramsEval.clearanceMode = 'segment';

    state = evaluateCSSCGlobal(P, obstacles, paramsEval);
    metrics.state = state;
    metrics.timing = state.timing;
    metrics.paramsUsed = paramsEval;

    metrics.minClear = state.minClear;
    metrics.minPoint = state.minPoint;
    metrics.minU = state.minU;
    metrics.minIdx = state.minIdx;
    metrics.success = ~isnan(state.minClear) && state.minClear > 0;
    metrics.dMinSatisfied = ~isnan(state.minClear) && state.minClear >= opts.dMin;
    metrics.numValidChords = nnz(state.validChord);
    % Segment mode evaluates every geometrically valid fixed-length chord.
    metrics.numEvaluatedChords = nnz(state.validChord);
    metrics.chordCoverage = metrics.numEvaluatedChords / ...
        max(1, metrics.numValidChords);
    metrics = evaluateFTLGeometry(metrics, state);

    path = state.pathSample;
    if isempty(path)
        w = linspace(opts.envOpts.uRange(1), opts.envOpts.uRange(2), opts.pathSampleN).';
        path = evalBSplinePath2D(P, w, opts.degree, opts.knot);
    end
    metrics.pathSample = path;
    metrics.numPathSamples = size(path, 1);
    metrics.pathLength = polylineLengthLocal(path);
    [metrics.turnAbsSum, metrics.turnSqSum, metrics.meanAbsTurn] = ...
        pathTurnMetricsLocal(path);

    [metrics.pointMinClear, metrics.pointMinPoint] = ...
        sampledPointClearance(path, obstacles, opts.pointClearanceResolution);
    metrics.pointSuccess = isfinite(metrics.pointMinClear) && metrics.pointMinClear > 0;

    metrics.evalTimeSec = toc(tAll);
    metrics.message = 'ok';
end

function metrics = evaluateFTLGeometry(metrics, state)
% A lightweight common feasibility check for ideal-joint FTL motion.
% It verifies that every chord declared geometrically feasible by the
% fixed-chord solver is constructed with finite endpoints and acceptable
% length residual. Joint-angle or actuator limits are intentionally not
% part of this benchmark-level criterion.

    env = state.env;
    nFeasible = getNestedField(env, {'stats','numFeasible'}, 0);
    nValid = nnz(state.validChord);
    residual = getNestedField(env, {'chordResidual'}, nan(size(state.validChord)));
    validResidual = residual(state.validChord);
    acceptTol = getNestedField(env, {'opts','vAcceptTol'}, 5e-4);

    finiteGeometry = all(isfinite(state.M(state.validChord,:)), 'all') && ...
        all(isfinite(state.N(state.validChord,:)), 'all');
    residualOK = ~isempty(validResidual) && ...
        all(isfinite(validResidual)) && ...
        all(abs(validResidual) <= acceptTol);

    metrics.numFeasibleChordSamples = nFeasible;
    metrics.chordConstructionSuccess = nFeasible > 0 && ...
        nValid == nFeasible && finiteGeometry && residualOK;
    metrics.ftlGeometricRealizable = metrics.chordConstructionSuccess;
    metrics.feasibilityNumericalFailure = nFeasible > nValid || ...
        (nValid > 0 && (~finiteGeometry || ~residualOK));
    if isempty(validResidual)
        metrics.maxChordLengthResidual = nan;
    else
        metrics.maxChordLengthResidual = max(abs(validResidual), [], 'omitnan');
    end
end

function value = getNestedField(s, names, defaultValue)
    value = s;
    for i = 1:numel(names)
        if ~isstruct(value) || ~isfield(value, names{i})
            value = defaultValue;
            return;
        end
        value = value.(names{i});
    end
end

function opts = setDefaults(opts, nCtrl)
    if ~isfield(opts, 'L'); opts.L = 0.15; end
    if ~isfield(opts, 'dMin'); opts.dMin = 0.02; end
    if ~isfield(opts, 'degree'); opts.degree = 3; end
    if (~isfield(opts, 'knot') || isempty(opts.knot)) && nCtrl >= opts.degree + 1
        opts.knot = makeClampedUniformKnot(nCtrl, opts.degree);
    end

    if ~isfield(opts, 'envOpts') || isempty(opts.envOpts)
        opts.envOpts = struct();
    end
    if ~isfield(opts.envOpts, 'uRange'); opts.envOpts.uRange = [0, 1]; end
    if ~isfield(opts.envOpts, 'vSearchRange'); opts.envOpts.vSearchRange = [0, 1]; end
    if ~isfield(opts.envOpts, 'nU'); opts.envOpts.nU = 480; end
    if ~isfield(opts.envOpts, 'epsV'); opts.envOpts.epsV = 1e-6; end
    if ~isfield(opts.envOpts, 'tolDen'); opts.envOpts.tolDen = 1e-6; end

    if ~isfield(opts, 'pathSampleN'); opts.pathSampleN = 1200; end
    if ~isfield(opts, 'pointClearanceResolution')
        opts.pointClearanceResolution = 0.001;
    end
end

function metrics = emptyMetrics()
    metrics = struct();
    metrics.success = false;
    metrics.dMinSatisfied = false;
    metrics.pointSuccess = false;
    metrics.dMin = nan;
    metrics.minClear = nan;
    metrics.minPoint = [nan, nan];
    metrics.minU = nan;
    metrics.minIdx = nan;
    metrics.pointMinClear = nan;
    metrics.pointMinPoint = [nan, nan];
    metrics.pathLength = nan;
    metrics.turnAbsSum = nan;
    metrics.turnSqSum = nan;
    metrics.meanAbsTurn = nan;
    metrics.numPathSamples = 0;
    metrics.numValidChords = 0;
    metrics.numEvaluatedChords = 0;
    metrics.chordCoverage = nan;
    metrics.numFeasibleChordSamples = 0;
    metrics.chordConstructionSuccess = false;
    metrics.ftlGeometricRealizable = false;
    metrics.feasibilityNumericalFailure = false;
    metrics.maxChordLengthResidual = nan;
    metrics.pathSample = zeros(0, 2);
    metrics.state = [];
    metrics.timing = [];
    metrics.paramsUsed = [];
    metrics.evalTimeSec = nan;
    metrics.message = '';
end

function [minClear, minPoint] = sampledPointClearance(path, obstacles, resolution)
    samples = samplePolylineLocal(path, resolution);
    minClear = inf;
    minPoint = [nan, nan];

    for i = 1:size(samples, 1)
        d = queryObstaclePointSDF2D(samples(i,:), obstacles);
        if d < minClear
            minClear = d;
            minPoint = samples(i,:);
        end
    end

    if isempty(samples)
        minClear = nan;
    end
end

function samples = samplePolylineLocal(path, resolution)
    samples = zeros(0, 2);
    if size(path, 1) < 2
        samples = path;
        return;
    end

    for i = 1:size(path, 1)-1
        a = path(i,:);
        b = path(i+1,:);
        len = norm(b - a);
        n = max(2, ceil(len / resolution) + 1);
        t = linspace(0, 1, n).';
        seg = (1 - t) .* a + t .* b;
        if i > 1
            seg = seg(2:end,:);
        end
        samples = [samples; seg]; %#ok<AGROW>
    end
end

function len = polylineLengthLocal(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

function [turnAbsSum, turnSqSum, meanAbsTurn] = pathTurnMetricsLocal(path)
    if size(path, 1) < 3
        turnAbsSum = 0;
        turnSqSum = 0;
        meanAbsTurn = 0;
        return;
    end

    v1 = diff(path(1:end-1,:), 1, 1);
    v2 = diff(path(2:end,:), 1, 1);
    crossVal = v1(:,1).*v2(:,2) - v1(:,2).*v2(:,1);
    dotVal = sum(v1 .* v2, 2);
    ang = atan2(crossVal, dotVal);

    turnAbsSum = sum(abs(ang));
    turnSqSum = sum(ang.^2);
    meanAbsTurn = mean(abs(ang));
end
