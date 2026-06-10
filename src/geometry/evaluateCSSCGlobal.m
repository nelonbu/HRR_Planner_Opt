function state = evaluateCSSCGlobal(P, obstacles, params)
%EVALUATECSSCGLOBAL Evaluate CSSC swept-segment clearance globally.
%
% This evaluator computes fixed-length chords M=r(u), N=r(v(u)) and then
% queries segment-obstacle clearance for each valid chord. It also computes
% optional point clearances for G/M/N for visualization.
%
% Timing:
%   Set params.printEvalTiming = true / false to control printing.
%   Timing results are saved in state.timing.

    tTotal = tic;

    %% 0. Defaults
    t = tic;
    params = setDefaultParams(params);
    timeDefault = toc(t);

    %% 1. Knot and function handles
    t = tic;
    nCtrl = size(P,1);
    degree = params.degree;

    if isfield(params, 'knot') && ~isempty(params.knot)
        knot = params.knot;
    else
        knot = makeClampedUniformKnot(nCtrl, degree);
    end

    rFun  = @(w) evalBSplinePath2D(P, w, degree, knot);
    drFun = @(w) evalBSplineDeriv(P, w, degree, knot);
    timeSetup = toc(t);

    %% 2. Fixed chord envelope
    t = tic;
    env = fixedChordEnvelope(rFun, drFun, params.L, params.envOpts);
    timeEnvelope = toc(t);

    n = numel(env.u);

    %% 3. Allocate arrays
    t = tic;

    clearanceSegment = nan(n,1);
    closestPoint = nan(n,2);
    closestAlpha = nan(n,1);
    closestNormal = nan(n,2);
    nearestObsId = nan(n,1);
    nearestObsType = strings(n,1);

    clearanceG = nan(n,1);
    clearanceM = nan(n,1);
    clearanceN = nan(n,1);

    validLineMask = env.validLine ...
        & all(isfinite(env.M), 2) ...
        & all(isfinite(env.N), 2);

    validSegmentMask = env.validSegment ...
        & all(isfinite(env.G), 2);

    idxValidLine = find(validLineMask);
    idxValidSegment = find(validSegmentMask);

    timeAllocate = toc(t);

    %% 4. Segment-obstacle clearance
    t = tic;

    for kk = 1:numel(idxValidLine)
        i = idxValidLine(kk);

        out = querySegmentObstacleClearance2D( ...
            env.M(i,:), env.N(i,:), obstacles);

        clearanceSegment(i) = out.clearance;
        closestPoint(i,:) = out.closestPoint;
        closestAlpha(i) = out.alpha;
        closestNormal(i,:) = out.normal;
        nearestObsId(i) = out.obsId;
        nearestObsType(i) = string(out.obsType);
    end

    timeSegmentClearance = toc(t);

    %% 5. M/N endpoint point-SDF clearance
    t = tic;

    for kk = 1:numel(idxValidLine)
        i = idxValidLine(kk);

        [dM, ~] = queryObstaclePointSDF2D(env.M(i,:), obstacles);
        [dN, ~] = queryObstaclePointSDF2D(env.N(i,:), obstacles);

        clearanceM(i) = dM;
        clearanceN(i) = dN;
    end

    timePointMN = toc(t);

    %% 6. G envelope point-SDF clearance
    t = tic;

    for kk = 1:numel(idxValidSegment)
        i = idxValidSegment(kk);

        [dG, ~] = queryObstaclePointSDF2D(env.G(i,:), obstacles);
        clearanceG(i) = dG;
    end

    timePointG = toc(t);

    %% 7. Minimum clearance extraction
    t = tic;

    [minClear, minIdx] = min(clearanceSegment, [], 'omitnan');

    if isempty(minClear) || isnan(minClear)
        minClear = nan;
        minIdx = nan;
        minU = nan;
        minPoint = [nan, nan];
        minType = 'none';
    else
        minU = env.u(minIdx);
        minPoint = closestPoint(minIdx,:);
        minType = 'segment';
    end

    timeMin = toc(t);

    %% 8. Path sampling for visualization
    t = tic;

    if params.enablePathSample
        wPlot = linspace(params.envOpts.uRange(1), params.envOpts.uRange(2), params.pathSampleN).';
        [pathSample, pathDeriv] = evalBSplinePath2D(P, wPlot, degree, knot);
    else
        wPlot = [];
        pathSample = [];
        pathDeriv = [];
    end

    timePathSample = toc(t);

    %% 9. Pack output state
    t = tic;

    state = struct();
    state.P = P;
    state.knot = knot;
    state.env = env;

    state.u = env.u;
    state.v = env.v;
    state.M = env.M;
    state.N = env.N;
    state.G = env.G;
    state.lambda = env.lambda;
    state.validLine = env.validLine;
    state.validSegment = env.validSegment;

    state.clearance = clearanceSegment;
    state.clearanceSegment = clearanceSegment;
    state.clearanceG = clearanceG;
    state.clearanceM = clearanceM;
    state.clearanceN = clearanceN;

    state.closestPoint = closestPoint;
    state.closestAlpha = closestAlpha;
    state.closestNormal = closestNormal;
    state.nearestObsId = nearestObsId;
    state.nearestObsType = nearestObsType;

    state.minClear = minClear;
    state.minIdx = minIdx;
    state.minU = minU;
    state.minPoint = minPoint;
    state.minType = minType;

    state.pathW = wPlot;
    state.pathSample = pathSample;
    state.pathDeriv = pathDeriv;
    state.paramsUsed = params;

    timePack = toc(t);

    %% 10. Timing summary
    timeTotal = toc(tTotal);

    timing = struct();
    timing.total = timeTotal;
    timing.defaultParams = timeDefault;
    timing.setup = timeSetup;
    timing.envelope = timeEnvelope;
    timing.allocate = timeAllocate;
    timing.segmentClearance = timeSegmentClearance;
    timing.pointMN = timePointMN;
    timing.pointG = timePointG;
    timing.minExtraction = timeMin;
    timing.pathSample = timePathSample;
    timing.packState = timePack;

    timing.numU = n;
    timing.numValidLine = numel(idxValidLine);
    timing.numValidSegment = numel(idxValidSegment);
    timing.numObstacles = numel(obstacles);

    timing.hasEnvelopeStats = isfield(env, 'stats');

    if isfield(env, 'stats')
        timing.envelopeStats = env.stats;
    end

    state.timing = timing;

    if params.printEvalTiming
        printEvaluateTiming(timing);
    end
end

function params = setDefaultParams(params)
    if ~isfield(params, 'degree'); params.degree = 3; end

    if ~isfield(params, 'envOpts') || isempty(params.envOpts)
        params.envOpts = struct();
    end

    if ~isfield(params.envOpts, 'uRange'); params.envOpts.uRange = [0,1]; end
    if ~isfield(params.envOpts, 'vSearchRange'); params.envOpts.vSearchRange = [0,1]; end
    if ~isfield(params.envOpts, 'nU'); params.envOpts.nU = 140; end
    if ~isfield(params.envOpts, 'nVGrid'); params.envOpts.nVGrid = 180; end
    if ~isfield(params.envOpts, 'epsV'); params.envOpts.epsV = 1e-8; end
    if ~isfield(params.envOpts, 'tolDen'); params.envOpts.tolDen = 1e-10; end
    if ~isfield(params.envOpts, 'lambdaTol'); params.envOpts.lambdaTol = 1e-9; end

    % For new warm-start fixedChordEnvelope.
    if ~isfield(params.envOpts, 'maxNewtonIter'); params.envOpts.maxNewtonIter = 6; end
    if ~isfield(params.envOpts, 'vResidualTol'); params.envOpts.vResidualTol = 1e-5; end
    if ~isfield(params.envOpts, 'vAcceptTol'); params.envOpts.vAcceptTol = 5e-4; end
    if ~isfield(params.envOpts, 'fallbackNGrid'); params.envOpts.fallbackNGrid = 30; end
    if ~isfield(params.envOpts, 'enableFallback'); params.envOpts.enableFallback = true; end

    % Timing print switch.
    if ~isfield(params, 'printEvalTiming')
        params.printEvalTiming = true;
    end

    % Path sample can be disabled during optimization for speed.
    if ~isfield(params, 'enablePathSample')
        params.enablePathSample = true;
    end

    if ~isfield(params, 'pathSampleN')
        params.pathSampleN = 600;
    end
end

function dpos = evalBSplineDeriv(P, w, degree, knot)
    [~, dpos] = evalBSplinePath2D(P, w, degree, knot);
end

function printEvaluateTiming(timing)
    total = max(timing.total, eps);

    fprintf('\n[EVALUATECSSCGLOBAL TIMING]\n');
    fprintf('  total                    : %8.3f ms\n', 1000 * timing.total);

    fprintf('  default params           : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.defaultParams, 100 * timing.defaultParams / total);

    fprintf('  setup knot/functions     : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.setup, 100 * timing.setup / total);

    fprintf('  fixedChordEnvelope       : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.envelope, 100 * timing.envelope / total);

    fprintf('  allocate arrays          : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.allocate, 100 * timing.allocate / total);

    fprintf('  segment clearance        : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.segmentClearance, 100 * timing.segmentClearance / total);

    fprintf('  point SDF M/N            : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.pointMN, 100 * timing.pointMN / total);

    fprintf('  point SDF G              : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.pointG, 100 * timing.pointG / total);

    fprintf('  min extraction           : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.minExtraction, 100 * timing.minExtraction / total);

    fprintf('  path sample              : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.pathSample, 100 * timing.pathSample / total);

    fprintf('  pack state               : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.packState, 100 * timing.packState / total);

    fprintf('  validLine                : %d / %d\n', ...
        timing.numValidLine, timing.numU);

    fprintf('  validSegment             : %d / %d\n', ...
        timing.numValidSegment, timing.numU);

    fprintf('  obstacles                : %d\n', timing.numObstacles);

    if timing.hasEnvelopeStats
        es = timing.envelopeStats;
        fprintf('  envelope Newton success  : %d\n', es.numNewtonSuccess);
        fprintf('  envelope fallback used   : %d\n', es.numFallbackUsed);
        fprintf('  envelope avg Newton iter : %.3f\n', es.avgNewtonIters);
    else
        fprintf('  envelope stats           : not found, maybe old fixedChordEnvelope is used\n');
    end
end
