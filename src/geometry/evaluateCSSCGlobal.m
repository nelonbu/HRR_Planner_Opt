function state = evaluateCSSCGlobal(P, obstacles, params)
%EVALUATECSSCGLOBAL Evaluate CSSC swept-segment clearance globally.
%
% This evaluator computes fixed-length chords M=r(u), N=r(v(u)) and then
% evaluates clearance according to params.clearanceMode:
%   'segment'  : exact segment-obstacle clearance for each valid chord.
%   'envelope' : fast G/M/N point-SDF probe, approximate.
%   'hybrid'   : G/M/N point-SDF probe plus selective segment refinement.
% It also keeps optional point clearances for visualization/diagnostics.
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

    %% 3. Clearance backend
    t = tic;

    clearanceOut = queryChordClearanceSet2D(env, obstacles, params);
    validLineMask = clearanceOut.validLineMask;
    validSegmentMask = clearanceOut.validSegmentMask;
    idxValidLine = clearanceOut.idxValidLine;
    idxValidSegment = clearanceOut.idxValidSegment;

    clearance = clearanceOut.clearance;
    clearanceSegment = clearanceOut.clearanceSegment;
    clearanceProbe = clearanceOut.clearanceProbe;
    clearanceG = clearanceOut.clearanceG;
    clearanceM = clearanceOut.clearanceM;
    clearanceN = clearanceOut.clearanceN;
    closestPoint = clearanceOut.closestPoint;
    closestAlpha = clearanceOut.closestAlpha;
    closestNormal = clearanceOut.closestNormal;
    clearanceSource = clearanceOut.clearanceSource;
    probeSource = clearanceOut.probeSource;
    isExactSegment = clearanceOut.isExactSegment;
    nearestObsId = clearanceOut.nearestObsId;
    nearestObsType = clearanceOut.nearestObsType;

    timeClearanceBackend = toc(t);
    timeAllocate = max(0, clearanceOut.timing.total ...
        - clearanceOut.timing.segmentClearance ...
        - clearanceOut.timing.pointMN ...
        - clearanceOut.timing.pointG);
    timeSegmentClearance = clearanceOut.timing.segmentClearance;
    timePointMN = clearanceOut.timing.pointMN;
    timePointG = clearanceOut.timing.pointG;

    %% 4. Minimum clearance extraction
    t = tic;

    [minClear, minIdx] = min(clearance, [], 'omitnan');

    if isempty(minClear) || isnan(minClear)
        minClear = nan;
        minIdx = nan;
        minU = nan;
        minPoint = [nan, nan];
        minType = 'none';
    else
        minU = env.u(minIdx);
        minPoint = closestPoint(minIdx,:);
        minType = char(clearanceSource(minIdx));
    end

    timeMin = toc(t);

    %% 5. Path sampling for visualization
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

    %% 6. Pack output state
    t = tic;

    state = struct();
    state.P = P;
    state.knot = knot;
    state.env = env;
    state.clearanceMode = params.clearanceMode;

    state.u = env.u;
    state.v = env.v;
    state.M = env.M;
    state.N = env.N;
    state.G = env.G;
    state.lambda = env.lambda;
    state.validLine = env.validLine;
    state.validSegment = env.validSegment;

    state.clearance = clearance;
    state.clearanceSegment = clearanceSegment;
    state.clearanceProbe = clearanceProbe;
    state.clearanceG = clearanceG;
    state.clearanceM = clearanceM;
    state.clearanceN = clearanceN;
    state.clearanceSource = clearanceSource;
    state.probeSource = probeSource;
    state.isExactSegment = isExactSegment;

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

    %% 7. Timing summary
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
    timing.clearanceBackend = timeClearanceBackend;
    timing.clearanceOther = timeAllocate;
    timing.minExtraction = timeMin;
    timing.pathSample = timePathSample;
    timing.packState = timePack;

    timing.numU = n;
    timing.numValidLine = numel(idxValidLine);
    timing.numValidSegment = numel(idxValidSegment);
    timing.numExactSegment = clearanceOut.timing.numExactSegment;
    timing.exactFraction = clearanceOut.timing.exactFraction;
    timing.numProbe = clearanceOut.timing.numProbe;
    timing.clearanceMode = params.clearanceMode;
    timing.hybridTriggerThreshold = clearanceOut.timing.hybridTriggerThreshold;
    timing.obstacleFilter = clearanceOut.timing.obstacleFilter;
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
        params.printEvalTiming = false;
    end

    % Path sample can be disabled during optimization for speed.
    if ~isfield(params, 'enablePathSample')
        params.enablePathSample = true;
    end

    % Point clearance for M/N/G is diagnostic/visualization-only.
    if ~isfield(params, 'enablePointClearance')
        params.enablePointClearance = true;
    end

    % Clearance backend. Segment is the conservative default used by the
    % proposed optimizer and high-precision evaluator.
    if ~isfield(params, 'clearanceMode') || isempty(params.clearanceMode)
        params.clearanceMode = 'segment';
    end
    if ~isfield(params, 'hybrid') || isempty(params.hybrid)
        params.hybrid = struct();
    end
    if ~isfield(params.hybrid, 'triggerFactor')
        params.hybrid.triggerFactor = 2.0;
    end
    if ~isfield(params.hybrid, 'refineActiveTopK')
        params.hybrid.refineActiveTopK = true;
    end
    if ~isfield(params.hybrid, 'forceExactStride')
        params.hybrid.forceExactStride = 0;
    end

    % Optional bounding-circle broad-phase obstacle filtering.
    if ~isfield(params, 'obstacleFilter') || isempty(params.obstacleFilter)
        params.obstacleFilter = struct();
    end
    if ~isfield(params.obstacleFilter, 'enable')
        params.obstacleFilter.enable = false;
    end
    if ~isfield(params.obstacleFilter, 'useForSegment')
        params.obstacleFilter.useForSegment = true;
    end
    if ~isfield(params.obstacleFilter, 'useForPoint')
        params.obstacleFilter.useForPoint = true;
    end
    if ~isfield(params.obstacleFilter, 'minCandidates')
        params.obstacleFilter.minCandidates = 1;
    end
    if ~isfield(params.obstacleFilter, 'stopTol')
        params.obstacleFilter.stopTol = 1e-12;
    end

    % Nearest obstacle id/type is diagnostic metadata, not needed by the optimizer.
    if ~isfield(params, 'enableObstacleMetadata')
        params.enableObstacleMetadata = true;
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
    fprintf('  clearance mode           : %s\n', char(timing.clearanceMode));
    fprintf('  total                    : %8.3f ms\n', 1000 * timing.total);

    fprintf('  default params           : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.defaultParams, 100 * timing.defaultParams / total);

    fprintf('  setup knot/functions     : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.setup, 100 * timing.setup / total);

    fprintf('  fixedChordEnvelope       : %8.3f ms  (%5.1f%%)\n', ...
        1000 * timing.envelope, 100 * timing.envelope / total);

    fprintf('  clearance overhead       : %8.3f ms  (%5.1f%%)\n', ...
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

    fprintf('  exact segment queries    : %d / %d  (%.1f%%)\n', ...
        timing.numExactSegment, timing.numValidLine, 100 * timing.exactFraction);

    if isfield(timing, 'obstacleFilter') && timing.obstacleFilter.totalObstaclePossible > 0
        ofs = timing.obstacleFilter;
        fprintf('  obstacle checks          : %d / %d  (%.1f%%)\n', ...
            ofs.totalObstacleChecks, ofs.totalObstaclePossible, ...
            100 * ofs.checkFraction);
    end

    if strcmpi(timing.clearanceMode, 'hybrid')
        fprintf('  hybrid trigger threshold : %.6g\n', timing.hybridTriggerThreshold);
    end

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
