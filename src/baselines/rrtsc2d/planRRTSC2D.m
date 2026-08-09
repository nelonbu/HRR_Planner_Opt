function [Paccepted, pathAccepted, info] = planRRTSC2D(startPt, goalPt, obstacles, opts)
%PLANRRTSC2D Global-replanning RRTSC baseline for the common 2-D FTL model.
%
% Pipeline per attempt:
%   RRT -> B-spline smoothing -> centerline validation ->
%   fixed-length chord-sweep validation.
% Any failed validation is rejected for strict acceptance and restarts RRT.
% The best complete rejected candidate is retained only as a fallback
% output when no strict-safe candidate is found within the limits.
% No shortcut, local repair, clearance optimization, or CSSC optimizer is
% used.
%
% RRTSC-2D retains the RRT-smoothing-leading-link validation-global
% replanning mechanism of RRTSC, while adapting the path representation,
% obstacle model, and fixed-link validation to the common 2-D FTL
% framework used in this project.

    if nargin < 4
        opts = struct();
    end
    opts = validateRRTSC2DParams(opts, startPt, goalPt);
    startPt = startPt(:).';
    goalPt = goalPt(:).';

    Paccepted = zeros(0, 2);
    pathAccepted = zeros(0, 2);
    info = initInfo(opts);
    attemptLogs = repmat(emptyAttemptLog(), opts.maxAttempts, 1);
    bestFallback = emptyFallback();
    tPlanning = tic;

    for attempt = 1:opts.maxAttempts
        if toc(tPlanning) >= opts.maxTotalTime
            info.terminationReason = 'maxTotalTime';
            break;
        end

        attemptLog = emptyAttemptLog();
        attemptLog.attempt = attempt;
        attemptLog.seed = deriveAttemptSeed( ...
            opts.seed, attempt, opts.attemptSeedStride);
        info.attemptCount = attempt;

        rrtOpts = opts.rrt;
        rrtOpts.seed = attemptLog.seed;
        tStage = tic;
        [pathRRT, rrtInfo] = planRRT2D( ...
            startPt, goalPt, obstacles, rrtOpts);
        attemptLog.rrtTimeSec = toc(tStage);
        attemptLog.rrtSuccess = rrtInfo.success;
        attemptLog.rrtNumIter = getField(rrtInfo, 'numIter', nan);
        attemptLog.rrtNumNodes = getField(rrtInfo, 'numNodes', nan);
        attemptLog.rrtPathLength = getField(rrtInfo, 'pathLength', nan);
        attemptLog.message = rrtInfo.message;
        info.timing.rrtTimeSec = info.timing.rrtTimeSec + ...
            attemptLog.rrtTimeSec;

        if ~rrtInfo.success
            info.rrtFailureCount = info.rrtFailureCount + 1;
            attemptLog.outcome = 'rrtFailure';
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end

        if toc(tPlanning) >= opts.maxTotalTime
            attemptLog.outcome = 'maxTotalTime';
            attemptLogs(attempt) = attemptLog;
            info.terminationReason = 'maxTotalTime';
            break;
        end

        try
            tStage = tic;
            approxLen = polylineLength(pathRRT);
            nCtrl = max(opts.degree + 1, ...
                ceil(approxLen / (opts.controlPointSpacingFactor * opts.L)) + 1);
            [Pcandidate, splineInfo] = polylineToBSplineInit2D( ...
                pathRRT, struct('degree', opts.degree, 'nCtrl', nCtrl));
            Pcandidate(1,:) = startPt;
            Pcandidate(end,:) = goalPt;
            attemptLog.splineTimeSec = toc(tStage);
            attemptLog.nCtrl = size(Pcandidate, 1);
            info.timing.splineTimeSec = info.timing.splineTimeSec + ...
                attemptLog.splineTimeSec;
        catch ME
            attemptLog.splineTimeSec = toc(tStage);
            info.timing.splineTimeSec = info.timing.splineTimeSec + ...
                attemptLog.splineTimeSec;
            info.numericalFailureCount = info.numericalFailureCount + 1;
            attemptLog.outcome = 'numericalFailure';
            attemptLog.message = ME.message;
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end

        validationOpts = opts;
        validationOpts.knot = splineInfo.knot;
        centerValidation = validateRRTSCCenterline2D( ...
            Pcandidate, obstacles, validationOpts);
        attemptLog.centerlineTimeSec = centerValidation.timeSec;
        attemptLog.centerlineSafe = centerValidation.safe;
        attemptLog.centerlineMinClear = centerValidation.minClear;
        info.timing.centerlineValidationTimeSec = ...
            info.timing.centerlineValidationTimeSec + centerValidation.timeSec;

        if centerValidation.numericalFailure
            info.numericalFailureCount = info.numericalFailureCount + 1;
            attemptLog.outcome = 'numericalFailure';
            attemptLog.message = centerValidation.message;
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end
        centerlineCollisionFree = isfinite(centerValidation.minClear) && ...
            centerValidation.minClear > 0;
        if ~centerlineCollisionFree
            info.centerlineRejectCount = info.centerlineRejectCount + 1;
            attemptLog.outcome = 'centerlineReject';
            attemptLog.message = centerValidation.message;
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end

        if toc(tPlanning) >= opts.maxTotalTime
            bestFallback = updateBestFallback(bestFallback, ...
                Pcandidate, pathRRT, attempt, rrtInfo, splineInfo, ...
                centerValidation, [], opts);
            attemptLog.outcome = 'maxTotalTime';
            attemptLogs(attempt) = attemptLog;
            info.terminationReason = 'maxTotalTime';
            break;
        end

        chordValidation = validateRRTSCChordSweep2D( ...
            Pcandidate, obstacles, validationOpts);
        attemptLog.chordTimeSec = chordValidation.timeSec;
        attemptLog.chordSafe = chordValidation.safe;
        attemptLog.chordMinClear = chordValidation.minClear;
        attemptLog.refinementCount = chordValidation.refinementCount;
        attemptLog.numValidChords = chordValidation.numValidChords;
        info.timing.chordValidationTimeSec = ...
            info.timing.chordValidationTimeSec + chordValidation.timeSec;

        bestFallback = updateBestFallback(bestFallback, ...
            Pcandidate, pathRRT, attempt, rrtInfo, splineInfo, ...
            centerValidation, chordValidation, opts);

        if ~centerValidation.safe
            info.centerlineRejectCount = info.centerlineRejectCount + 1;
            attemptLog.outcome = 'centerlineReject';
            attemptLog.message = ...
                'Centerline is collision-free but below safetyMargin.';
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end

        if chordValidation.numericalFailure
            info.numericalFailureCount = info.numericalFailureCount + 1;
            attemptLog.outcome = 'numericalFailure';
            attemptLog.message = chordValidation.message;
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end
        if ~chordValidation.safe
            info.chordRejectCount = info.chordRejectCount + 1;
            attemptLog.outcome = 'chordReject';
            attemptLog.message = chordValidation.message;
            attemptLogs(attempt) = attemptLog;
            printAttempt(opts, attemptLog);
            continue;
        end

        Paccepted = Pcandidate;
        pathAccepted = pathRRT;
        info.success = true;
        info.outputAvailable = true;
        info.strictAccepted = true;
        info.fallbackReturned = false;
        info.acceptedAttempt = attempt;
        info.returnedAttempt = attempt;
        info.solutionStatus = 'strict-safe';
        info.terminationReason = 'success';
        info.finalRRTInfo = rrtInfo;
        info.finalSplineInfo = splineInfo;
        info.centerlineValidation = centerValidation;
        info.chordValidation = chordValidation;
        attemptLog.outcome = 'accepted';
        attemptLog.message = 'Accepted by both validation stages.';
        attemptLogs(attempt) = attemptLog;
        printAttempt(opts, attemptLog);
        break;
    end

    info.attempts = info.attemptCount;
    info.attemptLog = attemptLogs(1:info.attemptCount);
    info.globalReplanningCount = max(0, info.attemptCount - 1);
    info.planningTimeSec = toc(tPlanning);
    info.timing.planningTimeSec = info.planningTimeSec;
    info.timing.accountedTimeSec = info.timing.rrtTimeSec + ...
        info.timing.splineTimeSec + ...
        info.timing.centerlineValidationTimeSec + ...
        info.timing.chordValidationTimeSec;

    if ~info.success && strcmp(info.terminationReason, 'running')
        info.terminationReason = classifyAttemptLimit(info);
    end
    if ~info.success && bestFallback.available
        Paccepted = bestFallback.P;
        pathAccepted = bestFallback.path;
        info.outputAvailable = true;
        info.fallbackReturned = true;
        info.returnedAttempt = bestFallback.attempt;
        info.finalRRTInfo = bestFallback.rrtInfo;
        info.finalSplineInfo = bestFallback.splineInfo;
        info.centerlineValidation = bestFallback.centerValidation;
        info.chordValidation = bestFallback.chordValidation;
        info.fallbackInfo = bestFallback;
        info.solutionStatus = bestFallback.solutionStatus;
    end
    info.message = terminationMessage(info);
    if info.fallbackReturned
        info.message = sprintf('%s Returned fallback attempt %d (%s).', ...
            info.message, info.returnedAttempt, info.solutionStatus);
    end
end

function info = initInfo(opts)
    info = struct();
    info.method = 'RRTSC-2D';
    info.success = false;
    info.outputAvailable = false;
    info.strictAccepted = false;
    info.fallbackReturned = false;
    info.attempts = 0;
    info.attemptLog = repmat(emptyAttemptLog(), 0, 1);
    info.attemptCount = 0;
    info.rrtFailureCount = 0;
    info.centerlineRejectCount = 0;
    info.chordRejectCount = 0;
    info.numericalFailureCount = 0;
    info.globalReplanningCount = 0;
    info.acceptedAttempt = nan;
    info.returnedAttempt = nan;
    info.solutionStatus = 'no-path';
    info.terminationReason = 'running';
    info.message = '';
    info.planningTimeSec = 0;
    info.finalRRTInfo = [];
    info.finalSplineInfo = [];
    info.centerlineValidation = [];
    info.chordValidation = [];
    info.fallbackInfo = emptyFallback();
    info.paramsUsed = opts;
    info.timing = struct( ...
        'rrtTimeSec', 0, ...
        'splineTimeSec', 0, ...
        'centerlineValidationTimeSec', 0, ...
        'chordValidationTimeSec', 0, ...
        'accountedTimeSec', 0, ...
        'planningTimeSec', 0);
end

function fallback = emptyFallback()
    fallback = struct();
    fallback.available = false;
    fallback.P = zeros(0,2);
    fallback.path = zeros(0,2);
    fallback.attempt = nan;
    fallback.rrtInfo = [];
    fallback.splineInfo = [];
    fallback.centerValidation = [];
    fallback.chordValidation = [];
    fallback.centerMinClear = nan;
    fallback.chordMinClear = nan;
    fallback.numericallyCertified = false;
    fallback.tier = 0;
    fallback.pathLength = inf;
    fallback.solutionStatus = 'no-path';
end

function best = updateBestFallback(best, P, path, attempt, ...
        rrtInfo, splineInfo, centerValidation, chordValidation, opts)
    if isempty(P) || isempty(path) || ...
            ~isfinite(centerValidation.minClear) || ...
            centerValidation.minClear <= 0
        return;
    end

    candidate = emptyFallback();
    candidate.available = true;
    candidate.P = P;
    candidate.path = path;
    candidate.attempt = attempt;
    candidate.rrtInfo = rrtInfo;
    candidate.splineInfo = splineInfo;
    candidate.centerValidation = centerValidation;
    candidate.chordValidation = chordValidation;
    candidate.centerMinClear = centerValidation.minClear;
    candidate.pathLength = polylineLength(path);

    hasCertifiedChord = isstruct(chordValidation) && ...
        isfield(chordValidation, 'numericalFailure') && ...
        ~chordValidation.numericalFailure && ...
        isfield(chordValidation, 'minClear') && ...
        isfinite(chordValidation.minClear);
    if hasCertifiedChord
        candidate.numericallyCertified = true;
        candidate.chordMinClear = chordValidation.minClear;
        if candidate.chordMinClear >= 0
            candidate.tier = 3;
            if candidate.chordMinClear >= opts.safetyMargin && ...
                    centerValidation.safe
                candidate.tier = 4;
                candidate.solutionStatus = 'strict-safe';
            elseif candidate.chordMinClear >= opts.safetyMargin
                candidate.solutionStatus = ...
                    'centerline-below-margin-chord-safe';
            else
                candidate.solutionStatus = ...
                    'collision-free-below-dmin';
            end
        else
            candidate.tier = 2;
            candidate.solutionStatus = 'centerline-only';
        end
    else
        candidate.tier = 1;
        candidate.solutionStatus = 'numerically-uncertified';
    end

    if isBetterFallback(candidate, best)
        best = candidate;
    end
end

function tf = isBetterFallback(candidate, best)
    if ~best.available
        tf = true;
        return;
    end
    if candidate.tier ~= best.tier
        tf = candidate.tier > best.tier;
        return;
    end

    candidateChord = candidate.chordMinClear;
    bestChord = best.chordMinClear;
    if ~isfinite(candidateChord); candidateChord = -inf; end
    if ~isfinite(bestChord); bestChord = -inf; end
    if candidateChord ~= bestChord
        tf = candidateChord > bestChord;
    elseif candidate.centerMinClear ~= best.centerMinClear
        tf = candidate.centerMinClear > best.centerMinClear;
    else
        tf = candidate.pathLength < best.pathLength;
    end
end

function log = emptyAttemptLog()
    log = struct();
    log.attempt = 0;
    log.seed = 0;
    log.outcome = '';
    log.rrtSuccess = false;
    log.rrtTimeSec = 0;
    log.splineTimeSec = 0;
    log.centerlineTimeSec = 0;
    log.chordTimeSec = 0;
    log.rrtNumIter = nan;
    log.rrtNumNodes = nan;
    log.rrtPathLength = nan;
    log.nCtrl = nan;
    log.centerlineSafe = false;
    log.centerlineMinClear = nan;
    log.chordSafe = false;
    log.chordMinClear = nan;
    log.refinementCount = 0;
    log.numValidChords = 0;
    log.message = '';
end

function seed = deriveAttemptSeed(masterSeed, attempt, stride)
    seed = mod(double(masterSeed) - 1 + ...
        double(attempt - 1) * double(stride), 2147483646) + 1;
end

function reason = classifyAttemptLimit(info)
    if info.attemptCount > 0 && ...
            info.rrtFailureCount == info.attemptCount
        reason = 'noRRTPath';
    elseif info.numericalFailureCount > 0 && ...
            info.numericalFailureCount + info.rrtFailureCount == ...
            info.attemptCount
        reason = 'numericalFailure';
    else
        reason = 'maxAttempts';
    end
end

function message = terminationMessage(info)
    switch info.terminationReason
        case 'success'
            message = sprintf('Accepted attempt %d.', info.acceptedAttempt);
        case 'maxTotalTime'
            message = 'Maximum total planning time reached.';
        case 'noRRTPath'
            message = 'All attempts failed to produce an RRT path.';
        case 'numericalFailure'
            message = 'All non-RRT failures were numerical.';
        otherwise
            message = 'Maximum number of attempts reached.';
    end
end

function printAttempt(opts, log)
    if ~opts.verbose
        return;
    end
    fprintf(['RRTSC attempt %3d | seed=%10d | %-18s | ' ...
        'center=%+.4f chord=%+.4f | %.3fs\n'], ...
        log.attempt, log.seed, log.outcome, ...
        log.centerlineMinClear, log.chordMinClear, ...
        log.rrtTimeSec + log.splineTimeSec + ...
        log.centerlineTimeSec + log.chordTimeSec);
end

function len = polylineLength(path)
    if size(path,1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

function value = getField(s, name, defaultValue)
    if isstruct(s) && isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end
