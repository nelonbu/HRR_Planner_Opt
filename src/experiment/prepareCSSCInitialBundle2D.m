function bundle = prepareCSSCInitialBundle2D( ...
        envInfo, obstacles, plannerOpts, paramsOpt, initOpts)
%PREPARECSSCINITIALBUNDLE2D Generate and cache paired CSSC initial paths.
%
% The selected candidate reproduces the current CSSC policy: maximize exact
% fixed-chord segment clearance and break ties by shorter polyline length.
% The first valid candidate is also retained for the single-init ablation.

    initOpts = setDefaults(initOpts);
    bundle = emptyBundle();
    tAll = tic;
    candidates = repmat(emptyCandidate(), initOpts.maxCandidates, 1);
    nAttempt = 0;
    nValid = 0;
    selectedIndex = 0;

    for attempt = 1:initOpts.maxCandidates
        if toc(tAll) >= initOpts.maxTimeSec
            break;
        end
        nAttempt = attempt;
        candidateSeed = initOpts.plannerSeed + ...
            (attempt - 1) * initOpts.seedStride;
        opts = plannerOpts;
        opts.method = initOpts.frontendMethod;
        opts.seed = candidateSeed;
        opts.maxTimeSec = max(0, initOpts.maxTimeSec - toc(tAll));

        tFront = tic;
        [pathRRT, frontendInfo] = planCSSCFrontend2D( ...
            envInfo.startPt, envInfo.goalPt, obstacles, opts);
        frontTime = toc(tFront);

        candidate = emptyCandidate();
        candidate.attempt = attempt;
        candidate.seed = candidateSeed;
        candidate.frontendSuccess = frontendInfo.success;
        candidate.frontendInfo = frontendInfo;
        candidate.frontendTimeSec = frontTime;
        candidate.pathRRT = pathRRT;
        candidate.cumulativeTimeSec = toc(tAll);

        if ~frontendInfo.success
            candidates(attempt) = candidate;
            continue;
        end

        tInit = tic;
        shortcutOpts = plannerOpts;
        shortcutOpts.seed = initOpts.shortcutSeedBase + candidateSeed;
        shortcutOpts.numShortcut = initOpts.numShortcut;
        [pathShort, shortcutInfo] = shortcutPath2D( ...
            pathRRT, obstacles, shortcutOpts);
        pathLength = polylineLengthLocal(pathShort);
        nCtrl = max(paramsOpt.degree + 1, ...
            ceil(pathLength / (0.5 * paramsOpt.L)) + 1);
        [Pinit, splineInfo] = polylineToBSplineInit2D( ...
            pathShort, struct('degree', paramsOpt.degree, 'nCtrl', nCtrl));

        scoreParams = paramsOpt;
        scoreParams.knot = splineInfo.knot;
        scoreParams.clearanceMode = 'segment';
        scoreParams.enablePathSample = false;
        scoreParams.enablePointClearance = false;
        scoreParams.enableObstacleMetadata = false;
        state = evaluateCSSCGlobal(Pinit, obstacles, scoreParams);

        candidate.valid = isfinite(state.minClear);
        candidate.pathShortcut = pathShort;
        candidate.shortcutInfo = shortcutInfo;
        candidate.Pinit = Pinit;
        candidate.splineInfo = splineInfo;
        candidate.minClear = state.minClear;
        candidate.pathLength = pathLength;
        candidate.initTimeSec = toc(tInit);
        candidate.cumulativeTimeSec = toc(tAll);
        candidate.safe = candidate.valid && state.minClear >= ...
            paramsOpt.dMin + initOpts.acceptMargin;
        candidates(attempt) = candidate;

        if ~candidate.valid
            continue;
        end
        nValid = nValid + 1;
        if bundle.firstValidIndex == 0
            bundle.firstValidIndex = attempt;
        end
        if selectedIndex == 0 || ...
                candidate.minClear > candidates(selectedIndex).minClear || ...
                (candidate.minClear == candidates(selectedIndex).minClear && ...
                candidate.pathLength < candidates(selectedIndex).pathLength)
            selectedIndex = attempt;
        end
        if candidates(selectedIndex).safe
            break;
        end
    end

    bundle.elapsedTimeSec = toc(tAll);
    bundle.numAttempts = nAttempt;
    bundle.numValidCandidates = nValid;
    bundle.candidates = candidates(1:nAttempt);
    if nAttempt > 0
        bundle.firstAttempt = candidates(1);
    end
    bundle.selectedIndex = selectedIndex;
    bundle.success = selectedIndex > 0;
    if bundle.success
        bundle.selected = candidates(selectedIndex);
        bundle.firstValid = candidates(bundle.firstValidIndex);
        bundle.terminationReason = 'candidateSelected';
        bundle.message = sprintf( ...
            'Selected candidate %d from %d attempts.', ...
            selectedIndex, nAttempt);
    elseif toc(tAll) >= initOpts.maxTimeSec
        bundle.terminationReason = 'maxInitializationTime';
        bundle.message = 'Initialization reached its time budget.';
    else
        bundle.terminationReason = 'noValidInitialCandidate';
        bundle.message = 'Initialization produced no valid candidate.';
    end
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'maxCandidates'); opts.maxCandidates = 6; end
    if ~isfield(opts, 'maxTimeSec'); opts.maxTimeSec = inf; end
    if ~isfield(opts, 'plannerSeed'); opts.plannerSeed = 1; end
    if ~isfield(opts, 'seedStride'); opts.seedStride = 10000; end
    if ~isfield(opts, 'shortcutSeedBase'); opts.shortcutSeedBase = 710000; end
    if ~isfield(opts, 'numShortcut'); opts.numShortcut = 180; end
    if ~isfield(opts, 'acceptMargin'); opts.acceptMargin = 0; end
    if ~isfield(opts, 'frontendMethod'); opts.frontendMethod = 'rrt'; end
end

function bundle = emptyBundle()
    bundle = struct('success', false, 'numAttempts', 0, ...
        'numValidCandidates', 0, 'selectedIndex', 0, ...
        'firstValidIndex', 0, 'elapsedTimeSec', 0, ...
        'terminationReason', '', 'message', '', ...
        'selected', emptyCandidate(), 'firstAttempt', emptyCandidate(), ...
        'firstValid', emptyCandidate(), ...
        'candidates', repmat(emptyCandidate(), 0, 1));
end

function candidate = emptyCandidate()
    candidate = struct('attempt', 0, 'seed', 0, ...
        'valid', false, 'safe', false, 'frontendSuccess', false, ...
        'minClear', nan, 'pathLength', nan, 'frontendTimeSec', 0, ...
        'initTimeSec', 0, 'cumulativeTimeSec', 0, ...
        'pathRRT', zeros(0,2), ...
        'pathShortcut', zeros(0,2), 'Pinit', zeros(0,2), ...
        'frontendInfo', struct(), 'shortcutInfo', struct(), ...
        'splineInfo', struct());
end

function value = polylineLengthLocal(path)
    if size(path,1) < 2
        value = 0;
    else
        value = sum(vecnorm(diff(path,1,1), 2, 2));
    end
end
