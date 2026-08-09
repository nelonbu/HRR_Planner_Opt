function validation = validateRRTSCChordSweep2D(P, obstacles, opts)
%VALIDATERRTSCCHORDSWEEP2D Binary fixed-length swept-link safety check.
%
% The function uses fixedChordEnvelope only to construct exact length-L
% chords, then calls the common segment-clearance backend. It does not call
% an objective, gradient, local repair, or optimizer.

    tAll = tic;
    validation = emptyValidation();

    if isfield(opts, 'knot') && ~isempty(opts.knot)
        knot = opts.knot;
    else
        knot = makeClampedUniformKnot(size(P,1), opts.degree);
    end

    rFun = @(u) evalBSplinePath2D(P, u, opts.degree, knot);
    drFun = @(u) evalBSplineDerivativeLocal(P, u, opts.degree, knot);

    for refinement = 0:opts.chord.maxRefinement
        envOpts = opts.chord.envOpts;
        envOpts.nU = round(opts.chord.nU * ...
            opts.chord.refinementFactor^refinement);

        env = fixedChordEnvelope(rFun, drFun, opts.L, envOpts);
        [numericallyValid, message] = chordConstructionValid(env, opts);
        if ~numericallyValid
            validation.refinementCount = refinement;
            validation.numSamples = envOpts.nU;
            validation.numFeasible = getStat(env, 'numFeasible');
            validation.numValidChords = getStat(env, 'numValidChord');
            validation.message = message;
            continue;
        end

        envQuery = chordOnlyEnvironment(env);
        paramsQuery = struct();
        paramsQuery.clearanceMode = 'segment';
        paramsQuery.dMin = opts.safetyMargin;
        paramsQuery.enablePointClearance = false;
        paramsQuery.enableObstacleMetadata = false;
        paramsQuery.obstacleFilter = struct( ...
            'enable', logical(opts.chord.useObstacleFilter), ...
            'useForSegment', true, ...
            'useForPoint', false);

        clearanceOut = queryChordClearanceSet2D( ...
            envQuery, obstacles, paramsQuery);
        c = clearanceOut.clearance(envQuery.validLine);
        if isempty(c) || any(isnan(c))
            validation.refinementCount = refinement;
            validation.numSamples = envOpts.nU;
            validation.numFeasible = getStat(env, 'numFeasible');
            validation.numValidChords = nnz(envQuery.validLine);
            validation.message = 'Non-finite chord clearance result.';
            continue;
        end

        [minClear, localIdx] = min(c);
        validIdx = find(envQuery.validLine);
        minIdx = validIdx(localIdx);

        validation.safe = isfiniteOrPositiveInf(minClear) && ...
            minClear >= opts.safetyMargin;
        validation.numericalFailure = false;
        validation.minClear = minClear;
        validation.minPoint = clearanceOut.closestPoint(minIdx,:);
        validation.minIndex = minIdx;
        validation.numSamples = envOpts.nU;
        validation.numFeasible = getStat(env, 'numFeasible');
        validation.numValidChords = nnz(envQuery.validLine);
        validation.refinementCount = refinement;
        validation.env = envQuery;
        validation.clearance = clearanceOut.clearance;
        validation.message = ternary(validation.safe, 'safe', 'unsafe');
        validation.timeSec = toc(tAll);
        return;
    end

    validation.numericalFailure = true;
    validation.safe = false;
    validation.timeSec = toc(tAll);
end

function [tf, message] = chordConstructionValid(env, opts)
    tf = false;
    message = 'Fixed-chord construction failed.';

    required = {'validChord','vChord','NChord','chordResidual'};
    for i = 1:numel(required)
        if ~isfield(env, required{i})
            message = ['fixedChordEnvelope missing field: ' required{i}];
            return;
        end
    end

    numFeasible = getStat(env, 'numFeasible');
    numValid = nnz(env.validChord);
    if numFeasible < opts.chord.minValidChords
        message = 'Path has too few feasible length-L chords.';
        return;
    end
    if numValid < opts.chord.minValidChords || numValid < numFeasible
        message = sprintf('Only %d/%d feasible chords converged.', ...
            numValid, numFeasible);
        return;
    end

    residual = env.chordResidual(env.validChord);
    if any(~isfinite(residual)) || ...
            any(abs(residual) > env.opts.vAcceptTol)
        message = 'Fixed-chord residual exceeds acceptance tolerance.';
        return;
    end

    tf = true;
    message = 'ok';
end

function envQuery = chordOnlyEnvironment(env)
    envQuery = env;
    envQuery.v = env.vChord;
    envQuery.N = env.NChord;
    envQuery.validLine = env.validChord;
    envQuery.validSegment = false(size(env.validChord));
    envQuery.G = nan(size(env.M));
    envQuery.lambda = nan(size(env.validChord));
    envQuery.residual = env.chordResidual;
end

function d = evalBSplineDerivativeLocal(P, u, degree, knot)
    [~, d] = evalBSplinePath2D(P, u, degree, knot);
end

function value = getStat(env, name)
    value = nan;
    if isfield(env, 'stats') && isfield(env.stats, name)
        value = env.stats.(name);
    end
end

function validation = emptyValidation()
    validation = struct();
    validation.safe = false;
    validation.numericalFailure = false;
    validation.minClear = nan;
    validation.minPoint = [nan, nan];
    validation.minIndex = nan;
    validation.numSamples = 0;
    validation.numFeasible = 0;
    validation.numValidChords = 0;
    validation.refinementCount = 0;
    validation.env = [];
    validation.clearance = [];
    validation.message = '';
    validation.timeSec = 0;
end

function tf = isfiniteOrPositiveInf(x)
    tf = isfinite(x) || isinf(x) && x > 0;
end

function value = ternary(tf, a, b)
    if tf
        value = a;
    else
        value = b;
    end
end
